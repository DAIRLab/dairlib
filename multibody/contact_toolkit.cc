#include "multibody/contact_toolkit.h"
#include "drake/math/orthonormal_basis.h"

namespace dairlib {
namespace multibody {

using std::vector;

using drake::multibody::MultibodyPlant;
using drake::multibody::BodyIndex;
using drake::systems::Context;
using drake::math::DiscardGradient;
using drake::MatrixX;
using drake::VectorX;
using Eigen::Map;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::Matrix3Xd;

template <typename T>
ContactToolkit<T>::ContactToolkit(const MultibodyPlant<T>& plant,
                                ContactInfo<T> contact_info)
    : plant_(plant),
      contact_info_(contact_info),
      num_contacts_(contact_info.frameA.size()) {}

template <typename T>
drake::MatrixX<T> ContactToolkit<T>::CalcContactJacobian(
    const Context<T>& context) const {
  // The normals at each contact are always facing upwards into z
  Vector3d normal;
  normal << 0, 0, 1;

  // compute tangential basis
  const Matrix3d R_contact = drake::math::ComputeBasisFromAxis(2, normal);
  const Vector3d t_hat_1 = R_contact.col(0);
  const Vector3d t_hat_2 = R_contact.col(1);


  // Contact Jacobians
  MatrixX<T> J(num_contacts_ * 3, plant_.num_positions());

  MatrixX<T> Ji(3, plant_.num_positions());

  const drake::multibody::Frame<T>& world = plant_.world_frame();
  for (int i = 0; i < num_contacts_; i++) {
    // .template cast<T> converts xA, as a double, into type T
    VectorX<T> xA_i = contact_info_.xA.col(i).template cast<T>();

    plant_.CalcJacobianTranslationalVelocity(
      context, drake::multibody::JacobianWrtVariable::kV,
      *contact_info_.frameA.at(i), xA_i, world, world, &Ji);

    J.row(i*3) = normal.transpose() * Ji;
    J.row(i*3) = t_hat_1.transpose() * Ji;
    J.row(i*3) = t_hat_2.transpose() * Ji;
  }

  return J;
}

template <typename T>
VectorX<T> ContactToolkit<T>::CalcMVDot(const Context<T>& context,
                                       VectorX<T> lambda) const {
  // const int num_positions = plant_.num_positions();
  const int num_velocities = plant_.num_velocities();
  const int num_efforts = plant_.num_actuators();
  // TODO(mposa): finish when drake is ready for constraints
  const int num_position_constraints = 0;

  // Making sure that the size of lambda is correct
  DRAKE_THROW_UNLESS(num_position_constraints + num_contacts_ * 3 ==
                     lambda.size());

  VectorX<T> right_hand_side(num_velocities);
  plant_.CalcBiasTerm(context, &right_hand_side);

  if (num_efforts > 0) {
    // will get easier after upcoming PR
    // plant_::get_actuation_input_port().Eval(context)
    VectorX<T> u = getInput(plant_, context);
    right_hand_side = -right_hand_side + plant_.MakeActuationMatrix() * u + 
        plant_.CalcGravityGeneralizedForces(context);
  }

  // TODO(mposa) update when drake is ready
  // // Position constraints Jacocbian
  // if (num_position_constraints) {
  //   MatrixX<T> J_position = tree_.positionConstraintsJacobian(k_cache);
  //   right_hand_side +=
  //       J_position.transpose() * lambda.head(num_position_constraints);
  // }

  // Contact Jacobian
  if (num_contacts_ > 0) {
    MatrixX<T> J_contact = CalcContactJacobian(context);
    right_hand_side += J_contact.transpose() * lambda.tail(num_contacts_ * 3);
  }

  // Returning right_hand_side (which is Mvdot) directly
  return right_hand_side;
}

template <typename T>
VectorX<T> ContactToolkit<T>::CalcTimeDerivatives(const Context<T>& context,
                                                 VectorX<T> lambda) const {
  const int num_positions = plant_.num_positions();
  const int num_velocities = plant_.num_velocities();

  const auto x = plant_.GetPositionsAndVelocities(context);
  const auto v = x.tail(num_velocities);


  MatrixX<T> M(num_velocities, num_velocities);
  plant_.CalcMassMatrixViaInverseDynamics(context, &M);

  // Reusing the code in CalcMVDot as the same computation is required.
  VectorX<T> right_hand_side = CalcMVDot(context, lambda);

  VectorX<T> v_dot = M.llt().solve(right_hand_side);

  VectorX<T> x_dot(num_positions + num_velocities);
  VectorX<T> q_dot(num_positions);

  plant_.MapVelocityToQDot(context, v, &q_dot);

  x_dot << q_dot, v_dot;
  return x_dot;
}

template <typename T>
ContactInfo<T> ContactToolkit<T>::get_contact_info() {
  return contact_info_;
}

template <typename T>
int ContactToolkit<T>::get_num_contacts() {
  return num_contacts_;
}

template <typename T>
void ContactToolkit<T>::set_contact_info(ContactInfo<T> contact_info) {
  contact_info_ = contact_info;
  num_contacts_ = contact_info.frameA.size();
}

}  // namespace multibody
}  // namespace dairlib

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::ContactToolkit);
