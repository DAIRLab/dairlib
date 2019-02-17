#include "attic/multibody/contact_toolkit.h"

namespace dairlib {
namespace multibody {

using std::vector;

using dairlib::multibody::GetBodyIndexFromName;
using drake::math::DiscardGradient;
using drake::MatrixX;
using drake::VectorX;
using Eigen::Map;
using Eigen::MatrixXd;
using Eigen::Matrix3Xd;

template <typename T>
ContactToolkit<T>::ContactToolkit(const RigidBodyTree<double>& tree,
                                ContactInfo contact_info)
    : tree_(tree),
      contact_info_(contact_info),
      num_contacts_(contact_info.idxA.size()) {}

template <typename T>
drake::MatrixX<T> ContactToolkit<T>::CalcContactJacobian(
    drake::VectorX<T> x) const {
  drake::VectorX<T> q = x.head(tree_.get_num_positions());
  drake::VectorX<T> v = x.tail(tree_.get_num_velocities());

  KinematicsCache<T> k_cache = tree_.doKinematics(q, v);

  // Index of "world" within the RBT
  const int world_ind = GetBodyIndexFromName(tree_, "world");

  // The normals at each contact are always facing upwards into z
  Matrix3Xd normal(3, num_contacts_);
  for (int i = 0; i < num_contacts_; i++) {
    normal(0, i) = 0;
    normal(1, i) = 0;
    normal(2, i) = 1;
  }

  const Map<Matrix3Xd> normal_map(normal.data(), 3, num_contacts_);

  vector<Map<Matrix3Xd>> tangents_map_vector;
  Matrix3Xd mat1 = Matrix3Xd::Zero(3, num_contacts_);
  Map<Matrix3Xd> map1(mat1.data(), 3, num_contacts_);
  Matrix3Xd mat2 = Matrix3Xd::Zero(3, num_contacts_);
  Map<Matrix3Xd> map2(mat1.data(), 3, num_contacts_);
  tangents_map_vector.push_back(map1);
  tangents_map_vector.push_back(map2);

  tree_.surfaceTangents(normal_map, tangents_map_vector);

  vector<MatrixX<T>> J_diff(num_contacts_);

  for (int i = 0; i < num_contacts_; i++) {
    auto Ja = tree_.transformPointsJacobian(k_cache, contact_info_.xA.col(i),
                                            contact_info_.idxA.at(i), world_ind,
                                            true);
    auto Jb = tree_.transformPointsJacobian(k_cache, contact_info_.xB.col(i),
                                            world_ind, world_ind, true);
    J_diff.at(i) = Jb - Ja;
  }

  // Contact Jacobians
  MatrixX<T> J(num_contacts_ * 3, tree_.get_num_positions());

  for (int i = 0; i < num_contacts_; i++) {
    // Jacobian for the individual constraints
    MatrixX<T> J_constraint(3, tree_.get_num_positions());
    // Normal
    J_constraint.row(0) = normal.col(i).transpose() * J_diff.at(i);
    // Both the surface tangents
    J_constraint.row(1) =
        tangents_map_vector.at(0).col(i).transpose() * J_diff.at(i);
    J_constraint.row(2) =
        tangents_map_vector.at(1).col(i).transpose() * J_diff.at(i);

    J.block(i * 3, 0, 3, tree_.get_num_positions()) = J_constraint;
  }

  return J;
}

template <typename T>
VectorX<T> ContactToolkit<T>::CalcMVDot(VectorX<T> x, VectorX<T> u,
                                       VectorX<T> lambda) const {
  const int num_positions = tree_.get_num_positions();
  const int num_velocities = tree_.get_num_velocities();
  const int num_efforts = tree_.get_num_actuators();
  const int num_position_constraints = tree_.getNumPositionConstraints();

  // Making sure that the size of lambda is correct
  DRAKE_THROW_UNLESS(num_position_constraints + num_contacts_ * 3 ==
                     lambda.size());

  VectorX<T> q = x.head(num_positions);
  VectorX<T> v = x.tail(num_velocities);

  KinematicsCache<T> k_cache = tree_.doKinematics(q, v);
  const MatrixX<T> M = tree_.massMatrix(k_cache);
  const typename RigidBodyTree<T>::BodyToWrenchMap no_external_wrenches;

  VectorX<T> right_hand_side =
      -tree_.dynamicsBiasTerm(k_cache, no_external_wrenches);

  if (num_efforts > 0) {
    right_hand_side += tree_.B * u;
  }

  // Position constraints Jacocbian
  if (num_position_constraints) {
    MatrixX<T> J_position = tree_.positionConstraintsJacobian(k_cache);
    right_hand_side +=
        J_position.transpose() * lambda.head(num_position_constraints);
  }

  // Contact Jacobian
  if (num_contacts_ > 0) {
    MatrixX<T> J_contact = CalcContactJacobian(x);
    right_hand_side += J_contact.transpose() * lambda.tail(num_contacts_ * 3);
  }

  // Returning right_hand_side (which is Mvdot) directly
  return right_hand_side;
}

template <typename T>
VectorX<T> ContactToolkit<T>::CalcTimeDerivatives(VectorX<T> x, VectorX<T> u,
                                                 VectorX<T> lambda) const {
  const int num_positions = tree_.get_num_positions();
  const int num_velocities = tree_.get_num_velocities();

  VectorX<T> q = x.head(num_positions);
  VectorX<T> v = x.tail(num_velocities);

  KinematicsCache<T> k_cache = tree_.doKinematics(q, v);

  const MatrixX<T> M = tree_.massMatrix(k_cache);
  // Reusing the code in CalcMVDot as the same computation is required.
  VectorX<T> right_hand_side = CalcMVDot(x, u, lambda);

  VectorX<T> v_dot = M.completeOrthogonalDecomposition().solve(right_hand_side);

  VectorX<T> x_dot(num_positions + num_velocities);
  x_dot << tree_.transformVelocityToQDot(k_cache, v), v_dot;
  return x_dot;
}

template <typename T>
ContactInfo ContactToolkit<T>::get_contact_info() {
  return contact_info_;
}

template <typename T>
int ContactToolkit<T>::get_num_contacts() {
  return num_contacts_;
}

template <typename T>
void ContactToolkit<T>::set_contact_info(ContactInfo contact_info) {
  contact_info_ = contact_info;
}

}  // namespace multibody
}  // namespace dairlib

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::ContactToolkit);
