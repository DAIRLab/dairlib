#include "multibody/pinocchio_plant.h"
#include "multibody/multibody_utils.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/multibody/joint/joint-free-flyer.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include <iostream>

namespace dairlib {
namespace multibody {

using drake::AutoDiffXd;
using drake::VectorX;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::map;
using std::string;
using std::cout;
using std::endl;

template <typename T>
PinocchioPlant<T>::PinocchioPlant(double time_step, const std::string& urdf) :
      MultibodyPlant<T>(time_step), urdf_(urdf) {
}

template<typename T>
void PinocchioPlant<T>::Finalize() {
  MultibodyPlant<T>::Finalize();

  if (isQuaternion(*this)) {
    pinocchio::urdf::buildModel(urdf_, pinocchio::JointModelFreeFlyer(),
                                pinocchio_model_);
  } else {
    pinocchio::urdf::buildModel(urdf_, pinocchio_model_);
  }

  // Do i need to buildReducedModel? Do What about joints?
  pinocchio_data_ = pinocchio::Data(pinocchio_model_);

  BuildPermutations();

  // Check that models match
  int nq = this->num_positions();
  int nv = this->num_velocities();
  int nu = this->num_actuators();

  VectorX<T> x =  VectorX<T>::Random(nq + nv);
  VectorX<T> u = 0*VectorX<T>::Random(nu);
  VectorX<T> vdot =  0*VectorX<T>::Random(nv);
  if (isQuaternion(*this)) {
    // x.head(4) = x.head(4) / x.head(4).norm();
    x = 0 * x;
    x(0) = 1;
  }
  //  x.head(nq) << 0.0195003, -0.0195003, 0, 0, 0.46061, 0.46061, -1.17829,
  //      -1.17829, -0.0118189, -0.0192588, 1.45555, 1.45425, -0.0466754,
  //      -1.59953, -0.0319659, -1.59137;

  //  x.head(nq) << 0.0194984, -0.0194984, 0, 0, 0.499801, 0.499801, -1.22315,
  //      -1.22315, 1.44726, 1.4471, -1.5974, -1.59782;

  auto context = createContext<T>(*this, x, u);
  // TODO: need to test actual forces
  drake::multibody::MultibodyForces<T> forces(*this);
  this->CalcForceElementsContribution(*context, &forces);

  if (!TestInverseDynamics(*context, vdot, forces, 1e-6)) {
    std::cout << "PinocchioPlant TestInverseDynamics FAILED!!" << std::endl;
  }
  if (!TestMassMatrix(*context, 1e-6)) {
    std::cout << "PinocchioPlant TestMassMatrix FAILED!!" << std::endl;
  }
  if (isQuaternion(*this)) {
    // Pinocchio doesn't take the fixed-base body into account when computing
    // the COM
    if (!TestCenterOfMass(*context, 1e-6)) {
      std::cout << "PinocchioPlant TestCenterOfMass FAILED!!" << std::endl;
    }
    if (!TestCenterOfMassVel(*context, 1e-6)) {
      std::cout << "PinocchioPlant TestCenterOfMassVel FAILED!!" << std::endl;
    }
    if (!TestCenterOfMassJ(*context, 1e-6)) {
      std::cout << "PinocchioPlant TestCenterOfMassJ FAILED!!" << std::endl;
    }
  }
}

template<typename T>
void PinocchioPlant<T>::BuildPermutations() {
  map<string, int> pos_map = makeNameToPositionsMap(*this);
  map<string, int> vel_map = makeNameToVelocitiesMap(*this);
  int nq = this->num_positions();
  int nv = this->num_velocities();
  Eigen::VectorXi pos_indices(nq);
  Eigen::VectorXi vel_indices(nv);
  cout << "nq = " << nq << endl;
  cout << "nv = " << nv << endl;

  int q_idx = 0;
  int v_idx = 0;
  for (int name_idx = 1; name_idx < pinocchio_model_.names.size(); name_idx++) {
    // TODO: floating base options
    // Skipping i=0 for the world (TODO--doesn't handle floating base yet)
    // Assumes that URDF root is welded to the world
    const auto& name = pinocchio_model_.names[name_idx];

    if (name == "root_joint") {
      pos_indices.head<7>() << 4, 5, 6, 0, 1, 2, 3;
      vel_indices.head<6>() << 3, 4, 5, 0, 1, 2;
      q_idx += 7;
      v_idx += 6;
    } else {
      if (pos_map.count(name) == 0) {
        throw std::runtime_error("PinocchioPlant::BuildPermutations: " + name +
                                 " was not found in the position map.");
      }

      if (vel_map.count(name + "dot") == 0) {
        throw std::runtime_error("PinocchioPlant::BuildPermutations: " + name +
                                 " was not found in the velocity map.");
      }

      pos_indices(q_idx) = pos_map[name];
      vel_indices(v_idx) = vel_map[name + "dot"];
      q_idx++;
      v_idx++;
    }
  }
  cout << "pos_indices = \n" << pos_indices << endl;
  cout << "vel_indices = \n" << vel_indices << endl;

  q_perm_.indices() = pos_indices;
  v_perm_.indices() = vel_indices;

  cout << "q_perm_ = \n" << q_perm_.toDenseMatrix() << endl;
}

template <>
VectorXd PinocchioPlant<double>::CalcInverseDynamics(
    const drake::systems::Context<double>& context, const VectorXd& known_vdot,
    const drake::multibody::MultibodyForces<double>& external_forces) const {

  // TODO: support body forces
  if (external_forces.body_forces().size() > 0) {
    // throw std::runtime_error(
    // "PinocchioPlant::CalcInverseDynamics: body forces not yet supported");
  }

  auto f_pin = pinocchio::rnea(pinocchio_model_, pinocchio_data_,
                               q_perm_.inverse() * GetPositions(context),
                               v_perm_.inverse() * GetVelocities(context),
                               v_perm_.inverse() * known_vdot);
  return v_perm_ * f_pin - external_forces.generalized_forces();
}

template <>
void PinocchioPlant<double>::CalcMassMatrix(const Context<double>& context,
    drake::EigenPtr<Eigen::MatrixXd> M) const {
  pinocchio::crba(pinocchio_model_, pinocchio_data_,
                  q_perm_.inverse() * GetPositions(context));

  // Pinocchio builds an upper triangular matrix, skipping the parts
  // below the diagonal. Fill those in here.
  *M = pinocchio_data_.M;
  for (int i = 0; i < M->cols(); i++) {
    for (int j = i + 1; j < M->rows(); j++) {
      (*M)(j, i) = (*M)(i, j);
    }
  }
  *M = v_perm_ * (*M) * v_perm_.inverse();
}

template<>
void PinocchioPlant<AutoDiffXd>::CalcMassMatrix(
    const Context<AutoDiffXd>& context,
    drake::EigenPtr<drake::MatrixX<AutoDiffXd>> M) const {
  throw std::domain_error("CalcMassMatrix not implemented with AutoDiffXd");
}

template <>
void PinocchioPlant<double>::CalcCenterOfMassPositionInWorld(
    const Context<double>& context,
    drake::EigenPtr<drake::VectorX<double>> r_com) const {
  pinocchio::centerOfMass(pinocchio_model_, pinocchio_data_,
                          q_perm_.inverse() * GetPositions(context));

  *r_com = pinocchio_data_.com[0];
}

template <>
void PinocchioPlant<AutoDiffXd>::CalcCenterOfMassPositionInWorld(
    const Context<AutoDiffXd>& context,
    drake::EigenPtr<drake::VectorX<AutoDiffXd>> r_com) const {
  throw std::domain_error(
      "CalcCenterOfMassPositionInWorld not implemented with AutoDiffXd");
}

template <>
void PinocchioPlant<double>::CalcCenterOfMassTranslationalVelocityInWorld(
    const Context<double>& context,
    drake::EigenPtr<drake::VectorX<double>> v_com) const {
  pinocchio::centerOfMass(pinocchio_model_, pinocchio_data_,
                          q_perm_.inverse() * GetPositions(context),
                          v_perm_.inverse() * GetVelocities(context));
  *v_com = pinocchio_data_.vcom[0];
}

template <>
void PinocchioPlant<AutoDiffXd>::CalcCenterOfMassTranslationalVelocityInWorld(
    const Context<AutoDiffXd>& context,
    drake::EigenPtr<drake::VectorX<AutoDiffXd>> v_com) const {
  throw std::domain_error(
      "CalcCenterOfMassTranslationalVelocityInWorld not implemented with "
      "AutoDiffXd");
}

template <>
void PinocchioPlant<double>::CalcJacobianCenterOfMassTranslationalVelocity(
    const Context<double>& context, drake::EigenPtr<Eigen::MatrixXd> J) const {
  *J = pinocchio::jacobianCenterOfMass(
           pinocchio_model_, pinocchio_data_,
           q_perm_.inverse() * GetPositions(context)) *
       v_perm_.inverse();
}

template <>
void PinocchioPlant<AutoDiffXd>::CalcJacobianCenterOfMassTranslationalVelocity(
    const Context<AutoDiffXd>& context,
    drake::EigenPtr<drake::MatrixX<AutoDiffXd>> J) const {
  throw std::domain_error("CalcMassMatrix not implemented with AutoDiffXd");
}

/// Comparisons against MultibodyPlant

template <>
::testing::AssertionResult PinocchioPlant<double>::TestInverseDynamics(
    const drake::systems::Context<double>& context, const VectorXd& known_vdot,
    const drake::multibody::MultibodyForces<double>& external_forces,
    double tol) const {
  auto f = MultibodyPlant<double>::CalcInverseDynamics(context, known_vdot,
                                                       external_forces);
  auto pin_f = CalcInverseDynamics(context, known_vdot, external_forces);

  return drake::CompareMatrices(f, pin_f, tol);
}

template<>
::testing::AssertionResult PinocchioPlant<double>::TestMassMatrix(
    const Context<double>& context, double tol) const {
  int nv = num_velocities();

  MatrixXd M(nv, nv);
  MatrixXd pin_M(nv, nv);

  MultibodyPlant<double>::CalcMassMatrix(context, &M);

  CalcMassMatrix(context, &pin_M);

  return drake::CompareMatrices(M, pin_M, tol);
}

template<>
::testing::AssertionResult PinocchioPlant<double>::TestCenterOfMass(
    const Context<double>& context, double tol) const {

  Eigen::Vector3d com;
  Eigen::Vector3d pin_com;

  com = MultibodyPlant<double>::CalcCenterOfMassPositionInWorld(context);

  CalcCenterOfMassPositionInWorld(context, &pin_com);
  std::cout << "com = " << com.transpose() << std::endl;
  std::cout << "pin_com = " << pin_com.transpose() << std::endl;

  return drake::CompareMatrices(com, pin_com, tol);
}

template <>
::testing::AssertionResult PinocchioPlant<double>::TestCenterOfMassVel(
    const Context<double>& context, double tol) const {
  Eigen::Vector3d com_vel;
  Eigen::Vector3d pin_com_vel;

  com_vel =
      MultibodyPlant<double>::CalcCenterOfMassTranslationalVelocityInWorld(
          context);

  CalcCenterOfMassTranslationalVelocityInWorld(context, &pin_com_vel);
  std::cout << "com_vel = " << com_vel.transpose() << std::endl;
  std::cout << "pin_com_vel = " << pin_com_vel.transpose() << std::endl;

  return drake::CompareMatrices(com_vel, pin_com_vel, tol);
}

template <>
::testing::AssertionResult PinocchioPlant<double>::TestCenterOfMassJ(
    const Context<double>& context, double tol) const {
  int nv = num_velocities();

  MatrixXd J(3, nv);
  MatrixXd pin_J(3, nv);
  //  MatrixXd pin_J(3, num_positions());

  MultibodyPlant<double>::CalcJacobianCenterOfMassTranslationalVelocity(
      context, drake::multibody::JacobianWrtVariable::kV, this->world_frame(),
      this->world_frame(), &J);

  CalcJacobianCenterOfMassTranslationalVelocity(context, &pin_J);
  std::cout << "J = \n" << J << std::endl;
  std::cout << "pin_J = \n" << pin_J << std::endl;
  std::cout << "J - pin_J = \n" << J - pin_J << std::endl;

  return drake::CompareMatrices(J, pin_J, tol);
}

template<>
::testing::AssertionResult PinocchioPlant<AutoDiffXd>::TestMassMatrix(
    const Context<AutoDiffXd>& context, double tol) const {
  throw std::domain_error("TestMassMatrix not implemented with AutoDiffXd");
}

template <>
::testing::AssertionResult PinocchioPlant<AutoDiffXd>::TestCenterOfMass(
    const Context<AutoDiffXd>& context, double tol) const {
  throw std::domain_error(
      "CalcCenterOfMassPositionInWorld not implemented with AutoDiffXd");
}

template <>
::testing::AssertionResult PinocchioPlant<AutoDiffXd>::TestCenterOfMassVel(
    const Context<AutoDiffXd>& context, double tol) const {
  throw std::domain_error(
      "CalcCenterOfMassPositionInWorld not implemented with AutoDiffXd");
}

template <>
::testing::AssertionResult PinocchioPlant<AutoDiffXd>::TestCenterOfMassJ(
    const Context<AutoDiffXd>& context, double tol) const {
  throw std::domain_error("TestCenterOfMassJ not implemented with AutoDiffXd");
}

}  // namespace multibody
}  // namespace dairlib


DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class dairlib::multibody::PinocchioPlant)
