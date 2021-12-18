#include "multibody/pinocchio_plant.h"

#include <iostream>

#include "multibody/multibody_utils.h"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/multibody/joint/joint-free-flyer.hpp"
#include "pinocchio/parsers/urdf.hpp"

namespace dairlib {
namespace multibody {

using drake::AutoDiffXd;
using drake::VectorX;
using drake::multibody::Frame;
using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::map;
using std::string;

template <typename T>
PinocchioPlant<T>::PinocchioPlant(double time_step, const std::string& urdf)
    : MultibodyPlant<T>(time_step), urdf_(urdf) {}

template <typename T>
void PinocchioPlant<T>::Finalize() {
  MultibodyPlant<T>::Finalize();

  is_floating_base_ = isQuaternion(*this);
  if (is_floating_base_) {
    pinocchio::urdf::buildModel(urdf_, pinocchio::JointModelFreeFlyer(),
                                pinocchio_model_);
  } else {
    pinocchio::urdf::buildModel(urdf_, pinocchio_model_);
  }

  pinocchio_data_ = pinocchio::Data(pinocchio_model_);

  BuildPermutations();

  // Warnings
  for (int i = 0; i < this->num_actuators(); ++i) {
    auto& joint_actuator = this->get_mutable_joint_actuator(
        drake::multibody::JointActuatorIndex(i));
    if (joint_actuator.default_rotor_inertia() > 0) {
      cout << "WARNING: currently PinoccioPlant doesn't have reflected "
              "inertia!\n";
      break;
    }
  }

  // Check that models match
  int nq = this->num_positions();
  int nv = this->num_velocities();
  int nu = this->num_actuators();

  VectorX<T> x = VectorX<T>::Random(nq + nv);
  VectorX<T> u = 0 * VectorX<T>::Random(nu);
  VectorX<T> vdot = 0 * VectorX<T>::Random(nv);
  if (is_floating_base_) {
    x.head(4) = x.head(4) / x.head(4).norm();
    //        x = 0 * x;
    //        x(0) = 1;

    //    x.head(4) << 1, 0, 0, 0;
    //
    //    x(4) = 1;
    //
    //    x(nq + 3) = 0.5;
    //    x(nq + 4) = 0.6;
    //    x(nq + 5) = 0.7;
  }

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
  if (is_floating_base_) {
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

template <typename T>
void PinocchioPlant<T>::BuildPermutations() {
  map<string, int> pos_map = makeNameToPositionsMap(*this);
  map<string, int> vel_map = makeNameToVelocitiesMap(*this);
  int nq = this->num_positions();
  int nv = this->num_velocities();
  Eigen::VectorXi pos_indices(nq);
  Eigen::VectorXi vel_indices(nv);

  int q_idx = 0;
  int v_idx = 0;
  for (int name_idx = 1; name_idx < pinocchio_model_.names.size(); name_idx++) {
    // TODO: floating base options
    // Skipping i=0 for the world (TODO--doesn't handle floating base yet)
    // Assumes that URDF root is welded to the world
    const auto& name = pinocchio_model_.names[name_idx];

    if (name == "root_joint") {
      // Pinocchio's floating base position is (x, y, z, qx,qy,qz,qw)
      // Pinocchio's floating base velocity is (vx,vy,vz,wx,wy,wz)
      // Note that
      // 1. the first element of quaternion is qx instead of qw.
      // 2. the velocity seems to be expressed in local frame
      pos_indices.head<7>() << 4, 5, 6, 1, 2, 3, 0;
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

  q_perm_.indices() = pos_indices;
  v_perm_.indices() = vel_indices;
}

template <typename T>
drake::VectorX<T> PinocchioPlant<T>::MapPositionFromDrakeToPinocchio(
    const drake::VectorX<T>& q) const {
  return q_perm_.inverse() * q;
}

template <typename T>
drake::VectorX<T> PinocchioPlant<T>::MapVelocityFromDrakeToPinocchio(
    const drake::VectorX<T>& quat, const drake::VectorX<T>& v) const {
  if (is_floating_base_) {
    drake::MatrixX<T> rot =
        Eigen::Quaternion<T>(quat(0), quat(1), quat(2), quat(3))
            .toRotationMatrix()
            .transpose();
    drake::VectorX<T> v_rotated = v;
    v_rotated.template head<3>() = rot * v_rotated.template head<3>();
    v_rotated.template segment<3>(3) = rot * v_rotated.template segment<3>(3);
    return v_perm_.inverse() * v_rotated;
  } else {
    return v_perm_.inverse() * v;
  }
}

template <typename T>
drake::VectorX<T> PinocchioPlant<T>::MapVelocityFromPinocchioToDrake(
    const drake::VectorX<T>& quat, const drake::VectorX<T>& v) const {
  if (is_floating_base_) {
    drake::MatrixX<T> rot =
        Eigen::Quaternion<T>(quat(0), quat(1), quat(2), quat(3))
            .toRotationMatrix();
    drake::VectorX<T> v_rotated = v;
    v_rotated.template head<3>() = rot * v_rotated.template head<3>();
    v_rotated.template segment<3>(3) = rot * v_rotated.template segment<3>(3);
    return v_perm_ * v_rotated;
  } else {
    return v_perm_ * v;
  }
}

template <typename T>
drake::MatrixX<T> PinocchioPlant<T>::GetVelocityMapFromDrakeToPinocchio(
    const drake::VectorX<T>& quat) const {
  drake::MatrixX<T> ret = drake::MatrixX<T>::Identity(this->num_velocities(),
                                                      this->num_velocities());
  if (is_floating_base_) {
    drake::MatrixX<T> rot =
        Eigen::Quaternion<T>(quat(0), quat(1), quat(2), quat(3))
            .toRotationMatrix()
            .transpose();
    ret.template block<3, 3>(0, 0) = rot;
    ret.template block<3, 3>(3, 3) = rot;
  }
  return v_perm_.inverse() * ret;
}

template <typename T>
drake::MatrixX<T> PinocchioPlant<T>::GetVelocityMapFromPinocchioToDrake(
    const drake::VectorX<T>& quat) const {
  drake::MatrixX<T> ret = drake::MatrixX<T>::Identity(this->num_velocities(),
                                                      this->num_velocities());
  if (is_floating_base_) {
    drake::MatrixX<T> rot =
        Eigen::Quaternion<T>(quat(0), quat(1), quat(2), quat(3))
            .toRotationMatrix();
    ret.template block<3, 3>(0, 0) = rot;
    ret.template block<3, 3>(3, 3) = rot;
  }
  return v_perm_ * ret;
}

template <typename T>
void PinocchioPlant<T>::RightMultiplicationFromDrakeToPinocchio(
    const drake::VectorX<T>& quat, drake::EigenPtr<drake::MatrixX<T>> M) const {
  (*M) = (*M) * v_perm_.inverse();
  if (is_floating_base_) {
    drake::MatrixX<T> rot =
        Eigen::Quaternion<T>(quat(0), quat(1), quat(2), quat(3))
            .toRotationMatrix()
            .transpose();
    M->template leftCols<3>() = M->template leftCols<3>() * rot;
    M->template middleCols<3>(3) = M->template middleCols<3>(3) * rot;
  }
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

  // TODO: currently CalcInverseDynamics doesn't pass the test when the MBP has
  // floating base

  auto f_pin = pinocchio::rnea(
      pinocchio_model_, pinocchio_data_,
      MapPositionFromDrakeToPinocchio(GetPositions(context)),
      MapVelocityFromDrakeToPinocchio(GetPositions(context).head<4>(),
                                      GetVelocities(context)),
      MapVelocityFromDrakeToPinocchio(GetPositions(context).head<4>(),
                                      known_vdot));
  return MapVelocityFromPinocchioToDrake(GetPositions(context).head<4>(),
                                         f_pin) -
      external_forces.generalized_forces();
}

template <>
void PinocchioPlant<double>::CalcMassMatrix(
    const Context<double>& context, drake::EigenPtr<Eigen::MatrixXd> M) const {
  pinocchio::crba(pinocchio_model_, pinocchio_data_,
                  MapPositionFromDrakeToPinocchio(GetPositions(context)));

  // Pinocchio builds an upper triangular matrix, skipping the parts
  // below the diagonal. Fill those in here.
  *M = pinocchio_data_.M;
  for (int i = 0; i < M->cols(); i++) {
    for (int j = i + 1; j < M->rows(); j++) {
      (*M)(j, i) = (*M)(i, j);
    }
  }
  // TODO: we can speed this up by not doing full matrix multiplication here.
  // Similar to RightMultiplicationFromDrakeToPinocchio
  *M = GetVelocityMapFromPinocchioToDrake(GetPositions(context).head<4>()) *
      (*M) *
      GetVelocityMapFromDrakeToPinocchio(GetPositions(context).head<4>());
}

template <>
void PinocchioPlant<AutoDiffXd>::CalcMassMatrix(
    const Context<AutoDiffXd>& context,
    drake::EigenPtr<drake::MatrixX<AutoDiffXd>> M) const {
  throw std::domain_error("CalcMassMatrix not implemented with AutoDiffXd");
}

template <>
drake::Vector3<double> PinocchioPlant<double>::CalcCenterOfMassPositionInWorld(
    const Context<double>& context) const {
  pinocchio::centerOfMass(
      pinocchio_model_, pinocchio_data_,
      MapPositionFromDrakeToPinocchio(GetPositions(context)));

  return pinocchio_data_.com[0];
}

template <>
drake::Vector3<AutoDiffXd>
PinocchioPlant<AutoDiffXd>::CalcCenterOfMassPositionInWorld(
    const Context<AutoDiffXd>& context) const {
  throw std::domain_error(
      "CalcCenterOfMassPositionInWorld not implemented with AutoDiffXd");
}

template <>
drake::Vector3<double>
PinocchioPlant<double>::CalcCenterOfMassTranslationalVelocityInWorld(
    const Context<double>& context) const {
  pinocchio::centerOfMass(
      pinocchio_model_, pinocchio_data_,
      MapPositionFromDrakeToPinocchio(GetPositions(context)),
      MapVelocityFromDrakeToPinocchio(GetPositions(context).head<4>(),
                                      GetVelocities(context)));
  return pinocchio_data_.vcom[0];
}

template <>
drake::Vector3<AutoDiffXd>
PinocchioPlant<AutoDiffXd>::CalcCenterOfMassTranslationalVelocityInWorld(
    const Context<AutoDiffXd>& context) const {
  throw std::domain_error(
      "CalcCenterOfMassTranslationalVelocityInWorld not implemented with "
      "AutoDiffXd");
}

template <>
void PinocchioPlant<double>::CalcJacobianCenterOfMassTranslationalVelocity(
    const Context<double>& context, JacobianWrtVariable with_respect_to,
    const Frame<double>& frame_A, const Frame<double>& frame_E,
    drake::EigenPtr<drake::Matrix3X<double>> J) const {
  DRAKE_DEMAND(frame_A.is_world_frame());
  DRAKE_DEMAND(frame_E.is_world_frame());
  *J = pinocchio::jacobianCenterOfMass(
      pinocchio_model_, pinocchio_data_,
      MapPositionFromDrakeToPinocchio(GetPositions(context)));
  RightMultiplicationFromDrakeToPinocchio(GetPositions(context).head<4>(), J);
}

template <>
void PinocchioPlant<AutoDiffXd>::CalcJacobianCenterOfMassTranslationalVelocity(
    const Context<AutoDiffXd>& context, JacobianWrtVariable with_respect_to,
    const Frame<AutoDiffXd>& frame_A, const Frame<AutoDiffXd>& frame_E,
    drake::EigenPtr<drake::Matrix3X<AutoDiffXd>> J) const {
  throw std::domain_error("CalcMassMatrix not implemented with AutoDiffXd");
}

/// Comparisons against MultibodyPlant

template <>
bool PinocchioPlant<double>::TestInverseDynamics(
    const drake::systems::Context<double>& context, const VectorXd& known_vdot,
    const drake::multibody::MultibodyForces<double>& external_forces,
    double tol) const {
  auto f = MultibodyPlant<double>::CalcInverseDynamics(context, known_vdot,
                                                       external_forces);
  auto pin_f = CalcInverseDynamics(context, known_vdot, external_forces);

  return (f - pin_f).norm() < tol;
}

template <>
bool PinocchioPlant<double>::TestMassMatrix(const Context<double>& context,
                                            double tol) const {
  int nv = num_velocities();

  MatrixXd M(nv, nv);
  MatrixXd pin_M(nv, nv);

  MultibodyPlant<double>::CalcMassMatrix(context, &M);

  CalcMassMatrix(context, &pin_M);

  return (M - pin_M).norm() < tol;
}

template <>
bool PinocchioPlant<double>::TestCenterOfMass(const Context<double>& context,
                                              double tol) const {
  Eigen::Vector3d com =
      MultibodyPlant<double>::CalcCenterOfMassPositionInWorld(context);
  Eigen::Vector3d pin_com = CalcCenterOfMassPositionInWorld(context);
  std::cout << "com = " << com.transpose() << std::endl;
  std::cout << "pin_com = " << pin_com.transpose() << std::endl;

  return (com - pin_com).norm() < tol;
}

template <>
bool PinocchioPlant<double>::TestCenterOfMassVel(const Context<double>& context,
                                                 double tol) const {
  Eigen::Vector3d com_vel =
      MultibodyPlant<double>::CalcCenterOfMassTranslationalVelocityInWorld(
          context);
  Eigen::Vector3d pin_com_vel =
      CalcCenterOfMassTranslationalVelocityInWorld(context);
  std::cout << "com_vel = " << com_vel.transpose() << std::endl;
  std::cout << "pin_com_vel = " << pin_com_vel.transpose() << std::endl;

  return (com_vel - pin_com_vel).norm() < tol;
}

template <>
bool PinocchioPlant<double>::TestCenterOfMassJ(const Context<double>& context,
                                               double tol) const {
  int nv = num_velocities();

  MatrixXd J(3, nv);
  MatrixXd pin_J(3, nv);

  MultibodyPlant<double>::CalcJacobianCenterOfMassTranslationalVelocity(
      context, JacobianWrtVariable::kV, this->world_frame(),
      this->world_frame(), &J);

  CalcJacobianCenterOfMassTranslationalVelocity(
      context, JacobianWrtVariable::kV, this->world_frame(),
      this->world_frame(), &pin_J);
  //  std::cout << "J = \n" << J << std::endl;
  //  std::cout << "pin_J = \n" << pin_J << std::endl;
  //  std::cout << "J - pin_J = \n" << J - pin_J << std::endl;

  return (J - pin_J).norm() < tol;
}

template <>
bool PinocchioPlant<AutoDiffXd>::TestInverseDynamics(
    const drake::systems::Context<AutoDiffXd>& context,
    const drake::VectorX<AutoDiffXd>& known_vdot,
    const drake::multibody::MultibodyForces<AutoDiffXd>& external_forces,
    double tol) const {
  throw std::domain_error(
      "TestInverseDynamics not implemented with AutoDiffXd");
}

template <>
bool PinocchioPlant<AutoDiffXd>::TestMassMatrix(
    const Context<AutoDiffXd>& context, double tol) const {
  throw std::domain_error("TestMassMatrix not implemented with AutoDiffXd");
}

template <>
bool PinocchioPlant<AutoDiffXd>::TestCenterOfMass(
    const Context<AutoDiffXd>& context, double tol) const {
  throw std::domain_error(
      "CalcCenterOfMassPositionInWorld not implemented with AutoDiffXd");
}

template <>
bool PinocchioPlant<AutoDiffXd>::TestCenterOfMassVel(
    const Context<AutoDiffXd>& context, double tol) const {
  throw std::domain_error(
      "CalcCenterOfMassPositionInWorld not implemented with AutoDiffXd");
}

template <>
bool PinocchioPlant<AutoDiffXd>::TestCenterOfMassJ(
    const Context<AutoDiffXd>& context, double tol) const {
  throw std::domain_error("TestCenterOfMassJ not implemented with AutoDiffXd");
}


template<>
void PinocchioPlant<double>::CalcCentroidalMomentumAndDerivatives(
    const drake::systems::Context<double> &context,
    const drake::EigenPtr<Eigen::VectorXd>& h,
    const drake::EigenPtr<Eigen::MatrixXd>& A,
    const drake::EigenPtr<Eigen::MatrixXd>& Adot) const {

  auto dAg = pinocchio::dccrba(pinocchio_model_, pinocchio_data_,
                                 q_perm_.inverse() * GetPositions(context),
                                 v_perm_.inverse() * GetVelocities(context));

  *A = v_perm_ *  pinocchio_data_.Ag;
  *h = (*A) * GetVelocities(context);
  *Adot = v_perm_ * dAg;
}

}  // namespace multibody
}  // namespace dairlib

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class dairlib::multibody::PinocchioPlant)
