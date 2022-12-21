#include "multibody/pinocchio_plant.h"

#include <iostream>

#include "multibody/multibody_utils.h"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/centroidal-derivatives.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames-derivatives.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/multibody/frame.hpp"
#include "pinocchio/multibody/joint/joint-free-flyer.hpp"
#include "pinocchio/parsers/urdf.hpp"

namespace dairlib {
namespace multibody {

using drake::AutoDiffXd;
using drake::Matrix3X;
using drake::Matrix6X;
using drake::MatrixX;
using drake::Vector3;
using drake::VectorX;
using drake::math::ExtractValue;
using drake::multibody::Frame;
using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::multibody::SpatialMomentum;
using drake::systems::Context;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using multibody::HasQuaternion;
using std::cout;
using std::endl;
using std::map;
using std::string;

// template <typename T>
// PinocchioPlant<T>::PinocchioPlant(const MultibodyPlant<double>& plant, const
// std::string& urdf)
//    : MultibodyPlant<T>(plant), urdf_(urdf) {}
template <>
PinocchioPlant<AutoDiffXd>::PinocchioPlant(const MultibodyPlant<double>& plant,
                                           const std::string& urdf)
    : MultibodyPlant<AutoDiffXd>(plant), urdf_(urdf) {}

// template <typename T>
// PinocchioPlant<T>::PinocchioPlant(double time_step, const std::string& urdf)
//    : MultibodyPlant<T>(time_step), urdf_(urdf) {}
template <>
PinocchioPlant<double>::PinocchioPlant(double time_step,
                                       const std::string& urdf)
    : MultibodyPlant<double>(time_step), urdf_(urdf) {}

template <typename T>
void PinocchioPlant<T>::FinalizePlant() {
  //  MultibodyPlant<T>::Finalize();

  is_floating_base_ = HasQuaternion(*this);
  if (is_floating_base_) {
    pinocchio::urdf::buildModel(urdf_, pinocchio::JointModelFreeFlyer(),
                                pinocchio_model_);
  } else {
    pinocchio::urdf::buildModel(urdf_, pinocchio_model_);
  }

  // Do i need to buildReducedModel? Do What about joints?
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

  n_q_ = nq;
  n_v_ = nv;
}

template <typename T>
void PinocchioPlant<T>::BuildPermutations() {
  map<string, int> pos_map = MakeNameToPositionsMap(*this);
  map<string, int> vel_map = MakeNameToVelocitiesMap(*this);
  int nq = this->num_positions();
  int nv = this->num_velocities();
  Eigen::VectorXi pos_indices(nq);
  Eigen::VectorXi vel_indices(nv);
  vq_perm_ = MatrixXd::Zero(nv, nq);

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
      vq_perm_(3, 1) = 1;
      vq_perm_(4, 2) = 1;
      vq_perm_(5, 3) = 1;
      vq_perm_(0, 4) = 1;
      vq_perm_(1, 5) = 1;
      vq_perm_(2, 6) = 1;
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
      vq_perm_(vel_map[name + "dot"], q_idx) = 1;
      vel_indices(v_idx) = vel_map[name + "dot"];
      q_idx++;
      v_idx++;
    }
  }

  q_perm_.indices() = pos_indices;
  v_perm_.indices() = vel_indices;
  std::cout << "rows: " << vq_perm_.rows() << ", cols: " << vq_perm_.cols()
            << std::endl;
}

template <typename T>
drake::VectorX<double> PinocchioPlant<T>::MapPositionFromDrakeToPinocchio(
    const drake::VectorX<double>& q) const {
  //  return q_perm_.inverse() * q;
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
drake::MatrixX<double> PinocchioPlant<T>::GetVelocityMapFromPinocchioToDrake(
    const drake::VectorX<double>& quat) const {
  drake::MatrixX<double> ret = drake::MatrixX<double>::Identity(
      this->num_velocities(), this->num_velocities());
  if (is_floating_base_) {
    drake::MatrixX<double> rot =
        Eigen::Quaternion<double>(quat(0), quat(1), quat(2), quat(3))
            .toRotationMatrix();
    ret.template block<3, 3>(0, 0) = rot;
    ret.template block<3, 3>(3, 3) = rot;
  }
  return v_perm_ * ret;
}

template <typename T>
void PinocchioPlant<T>::RightMultiplicationFromDrakeToPinocchio(
    const drake::VectorX<double>& quat,
    drake::EigenPtr<drake::MatrixX<double>> M) const {
  (*M) = (*M) * v_perm_.inverse();
  if (is_floating_base_) {
    drake::MatrixX<double> rot =
        Eigen::Quaternion<double>(quat(0), quat(1), quat(2), quat(3))
            .toRotationMatrix()
            .transpose();
    M->leftCols(3) = M->leftCols(3) * rot;
    M->middleCols(3, 3) = M->middleCols(3, 3) * rot;
  }
}

template <>
void PinocchioPlant<AutoDiffXd>::UpdateForwardKinematicsDerivatives(
    const Context<AutoDiffXd>& context) {
  drake::VectorX<double> a = drake::VectorX<double>::Zero(n_v_);
  drake::VectorX<double> q =
      MapPositionFromDrakeToPinocchio(ExtractValue(GetPositions(context)));
  drake::VectorX<double> v = ExtractValue(MapVelocityFromDrakeToPinocchio(
      GetPositions(context).head<4>(), GetVelocities(context)));
  pinocchio::computeForwardKinematicsDerivatives(pinocchio_model_,
                                                 pinocchio_data_, q, v, a);
}

template <>
void PinocchioPlant<double>::UpdateCentroidalDynamics(
    const Context<double>& context) const {
  drake::VectorX<double> a = drake::VectorX<double>::Zero(n_v_);
//  drake::Matrix6X<double> dhdq(6, n_v_);
//  drake::Matrix6X<double> dhdotdq(6, n_v_);
//  drake::Matrix6X<double> dhdotdv(6, n_v_);
//  drake::Matrix6X<double> dhdotda(6, n_v_);
  pinocchio::computeCentroidalMomentum(
      pinocchio_model_, pinocchio_data_,
      MapPositionFromDrakeToPinocchio(GetPositions(context)),
      MapVelocityFromDrakeToPinocchio(GetPositions(context).head<4>(),
                                      GetVelocities(context)));
}

template <>
void PinocchioPlant<AutoDiffXd>::UpdateCentroidalDynamicsDerivatives(
    const Context<AutoDiffXd>& context) const {
  drake::VectorX<double> a = drake::VectorX<double>::Zero(n_v_);
  drake::Matrix6X<double> dhdq(6, n_v_);
  drake::Matrix6X<double> dhdotdq(6, n_v_);
  drake::Matrix6X<double> dhdotdv(6, n_v_);
  drake::Matrix6X<double> dhdotda(6, n_v_);
  //  std::cout << "here: " << std::endl;
  drake::VectorX<double> q =
      MapPositionFromDrakeToPinocchio(ExtractValue(GetPositions(context)));
  drake::VectorX<double> v = ExtractValue(MapVelocityFromDrakeToPinocchio(
      GetPositions(context).head<4>(), GetVelocities(context)));
  pinocchio::computeCentroidalDynamicsDerivatives(
      pinocchio_model_, pinocchio_data_, q, v, a, dhdq, dhdotdq, dhdotdv,
      dhdotda);
  // the centroidal momentum matrix is equivalent to dhdot_da.
  pinocchio_data_.Ag = dhdotda;
  pinocchio_data_.dHdq = dhdq;
  // Filling in data.dFdq is a hack since data.dhdq is not available
  //  pinocchio_data_.dFdq.setZero(6, n_q_);
  //  pinocchio_data_.dFdq.template middleCols<3>(3) =
  //      getCentroidalMomentumZyxGradient(interface, info, q, v);
  //  pinocchio::updateFramePlacements(pinocchio_model_, pinocchio_data_);
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
  VectorXd q_drake = ExtractValue(GetPositions(context));
  VectorXd q_pin = MapPositionFromDrakeToPinocchio(q_drake);
  auto drake_quat =
      Eigen::Quaternion<double>(q_drake(0), q_drake(1), q_drake(2), q_drake(3));
  drake::MatrixX<double> rot = drake_quat.toRotationMatrix().transpose();
  Matrix3X<double> gradient = MatrixXd(3, n_q_ + n_v_);
  Vector3<double> gradient_qw = Vector3d::Zero();
  Matrix3X<double> gradient_q =
      pinocchio::jacobianCenterOfMass(pinocchio_model_, pinocchio_data_, q_pin);
  Matrix3X<double> gradient_v = MatrixXd::Zero(3, n_v_);
  auto map = drake::multibody::internal::QuaternionFloatingMobilizer<
      double>::AngularVelocityToQuaternionRateMatrix(drake_quat);
  MatrixXd gradient_quat = 4 * map * rot.transpose() * gradient_q.block<3, 3>(0, 3).transpose();
  Matrix3X<double> gradient_pos = gradient_q.block<3, 3>(0, 0) * rot;
  gradient_quat.transposeInPlace();
  Matrix3X<double> drake_gradient = Matrix3X<double>::Zero(3, n_q_);
  drake_gradient << gradient_quat, gradient_pos, gradient_q.rightCols(n_q_ - 7);
  gradient << drake_gradient, gradient_v;
  return drake::math::InitializeAutoDiff(pinocchio_data_.com[0], gradient);
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

template <>
SpatialMomentum<double>
PinocchioPlant<double>::CalcSpatialMomentumInWorldAboutPoint(
    const Context<double>& context, const Vector3<double>& p_WoP_W) const {
  UpdateCentroidalDynamics(context);
  VectorXd hg = pinocchio_data_.hg.toVector();
  drake::multibody::SpatialMomentum<double> h(hg.tail(3), hg.head(3));

  return h;
}

template <>
SpatialMomentum<AutoDiffXd>
PinocchioPlant<AutoDiffXd>::CalcSpatialMomentumInWorldAboutPoint(
    const Context<AutoDiffXd>& context,
    const Vector3<AutoDiffXd>& p_WoP_W) const {
  UpdateCentroidalDynamicsDerivatives(context);
  VectorXd hg = pinocchio_data_.hg.toVector();

  auto map_pinocchio_to_drake = GetVelocityMapFromPinocchioToDrake(
      ExtractValue(GetPositions(context).head<4>()));
  Matrix6X<double> dhdq = pinocchio_data_.dHdq;
  Matrix6X<double> dhdv = pinocchio_data_.Ag * map_pinocchio_to_drake.transpose();

  VectorXd q_drake = ExtractValue(GetPositions(context));
  VectorXd q_pin = MapPositionFromDrakeToPinocchio(q_drake);
  auto drake_quat =
      Eigen::Quaternion<double>(q_drake(0), q_drake(1), q_drake(2), q_drake(3));
  drake::MatrixX<double> rot = drake_quat.toRotationMatrix().transpose();
  auto map = drake::multibody::internal::QuaternionFloatingMobilizer<
      double>::AngularVelocityToQuaternionRateMatrix(drake_quat);
  MatrixXd gradient_quat = 4 * map * rot.transpose() * dhdq.block<6, 3>(0, 3).transpose();
  Matrix6X<double> gradient_pos = dhdq.block<6, 3>(0, 0) * rot;
  gradient_quat.transposeInPlace();

  MatrixXd dhdx = MatrixXd(6, n_q_ + n_v_);
  dhdx << gradient_quat, gradient_pos, dhdq.rightCols(n_q_ - 7), dhdv;
  Vector3<AutoDiffXd> h_linear =
      drake::math::InitializeAutoDiff(hg.head(3), dhdx.topRows(3));
  Vector3<AutoDiffXd> h_angular =
      drake::math::InitializeAutoDiff(hg.tail(3), dhdx.bottomRows(3));
  drake::multibody::SpatialMomentum<AutoDiffXd> h(h_angular, h_linear);
  return h;
}

template <>
void PinocchioPlant<double>::CalcPointsPositions(
    const drake::systems::Context<double>& context,
    const drake::multibody::Frame<double>& frame_B,
    const Eigen::Ref<const drake::MatrixX<double>>& p_BQi,
    const drake::multibody::Frame<double>& frame_A,
    drake::EigenPtr<MatrixX<double>> p_AQi) const {
  pinocchio::ReferenceFrame rf;
  if (frame_A.is_world_frame()) {
    rf = pinocchio_world_;
  } else {
    rf = pinocchio::ReferenceFrame::LOCAL;
  }
  pinocchio::FrameIndex frame_id = pinocchio_model_.getFrameId(frame_B.name());
  DRAKE_DEMAND(p_AQi);
  *p_AQi = pinocchio_data_.oMf[frame_id].translation();
}

template <>
void PinocchioPlant<AutoDiffXd>::CalcPointsPositions(
    const drake::systems::Context<AutoDiffXd>& context,
    const drake::multibody::Frame<AutoDiffXd>& frame_B,
    const Eigen::Ref<const drake::MatrixX<AutoDiffXd>>& p_BQi,
    const drake::multibody::Frame<AutoDiffXd>& frame_A,
    drake::EigenPtr<MatrixX<AutoDiffXd>> p_AQi) const {
  pinocchio::ReferenceFrame rf;
  if (frame_A.is_world_frame()) {
    rf = pinocchio_world_;
  } else {
    rf = pinocchio::ReferenceFrame::LOCAL;
  }
  pinocchio::FrameIndex frame_id = pinocchio_model_.getFrameId(frame_B.name());

  Vector3d position = pinocchio_data_.oMf[frame_id].translation();
  Matrix6X<double> J = MatrixXd::Zero(6, n_v_);
  pinocchio::computeFrameJacobian(
      pinocchio_model_, pinocchio_data_,
      MapPositionFromDrakeToPinocchio(ExtractValue(GetPositions(context))),
      frame_id, rf, J);
  Matrix3X<double> J_translation = J.topRows(3);
  *p_AQi = drake::math::InitializeAutoDiff(position, J_translation);
}

}  // namespace multibody
}  // namespace dairlib

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class dairlib::multibody::PinocchioPlant)
