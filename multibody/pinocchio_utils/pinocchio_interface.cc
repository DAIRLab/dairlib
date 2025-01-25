#include "pinocchio_interface.h"
#include "pinocchio/parsers/urdf.hpp"

#include "multibody/multibody_utils.h"

namespace dairlib::multibody {

using Eigen::VectorXd;
using Eigen::Quaternion;

using drake::multibody::MultibodyPlant;
using drake::VectorX;
using drake::MatrixX;

PinocchioInterface::PinocchioInterface(
    const MultibodyPlant<double> &plant,
    const std::string &urdf) : plant_(plant) {

  DRAKE_DEMAND(plant_.is_finalized());

  is_floating_base_ = HasQuaternion(plant);
  if (is_floating_base_) {
    pinocchio::urdf::buildModel(urdf, pinocchio::JointModelFreeFlyer(),
                                pinocchio_model_);
  } else {
    pinocchio::urdf::buildModel(urdf, pinocchio_model_);
  }
  BuildPermutations();
  CopyReflectedInertiaToPinocchioModel();
}

void PinocchioInterface::CopyReflectedInertiaToPinocchioModel() {
  // Add reflected inertia
  VectorXd gear_ratios = VectorXd::Ones(plant_.num_velocities());
  VectorXd rotor_inertias = VectorXd::Zero(plant_.num_velocities());
  VectorXd armature = VectorXd::Zero(plant_.num_velocities());
  auto vel_map = MakeNameToVelocitiesMap(plant_);

  for (int i = 0; i < plant_.num_actuators(); ++i) {
    auto& joint_actuator = plant_.get_joint_actuator(
        drake::multibody::JointActuatorIndex(i));
    auto name = joint_actuator.joint().name();
    int idx = vel_map.at(name + "dot");
    gear_ratios(idx) = joint_actuator.default_gear_ratio();
    rotor_inertias(idx) = joint_actuator.default_rotor_inertia();
    armature(idx) = gear_ratios(idx) * gear_ratios(idx) * rotor_inertias(idx);
  }

  pinocchio_model_.rotorInertia = v_perm_p2d_.transpose() * rotor_inertias;
  pinocchio_model_.rotorGearRatio = v_perm_p2d_.transpose() * gear_ratios;
  pinocchio_model_.armature = pinocchio_model_.armature +
      v_perm_p2d_.transpose() * armature;
}

void PinocchioInterface::BuildPermutations() {
  std::map<std::string, int> pos_map = MakeNameToPositionsMap(plant_);
  std::map<std::string, int> vel_map = MakeNameToVelocitiesMap(plant_);
  int nq = plant_.num_positions();
  int nv = plant_.num_velocities();
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
      // 2. the velocity is to be expressed in local frame
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
  q_perm_p2d_.indices() = pos_indices;
  v_perm_p2d_.indices() = vel_indices;
}

template <typename T>
void PinocchioInterface::MapPositionsToDrake(
    VectorX<T> &positions) const {
  positions = v_perm_p2d_ * positions;
}

template <typename T>
void PinocchioInterface::MapPositionsToPinocchio(
    VectorX<T> &positions) const {
  positions = v_perm_p2d_.inverse() * positions;
}

template <typename T>
void PinocchioInterface::MapVelocitiesToDrake(
    const VectorX<T> &q, VectorX<T> &v) const {
  if (is_floating_base_) {
    MatrixX<T> rot =
        Eigen::Quaternion<T>(q(0), q(1), q(2), q(3)).toRotationMatrix();
    v.template head<3>() = rot * v.template head<3>();
    v.template segment<3>(3) = v.template segment<3>(3);
  }
  v = v_perm_p2d_ * v;
}

template <typename T>
void PinocchioInterface::MapVelocitiesToPinocchio(
    const VectorX<T> &q, VectorX<T> &v) const {
  if (is_floating_base_) {
    MatrixX<T> rot =
        Eigen::Quaternion<T>(q(0), q(1), q(2), q(3)).toRotationMatrix()
        .transpose();
    v.template head<3>() = rot * v.template head<3>();
    v.template segment<3>(3) = v.template segment<3>(3);
  }
  v = v_perm_p2d_.inverse() * v;
}

template <typename T>
void PinocchioInterface::MapJacobianToDrake(
    const VectorX<T> &q, MatrixX<T> &J) const {
  J = J * v_perm_p2d_.inverse();
  if (is_floating_base_) {
    MatrixX<T> rot =
        Eigen::Quaternion<T>(q(0), q(1), q(2), q(3)).toRotationMatrix()
        .transpose();
    J.template leftCols<3>() = J.template leftCols<3>() * rot;
    J.template middleCols<3>(3) = J.template middleCols<3>(3) * rot;
  }
}
}