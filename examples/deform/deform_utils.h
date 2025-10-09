#pragma once

#include <string>
#include <Eigen/Dense>
#include "drake/multibody/plant/multibody_plant.h"


namespace dairlib {

/// Constants.
// static constexpr const char* kShadowModel =
//   "/home/bibit/miniconda3/envs/genesis_cuda_env/lib/python3.10/site-packages/"
//   "genesis/assets/urdf/shadow_hand/visual_only.urdf";
// static constexpr const char* kShadowModel =
//   "package://sr_description/robots/sr_hand.urdf.xacro";
// static constexpr const char* kShadowModel =
//   "/mnt/data0/bibit/sr_common/sr_description/robots/sr_hand_no_collision.urdf";
// static constexpr const char* kShadowModel =
//   "package://sr_description/mujoco_models/urdfs/shadowhand_motor.urdf";
static constexpr const char* kHandModel =
  "package://drake_models/allegro_hand_description/urdf/allegro_hand_description_right.urdf";
// "package://drake_models/franka_description/urdf/panda_arm.urdf";
static const Eigen::Vector3d kRobotPosOffset = {0, 0.2, 0.2};
static const Eigen::Quaterniond kRobotRotOffset = {0.5, -0.5, -0.5, -0.5};

/// Add the robot hand to a given multibody plant and scene graph.
/// @param plant a pointer to the MultibodyPlant
/// @param scene_graph a pointer to the SceneGraph--may be nullptr (or omitted)
/// @return the ModelInstanceIndex of the robot hand in the plant
drake::multibody::ModelInstanceIndex AddRobotHandToPlant(
  drake::multibody::MultibodyPlant<double>* plant,
  drake::geometry::SceneGraph<double>* scene_graph = nullptr);

}