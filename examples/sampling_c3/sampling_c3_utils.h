#pragma once

#include <string>
#include <Eigen/Dense>

#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib {

/// Constants for the Franka and end effector.
static constexpr const char* kFrankaModel =
  "package://drake_models/franka_description/urdf/panda_arm.urdf";
static constexpr const char* kEndEffectorModel =
  "examples/sampling_c3/urdf/end_effector_full.urdf";
static constexpr const char* kEndEffectorSimpleModel =
  "examples/sampling_c3/urdf/end_effector_simple_model.urdf";
static constexpr const char* kEndEffectorName = "end_effector_tip";
static constexpr const char* kGroundModel =
  "examples/sampling_c3/urdf/ground.urdf";
static constexpr const char* kPlatformModel =
  "examples/sampling_c3/urdf/platform.urdf";

/// This is the offset from the Panda's link7 frame to its flange where an end
/// effector can be attached.
static const Eigen::Vector3d kToolAttachmentFrame = {0, 0, 0.107};

static const Eigen::Vector3d kFrankaToGroundOffset = {0, 0, -0.029};
static const Eigen::Vector3d kFrankaToPlatformOffset = {0, 0, -0.0145};
static const Eigen::Vector3d kWorldToFrankaOffset = {0, 0, 0};
static const Eigen::Vector3d kWorldToGroundOffset = kWorldToFrankaOffset +
                                                    kFrankaToGroundOffset;

/// Add the Franka to a given multibody plant and scene graph.
/// @param plant a pointer to the MultibodyPlant
/// @param scene_graph a pointer to the SceneGraph--may be nullptr (or omitted)
/// @param include_ground_and_platform whether to include the ground and
/// platform in the plant. If false, only the Franka and end effector will be
/// added.
/// @return the ModelInstanceIndex of the Franka in the plant
drake::multibody::ModelInstanceIndex AddFrankaToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph = nullptr,
    const bool& include_ee = true,
    const bool& include_ground_and_platform = true);

/// Add an object to a given multibody plant and scene graph.
/// @param plant a pointer to the MultibodyPlant
/// @param scene_graph a pointer to the SceneGraph--may be nullptr (or omitted)
/// @param object_model the model of the object to add to the plant
/// @return the ModelInstanceIndex of the object in the plant
drake::multibody::ModelInstanceIndex AddObjectToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph = nullptr,
    const std::string& object_model = "");

std::vector<drake::multibody::ModelInstanceIndex> AddObjectsToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph = nullptr,
    vector<const std::string> object_models = {});

void AddLCSModelToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph = nullptr,
    const std::string& object_model = "",
    const bool& include_end_effector_orientation = false);

/// Add LCS models to a given multibody plant and scene graph.
/// @param plant a pointer to the MultibodyPlant
/// @param scene_graph a pointer to the SceneGraph--may be nullptr (or omitted)
/// @param object_model the model of the object to add to the plant
/// @param include_end_effector_orientation whether to include the end effector
/// orientation as DOFs in the plant. True is currently unimplemented.
void AddLCSModelsToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph = nullptr,
    std::vector<std::string> object_models = {},
    const bool& include_end_effector_orientation = false);

}   // namespace dairlib
