#pragma once

#include <string>
#include <Eigen/Dense>
#include "examples/sampling_c3/parameter_headers/sampling_c3_options.h"

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

static constexpr const char* kLeftWallModel = "examples/sampling_c3/urdf/wall_left.urdf";
static constexpr const char* kRightWallModel = "examples/sampling_c3/urdf/wall_right.urdf";
static constexpr const char* kFrontWallModel = "examples/sampling_c3/urdf/wall_front.urdf";
  
/// This is the offset from the Panda's link7 frame to its flange where an end
/// effector can be attached.
static const Eigen::Vector3d kToolAttachmentFrame = {0, 0, 0.107};

static const Eigen::Vector3d kFrankaToGroundOffset = {0, 0, -0.029};
static const Eigen::Vector3d kFrankaToPlatformOffset = {0, 0, -0.0145};
static const Eigen::Vector3d kWorldToFrankaOffset = {0, 0, 0};
static const Eigen::Vector3d kWorldToGroundOffset = kWorldToFrankaOffset +
                                                    kFrankaToGroundOffset;
static Eigen::Vector3d kLeftWallToGroundOffset = {0, 0, 0};
static Eigen::Vector3d kRightWallToGroundOffset = {0, 0, 0};
static Eigen::Vector3d kFrontWallToGroundOffset = {0, 0, 0};

// Pulls workspace limits and gets offsets for wall welding
static void SetWallOffsets(SamplingC3Options sampling_c3_options) {
  double length = sampling_c3_options.workspace_limits[0][4];
  double width = sampling_c3_options.workspace_limits[1][4] - sampling_c3_options.workspace_limits[1][3];

  kLeftWallToGroundOffset[0] = -0.5 * length;
  kLeftWallToGroundOffset[1] = 0.5 * width;
  
  kRightWallToGroundOffset[0] = -0.5 * length;
  kRightWallToGroundOffset[1] = -0.5 * width;

  kFrontWallToGroundOffset[0] = -length;
} 

/// Add the Franka to a given multibody plant and scene graph.
/// @param plant a pointer to the MultibodyPlant
/// @param scene_graph a pointer to the SceneGraph--may be nullptr (or omitted)
/// @param add_walls whether to add border walls to workspace
/// @param sampling_c3_options if add_walls == true, must include to get workspace limits
/// @param include_ground_and_platform whether to include the ground and
/// platform in the plant. If false, only the Franka and end effector will be
/// added.
/// @return the ModelInstanceIndex of the Franka in the plant
drake::multibody::ModelInstanceIndex AddFrankaToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph = nullptr,
    const bool& include_ee = true,
    const bool& include_ground_and_platform = true,
    const bool& add_walls = false,
    SamplingC3Options* sampling_c3_options = nullptr);

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
    std::vector<std::string> object_models = {});

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
/// @param add_walls whether to add border walls to workspace
/// @param sampling_c3_options if add_walls == true, must include to get workspace limits
/// orientation as DOFs in the plant. True is currently unimplemented.
std::vector<drake::multibody::ModelInstanceIndex> AddLCSModelsToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph = nullptr,
    std::vector<std::string> object_models = {},
    const bool& include_end_effector_orientation = false,
    const bool& add_walls = false,
    SamplingC3Options* sampling_c3_options = nullptr);


}   // namespace dairlib
