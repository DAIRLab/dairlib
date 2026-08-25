#pragma once

#include <string>

#include <Eigen/Dense>

#include "examples/sampling_c3/parameter_headers/sampling_c3_options.h"

#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
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

/// Constants for the 3D printer and end effector.
static constexpr const char* k3DPrinterModel =
    "examples/sampling_c3/urdf/three_d_printer/3d_printer.urdf";
static constexpr const char* k3DPrinterRampModel =
    "examples/sampling_c3/urdf/three_d_printer/ramp/new_ramp.urdf";
static constexpr const char* k3dEndEffectorModel =
    "examples/sampling_c3/urdf/three_d_printer/EE.urdf";
static constexpr const char* k3dEndEffectorSimpleModel =
    "examples/sampling_c3/urdf/three_d_printer/"
    "three_d_printer_end_effector_simple_model.urdf";
static constexpr const char* k3dEndEffectorTipName = "end_effector_tip";
static constexpr const char* kBaseModel =
    "examples/sampling_c3/urdf/three_d_printer/3_d_printer_bed.urdf";

/// Constants for the walls (used in Franka examples).
static constexpr const char* kLeftWallModel =
    "examples/sampling_c3/urdf/wall_left.urdf";
static constexpr const char* kRightWallModel =
    "examples/sampling_c3/urdf/wall_right.urdf";
static constexpr const char* kFrontWallModel =
    "examples/sampling_c3/urdf/wall_front.urdf";

/// Franka offsets.
/// This is the offset from the Panda's link7 frame to its flange where an end
/// effector can be attached.
static const Eigen::Vector3d kToolAttachmentFrame = {0, 0, 0.107};
static const Eigen::Vector3d kFrankaToGroundOffset = {0, 0, -0.029};
static const Eigen::Vector3d kFrankaToPlatformOffset = {0, 0, -0.0145};
static const Eigen::Vector3d kWorldToFrankaOffset = {0, 0, 0};
static const Eigen::Vector3d kWorldToGroundOffset =
    kWorldToFrankaOffset + kFrankaToGroundOffset;

/// 3D printer offsets and other constants.
static const Eigen::Vector3d k3dPrinterToolAttachmentFrame = {0.0, 0.0,
                                                              -0.054582};
static const Eigen::Vector3d k3dPrinterRampAttachmentFrame = {0.0, 0.1, 0.0};
static const drake::math::RotationMatrix<double>
    k3dPrinterRampAttachmentRotationMatrix(
        drake::math::RollPitchYaw<double>(0, 0, 3.14159));
static const drake::multibody::PdControllerGains k3dPrinterPdGains{250000.0,
                                                                   25000.0};

/// Bin wall constants.
static const Eigen::Vector4d kWallColor = {0.7, 0.7, 0.7, 1.0};
static const drake::multibody::CoulombFriction<double> kWallFriction(0.5, 0.5);
static const float kWallHeight = 0.015;
static const float kWallWidth = 0.075;
static const float kWallLengthX = 0.7;
static const float kWallLengthY = 0.9;
static const float kWallCenterX = 0.5;
static const Eigen::Vector3d kGroundToLeftWallOffset = {
    kWallCenterX, (kWallLengthY + kWallWidth) / 2, kWallHeight / 2};
static const Eigen::Vector3d kGroundToRightWallOffset = {
    kWallCenterX, -(kWallLengthY + kWallWidth) / 2, kWallHeight / 2};
static const Eigen::Vector3d kGroundToFrontWallOffset = {
    kWallCenterX + (kWallLengthX + kWallWidth) / 2, 0, kWallHeight / 2};
static const Eigen::Vector3d kGroundToBackWallOffset = {
    kWallCenterX - (kWallLengthX + kWallWidth) / 2, 0, kWallHeight / 2};

/// Add the Franka to a given multibody plant and scene graph.
/// @param plant a pointer to the MultibodyPlant
/// @param scene_graph a pointer to the SceneGraph--may be nullptr (or omitted)
/// @param include_walls whether to add border walls to workspace
/// @param include_ground_and_platform whether to include the ground and
/// platform in the plant. If false, only the Franka and end effector will be
/// added.
/// @return the ModelInstanceIndex of the Franka in the plant
drake::multibody::ModelInstanceIndex AddFrankaToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph = nullptr,
    const bool& include_ee = true,
    const bool& include_ground_and_platform = true,
    const bool& include_walls = false);

/// Add bin walls to a given multibody plant and scene graph.
/// @param plant a pointer to the MultibodyPlant
/// @param scene_graph a pointer to the SceneGraph--may be nullptr (or omitted)
/// @param include_back_wall whether to include the back wall; may be desired to
/// exclude the back wall in the LCS model.

void AddWallsToPlant(drake::multibody::MultibodyPlant<double>* plant,
                     drake::geometry::SceneGraph<double>* scene_graph = nullptr,
                     const bool& include_back_wall = true);

/// Add a box to a given multibody plant.
/// @param plant a pointer to the MultibodyPlant
/// @param box_size the size of the box to add
/// @param box_name the name of the box
void AddBoxToPlant(drake::multibody::MultibodyPlant<double>* plant,
                   drake::geometry::SceneGraph<double>* scene_graph,
                   const Eigen::Vector3d& box_size,
                   const std::string& box_name);

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
    const bool& include_end_effector_orientation = false,
    const bool& include_walls = false);

/// Add LCS models to a given multibody plant and scene graph.
/// @param plant a pointer to the MultibodyPlant
/// @param scene_graph a pointer to the SceneGraph--may be nullptr (or omitted)
/// @param object_model the model of the object to add to the plant
/// @param include_end_effector_orientation whether to include the end effector
/// @param include_walls whether to add border walls to workspace
std::vector<drake::multibody::ModelInstanceIndex> AddLCSModelsToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph = nullptr,
    std::vector<std::string> object_models = {},
    const bool& include_end_effector_orientation = false,
    const bool& include_walls = false);

/// Add the 3D printer to a given multibody plant and scene graph.
/// @param plant a pointer to the MultibodyPlant
/// @param scene_graph a pointer to the SceneGraph--may be nullptr (or omitted)
/// @param include_walls whether to add border walls to workspace
/// @param include_ground_and_platform whether to include the ground and
/// platform in the plant. If false, only the Franka and end effector will be
/// added.
/// @return the ModelInstanceIndex of the Franka in the plant
drake::multibody::ModelInstanceIndex Add3DPrinterToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph = nullptr,
    const bool& include_ee = true,
    const bool& include_ground_and_platform = true,
    const bool& include_walls = false);

/// Add 3D printer LCS models to a given multibody plant and scene graph.
/// @param plant a pointer to the MultibodyPlant
/// @param scene_graph a pointer to the SceneGraph--may be nullptr (or omitted)
/// @param object_model the model of the object to add to the plant
std::vector<drake::multibody::ModelInstanceIndex> AddLCSModelsTo3DPrinterPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph = nullptr,
    std::vector<std::string> object_models = {});

}  // namespace dairlib
