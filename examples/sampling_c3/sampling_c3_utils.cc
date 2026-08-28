#include "sampling_c3_utils.h"

#include <iostream>

#include "common/find_resource.h"

#include "drake/multibody/parsing/parser.h"

namespace dairlib {

using drake::geometry::SceneGraph;
using drake::math::RigidTransform;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using std::vector;

ModelInstanceIndex AddFrankaToPlant(MultibodyPlant<double>* plant,
                                    SceneGraph<double>* scene_graph,
                                    const bool& include_ee,
                                    const bool& include_ground_and_platform,
                                    const bool& include_walls) {
  Parser parser(plant, scene_graph);
  parser.SetAutoRenaming(true);

  ModelInstanceIndex franka_index = parser.AddModelsFromUrl(kFrankaModel)[0];
  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("panda_link0"),
                    X_WI);

  if (include_ee) {
    parser.AddModels(FindResourceOrThrow(kEndEffectorModel));
    RigidTransform<double> T_Franka_EE = RigidTransform<double>(
        drake::math::RotationMatrix<double>(
            drake::math::RollPitchYaw<double>(3.1415, 0, 0)),
        kToolAttachmentFrame);
    plant->WeldFrames(plant->GetFrameByName("panda_link7"),
                      plant->GetFrameByName("end_effector_flange"),
                      T_Franka_EE);
  }

  if (include_ground_and_platform) {
    parser.AddModels(FindResourceOrThrow(kGroundModel));
    parser.AddModels(FindResourceOrThrow(kPlatformModel));

    RigidTransform<double> X_F_P = RigidTransform<double>(
        drake::math::RotationMatrix<double>(), kFrankaToPlatformOffset);
    RigidTransform<double> X_F_G_franka = RigidTransform<double>(
        drake::math::RotationMatrix<double>(), kFrankaToGroundOffset);

    plant->WeldFrames(plant->GetFrameByName("panda_link0"),
                      plant->GetFrameByName("ground"), X_F_G_franka);
    plant->WeldFrames(plant->GetFrameByName("panda_link0"),
                      plant->GetFrameByName("platform"), X_F_P);
  }

  if (include_walls) {
    AddWallsToPlant(plant, scene_graph);
  }

  return franka_index;
}

void AddWallsToPlant(drake::multibody::MultibodyPlant<double>* plant,
                     drake::geometry::SceneGraph<double>* scene_graph,
                     const bool& include_back_wall) {
  Eigen::Vector3d side_wall_size(kWallLengthX, kWallWidth, kWallHeight);
  AddBoxToPlant(plant, scene_graph, side_wall_size, "left_wall");
  AddBoxToPlant(plant, scene_graph, side_wall_size, "right_wall");
  Eigen::Vector3d wall_size(kWallWidth, kWallLengthY + 2 * kWallWidth,
                            kWallHeight);
  AddBoxToPlant(plant, scene_graph, wall_size, "front_wall");

  RigidTransform<double> X_G_LW = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), kGroundToLeftWallOffset);
  RigidTransform<double> X_G_RW = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), kGroundToRightWallOffset);
  RigidTransform<double> X_G_FW = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), kGroundToFrontWallOffset);

  plant->WeldFrames(plant->GetFrameByName("ground"),
                    plant->GetFrameByName("left_wall"), X_G_LW);
  plant->WeldFrames(plant->GetFrameByName("ground"),
                    plant->GetFrameByName("right_wall"), X_G_RW);
  plant->WeldFrames(plant->GetFrameByName("ground"),
                    plant->GetFrameByName("front_wall"), X_G_FW);

  if (include_back_wall) {
    AddBoxToPlant(plant, scene_graph, wall_size, "back_wall");
    RigidTransform<double> X_G_BW = RigidTransform<double>(
        drake::math::RotationMatrix<double>(), kGroundToBackWallOffset);
    plant->WeldFrames(plant->GetFrameByName("ground"),
                      plant->GetFrameByName("back_wall"), X_G_BW);
  }
}

void AddBoxToPlant(drake::multibody::MultibodyPlant<double>* plant,
                   drake::geometry::SceneGraph<double>* scene_graph,
                   const Eigen::Vector3d& box_size,
                   const std::string& box_name) {
  ModelInstanceIndex model_instance_index = plant->AddModelInstance(box_name);
  const drake::multibody::RigidBody<double>& body = plant->AddRigidBody(
      box_name, model_instance_index,
      drake::multibody::SpatialInertia<double>::SolidBoxWithMass(
          1.0, box_size(0) / 2, box_size(1) / 2, box_size(2) / 2));

  plant->RegisterVisualGeometry(
      body, RigidTransform<double>::Identity(),
      drake::geometry::Box(box_size(0), box_size(1), box_size(2)), box_name,
      kWallColor);
  plant->RegisterCollisionGeometry(
      body, RigidTransform<double>::Identity(),
      drake::geometry::Box(box_size(0), box_size(1), box_size(2)), box_name,
      kWallFriction);
}

ModelInstanceIndex AddObjectToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph,
    const std::string& object_model) {
  Parser parser(plant, scene_graph);
  parser.SetAutoRenaming(true);
  return parser.AddModels(FindResourceOrThrow(object_model))[0];
}

vector<ModelInstanceIndex> AddObjectsToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph,
    vector<std::string> object_models) {
  Parser parser(plant, scene_graph);
  parser.SetAutoRenaming(true);

  vector<ModelInstanceIndex> models;
  for (const auto& model : object_models) {
    models.push_back(parser.AddModels(FindResourceOrThrow(model))[0]);
  }
  return models;
}

void AddLCSModelToPlant(MultibodyPlant<double>* plant,
                        SceneGraph<double>* scene_graph,
                        const std::string& object_model,
                        const bool& include_end_effector_orientation,
                        const bool& include_walls) {
  // Cannot currently handle end effector orientation (would just require new
  // EE simple model with orientation DOFs).
  DRAKE_DEMAND(!include_end_effector_orientation);

  Parser parser_lcs(plant);
  parser_lcs.SetAutoRenaming(true);
  parser_lcs.AddModels(kEndEffectorSimpleModel);
  parser_lcs.AddModels(kGroundModel);
  parser_lcs.AddModels(object_model);

  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  RigidTransform<double> X_W_G = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), kWorldToGroundOffset);
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("base_link"),
                    X_WI);
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("ground"),
                    X_W_G);

  if (include_walls) {
    // TODO: may want to exclude the back wall for the LCS model.
    AddWallsToPlant(plant, scene_graph);  //, false);
  }
}

vector<ModelInstanceIndex> AddLCSModelsToPlant(
    MultibodyPlant<double>* plant, SceneGraph<double>* scene_graph,
    vector<std::string> object_models,
    const bool& include_end_effector_orientation, const bool& include_walls) {
  // Cannot currently handle end effector orientation (would just require new
  // EE simple model with orientation DOFs).
  DRAKE_ASSERT(!include_end_effector_orientation);

  vector<ModelInstanceIndex> obj_models;

  Parser parser_lcs(plant);
  parser_lcs.SetAutoRenaming(true);
  parser_lcs.AddModels(kEndEffectorSimpleModel);
  parser_lcs.AddModels(kGroundModel);

  for (const auto& model : object_models) {
    obj_models.push_back(parser_lcs.AddModels(FindResourceOrThrow(model))[0]);
  }

  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  RigidTransform<double> X_W_G = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), kWorldToGroundOffset);
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("base_link"),
                    X_WI);
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("ground"),
                    X_W_G);

  if (include_walls) {
    // TODO: may want to exclude the back wall for the LCS model.
    AddWallsToPlant(plant, scene_graph);  //, false);
  }

  return obj_models;
}

ModelInstanceIndex Add3DPrinterToPlant(MultibodyPlant<double>* plant,
                                       SceneGraph<double>* scene_graph,
                                       const bool& include_ee,
                                       const bool& include_ground_and_platform,
                                       const bool& include_walls) {
  Parser parser(plant, scene_graph);
  parser.SetAutoRenaming(true);

  ModelInstanceIndex printer_index =
      parser.AddModels(FindResourceOrThrow(k3DPrinterModel))[0];
  ModelInstanceIndex ramp_index =
      parser.AddModels(FindResourceOrThrow(k3DPrinterRampModel))[0];

  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("base_link"),
                    X_WI);
  RigidTransform<double> X_World_Ramp(k3dPrinterRampAttachmentRotationMatrix,
                                      k3dPrinterRampAttachmentFrame);
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("ramp_link"),
                    X_World_Ramp);

  // Disable gravity for the entire printer model instance.
  plant->set_gravity_enabled(printer_index, false);
  plant->set_gravity_enabled(ramp_index, false);

  plant->AddJointActuator("x_axis_actuator",
                          plant->GetJointByName("x_axis_joint", printer_index));
  plant->AddJointActuator("y_axis_actuator",
                          plant->GetJointByName("y_axis_joint", printer_index));
  plant->AddJointActuator("z_axis_actuator",
                          plant->GetJointByName("z_axis_joint", printer_index));

  if (plant->time_step() > 0.0) {
    plant
        ->get_mutable_joint_actuator(
            plant->GetJointActuatorByName("x_axis_actuator").index())
        .set_controller_gains(k3dPrinterXYAxesPdGains);
    plant
        ->get_mutable_joint_actuator(
            plant->GetJointActuatorByName("y_axis_actuator").index())
        .set_controller_gains(k3dPrinterXYAxesPdGains);
    plant
        ->get_mutable_joint_actuator(
            plant->GetJointActuatorByName("z_axis_actuator").index())
        .set_controller_gains(k3dPrinterZAxisPdGains);
  }

  if (include_ee) {
    ModelInstanceIndex ee_index = parser.AddModels(k3dEndEffectorModel)[0];
    RigidTransform<double> T_Printer_EE(k3dPrinterToolAttachmentFrame);
    plant->WeldFrames(plant->GetFrameByName("x_carriage"),
                      plant->GetFrameByName("end_effector_flange"),
                      T_Printer_EE);

    // Disable gravity for all end effector bodies.
    plant->set_gravity_enabled(ee_index, false);
  }

  return printer_index;
}

vector<ModelInstanceIndex> AddLCSModelsTo3DPrinterPlant(
    MultibodyPlant<double>* plant, SceneGraph<double>* scene_graph,
    vector<std::string> object_models) {
  vector<ModelInstanceIndex> obj_models;

  Parser parser_lcs(plant);
  parser_lcs.SetAutoRenaming(true);
  parser_lcs.AddModels(k3dEndEffectorSimpleModel);
  parser_lcs.AddModels(kBaseModel);
  parser_lcs.AddModels(FindResourceOrThrow(k3DPrinterRampModel));

  for (const auto& model : object_models) {
    obj_models.push_back(parser_lcs.AddModels(FindResourceOrThrow(model))[0]);
  }

  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("base_ee_link"),
                    X_WI);
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("ground"),
                    X_WI);

  RigidTransform<double> X_World_Ramp(k3dPrinterRampAttachmentRotationMatrix,
                                      k3dPrinterRampAttachmentFrame);

  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("ramp_link"),
                    X_World_Ramp);

  return obj_models;
}

}  // namespace dairlib
