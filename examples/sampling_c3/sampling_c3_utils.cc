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
    RigidTransform<double> T_EE_W = RigidTransform<double>(
        drake::math::RotationMatrix<double>(
            drake::math::RollPitchYaw<double>(3.1415, 0, 0)),
        kToolAttachmentFrame);
    plant->WeldFrames(plant->GetFrameByName("panda_link7"),
                      plant->GetFrameByName("end_effector_flange"), T_EE_W);
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

std::vector<ModelInstanceIndex> AddObjectsToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph,
    std::vector<std::string> object_models) {
  Parser parser(plant, scene_graph);
  parser.SetAutoRenaming(true);

  std::vector<ModelInstanceIndex> models;
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

std::vector<ModelInstanceIndex> AddLCSModelsToPlant(
    MultibodyPlant<double>* plant, SceneGraph<double>* scene_graph,
    std::vector<std::string> object_models,
    const bool& include_end_effector_orientation, const bool& include_walls) {
  // Cannot currently handle end effector orientation (would just require new
  // EE simple model with orientation DOFs).
  DRAKE_ASSERT(!include_end_effector_orientation);

  std::vector<ModelInstanceIndex> obj_models;

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

}  // namespace dairlib
