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
                                    const bool& add_walls,
                                    SamplingC3Options* sampling_c3_options) {

  
  Parser parser(plant, scene_graph);
  parser.SetAutoRenaming(true);

  ModelInstanceIndex franka_index = parser.AddModelsFromUrl(kFrankaModel)[0];
  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("panda_link0"), X_WI);

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

  if (add_walls) {
    DRAKE_DEMAND(sampling_c3_options != nullptr);
    SetWallOffsets(*sampling_c3_options);

    parser.AddModels(FindResourceOrThrow(kLeftWallModel));
    parser.AddModels(FindResourceOrThrow(kRightWallModel));
    parser.AddModels(FindResourceOrThrow(kFrontWallModel));

    RigidTransform<double> X_LW_G = RigidTransform<double>(
        drake::math::RotationMatrix<double>(), kLeftWallToGroundOffset);
    RigidTransform<double> X_RW_G = RigidTransform<double>(
        drake::math::RotationMatrix<double>(), kRightWallToGroundOffset);
    RigidTransform<double> X_FW_G = RigidTransform<double>(
        drake::math::RotationMatrix<double>(), kFrontWallToGroundOffset);

    plant->WeldFrames(plant->GetFrameByName("left_wall"),
                      plant->GetFrameByName("ground"), X_LW_G);
    plant->WeldFrames(plant->GetFrameByName("right_wall"),
                      plant->GetFrameByName("ground"), X_RW_G);
    plant->WeldFrames(plant->GetFrameByName("front_wall"),
                      plant->GetFrameByName("ground"), X_FW_G);
  }

  return franka_index;
}

drake::multibody::ModelInstanceIndex AddObjectToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph,
    const std::string& object_model) {
  Parser parser(plant, scene_graph);
  parser.SetAutoRenaming(true);
  return parser.AddModels(FindResourceOrThrow(object_model))[0];
}

std::vector<drake::multibody::ModelInstanceIndex> AddObjectsToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph,
    std::vector<std::string> object_models) {
  Parser parser(plant, scene_graph);
  parser.SetAutoRenaming(true);

  std::vector<drake::multibody::ModelInstanceIndex> models;
  for (const auto& model : object_models) {
      models.push_back(
        parser.AddModels(FindResourceOrThrow(model))[0]
      );
  } 
  return models;
}

void AddLCSModelToPlant(
    MultibodyPlant<double>* plant,
    SceneGraph<double>* scene_graph,
    const std::string& object_model,
    const bool& include_end_effector_orientation) {
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
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("base_link"), X_WI);
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("ground"), X_W_G);

}


std::vector<drake::multibody::ModelInstanceIndex> AddLCSModelsToPlant(
    MultibodyPlant<double>* plant,
    SceneGraph<double>* scene_graph,
    std::vector<std::string> object_models,
    const bool& include_end_effector_orientation,
    const bool& add_walls,
    SamplingC3Options* sampling_c3_options) {
  // Cannot currently handle end effector orientation (would just require new
  // EE simple model with orientation DOFs).
  DRAKE_ASSERT(!include_end_effector_orientation);

  std::vector<drake::multibody::ModelInstanceIndex> obj_models;

  Parser parser_lcs(plant);
  parser_lcs.SetAutoRenaming(true);
  parser_lcs.AddModels(kEndEffectorSimpleModel);
  parser_lcs.AddModels(kGroundModel);

  for (const auto& model : object_models) {
    obj_models.push_back(
      parser_lcs.AddModels(FindResourceOrThrow(model))[0]
    );
  } 

  RigidTransform<double> X_WI = RigidTransform<double>::Identity();

  RigidTransform<double> X_W_G = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), kWorldToGroundOffset);
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("base_link"), X_WI);
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("ground"), X_W_G);

  
  if (add_walls) {
    DRAKE_DEMAND(sampling_c3_options != nullptr);
    SetWallOffsets(*sampling_c3_options);

    parser_lcs.AddModels(FindResourceOrThrow(kLeftWallModel));
    parser_lcs.AddModels(FindResourceOrThrow(kRightWallModel));
    parser_lcs.AddModels(FindResourceOrThrow(kFrontWallModel));

    RigidTransform<double> X_LW_G = RigidTransform<double>(
        drake::math::RotationMatrix<double>(), kLeftWallToGroundOffset);
    RigidTransform<double> X_RW_G = RigidTransform<double>(
        drake::math::RotationMatrix<double>(), kRightWallToGroundOffset);
    RigidTransform<double> X_FW_G = RigidTransform<double>(
        drake::math::RotationMatrix<double>(), kFrontWallToGroundOffset);

    plant->WeldFrames(plant->GetFrameByName("left_wall"),
                      plant->GetFrameByName("ground"), X_LW_G);
    plant->WeldFrames(plant->GetFrameByName("right_wall"),
                      plant->GetFrameByName("ground"), X_RW_G);
    plant->WeldFrames(plant->GetFrameByName("front_wall"),
                      plant->GetFrameByName("ground"), X_FW_G);
  }

  return obj_models;
}

}   // namespace dairlib
