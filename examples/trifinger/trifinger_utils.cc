#include "trifinger_utils.h"

#include "common/find_resource.h"
#include "drake/multibody/parsing/parser.h"

namespace dairlib {

using drake::geometry::SceneGraph;
using drake::math::RigidTransform;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;


ModelInstanceIndex AddTrifingerToPlant(MultibodyPlant<double>* plant,
                                    SceneGraph<double>* scene_graph) {
  Parser parser(plant, scene_graph);
  parser.SetAutoRenaming(true);

  ModelInstanceIndex trifinger_index = parser.AddModels(kTrifingerModel)[0];
  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("base_link"),
                   X_WI);

  // Add the ground
  parser.AddModels(FindResourceOrThrow(kGroundModel));
  plant->WeldFrames(plant->GetFrameByName("base_link"),
                      plant->GetFrameByName("ground"), X_WI);
  return trifinger_index;
}

drake::multibody::ModelInstanceIndex AddObjectToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph,
    const std::string& object_model) {
  Parser parser(plant, scene_graph);
  parser.SetAutoRenaming(true);
  return parser.AddModels(FindResourceOrThrow(object_model))[0];
}

std::pair<ModelInstanceIndex, ModelInstanceIndex> AddLCSModelsToPlant(
    MultibodyPlant<double>* plant,
    SceneGraph<double>* scene_graph,
    const std::string& object_model) {
  Parser parser_lcs(plant);
  parser_lcs.SetAutoRenaming(true);
  ModelInstanceIndex trifinger_index = parser_lcs.AddModels(kTrifingerSimplifiedModel)[0];
  parser_lcs.AddModels(kGroundModel);
  ModelInstanceIndex object_index = parser_lcs.AddModels(object_model)[0];

  RigidTransform<double> X_WI = RigidTransform<double>::Identity();

  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("base_link"), X_WI);
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("ground"), X_WI);
  return {trifinger_index, object_index};
}

}   // namespace dairlib
