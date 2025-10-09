#include "deform_utils.h"
#include <iostream>
#include "common/find_resource.h"
#include "drake/multibody/parsing/parser.h"

namespace dairlib {

using drake::geometry::SceneGraph;
using drake::math::RigidTransform;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;


ModelInstanceIndex AddRobotHandToPlant(MultibodyPlant<double>* plant,
                                       SceneGraph<double>* scene_graph) {
  Parser parser(plant, scene_graph);
  ModelInstanceIndex hand_index = parser.AddModelsFromUrl(kHandModel)[0];
  RigidTransform<double> X_WH = RigidTransform<double>(
    kRobotRotOffset, kRobotPosOffset);
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("hand_root"), X_WH);

  return hand_index;
}

}