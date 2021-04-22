//
// Created by brian on 4/21/21.
//
#pragma once

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/systems/framework/context.h"
#include "multibody/multibody_utils.h"
#include "common/find_resource.h"

namespace dairlib {
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::geometry::SceneGraph;
using drake::systems::Context;

void LoadPlanarWalkerFromFile(MultibodyPlant<double>& plant) {
  Parser parser(&plant);

  std::string plant_file_name = FindResourceOrThrow("examples/PlanarWalker/PlanarWalkerWithTorso.urdf");
  parser.AddModelFromFile(plant_file_name);

  plant.WeldFrames(
      plant.world_frame(), plant.GetFrameByName("base"), drake::math::RigidTransform<double>());
}

void LoadPlanarWalkerFromFile(MultibodyPlant<double>& plant,
    SceneGraph<double>* scene_graph ) {

  Parser parser (&plant, scene_graph);
  std::string full_name = FindResourceOrThrow(
      "examples/PlanarWalker/PlanarWalkerWithTorso.urdf");
  parser.AddModelFromFile(full_name);

  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"),
                   drake::math::RigidTransform<double>());

}

void SetDefaultStance(MultibodyPlant<double>& plant, Context<double>* context) {

}
} // </dairlib>

