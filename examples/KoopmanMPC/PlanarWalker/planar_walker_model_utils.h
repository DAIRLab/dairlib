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

void PlanarWalkerFixedPointSolver(const drake::multibody::MultibodyPlant<double>& plant,
                                  const double height, const double foot_spread, const double mu,
                                  Eigen::VectorXd* q_result, Eigen::VectorXd* u_result);

void LoadPlanarWalkerFromFile(drake::multibody::MultibodyPlant<double>& plant);

void LoadPlanarWalkerFromFile(drake::multibody::MultibodyPlant<double>& plant,
                              drake::geometry::SceneGraph<double>* scene_graph );

} // </namespace dairlib>

