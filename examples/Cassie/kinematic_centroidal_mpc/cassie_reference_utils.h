
#pragma once

#include <drake/multibody/plant/multibody_plant.h>
#include <Eigen/Core>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include "multibody/kinematic/world_point_evaluator.h"

Eigen::VectorXd GenerateNominalStand(const drake::multibody::MultibodyPlant<double> &plant,
                                     double pelvis_height,
                                     double stance_width,
                                     bool visualize = false);