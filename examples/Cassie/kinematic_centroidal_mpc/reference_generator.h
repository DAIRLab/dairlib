
#pragma once

#include <drake/multibody/plant/multibody_plant.h>
#include <Eigen/Core>

Eigen::VectorXd GenerateNominalStand(const drake::multibody::MultibodyPlant<double> &plant,
                                     double pelvis_height,
                                     double stance_width,
                                     bool visualize = false);