/// This file contains utilities to procedurally generate terrains for
/// simulating difficult environments such as hiking and trail running

#pragma once
#include "drake/multibody/plant/multibody_plant.h"
#include "multibody/multibody_utils.h"

namespace dairlib::multibody {

std::vector<drake::math::RigidTransformd> GenerateRandomPoses(int n);
std::vector<drake::geometry::Box> GenerateRandomBoxes(int n);
std::vector<drake::multibody::CoulombFriction<double>> GenerateRandomFrictions(int n);

void addRandomTerrain(drake::multibody::MultibodyPlant<double> *plant,
                drake::geometry::SceneGraph<double> *scene_graph,
                double mu_static, double mu_kinetic,
                Eigen::Vector3d normal_W = Eigen::Vector3d(0, 0, 1));
}



