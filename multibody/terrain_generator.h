/// This file contains utilities to procedurally generate terrains for
/// simulating difficult environments such as hiking and trail running

#pragma once
#include "drake/multibody/plant/multibody_plant.h"
#include "multibody/multibody_utils.h"

namespace dairlib::multibody {


struct TerrainConfig {
  double min_cube_size=0.1;   // minimum cube side length
  double max_cube_size=1.0;   // maximum cube side length
  double clearing_radius=1.0; // radius around 0,0 to keep free of cubes
  double mu_flat;
  double mu_cube_min;
  double mu_cube_max;
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
};



std::vector<drake::math::RigidTransformd> GenerateRandomPoses(int n);
std::vector<drake::geometry::Box> GenerateRandomBoxes(
    int n, double s_min, double s_max);
std::vector<drake::multibody::CoulombFriction<double>> GenerateRandomFrictions(
    int n, double mu_min, double mu_max);

void addRandomTerrain(drake::multibody::MultibodyPlant<double> *plant,
                drake::geometry::SceneGraph<double> *scene_graph,
                TerrainConfig terrain_config);
}



