/// This file contains utilities to procedurally generate terrains for
/// simulating difficult environments such as hiking and trail running

#pragma once
#include "drake/multibody/plant/multibody_plant.h"
#include "multibody/multibody_utils.h"

namespace dairlib::multibody {


struct TerrainConfig {
  int xbound = 41;
  int ybound= 41;
  double mesh_res=0.25;
  double mu_flat=0.8;
  Eigen::Vector4d freq_scales = Eigen::Vector4d(1.0, 3.0, 3.0, 2.0);
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d rpy_bounds = Eigen::Vector3d::UnitZ();
};

void writeTriangleMeshToObj(
    const drake::geometry::TriangleSurfaceMesh<double> &mesh,
    std::string filename);

std::string makeRandomHeightMap(int nx, int ny,double x_resolution,
                                double y_resolution, Eigen::Vector4d freq_scales,
                                Eigen::Vector3d normal);

void addFlatHydroelasticTerrain(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph,
    double mu_static, double mu_kinetic,
    Eigen::Vector3d normal_W = Eigen::Vector3d(0, 0, 1));

std::vector<drake::math::RigidTransformd> GenerateRandomPoses(
    int n, double clearing_radius, Eigen::Vector3d rpy_bounds);
std::vector<drake::geometry::Box> GenerateRandomBoxes(
    int n, double s_min, double s_max);
std::vector<drake::geometry::Ellipsoid> GenerateRandomEllipsoids(
    int n, double s_min, double s_max);
std::vector<drake::multibody::CoulombFriction<double>> GenerateRandomFrictions(
    int n, double mu_min, double mu_max);

void addRandomTerrain(drake::multibody::MultibodyPlant<double> *plant,
                drake::geometry::SceneGraph<double> *scene_graph,
                const TerrainConfig& terrain_config);
}



