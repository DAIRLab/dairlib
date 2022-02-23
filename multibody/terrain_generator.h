/// This file contains utilities to procedurally generate terrains for
/// simulating difficult environments such as hiking and trail running

#pragma once
#include "drake/multibody/plant/multibody_plant.h"
#include "multibody/multibody_utils.h"

namespace dairlib::multibody {

struct TerrainConfig {
  int x_length;
  int y_length;
  double x_resolution;
  double y_resolution;
  double n_freq_components;
  std::vector<double> frequency_amplitudes;
  std::vector<double> frequencies;
  std::vector<double> average_normal;

  Eigen::VectorXd freq_amps;
  Eigen::VectorXd freqs;
  Eigen::Vector3d normal;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(x_length));
    a->Visit(DRAKE_NVP(y_length));
    a->Visit(DRAKE_NVP(x_resolution));
    a->Visit(DRAKE_NVP(y_resolution));
    a->Visit(DRAKE_NVP(n_freq_components));
    a->Visit(DRAKE_NVP(frequency_amplitudes));
    a->Visit(DRAKE_NVP(frequencies));
    a->Visit(DRAKE_NVP(average_normal));

    freq_amps = Eigen::Map<Eigen::Dynamic, Eigen::
  }
};

void writeTriangleMeshToObj(
    const drake::geometry::TriangleSurfaceMesh<double> &mesh,
    std::string filename);

std::string makeRandomHeightMap(int nx, int ny,double x_resolution,
                                double y_resolution, Eigen::VectorXd freq_scales,
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



