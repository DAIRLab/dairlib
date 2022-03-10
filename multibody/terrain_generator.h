/// This file contains utilities to procedurally generate terrains for
/// simulating difficult environments such as hiking and trail running

#pragma once
#include "drake/multibody/plant/multibody_plant.h"
#include "multibody/multibody_utils.h"

namespace dairlib::multibody {

struct TerrainConfig {
  int x_length;
  int y_length;
  int n_freq_components;
  double x_resolution;
  double y_resolution;
  double mu;
  double stiffness;
  double dissipation;

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
    a->Visit(DRAKE_NVP(mu));
    a->Visit(DRAKE_NVP(stiffness));
    a->Visit(DRAKE_NVP(dissipation));
    a->Visit(DRAKE_NVP(n_freq_components));
    a->Visit(DRAKE_NVP(frequency_amplitudes));
    a->Visit(DRAKE_NVP(frequencies));
    a->Visit(DRAKE_NVP(average_normal));
    DRAKE_DEMAND(frequencies.size() == n_freq_components);
    DRAKE_DEMAND(frequency_amplitudes.size() == n_freq_components);

    freq_amps = Eigen::Map<Eigen::VectorXd>(
        frequency_amplitudes.data(), n_freq_components);
    freqs = Eigen::Map<Eigen::VectorXd>(
        frequencies.data(), n_freq_components);
    normal = Eigen::Map<Eigen::Vector3d>(average_normal.data());
  }
};

void writeTriangleMeshToObj(
    const drake::geometry::TriangleSurfaceMesh<double> &mesh,
    std::string filename);

std::string makeRandomHeightMapAsMesh(int nx, int ny, double x_resolution,
                                      double y_resolution, int n_freq,
                                      Eigen::VectorXd frequencies,
                                      Eigen::VectorXd freq_scales,
                                      Eigen::Vector3d normal);

void addFlatHydroelasticTerrain(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph,
    double mu_static, double mu_kinetic,
    Eigen::Vector3d normal_W = Eigen::Vector3d(0, 0, 1));

void addRandomTerrain(drake::multibody::MultibodyPlant<double> *plant,
                drake::geometry::SceneGraph<double> *scene_graph,
                const TerrainConfig& terrain_config);
}



