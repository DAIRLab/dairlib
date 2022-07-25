#pragma once

#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/geometry/scene_graph.h"
#include "boxy_height_map.h"

namespace dairlib::multibody {
class CubeHeightMap {
 public:
  CubeHeightMap(Eigen::MatrixXd heights, double dim_x, double dim_y,
                double dim_z, const Eigen::Vector3d& normal,
                const Eigen::Vector3d& origin, double rot_z, double mu);

  void AddHeightMapToPlant(drake::multibody::MultibodyPlant<double>* plant,
                           drake::geometry::SceneGraph<double>* scene_graph);

  static CubeHeightMap MakeRandomMap(const Eigen::Vector3d& normal,
                                          double rotz, double mu);

 private:
  drake::math::RigidTransformd X_WM_;
  int nx_;
  int ny_;
  double x_resolution_;
  double y_resolution_;
  double dim_x_;
  double dim_y_;
  double dim_z_;
  double mu_;
  Eigen::MatrixXd heights_;
};
}
