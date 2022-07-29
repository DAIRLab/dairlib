#include "cube_height_map.h"
#include "multibody_utils.h"

namespace dairlib::multibody {

using Eigen::MatrixXd;
using Eigen::Vector3d;
using drake::math::RotationMatrixd;
using drake::math::RigidTransformd;

CubeHeightMap::CubeHeightMap(MatrixXd heights, double dim_x, double dim_y,
                             double dim_z, const Vector3d& normal,
                             const Vector3d& origin,
                             double rot_z, double mu)  :
                             heights_(heights),
                             dim_x_(dim_x), dim_y_(dim_y), dim_z_(dim_z),
                             mu_(mu) {
    X_WM_ =  RigidTransformd(BoxyHeightMap::MakeRotation(normal, rot_z), origin);
    nx_ = heights.rows();
    ny_ = heights.cols();
    x_resolution_ = dim_x_ / nx_;
    y_resolution_ = dim_y_ / ny_;
}

void CubeHeightMap::AddHeightMapToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph) {
  for (int x_idx = 0; x_idx < nx_; x_idx ++) {
    for (int y_idx = 0; y_idx < ny_; y_idx++) {
      Vector3d box_center(x_resolution_ * (x_idx - (double)(nx_ - 1.0)/2.0),
                          y_resolution_ * (y_idx - (double)(ny_ - 1.0)/2.0),
                          heights_(x_idx, y_idx) - dim_z_ / 2);
      RigidTransformd X_WB(X_WM_.rotation(),
                           X_WM_.translation() +
                           X_WM_.rotation().matrix() * box_center);
      AddBox<double>(plant, scene_graph, X_WB,
                     Vector3d(x_resolution_, y_resolution_, dim_z_),
                     mu_);
    }
  }
}

CubeHeightMap CubeHeightMap::MakeRandomMap(
    const Eigen::Vector3d &normal, double rotz, double mu) {
  double dim_x = randd(3, 9);
  double dim_y = randd(3,9);
  int n = rand() % 10 + 10;
  // easier to initialize with an odd number of squares
  n = n + (1 - n % 2);
  MatrixXd heights = randd(0.05, 0.1) * MatrixXd::Random(n,n);
  heights -= MatrixXd::Ones(n,n) * heights(n/2, n/2);
  return {heights, dim_x, dim_y, 0.5, normal, Vector3d::Zero(),
                       rotz, mu};
}

}