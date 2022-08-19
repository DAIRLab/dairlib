//
// Created by brian on 8/19/22.
//

#include "stairs.h"
#include "multibody/multibody_utils.h"
#include "drake/math/rigid_transform.h"

namespace dairlib::multibody {

using Eigen::Vector3d;
using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;

void Stairs::AddToPlant(drake::multibody::MultibodyPlant<double> *plant,
                        drake::geometry::SceneGraph<double> *scene_graph) {
  auto boxy = BoxyHeightMap(Vector3d::UnitZ(), width_, height_ * 1.5, 0, mu_);
  boxy.AppendBox(0, 0.5);
  double sign = (stair_direction_ == kUp) ? 1.0 : -1.0;
  for (int i = 0; i < n_; i++) {
    boxy.AppendBox((i+1) * sign * height_, depth_);
  }
  boxy.AddToPlant(plant, scene_graph);
  if (add_walls_) {
    Vector3d wall_len_xyz(depth_ * n_ * 4.0, 0.5, height_ * n_ + 4.0);
    auto X_Wall_left = RigidTransformd(
        RotationMatrixd::Identity(),
        Vector3d(0, width_ / 2 + 0.25,  sign * ((height_ * n_) / 2.0 + 2)));
    auto X_Wall_right = RigidTransformd(
        RotationMatrixd::Identity(),
        Vector3d(0, -width_ / 2 - 0.25, sign * ((height_ * n_) / 2.0 + 2)));

    AddBox(plant, scene_graph, X_Wall_left, wall_len_xyz, 0.25);
    AddBox(plant, scene_graph, X_Wall_right, wall_len_xyz, 0.25);
  }
}

Stairs Stairs::MakeRandomMap(double mu) {
  double width = randd(1.0, 2.5);
  double depth = randd(0.24, 0.31);
  double height = randd(0.1, 0.21);
  double n =  3 + rand() % 5;
  bool add_walls = rand() % 2;
  StairDirection direction = (rand() % 2) ? kUp : kDown;
  return Stairs(width, depth, height, mu, n, add_walls, direction);
}

}