#include "boxy_height_map.h"
#include "multibody_utils.h"

namespace dairlib::multibody {

using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::MatrixXd;
using drake::math::RotationMatrix;
using drake::math::RigidTransform;
using drake::geometry::GeometryId;


double randd(double a, double b) {
  return a + (b - a) * (static_cast<double>(rand())  /
      static_cast<double>(RAND_MAX));
}

void BoxyHeightMap::AppendBox(double h, double w) {
  double box_start =
      box_w_.empty() ? -w / 2.0 : box_start_.back() + box_w_.back();
  box_start_.push_back(box_start);
  box_h_.push_back(h);
  box_w_.push_back(w);
}

void BoxyHeightMap::AddHeightMapToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph) {
  for (int i = 0; i < box_w_.size(); i++) {
    Vector3d box_pos(
        box_start_.at(i) + box_w_.at(i) / 2,
        0,
        box_h_.at(i) - dim_z_ / 2.0);
    RigidTransform<double> X_WB(R_, R_.matrix() * box_pos);
    AddBox<double>(plant, scene_graph, X_WB,
           Vector3d(box_w_.at(i), dim_y_, dim_z_),
           mu_);
  }
}

double BoxyHeightMap::GetHeightInWorld(const Vector2d &xy_pos) const {
  double x = xmap_ * xy_pos;
  double h = 0;
  for (int i = 0; box_start_.at(i) < x; i++){h = box_h_.at(i);}
  return h;
}

Eigen::MatrixXd BoxyHeightMap::GetHeightMap(
    const Eigen::VectorXd &x_grid, const Eigen::VectorXd &y_grid) const {
  MatrixXd map = MatrixXd::Zero(x_grid.size(), y_grid.size());
  for (int i = 0; i < x_grid.size(); i++) {
    double x = x_grid(i);
    for (int j = 0; j < y_grid.size(); j++) {
      map(i, j) = GetHeightInWorld(Vector2d(x, y_grid(i)));
    }
  }
  return map;
}

RotationMatrix<double> BoxyHeightMap::MakeRotation(
    const Vector3d& normal, double rotz) {

  // Make a coordinate frame with the x axis aligned with the parent frame
  // x-z plane and the z axis aligned with normal
  Vector3d b_z = normal.normalized();
  Vector3d b_x (b_z(2), 0, -b_z(0));
  b_x.normalize();
  Vector3d b_y = b_z.cross(b_x);
  RotationMatrix<double> R_BW =
      RotationMatrix<double>::MakeFromOrthonormalColumns(b_x, b_y, b_z);

  // rotate the whole thing about the parent frame z-axis by rotz
  return R_BW * RotationMatrix<double>::MakeZRotation(rotz);
}

BoxyHeightMap BoxyHeightMap::MakeRandomMap() {
  int n_boxes = rand() % 10 + 5;
  BoxyHeightMap boxy(Vector3d::UnitZ(), 5, 0.5,
                     randd(-0.5, 0.5), 0.8);
  boxy.AppendBox(0, 0.4);
  for (int i = 0; i < n_boxes; i++) {
    boxy.AppendBox(randd(-0.1, 0.1), randd(0.2, 0.4));
  }
  return boxy;
}

}