#pragma once
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/geometry/scene_graph.h"

namespace dairlib::multibody {

double randd(double a, double b);

struct box2d {
  double h;
  double w;
};

class BoxyHeightMap {
 public:
  BoxyHeightMap()=default;
  BoxyHeightMap(const Eigen::Vector3d &normal, double dim_y, double dim_z,
                double rot_z, double mu) : R_(MakeRotation(normal, rot_z)),
                                xmap_(MakeRotation(normal, rot_z).inverse().matrix().block(0,0,1,2)),
                                dim_y_(dim_y),
                                dim_z_(dim_z),
                                mu_(mu) {};

  void AppendBox(double h, double w);
  void AppendBox(Eigen::Vector2d hw) { AppendBox(hw(0), hw(1));}
  void AddHeightMapToPlant(drake::multibody::MultibodyPlant<double>* plant,
                           drake::geometry::SceneGraph<double>* scene_graph);

  double GetHeightInWorld(const Eigen::Vector2d &xy_pos) const ;
  Eigen::MatrixXd GetHeightMap(const Eigen::VectorXd &x_grid,
                               const Eigen::VectorXd &y_grid) const ;

  static BoxyHeightMap MakeRandomMap();
  static BoxyHeightMap MakeRandomMap(
      const Eigen::Vector3d& normal, double yaw, double mu, double height);
  static drake::math::RotationMatrix<double> MakeRotation(
      const Eigen::Vector3d& normal, double rotz);

 private:


  drake::math::RotationMatrixd R_;
  Eigen::RowVector2d xmap_;
  double dim_y_;
  double dim_z_;
  double mu_;
  std::vector<double> box_start_;
  std::vector<double> box_w_;
  std::vector<double> box_h_;
};
}

