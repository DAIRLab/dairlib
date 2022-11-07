#pragma once

#include "geometry/convex_foothold.h"

#include "drake/math/rigid_transform.h"
#include "drake/common/yaml/yaml_read_archive.h"
#include "yaml-cpp/yaml.h"

using dairlib::geometry::ConvexFoothold;
using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;
using Eigen::Vector3d;



namespace dairlib::multibody {

struct SquareSteppingStone {

  std::vector<std::vector<std::vector<double>>> stones;
  std::vector<std::pair<RigidTransformd, Eigen::Vector3d>> cubes;
  std::vector<ConvexFoothold> footholds;

  template<typename Archive>
  void Serialize(Archive *a) {
    a->Visit(stones);

    for (auto &stone : stones) {
      auto center = Vector3d::Map(stone.at(0).data());
      auto normal = Vector3d::Map(stone.at(1).data());
      auto dims = Vector3d::Map(stone.at(2).data());
      double yaw = stone.at(3).at(0);

      // Make the cube rotation from the surface normal and yaw
      Vector3d b_z = normal.normalized();
      Vector3d b_x (b_z(2), 0, -b_z(0));
      b_x.normalize();
      Vector3d b_y = b_z.cross(b_x);
      RotationMatrixd R_WB =
          RotationMatrixd::MakeFromOrthonormalColumns(b_x, b_y, b_z);

      // rotate the whole thing about the parent frame z-axis by rotz
      R_WB = R_WB * RotationMatrixd::MakeZRotation(yaw);

      Vector3d box_center = center - 0.5 * dims(2) * R_WB.matrix() * Vector3d::UnitZ();
      cubes.push_back({RigidTransformd(R_WB, box_center), dims});

      ConvexFoothold foothold;
      foothold.SetContactPlane(normal, center);
      foothold.AddFace(normal, center + 0.5 * dims(0) * R_WB.matrix() * Vector3d::UnitX());
      foothold.AddFace(normal, center - 0.5 * dims(0) * R_WB.matrix() * Vector3d::UnitX());
      foothold.AddFace(normal, center + 0.5 * dims(1) * R_WB.matrix() * Vector3d::UnitY());
      foothold.AddFace(normal, center - 0.5 * dims(1) * R_WB.matrix() * Vector3d::UnitY());
    }
  }

};
}