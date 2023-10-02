#pragma once

#include "geometry/convex_foothold.h"
#include "multibody_utils.h"

#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/geometry/meshcat.h"

using dairlib::geometry::ConvexFoothold;
using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;
using Eigen::Vector3d;



namespace dairlib {
namespace multibody {

struct SquareSteppingStoneList {

  std::vector<std::vector<std::vector<double>>> stones;
  std::vector<std::pair<RigidTransformd, Eigen::Vector3d>> cubes;
  std::vector<ConvexFoothold> footholds;

  template<typename Archive>
  void Serialize(Archive *a) {
    a->Visit(DRAKE_NVP(stones));
    std::tie(this->footholds, this->cubes) = GetFootholdsWithMargin(stones, 0.12);
  }

  static std::pair<std::vector<ConvexFoothold>,
                   std::vector<std::pair<RigidTransformd, Eigen::Vector3d>>>
  GetFootholdsWithMargin(
      std::vector<std::vector<std::vector<double>>> stones, double margin) {

    DRAKE_ASSERT(margin >= 0);

    std::vector<ConvexFoothold> footholds;
    std::vector<std::pair<RigidTransformd, Eigen::Vector3d>> cubes;
    for (auto &stone : stones) {
      auto center = Vector3d::Map(stone.at(0).data());
      auto normal = Vector3d::Map(stone.at(1).data());
      auto dims = Vector3d::Map(stone.at(2).data());
      double yaw = stone.at(3).at(0);

      // Make the cube rotation from the surface normal and yaw
      Vector3d b_z = normal.normalized();
      Vector3d b_x(b_z(2), 0, -b_z(0));
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

      double e = margin;
      // stepping stone boundaries
      foothold.AddFace(
          R_WB.matrix() * Vector3d::UnitX(),
          center + 0.5 * (dims(0) - 2 * e) * R_WB.matrix() * Vector3d::UnitX());
      foothold.AddFace(
          -R_WB.matrix() * Vector3d::UnitX(),
          center - 0.5 * (dims(0) - 2 * e) * R_WB.matrix() * Vector3d::UnitX());
      foothold.AddFace(
          R_WB.matrix() * Vector3d::UnitY(),
          center + 0.5 * (dims(1) - e) * R_WB.matrix() * Vector3d::UnitY());
      foothold.AddFace(
          -R_WB.matrix() * Vector3d::UnitY(),
          center - 0.5 * (dims(1) - e) * R_WB.matrix() * Vector3d::UnitY());
      footholds.push_back(foothold);
    }
    return {footholds, cubes};
  }
};

SquareSteppingStoneList LoadSteppingStonesFromYaml(const std::string& filename);

void AddSteppingStonesToSim(drake::multibody::MultibodyPlant<double>* plant,
                            drake::geometry::SceneGraph<double>* scene_graph,
                            SquareSteppingStoneList stones,
                            double mu);

void AddSteppingStonesToMeshcatFromYaml(std::shared_ptr<drake::geometry::Meshcat> meshcat,
                                        const std::string& filename);

void AddSteppingStonesToSimFromYaml(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph,
    const std::string& filename, double mu);
}
}