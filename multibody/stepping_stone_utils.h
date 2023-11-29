#pragma once

#include "geometry/convex_polygon.h"
#include "multibody_utils.h"

#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/geometry/meshcat.h"

using dairlib::geometry::ConvexPolygon;
using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;
using Eigen::Vector3d;



namespace dairlib {
namespace multibody {

struct SquareSteppingStoneList {

  std::vector<std::vector<std::vector<double>>> stones;
  std::vector<std::pair<RigidTransformd, Eigen::Vector3d>> cubes;
  std::vector<ConvexPolygon> footholds;

  template<typename Archive>
  void Serialize(Archive *a) {
    a->Visit(DRAKE_NVP(stones));
    std::tie(this->footholds, this->cubes) = GetFootholdsWithMargin(stones, 0.12);
  }

  static std::pair<std::vector<ConvexPolygon>,
                   std::vector<std::pair<RigidTransformd, Eigen::Vector3d>>>
  GetFootholdsWithMargin(
      std::vector<std::vector<std::vector<double>>> stones, double margin) {

    DRAKE_ASSERT(margin >= 0);

    std::vector<ConvexPolygon> footholds;
    std::vector<std::pair<RigidTransformd, Eigen::Vector3d>> cubes;
    for (auto &stone : stones) {
      auto center = Vector3d::Map(stone.at(0).data());
      auto normal = Vector3d::Map(stone.at(1).data());
      auto dims = Vector3d::Map(stone.at(2).data());
      double yaw = stone.at(3).at(0);

      // stepping stone must be right side up
      DRAKE_DEMAND(normal(2) > 0);

      // Make the cube rotation from the surface normal and yaw
      Vector3d b_z = normal.normalized();
      Vector3d b_x(b_z(2), 0, -b_z(0));
      b_x.normalize();
      Vector3d b_y = b_z.cross(b_x);
      RotationMatrixd R_WB =
          RotationMatrixd::MakeFromOrthonormalColumns(b_x, b_y, b_z);
      // rotate the whole thing about the parent frame z-axis by rotz
      R_WB = R_WB * RotationMatrixd::MakeZRotation(yaw);

      // local x, y, and z axes of the box, expressed in the world frame
      Vector3d Bx = R_WB.col(0);
      Vector3d By = R_WB.col(1);
      Vector3d Bz = R_WB.col(2);

      // make the faces parallel to the world z-axis to make
      // 2d violation accurate for calculating height maps based on 2d location
      double rx = -Bx(2) / Bz(2);
      double ry = -By(2) / Bz(2);
      Vector3d nx = Bx + rx * Bz;
      Vector3d ny = By + ry * Bz;
      nx.normalize();
      ny.normalize();

      Vector3d box_center = center - 0.5 * dims(2) * R_WB.col(2);
      cubes.push_back({RigidTransformd(R_WB, box_center), dims});

      ConvexPolygon foothold;
      foothold.SetPlane(normal, center);

      double e = margin;
      // stepping stone boundaries
      foothold.AddFace(nx, center + 0.5 * (dims(0) - 2 * e) * Bx);
      foothold.AddFace(-nx, center - 0.5 * (dims(0) - 2 * e) * Bx);
      foothold.AddFace(ny, center + 0.5 * (dims(1) - e) * By);
      foothold.AddFace(-ny, center - 0.5 * (dims(1) - e) * By);
      foothold.CalcBoundingBox();
      footholds.push_back(foothold);
    }
    return {footholds, cubes};
  }
};

SquareSteppingStoneList LoadSteppingStonesFromYaml(const std::string& filename);

void AddSteppingStonesToSim(drake::multibody::MultibodyPlant<double>* plant,
                            drake::geometry::SceneGraph<double>* scene_graph,
                            const SquareSteppingStoneList& stones,
                            double mu);

void AddSteppingStonesToSim(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph,
    const std::variant<std::string, SquareSteppingStoneList>& stones,
    double mu);

void AddSteppingStonesToMeshcatFromYaml(std::shared_ptr<drake::geometry::Meshcat> meshcat,
                                        const std::string& filename);

void AddSteppingStonesToSimFromYaml(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph,
    const std::string& filename, double mu);
}
}