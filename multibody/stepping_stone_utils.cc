#include "stepping_stone_utils.h"
#include "drake/common/yaml/yaml_io.h"

using drake::multibody::MultibodyPlant;
using drake::geometry::SceneGraph;
using drake::geometry::Box;

using Eigen::Vector3d;

namespace dairlib::multibody {

void AddSteppingStonesToSim(MultibodyPlant<double>* plant,
                            SceneGraph<double>* scene_graph,
                            const SquareSteppingStoneList& stones,
                            double mu) {
  for (const auto& cube: stones.cubes) {
    AddBox(plant, scene_graph, cube.first, cube.second,mu);
  }
}

void AddSteppingStonesToSim(
    MultibodyPlant<double>* plant,
    SceneGraph<double>* scene_graph,
    const std::variant<std::string, SquareSteppingStoneList>& stones,
    double mu) {

  const auto& stepping_stone_list = std::holds_alternative<std::string>(stones) ?
                     LoadSteppingStonesFromYaml(std::get<std::string>(stones)) :
                                    std::get<SquareSteppingStoneList>(stones);
  AddSteppingStonesToSim(plant, scene_graph, stepping_stone_list, mu);
}

void AddSteppingStonesToSimFromYaml(MultibodyPlant<double>* plant,
                                    SceneGraph<double>* scene_graph,
                                    const std::string& filename, double mu) {
  AddSteppingStonesToSim(
      plant,
      scene_graph,
      LoadSteppingStonesFromYaml(filename),
      mu
  );
}

SquareSteppingStoneList
LoadSteppingStonesFromYaml(const std::string& filename) {
  return drake::yaml::LoadYamlFile<SquareSteppingStoneList>(filename);
}

void AddSteppingStonesToMeshcatFromYaml(
    std::shared_ptr<drake::geometry::Meshcat> meshcat,
    const std::string& filename) {
  auto boxes = LoadSteppingStonesFromYaml(filename).cubes;
  static int i = 0;
  for (const auto& box_params : boxes) {
    std::string path = "box" + std::to_string(++i);
    const auto box = drake::geometry::Box(box_params.second);
    meshcat->SetObject(path,box);
    meshcat->SetTransform(path, box_params.first);
  }
}

std::vector<ConvexPolygon>
SquareSteppingStoneList::GetConvexPolygonsForHeightmapSimulation (
    std::vector<std::vector<std::vector<double>>> stone_list) {
    std::vector<ConvexPolygon> footholds_out;

  for (auto &stone : stone_list) {
    auto center = Vector3d::Map(stone.at(0).data());
    auto normal = Vector3d::Map(stone.at(1).data());
    auto dims = Vector3d::Map(stone.at(2).data());
    double yaw = stone.at(3).at(0);

    // stepping stone must be right side up
    DRAKE_DEMAND(normal(2) > 0);

    // Make the cube rotation from the surface normal and yaw
    Vector3d b_z = normal.normalized();
    Vector3d b_x (b_z(2), 0, -b_z(0));

    // normal is too close to the world y axis for
    // y.cross(normal) to give a good x axis, pick -90
    // degree rotation about world z axis instead
    if (b_x.squaredNorm() < 0.001) {
      b_x = b_z.cross(Vector3d::UnitZ());
    }
    b_x.normalize();
    Vector3d b_y = b_z.cross(b_x).normalized();
    RotationMatrixd R_WB =
        RotationMatrixd::MakeFromOrthonormalColumns(b_x, b_y, b_z);
    // rotate the whole thing about the parent frame z-axis by yaw
    R_WB = R_WB * RotationMatrixd::MakeZRotation(yaw);

    // local x, y, and z axes of the box, expressed in the world frame
    Vector3d Bx = R_WB.col(0);
    Vector3d By = R_WB.col(1);
    Vector3d Bz = R_WB.col(2);

    Vector3d box_center = center - 0.5 * dims(2) * Bz;

    if (abs(Bx(2)) > 1e-2) {
      Vector3d nx = Bx(2) >= 0 ? Bx : -Bx;

      footholds_out.push_back(
          ConvexPolygon::MakeBoxFaceForTerrain(
              nx,
              box_center + 0.5 * dims(0) * nx,
              0.5 * dims(1) * By,
              0.5 * dims(2) * Bz)
      );
    }
    if (abs(By(2)) > 1e-2) {
      Vector3d ny = By(2) >= 0 ? By : -By;
      footholds_out.push_back(
          ConvexPolygon::MakeBoxFaceForTerrain(
              ny,
              box_center + 0.5 * dims(1) * ny,
              0.5 * dims(2) * Bz,
              0.5 * dims(0) * Bx)
      );
    }
    if (abs(Bz(2)) > 1e-2) {
      Vector3d nz = Bz(2) >= 0 ? Bz : -Bz;
      footholds_out.push_back(
          ConvexPolygon::MakeBoxFaceForTerrain(
              nz,
              box_center + 0.5 * dims(2) * nz,
              0.5 * dims(0) * Bx,
              0.5 * dims(1) * By)
      );
    }
  }
  return footholds;
}

}

