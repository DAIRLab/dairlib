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

}

