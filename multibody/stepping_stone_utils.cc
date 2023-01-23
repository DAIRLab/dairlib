#include "stepping_stone_utils.h"
#include "drake/common/yaml/yaml_io.h"

using drake::multibody::MultibodyPlant;
using drake::geometry::SceneGraph;
using Eigen::Vector3d;

namespace dairlib::multibody {

void AddSteppingStonesToSim(MultibodyPlant<double>* plant,
                            SceneGraph<double>* scene_graph,
                            SquareSteppingStoneList stones,
                            double mu) {
  for (const auto& cube: stones.cubes) {
    AddBox(plant, scene_graph, cube.first, cube.second,mu);
  }
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

}

