#include <memory>
#include <utility>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/urdf_parser.h"
#include "multibody/mbt_utils.h"
#include "common/find_resource.h"

namespace dairlib {
namespace multibody {


using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::parsing::AddModelFromUrdfFile;


int do_main()  {
  MultibodyPlant<double> plant;
  drake::geometry::SceneGraph<double> scene_graph;
  std::string full_name = FindResourceOrThrow(
      "examples/Cassie/urdf/cassie_v2.urdf");
  AddModelFromUrdfFile(full_name, &plant, &scene_graph);
  plant.Finalize();

  auto positions_map = utils::makeNameToPositionsMap(plant);

  for (const auto &p : positions_map) {
      std::cout << "positions[" << p.first << "] = " << p.second << '\n';
  }

  auto velocities_map = utils::makeNameToVelocitiesMap(plant);

  for (const auto &p : velocities_map) {
      std::cout << "velocities[" << p.first << "] = " << p.second << '\n';
  }


  auto actuators_map = utils::makeNameToActuatorsMap(plant);

  for (const auto &p : actuators_map) {
      std::cout << "actuators[" << p.first << "] = " << p.second << '\n';
  }

  return 0;
}


}  // namespace multibody
}  // namespace dairlib


int main(int argc, char* argv[]) {
  return dairlib::multibody::do_main();
}