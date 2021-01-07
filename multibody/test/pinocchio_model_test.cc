//
// Created by brian on 11/10/20.
//

#include <memory>
#include <utility>

#include "examples/Cassie/cassie_utils.h"
#include "multibody/multibody_utils.h"
#include "multibody/pinocchio_model.h"
#include "common/find_resource.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"


namespace dairlib {
namespace multibody {

using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::geometry::SceneGraph;

int PinocchioModelTestMain(int argc, char **argv) {
  SceneGraph<double> scene_graph;

  MultibodyPlant<double> plant(0);
  addCassieMultibody(&plant, nullptr, true, "examples/Cassie/urdf/cassie_v2.urdf",
      true, false);
  plant.Finalize();

  std::cout << "Build Pinocchio Model" <<std::endl;
  PinocchioModel pinocchio_model(plant, "examples/Cassie/urdf/cassie_v2.urdf", false);

  std::cout << "Nq: " << plant.num_positions() << std::endl;
  std::cout << "Nv: " << plant.num_velocities() << std::endl;

  std::cout << std::endl << pinocchio_model.GetPositionMapDrakeToPinocchio() << std::endl;
  std::cout << std::endl << pinocchio_model.GetVelocityMapDrakeToPinocchio() << std::endl;
  std::cout << std::endl << pinocchio_model.GetPositionMapPinocchioToDrake() << std::endl;
  std::cout << std::endl << pinocchio_model.GetVelocityMapPinocchioToDrake() << std::endl;

  return 0;
}
}
}

int main(int argc, char **argv) {
    return dairlib::multibody::PinocchioModelTestMain(argc, argv);
}