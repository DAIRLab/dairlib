#include <gflags/gflags.h>

#include "dairlib/lcmt_robot_output.hpp"
#include "systems/framework/output_vector.h"
#include "systems/robot_lcm_systems.h"
#include "systems/framework/lcm_driven_loop.h"
#include "common/find_resource.h"
#include "cartpole_input_interface.h"

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"

DEFINE_string(channel_u, "CARTPOLE_INPUT",
              "input channel to subscribe to");

namespace dairlib::examples::cartpole {


using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::TriggerType;
using drake::systems::Simulator;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;

using systems::RobotInputReceiver;
using systems::LcmDrivenLoop;

int do_main(int argc, char*argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  DiagramBuilder<double> builder;

  MultibodyPlant<double> plant(0.0);
  Parser(&plant).AddModelFromFile(
      FindResourceOrThrow(
          "examples/cartpole/urdf/cartpole.urdf"));
  plant.Finalize();

  // setup lcm communications
  drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0") ;

  auto input_reciver = builder.AddSystem<RobotInputReceiver>(plant);
  auto input_interface = builder.AddSystem<CartpoleInputInterface>();
  builder.Connect(*input_reciver, *input_interface);

  auto diagram = builder.Build();
  input_interface->SetupEposDevice();

  LcmDrivenLoop<dairlib::lcmt_robot_input> loop(
      &lcm,
      std::move(diagram),
      input_reciver,
      FLAGS_channel_u,
      true);
  loop.Simulate();
  return 0;
}

}


int main(int argc, char* argv[]) {
  return dairlib::examples::cartpole::do_main(argc, argv);
}