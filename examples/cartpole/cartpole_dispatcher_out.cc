#include <gflags/gflags.h>

#include "dairlib/lcmt_robot_output.hpp"
#include "systems/framework/output_vector.h"
#include "systems/robot_lcm_systems.h"
#include "common/find_resource.h"
#include "cartpole_output_interface.h"

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"

namespace dairlib::examples::cartpole {

using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::TriggerType;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::Simulator;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;

using systems::RobotOutputSender;

DEFINE_string(channel_x, "CARTPOLE_STATE",
    "state channel to publish on");
DEFINE_double(pub_rate, 0.005, "LCM pubishing period (s).");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  DiagramBuilder<double> builder;

  MultibodyPlant<double> plant(0.0);
  Parser(&plant).AddModelFromFile(
      FindResourceOrThrow(
          "examples/cartpole/urdf/cartpole.urdf"));
  plant.Finalize();

  // setup lcm communications
  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();

  auto output_interface = builder.AddSystem<CartpoleOutputInterface>(plant);
  auto state_sender = builder.AddSystem<RobotOutputSender>(plant);
  auto state_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_robot_output>(
          FLAGS_channel_x, lcm,
          {TriggerType::kPeriodic}, FLAGS_pub_rate));

  builder.Connect(output_interface->get_output_port(),
      state_sender->get_input_port_state());
  builder.Connect(*state_sender, *state_publisher);

  auto diagram = builder.Build();
  output_interface->SetupOutputInterface();
  Simulator<double> simulator(std::move(diagram));
  simulator.Initialize();
  simulator.AdvanceTo(std::numeric_limits<double>::infinity());
  return 0;
}


}

int main(int argc, char* argv[]) {
  return dairlib::examples::cartpole::do_main(argc, argv);
}