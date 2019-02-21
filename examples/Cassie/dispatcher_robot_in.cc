#include <memory>

#include <gflags/gflags.h>
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/lcm/lcm_driven_loop.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

#include "attic/multibody/rigidbody_utils.h"
#include "systems/robot_lcm_systems.h"
#include "examples/Cassie/networking/cassie_udp_publisher.h"
#include "examples/Cassie/networking/cassie_input_translator.h"
#include "examples/Cassie/cassie_utils.h"
#include "dairlib/lcmt_cassie_out.hpp"
#include "dairlib/lcmt_robot_output.hpp"

namespace dairlib {
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::Context;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::UtimeMessageToSeconds;
using systems::RobotInputReceiver;
using systems::RobotCommandSender;
using drake::systems::TriggerType;

// Simulation parameters.
DEFINE_string(address, "127.0.0.1", "IPv4 address to publish to (UDP).");
DEFINE_int64(port, 5000, "Port to publish to (UDP).");
DEFINE_double(pub_rate, .02, "Network LCM pubishing period (s).");


/// Runs UDP driven loop for 10 seconds
/// Re-publishes any received messages as LCM
int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  const std::string channel_u = "CASSIE_INPUT";

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");
  drake::lcm::DrakeLcm lcm_network("udpm://239.255.76.67:7667?ttl=1");

  DiagramBuilder<double> builder;

  std::unique_ptr<RigidBodyTree<double>> tree = makeCassieTreePointer();

  // Crate LCM subscriber/receiver for commands
  auto command_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_robot_input>(channel_u,
          &lcm_local));
  auto command_receiver = builder.AddSystem<RobotInputReceiver>(*tree);
  builder.Connect(*command_sub, *command_receiver);

  // Create and connect translator
  auto input_translator =
      builder.AddSystem<systems::CassieInputTranslator>(*tree);
  builder.Connect(*command_receiver, *input_translator);

  // Create and connect input publisher.
  auto input_pub = builder.AddSystem(
      systems::CassieUDPPublisher::Make(FLAGS_address, FLAGS_port,
          {TriggerType::kForced}));
  builder.Connect(*input_translator, *input_pub);

  // Create and connect LCM command echo to network
  auto net_command_sender = builder.AddSystem<RobotCommandSender>(*tree);
  auto net_command_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          "NETWORK_CASSIE_INPUT", &lcm_network,
          {TriggerType::kPeriodic}, FLAGS_pub_rate));

  builder.Connect(*command_receiver, *net_command_sender);

  builder.Connect(*net_command_sender, *net_command_pub);

  auto diagram = builder.Build();

  drake::systems::lcm::LcmDrivenLoop loop(*diagram, *command_sub, nullptr,
      &lcm_local,
      std::make_unique<UtimeMessageToSeconds<dairlib::lcmt_robot_input>>());

  // caused an extra publish call?
  loop.set_publish_on_every_received_message(true);

  // Starts the loop.
  loop.RunToSecondsAssumingInitialized(1e6);
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::do_main(argc, argv);
}
