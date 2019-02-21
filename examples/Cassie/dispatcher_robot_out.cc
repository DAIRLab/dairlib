#include <memory>

#include <gflags/gflags.h>
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

#include "attic/multibody/rigidbody_utils.h"
#include "systems/robot_lcm_systems.h"
#include "systems/primitives/subvector_pass_through.h"
#include "examples/Cassie/networking/cassie_udp_subscriber.h"
#include "examples/Cassie/networking/cassie_output_sender.h"
#include "examples/Cassie/networking/udp_driven_loop.h"
#include "examples/Cassie/cassie_rbt_state_estimator.h"
#include "examples/Cassie/cassie_utils.h"
#include "dairlib/lcmt_cassie_out.hpp"
#include "dairlib/lcmt_robot_output.hpp"

namespace dairlib {
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::Context;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::TriggerType;

// Simulation parameters.
DEFINE_string(address, "127.0.0.1", "IPv4 address to receive from.");
DEFINE_int64(port, 5000, "Port to receive on.");
DEFINE_double(pub_rate, 0.02, "Network LCM pubishing period (s).");

/// Runs UDP driven loop for 10 seconds
/// Re-publishes any received messages as LCM
int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");
  drake::lcm::DrakeLcm lcm_network("udpm://239.255.76.67:7667?ttl=1");
  DiagramBuilder<double> builder;

  std::unique_ptr<RigidBodyTree<double>> tree = makeCassieTreePointer();

  // Create input receiver.
  auto input_sub = builder.AddSystem(
      systems::CassieUDPSubscriber::Make(FLAGS_address, FLAGS_port));

  // Create and connect CassieOutputSender publisher (low-rate for the network)
  // This echoes the messages from the robot
  auto output_sender = builder.AddSystem<systems::CassieOutputSender>();
  auto output_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_cassie_out>("CASSIE_OUTPUT",
      &lcm_network, {TriggerType::kPeriodic}, FLAGS_pub_rate));

  // connect cassie_out publisher
  builder.Connect(*input_sub, *output_sender);

  builder.Connect(*output_sender, *output_pub);

  // Create and connect state estimator
  auto state_estimator =
      builder.AddSystem<systems::CassieRbtStateEstimator>(*tree);
  builder.Connect(*input_sub, *state_estimator);

  // Create and connect RobotOutput publisher.
  auto state_sender = builder.AddSystem<systems::RobotOutputSender>(*tree);
  auto state_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          "CASSIE_STATE", &lcm_local,
          {TriggerType::kForced}));

  // Create and connect RobotOutput publisher (low-rate for the network)
  auto net_state_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          "NETWORK_CASSIE_STATE", &lcm_network,
          {TriggerType::kPeriodic}, FLAGS_pub_rate));

  // Pass through to drop all but positions and velocities
  auto passthrough = builder.AddSystem<systems::SubvectorPassThrough>(
    state_estimator->get_output_port(0).size(),
    0,
    state_sender->get_input_port(0).size());

  builder.Connect(*state_estimator, *passthrough);
  builder.Connect(*passthrough, *state_sender);
  builder.Connect(*state_sender, *state_pub);

  builder.Connect(*state_sender, *net_state_pub);

  auto diagram = builder.Build();

  systems::UDPDrivenLoop loop(*diagram, *input_sub, nullptr);

  // caused an extra publish call?
  loop.set_publish_on_every_received_message(true);

  // Starts the loop.
  loop.RunToSecondsAssumingInitialized(1e6);

  input_sub->StopPolling();
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::do_main(argc, argv);
}
