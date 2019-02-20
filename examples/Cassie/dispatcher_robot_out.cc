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

// Simulation parameters.
DEFINE_string(address, "127.0.0.1", "IPv4 address to receive from.");
DEFINE_int64(port, 5000, "Port to receive on.");
DEFINE_double(pub_rate, 0, "LCM pubishing rate.");


/// Runs UDP driven loop for 10 seconds
/// Re-publishes any received messages as LCM
int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  drake::lcm::DrakeLcm lcm;
  DiagramBuilder<double> builder;

  std::unique_ptr<RigidBodyTree<double>> tree = makeCassieTreePointer();

  // Create input receiver.
  auto input_sub = builder.AddSystem(
      systems::CassieUDPSubscriber::Make(FLAGS_address, FLAGS_port));

  // Create publisher--no publishing rate since this will be driven by LCM
  auto output_sender = builder.AddSystem<systems::CassieOutputSender>();
  auto output_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_cassie_out>("CASSIE_OUTPUT",
      &lcm, FLAGS_pub_rate));

  // connect cassie_out publisher
  builder.Connect(input_sub->get_output_port(),
                  output_sender->get_input_port(0));

  builder.Connect(output_sender->get_output_port(0),
                  output_pub->get_input_port());

  // Create and connect state estimator
  auto state_estimator =
      builder.AddSystem<systems::CassieRbtStateEstimator>(*tree);
  builder.Connect(input_sub->get_output_port(),
                  state_estimator->get_input_port(0));

  // Create and connect RobotOutput publisher.
  auto state_sender = builder.AddSystem<systems::RobotOutputSender>(*tree);
  auto state_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          "CASSIE_STATE", &lcm, FLAGS_pub_rate));

  // Pass through to drop all but positions and velocities
  auto passthrough = builder.AddSystem<systems::SubvectorPassThrough>(
    state_estimator->get_output_port(0).size(),
    0,
    state_sender->get_input_port(0).size());

  builder.Connect(state_estimator->get_output_port(0),
      passthrough->get_input_port());

  builder.Connect(passthrough->get_output_port(),
      state_sender->get_input_port_state());

  builder.Connect(state_sender->get_output_port(0),
                  state_pub->get_input_port());

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
