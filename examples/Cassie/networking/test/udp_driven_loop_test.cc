#include <memory>

#include <gflags/gflags.h>
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "examples/Cassie/networking/cassie_udp_subscriber.h"
#include "examples/Cassie/networking/cassie_output_sender.h"
#include "examples/Cassie/networking/udp_driven_loop.h"

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

/// Runs UDP driven loop for 10 seconds
/// Re-publishes any received messages as LCM
int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  drake::lcm::DrakeLcm lcm;
  DiagramBuilder<double> builder;

  // Create input receiver.
  auto input_sub = builder.AddSystem(
      systems::CassieUDPSubscriber::Make(FLAGS_address, FLAGS_port));

  // Create publisher--no publishing rate since this will be driven by LCM
  auto output_sender = builder.AddSystem<systems::CassieOutputSender>();
  auto output_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_cassie_out>("CASSIE_OUTPUT",
      &lcm, {TriggerType::kForced}));

  // connect state publisher
  builder.Connect(input_sub->get_output_port(),
                  output_sender->get_input_port(0));

  builder.Connect(output_sender->get_output_port(0),
                  output_pub->get_input_port());

  auto diagram = builder.Build();

  systems::UDPDrivenLoop loop(*diagram, *input_sub, nullptr);

  // caused an extra publish call?
  loop.set_publish_on_every_received_message(true);

  // Starts the loop.
  loop.RunToSecondsAssumingInitialized(10.0);

  input_sub->StopPolling();
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::do_main(argc, argv);
}
