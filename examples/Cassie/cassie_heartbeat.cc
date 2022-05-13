#include <gflags/gflags.h>

#include "dairlib/lcmt_cassie_out.hpp"

#include "examples/Cassie/networking/cassie_output_sender.h"
#include "examples/Cassie/networking/simple_cassie_udp_subscriber.h"

#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace dairlib {

using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::TriggerType;
using drake::systems::lcm::LcmPublisherSystem;


DEFINE_bool(broadcast_robot_state, false, "broadcast to planner thread");

// Simulation parameters.
DEFINE_string(address, "127.0.0.1", "IPv4 address to receive on.");
DEFINE_int64(port, 25001, "Port to receive on.");
DEFINE_double(pub_rate, 0.5, "Network LCM pubishing period (s).");

DEFINE_string(channel, "CASSIE_HEARTBEAT","lcm channel to send on");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  drake::lcm::DrakeLcm lcm_network("udpm://239.255.76.67:7667?ttl=1");
  DiagramBuilder<double> builder;


  // Create and connect CassieOutputSender publisher (low-rate for the network)
  // This echoes the messages from the robot
  auto output_sender = builder.AddSystem<systems::CassieOutputSender>();
  auto output_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_cassie_out>(
          "CASSIE_OUTPUT_ECHO", &lcm_network, {TriggerType::kPeriodic},
          FLAGS_pub_rate));
  // connect cassie_out publisher
  builder.Connect(*output_sender, *output_pub);

  // Create the diagram, simulator, and context.
  auto owned_diagram = builder.Build();
  const auto& diagram = *owned_diagram;
  drake::systems::Simulator<double> simulator(std::move(owned_diagram));
  auto& diagram_context = simulator.get_mutable_context();
  auto& output_sender_context =
      diagram.GetMutableSubsystemContext(*output_sender, &diagram_context);

  // Wait for the first message.
  SimpleCassieUdpSubscriber udp_sub(FLAGS_address, FLAGS_port);
  drake::log()->info("Waiting for first UDP message from Cassie");
  udp_sub.Poll();

  // Initialize the context based on the first message.
  const double t0 = udp_sub.message_time();
  diagram_context.SetTime(t0);
  auto& output_sender_value = output_sender->get_input_port(0).FixValue(
      &output_sender_context, udp_sub.message());

  drake::log()->info("heartbeat started");

  while (true) {
    udp_sub.Poll();
    output_sender_value.GetMutableData()->set_value(udp_sub.message());
    const double time = udp_sub.message_time();

    // Check if we are very far ahead or behind
    // (likely due to a restart of the driving clock)
    if (time > simulator.get_context().get_time() + 1.0 ||
        time < simulator.get_context().get_time()) {
      std::cout << "Dispatcher time is " << simulator.get_context().get_time()
                << ", but stepping to " << time << std::endl;
      std::cout << "Difference is too large, resetting dispatcher time."
                << std::endl;
      simulator.get_mutable_context().SetTime(time);
    }
    simulator.AdvanceTo(time);
    // Force-publish via the diagram
    diagram.Publish(diagram_context);
  }

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }
