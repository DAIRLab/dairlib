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

namespace dairlib {
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::Context;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::LcmPublisherSystem;

// Simulation parameters.
DEFINE_string(address, "127.0.0.1", "IPv4 address to receive from.");
DEFINE_int64(port, 5000, "Port to receive on.");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  
  drake::lcm::DrakeLcm lcm;
  DiagramBuilder<double> builder;

  // Create input receiver.
  auto input_sub = builder.AddSystem(
      systems::CassieUDPSubscriber::Make(FLAGS_address, FLAGS_port));

  // Create publisher.
  auto output_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_cassie_out>("CASSIE_OUTPUT",
                                                           &lcm, 1.0/100.0));
  auto output_sender = builder.AddSystem<systems::CassieOutputSender>();

  // connect state publisher
  builder.Connect(input_sub->get_output_port(),
                  output_sender->get_input_port(0));

  builder.Connect(output_sender->get_output_port(0),
                  output_pub->get_input_port());

  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();

  Simulator<double> simulator(*diagram, std::move(diagram_context));

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(1.0);
  simulator.Initialize();

  std::cout << "Initialized! Starting." << std::endl;
  simulator.AdvanceTo(std::numeric_limits<double>::infinity());
  std::cout << "Done. Waiting for threads to die." << std::endl;
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::do_main(argc, argv);
}
