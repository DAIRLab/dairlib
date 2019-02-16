#include <memory>

#include <gflags/gflags.h>
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "examples/Cassie/networking/cassie_udp_subscriber.h"

namespace dairlib {
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::Context;


// Simulation parameters.
DEFINE_string(address, "127.0.0.1", "IPv4 address to receive from.");
DEFINE_int64(port, 5000, "Port to receive on.");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  DiagramBuilder<double> builder;

  // Create input receiver.
  auto input_sub = builder.AddSystem(
      systems::CassieUDPSubscriber::Make(FLAGS_address, FLAGS_port));
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
  simulator.StepTo(std::numeric_limits<double>::infinity());
  std::cout << "Done. Waiting for threads to die." << std::endl;
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::do_main(argc, argv);
}
