#include <memory>

#include <gflags/gflags.h>
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "examples/Cassie/networking/cassie_udp_publisher.h"
#include "examples/Cassie/networking/cassie_output_sender.h"

namespace dairlib {
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::Context;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::LcmPublisherSystem;

// Simulation parameters.
DEFINE_string(address, "127.0.0.1", "IPv4 address to send to.");
DEFINE_int64(port, 5000, "Port to send to.");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  drake::lcm::DrakeLcm lcm;
  DiagramBuilder<double> builder;

  // Create input receiver.
  auto input_pub = builder.AddSystem(
      systems::CassieUDPPublisher::Make(FLAGS_address, FLAGS_port, 1.0/30.0));

  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();

  Simulator<double> simulator(*diagram, std::move(diagram_context));

  cassie_user_in_t cassie_in{};
  cassie_in.torque[1] = 100;
  cassie_in.torque[2] = 100;
  cassie_in.torque[3] = 100;
  cassie_in.torque[4] = 100;
  cassie_in.torque[5] = 100;
  cassie_in.torque[6] = 100;
  cassie_in.torque[7] = 100;
  auto& input_pub_context = diagram->GetMutableSubsystemContext(
      *input_pub, &simulator.get_mutable_context());
  input_pub_context.FixInputPort(input_pub->get_input_port().get_index(),
      std::make_unique<drake::Value<cassie_user_in_t>>(cassie_in));

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
