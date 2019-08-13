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
using systems::RobotInputReceiver;
using systems::RobotCommandSender;
using drake::systems::TriggerType;

// Simulation parameters.
DEFINE_string(address, "127.0.0.1", "IPv4 address to publish to (UDP).");
DEFINE_int64(port, 25000, "Port to publish to (UDP).");
DEFINE_double(pub_rate, .02, "Network LCM pubishing period (s).");

// Cassie model paramter
DEFINE_bool(floating_base, false, "Fixed or floating base model");

/// Runs UDP driven loop for 10 seconds
/// Re-publishes any received messages as LCM
int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  const std::string channel_u = "CASSIE_INPUT";

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");
  drake::lcm::DrakeLcm lcm_network("udpm://239.255.76.67:7667?ttl=1");

  DiagramBuilder<double> builder;

  std::unique_ptr<RigidBodyTree<double>> tree;
  if (FLAGS_floating_base)
    tree = makeCassieTreePointer("examples/Cassie/urdf/cassie_v2.urdf",
                                 drake::multibody::joints::kQuaternion);
  else
    tree = makeCassieTreePointer();

  // Create LCM receiver for commands
  auto command_receiver = builder.AddSystem<RobotInputReceiver>(*tree);

  // auto command_sub = builder.AddSystem(
  //     LcmSubscriberSystem::Make<dairlib::lcmt_robot_input>(channel_u,
  //         &lcm_local));
  // builder.Connect(*command_sub, *command_receiver);

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

  // Create the diagram, simulator, and context.
  auto owned_diagram = builder.Build();
  const auto& diagram = *owned_diagram;
  drake::systems::Simulator<double> simulator(std::move(owned_diagram));
  auto& diagram_context = simulator.get_mutable_context();
  auto& command_receiver_context =
      diagram.GetMutableSubsystemContext(*command_receiver, &diagram_context);


  // Wait for the first message.
  drake::log()->info("Waiting for first lcmt_robot_input");
  drake::lcm::Subscriber<dairlib::lcmt_robot_input> command_sub(&lcm_local,
      channel_u);
  LcmHandleSubscriptionsUntil(&lcm_local, [&]() {
      return command_sub.count() > 0; });

  // Initialize the context based on the first message.
  const double t0 = command_sub.message().utime * 1e-6;
  diagram_context.SetTime(t0);
  auto& command_value = command_receiver->get_input_port(0).FixValue(
      &command_receiver_context, command_sub.message());

  drake::log()->info("dispatcher_robot_in started");
  while (true) {
    // Wait for an lcmt_robot_input message.
    command_sub.clear();
    LcmHandleSubscriptionsUntil(&lcm_local, [&]() {
        return command_sub.count() > 0; });
    // Write the lcmt_robot_input message into the context and advance.
    command_value.GetMutableData()->set_value(command_sub.message());
    const double time = command_sub.message().utime * 1e-6;

    // Check if we are very far ahead or behind
    // (likely due to a restart of the driving clock)
    if (time > simulator.get_context().get_time() + 1.0 ||
        time < simulator.get_context().get_time() - 1.0) {
      std::cout << "Dispatcher time is " << simulator.get_context().get_time()
          << ", but stepping to " << time << std::endl;
      std::cout << "Difference is too large, resetting dispatcher time." <<
          std::endl;
      simulator.get_mutable_context().SetTime(time);
    }

    simulator.AdvanceTo(time);
    // Force-publish via the diagram
    diagram.Publish(diagram_context);
  }
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::do_main(argc, argv);
}
