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
#include "examples/Cassie/networking/simple_cassie_udp_subscriber.h"
#include "examples/Cassie/networking/cassie_output_sender.h"
#include "examples/Cassie/networking/cassie_output_receiver.h"
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
DEFINE_int64(port, 25001, "Port to receive on.");
DEFINE_double(pub_rate, 0.02, "Network LCM pubishing period (s).");
DEFINE_bool(simulation, false,
    "Simulated or real robot (default=false, real robot)");

// Cassie model paramter
DEFINE_bool(floating_base, false, "Fixed or floating base model");

/// Runs UDP driven loop for 10 seconds
/// Re-publishes any received messages as LCM
int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");
  drake::lcm::DrakeLcm lcm_network("udpm://239.255.76.67:7667?ttl=1");
  DiagramBuilder<double> builder;

  std::unique_ptr<RigidBodyTree<double>> tree;
  if (FLAGS_floating_base)
    tree = makeCassieTreePointer("examples/Cassie/urdf/cassie_v2.urdf",
                                 drake::multibody::joints::kQuaternion);
  else
    tree = makeCassieTreePointer();

  // Create state estimator
  auto state_estimator =
      builder.AddSystem<systems::CassieRbtStateEstimator>(*tree);

  // Create and connect CassieOutputSender publisher (low-rate for the network)
  // This echoes the messages from the robot
  auto output_sender = builder.AddSystem<systems::CassieOutputSender>();
  auto output_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_cassie_out>("CASSIE_OUTPUT_ECHO",
      &lcm_network, {TriggerType::kPeriodic}, FLAGS_pub_rate));
  // connect cassie_out publisher
  builder.Connect(*output_sender, *output_pub);

  // Connect appropriate input receiver for simulation
  systems::CassieOutputReceiver* input_receiver = nullptr;
  if (FLAGS_simulation) {
    input_receiver =
        builder.AddSystem<systems::CassieOutputReceiver>();
    builder.Connect(*input_receiver, *output_sender);
    builder.Connect(*input_receiver, *state_estimator);
  }

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



  // Create the diagram, simulator, and context.
  auto owned_diagram = builder.Build();
  const auto& diagram = *owned_diagram;
  drake::systems::Simulator<double> simulator(std::move(owned_diagram));
  auto& diagram_context = simulator.get_mutable_context();

  if (FLAGS_simulation) {
    auto& input_receiver_context =
      diagram.GetMutableSubsystemContext(*input_receiver, &diagram_context);

    // Wait for the first message.
    drake::log()->info("Waiting for first lcmt_cassie_out");
    drake::lcm::Subscriber<dairlib::lcmt_cassie_out> input_sub(&lcm_local,
        "CASSIE_OUTPUT");
    LcmHandleSubscriptionsUntil(&lcm_local, [&]() {
        return input_sub.count() > 0; });

    // Initialize the context based on the first message.
    const double t0 = input_sub.message().utime * 1e-6;
    diagram_context.SetTime(t0);
    auto& input_value = input_receiver->get_input_port(0).FixValue(
        &input_receiver_context, input_sub.message());

    drake::log()->info("dispatcher_robot_out started");
    while (true) {
      // Wait for an lcmt_cassie_out message.
      input_sub.clear();
      LcmHandleSubscriptionsUntil(&lcm_local, [&]() {
          return input_sub.count() > 0; });
      // Write the lcmt_robot_input message into the context and advance.
      input_value.GetMutableData()->set_value(input_sub.message());
      const double time = input_sub.message().utime * 1e-6;
      simulator.AdvanceTo(time);
      // Force-publish via the diagram
      diagram.Publish(diagram_context);
    }
  } else {
    auto& output_sender_context =
      diagram.GetMutableSubsystemContext(*output_sender, &diagram_context);
    auto& state_estimator_context =
      diagram.GetMutableSubsystemContext(*state_estimator, &diagram_context);

    // Wait for the first message.
    SimpleCassieUdpSubscriber udp_sub(FLAGS_address, FLAGS_port);
    drake::log()->info("Waiting for first UDP message from Cassie");
    udp_sub.Poll();

    // Initialize the context based on the first message.
    const double t0 = udp_sub.message_time();
    diagram_context.SetTime(t0);
    auto& output_sender_value = output_sender->get_input_port(0).FixValue(
        &output_sender_context, udp_sub.message());
    auto& state_estimator_value = state_estimator->get_input_port(0).FixValue(
        &state_estimator_context, udp_sub.message());
    drake::log()->info("dispatcher_robot_out started");

    while (true) {
      udp_sub.Poll();
      output_sender_value.GetMutableData()->set_value(udp_sub.message());
      state_estimator_value.GetMutableData()->set_value(udp_sub.message());
      const double time = udp_sub.message_time();
      simulator.AdvanceTo(time);
      // Force-publish via the diagram
      diagram.Publish(diagram_context);
    }
  }
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::do_main(argc, argv);
}
