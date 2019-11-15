#include <memory>
#include <limits>

#include <gflags/gflags.h>
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

#include "attic/multibody/rigidbody_utils.h"
#include "systems/robot_lcm_systems.h"
#include "examples/Cassie/input_supervisor.h"
#include "examples/Cassie/networking/cassie_udp_publisher.h"
#include "examples/Cassie/networking/cassie_input_translator.h"
#include "examples/Cassie/cassie_utils.h"
#include "dairlib/lcmt_cassie_out.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_controller_switch.hpp"

namespace dairlib {
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::Context;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::LcmPublisherSystem;
using systems::RobotInputReceiver;
using systems::RobotCommandSender;
using drake::systems::TriggerType;
using drake::lcm::Subscriber;

using std::map;
using std::string;

// Simulation parameters.
DEFINE_string(address, "127.0.0.1", "IPv4 address to publish to (UDP).");
DEFINE_int64(port, 25000, "Port to publish to (UDP).");
DEFINE_double(pub_rate, .02, "Network LCM pubishing period (s).");
DEFINE_double(max_joint_velocity, 10,
    "Maximum joint velocity before error is triggered");
DEFINE_double(input_limit, -1, "Maximum torque limit. Negative values are inf.");
DEFINE_int64(supervisor_N, 10,
    "Maximum allowed consecutive failures of velocity limit.");
DEFINE_string(state_channel_name, "CASSIE_STATE",
    "The name of the lcm channel that sends Cassie's state");

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

  // Create LCM receiver for commands
  auto command_receiver = builder.AddSystem<RobotInputReceiver>(*tree);

    // Create state estimate receiver, used for safety checks
  auto state_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(
          FLAGS_state_channel_name, &lcm_local));
  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(*tree);
  builder.Connect(*state_sub, *state_receiver);

  double input_supervisor_update_period = 1.0/1000.0;
  double input_limit = FLAGS_input_limit;
  if (input_limit < 0) {
    input_limit = std::numeric_limits<double>::max();
  }

  auto input_supervisor = builder.AddSystem<InputSupervisor>(*tree,
      FLAGS_max_joint_velocity, input_supervisor_update_period,
      FLAGS_supervisor_N, input_limit);
  builder.Connect(state_receiver->get_output_port(0),
      input_supervisor->get_input_port_state());
  builder.Connect(command_receiver->get_output_port(0),
      input_supervisor->get_input_port_command());

  // Create and connect translator
  auto input_translator =
      builder.AddSystem<systems::CassieInputTranslator>(*tree);
  builder.Connect(*input_supervisor, *input_translator);

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

  builder.Connect(*input_supervisor, *net_command_sender);

  builder.Connect(*net_command_sender, *net_command_pub);

  // Create the diagram, simulator, and context.
  auto owned_diagram = builder.Build();
  const auto& diagram = *owned_diagram;
  drake::systems::Simulator<double> simulator(std::move(owned_diagram));
  auto& diagram_context = simulator.get_mutable_context();
  auto& command_receiver_context =
      diagram.GetMutableSubsystemContext(*command_receiver, &diagram_context);

  // Channel name of the input switch
  std::string switch_channel = "INPUT_SWITCH";
  // Channel names of the controllers
  std::vector<std::string> input_channels;
  input_channels.push_back("CASSIE_INPUT");
  input_channels.push_back("PD_CONTROLLER");
  input_channels.push_back("OSC_CONTROLLER");
  std::string active_channel = input_channels.at(0);

  // Create subscribers
  Subscriber<dairlib::lcmt_controller_switch> input_switch_sub(&lcm_local,
                                                         switch_channel);
  map<string, Subscriber<dairlib::lcmt_robot_input>> name_to_sub_map;
  for (auto name : input_channels) {
    // TODO: double check that the scope of the next line is correct for
    // Subscriber (it's created locally, but should be copied to map)
    name_to_sub_map.insert(std::make_pair(name,
        Subscriber<dairlib::lcmt_robot_input>(&lcm_local,name)));
  }

  // Wait for the first message.
  drake::log()->info("Waiting for first lcmt_robot_input");
  LcmHandleSubscriptionsUntil(&lcm_local, [&]() {
      return name_to_sub_map.at(active_channel).count() > 0; });

  // Initialize the context based on the first message.
  const double t0 = name_to_sub_map.at(active_channel).message().utime * 1e-6;
  diagram_context.SetTime(t0);
  auto& command_value = command_receiver->get_input_port(0).FixValue(
      &command_receiver_context, name_to_sub_map.at(active_channel).message());

  // "Simulator" time
  double time = 0; // initialize the current time to 0
  const double end_time = std::numeric_limits<double>::infinity();
  double message_time;

  drake::log()->info("dispatcher_robot_in started");
  while (time < end_time) {
    // Update active channel name
    if (input_switch_sub.count() > 0) {
      active_channel = input_switch_sub.message().channel;
      input_switch_sub.clear();
    }

    // Wait for an lcmt_robot_input message.
    name_to_sub_map.at(active_channel).clear();
    LcmHandleSubscriptionsUntil(&lcm_local, [&]() {
      return (name_to_sub_map.at(active_channel).count() > 0); });

    // Write the lcmt_robot_input message into the context.
    command_value.GetMutableData()->set_value(
        name_to_sub_map.at(active_channel).message());

    // Get message time from the active channel to advance
    message_time = name_to_sub_map.at(active_channel).message().utime * 1e-6;
    // We cap the time from bottom just in case the message time is older around
    // the instant when we switch to a different controller
    if (message_time >= time) {
      time = message_time;
    }

    // Check if we are very far ahead or behind
    // (likely due to a restart of the driving clock)
    if (time > simulator.get_context().get_time() + 1.0 ||
        time < simulator.get_context().get_time()) {
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
