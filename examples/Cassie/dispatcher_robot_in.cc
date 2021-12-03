#include <limits>
#include <memory>

#include <gflags/gflags.h>

#include "dairlib/lcmt_controller_error.hpp"
#include "dairlib/lcmt_controller_switch.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/input_supervisor.h"
#include "examples/Cassie/networking/cassie_input_translator.h"
#include "examples/Cassie/networking/cassie_udp_publisher.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"

#include "drake/lcm/drake_lcm.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace dairlib {
using drake::lcm::Subscriber;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::TriggerType;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using systems::RobotCommandSender;
using systems::RobotInputReceiver;

using std::map;
using std::string;

// Simulation parameters.
DEFINE_string(address, "127.0.0.1", "IPv4 address to publish to (UDP).");
DEFINE_int64(port, 25000, "Port to publish to (UDP).");
DEFINE_double(pub_rate, .02, "Network LCM pubishing period (s).");
DEFINE_string(
    cassie_out_channel, "CASSIE_OUTPUT_ECHO",
    "The name of the channel to receive the cassie out structure from.");
DEFINE_double(max_joint_velocity, 20,
              "Maximum joint velocity before error is triggered");
DEFINE_double(input_limit, -1,
              "Maximum torque limit. Negative values are inf.");
DEFINE_int64(supervisor_N, 10,
             "Maximum allowed consecutive failures of velocity limit.");
DEFINE_string(state_channel_name, "CASSIE_STATE_DISPATCHER",
              "The name of the lcm channel that sends Cassie's state");
DEFINE_string(control_channel_name_1, "PD_CONTROL",
              "The name of the lcm channel that sends Cassie's state");
DEFINE_string(control_channel_name_2, "OSC_STANDING",
              "The name of the lcm channel that sends Cassie's state");
DEFINE_string(control_channel_name_3, "OSC_WALKING",
              "The name of the lcm channel that sends Cassie's state");
DEFINE_bool(
    sim, false,
    "Whether or not this dispatcher is being used with the simulated robot");

// Cassie model parameter
DEFINE_bool(floating_base, true, "Fixed or floating base model");

/// Runs UDP driven loop for 10 seconds
/// Re-publishes any received messages as LCM
int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");
  drake::lcm::DrakeLcm lcm_network("udpm://239.255.76.67:7667?ttl=1");

  DiagramBuilder<double> builder;

  // Build Cassie MBP
  drake::multibody::MultibodyPlant<double> plant(0.0);
  addCassieMultibody(&plant, nullptr, FLAGS_floating_base /*floating base*/,
                     "examples/Cassie/urdf/cassie_v2.urdf",
                     true /*spring model*/, false /*loop closure*/);
  plant.Finalize();

  std::cout << "channel_1: " << FLAGS_control_channel_name_1 << std::endl;
  std::cout << "channel_2: " << FLAGS_control_channel_name_2 << std::endl;
  std::cout << "channel_3: " << FLAGS_control_channel_name_3 << std::endl;

  // Channel name of the input switch
  std::string switch_channel = "INPUT_SWITCH";

  // Create LCM receiver for commands
  auto command_receiver = builder.AddSystem<RobotInputReceiver>(plant);

  // Create state estimate receiver, used for safety checks
  auto state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(
          FLAGS_state_channel_name, &lcm_local));
  auto controller_switch_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_controller_switch>(switch_channel,
                                                                 &lcm_local));
  auto cassie_out_receiver =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_cassie_out>(
          FLAGS_cassie_out_channel, &lcm_local));
  auto controller_error_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_controller_error>(
          "CONTROLLER_ERROR", &lcm_local));
  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);
  auto input_supervisor_status_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_input_supervisor_status>(
          "INPUT_SUPERVISOR_STATUS", &lcm_local, {TriggerType::kForced}));
  builder.Connect(*state_sub, *state_receiver);

  double input_supervisor_update_period = 1.0 / 1000.0;
  double input_limit = FLAGS_input_limit;
  if (input_limit < 0) {
    input_limit = std::numeric_limits<double>::max();
  }

  auto input_supervisor = builder.AddSystem<InputSupervisor>(
      plant, FLAGS_control_channel_name_1, FLAGS_max_joint_velocity,
      input_supervisor_update_period, FLAGS_supervisor_N, input_limit);
  builder.Connect(state_receiver->get_output_port(0),
                  input_supervisor->get_input_port_state());
  builder.Connect(command_receiver->get_output_port(0),
                  input_supervisor->get_input_port_command());
  builder.Connect(controller_switch_sub->get_output_port(),
                  input_supervisor->get_input_port_controller_switch());
  builder.Connect(controller_error_sub->get_output_port(),
                  input_supervisor->get_input_port_controller_error());
  builder.Connect(input_supervisor->get_output_port_status(),
                  input_supervisor_status_pub->get_input_port());
  builder.Connect(cassie_out_receiver->get_output_port(),
                  input_supervisor->get_input_port_cassie());

  // Create and connect translator
  auto input_translator =
      builder.AddSystem<systems::CassieInputTranslator>(plant);
  builder.Connect(input_supervisor->get_output_port_command(),
                  input_translator->get_input_port(0));

  // Create and connect input publisher.
  auto input_pub = builder.AddSystem(systems::CassieUDPPublisher::Make(
      FLAGS_address, FLAGS_port, {TriggerType::kForced}));
  builder.Connect(*input_translator, *input_pub);

  // Create and connect LCM command echo to network
  auto net_command_sender = builder.AddSystem<RobotCommandSender>(plant);

  LcmPublisherSystem* command_pub;
  if (FLAGS_sim) {
    command_pub =
        builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
            "CASSIE_INPUT", &lcm_local, {TriggerType::kForced}));
  } else {
    command_pub =
        builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
            "NETWORK_CASSIE_INPUT", &lcm_network, {TriggerType::kPeriodic},
            FLAGS_pub_rate));
  }

  builder.Connect(input_supervisor->get_output_port_command(),
                  net_command_sender->get_input_port(0));

  builder.Connect(*net_command_sender, *command_pub);

  // Finish building the diagram
  auto owned_diagram = builder.Build();
  owned_diagram->set_name("dispatcher_robot_in");

  // Channel names of the controllers
  std::vector<std::string> input_channels;
  input_channels.push_back(FLAGS_control_channel_name_1);
  input_channels.push_back(FLAGS_control_channel_name_2);
  input_channels.push_back(FLAGS_control_channel_name_3);

  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_input,
                         dairlib::lcmt_controller_switch>
      loop(&lcm_local, std::move(owned_diagram), command_receiver,
           input_channels, FLAGS_control_channel_name_1, switch_channel, true);
  loop.Simulate();

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }
