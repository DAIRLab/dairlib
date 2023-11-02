#include <limits>
#include <memory>

#include <gflags/gflags.h>

#include "dairlib/lcmt_controller_failure.hpp"
#include "dairlib/lcmt_controller_switch.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_lcm_driven_loop.h"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/systems/input_supervisor.h"
#include "examples/Cassie/networking/cassie_input_translator.h"
#include "examples/Cassie/networking/cassie_udp_publisher.h"
#include "multibody/multibody_utils.h"
#include "systems/controllers/linear_controller.h"
#include "systems/controllers/pd_config_lcm.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"


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
DEFINE_double(max_joint_velocity, 50,
              "Maximum joint velocity before error is triggered");
DEFINE_double(input_limit, 300,
              "Maximum torque limit. Negative values are inf.");
DEFINE_int64(supervisor_N, 10,
             "Maximum allowed consecutive failures of velocity limit.");
DEFINE_string(state_channel_name, "CASSIE_STATE_DISPATCHER",
              "The name of the lcm channel that sends Cassie's state");
DEFINE_string(control_channel_name_initial, "",
              "The initial LCM channel to listen for motor commands from");
DEFINE_string(control_channel_name_additional, "",
              "An additional LCM channel to listen for motor commands from");
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
  AddCassieMultibody(&plant, nullptr, FLAGS_floating_base /*floating base*/,
                     "examples/Cassie/urdf/cassie_v2_conservative.urdf",
                     true /*spring model*/, false /*loop closure*/);
  plant.Finalize();

  std::cout << "initial channel: " << FLAGS_control_channel_name_initial
            << std::endl;
  std::cout << "additional channel: " << FLAGS_control_channel_name_additional
            << std::endl;

  // Channel name of the input switch
  const std::string switch_channel = "INPUT_SWITCH";
  const std::string channel_config = "PD_CONFIG";

  // Create LCM receiver for commands
  auto command_receiver = builder.AddSystem<RobotInputReceiver>(plant);

  // Safety Controller
  auto safety_controller = builder.AddSystem<systems::LinearController>(
      plant.num_positions(), plant.num_velocities(), plant.num_actuators());
  // Create config receiver.
  auto config_receiver = builder.AddSystem<systems::PDConfigReceiver>(plant);

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
      LcmSubscriberSystem::Make<dairlib::lcmt_controller_failure>(
          "CONTROLLER_ERROR", &lcm_local));
  auto controller_error_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_controller_failure>(
          "CONTROLLER_ERROR", &lcm_local, {TriggerType::kForced}));
  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);
  auto input_supervisor_status_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_input_supervisor_status>(
          "INPUT_SUPERVISOR_STATUS", &lcm_network, {TriggerType::kPeriodic},
          FLAGS_pub_rate));
  builder.Connect(*state_sub, *state_receiver);

  double input_supervisor_update_period = 1.0 / 1000.0;

  // Get input limits
  int nu = plant.num_actuators();
  VectorXd input_limit(nu);
  for (drake::multibody::JointActuatorIndex i(0); i < nu; ++i) {
    input_limit[i] = plant.get_joint_actuator(i).effort_limit();
  }

  auto input_supervisor = builder.AddSystem<InputSupervisor>(
      plant, FLAGS_control_channel_name_initial, FLAGS_max_joint_velocity,
      input_supervisor_update_period, input_limit, FLAGS_supervisor_N);
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
  builder.Connect(state_receiver->get_output_port(0),
                  safety_controller->get_input_port_output());
  builder.Connect(config_receiver->get_output_port(0),
                  safety_controller->get_input_port_config());
  builder.Connect(safety_controller->get_output_port(0),
                  input_supervisor->get_input_port_safety_controller());
  builder.Connect(input_supervisor->get_output_port_failure(),
                  controller_error_pub->get_input_port());

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

  LcmPublisherSystem* command_pub_network;
  if (!FLAGS_sim) {
    command_pub_network =
        builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
            "NETWORK_CASSIE_INPUT", &lcm_network, {TriggerType::kPeriodic},
            FLAGS_pub_rate));
    builder.Connect(*net_command_sender, *command_pub_network);
  }
  auto command_pub_local =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          "CASSIE_INPUT", &lcm_local, {TriggerType::kForced}));

  builder.Connect(input_supervisor->get_output_port_command(),
                  net_command_sender->get_input_port(0));

  builder.Connect(*net_command_sender, *command_pub_local);

  // Finish building the diagram
  auto owned_diagram = builder.Build();
  owned_diagram->set_name("dispatcher_robot_in");

  // Channel names of the controllers
  std::vector<std::string> input_channels = {FLAGS_control_channel_name_initial,
                                             "PD_CONTROL",
                                             "OSC_STANDING",
                                             "OSC_WALKING",
                                             "OSC_JUMPING",
                                             "OSC_RUNNING"};
  if (!FLAGS_control_channel_name_additional.empty()) {
    input_channels.push_back(FLAGS_control_channel_name_additional);
  }

  // Run lcm-driven simulation
  CassieLcmDrivenLoop<dairlib::lcmt_robot_input,
                      dairlib::lcmt_controller_switch>
      loop(&lcm_local, std::move(owned_diagram), command_receiver,
           input_channels, FLAGS_control_channel_name_initial, switch_channel,
           true, FLAGS_state_channel_name);

  auto msg = dairlib::lcmt_pd_config();
  msg.timestamp = 0;
  msg.num_joints = 10;
  msg.joint_names = {"hip_roll_left_motor",  "hip_roll_right_motor",
                     "hip_yaw_left_motor",   "hip_yaw_right_motor",
                     "hip_pitch_left_motor", "hip_pitch_right_motor",
                     "knee_left_motor",      "knee_right_motor",
                     "toe_left_motor",       "toe_right_motor"};
  msg.desired_position = {-0.01, .01, 0, 0, 0.55, 0.55, -1.7, -1.7, -1.8, -1.8};
  msg.desired_velocity = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  msg.kp = {50, 50, 50, 50, 50, 50, 50, 50, 20, 20};
  msg.kd = {5, 5, 5, 5, 5, 5, 5, 5, 5, 5};
  config_receiver->get_input_port().FixValue(
      &(loop.get_diagram()->GetMutableSubsystemContext(
          *config_receiver, &loop.get_diagram_mutable_context())),
      msg);
//  DrawAndSaveDiagramGraph(*loop.get_diagram());
  loop.Simulate();

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }
