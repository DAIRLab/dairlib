#include <gflags/gflags.h>

#include "dairlib/lcmt_controller_failure.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/networking/cassie_input_translator.h"
#include "examples/Cassie/networking/cassie_udp_publisher.h"
#include "examples/Cassie/systems/input_supervisor.h"
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

DEFINE_double(echo_frequency, 50, "Network LCM pubishing frequency (Hz).");
DEFINE_double(publish_frequency, 2000, "ROS pubishing period (Hz).");
DEFINE_double(max_joint_velocity, 20,
              "Maximum joint velocity before error is triggered");
DEFINE_double(input_limit, 300,
              "Maximum torque limit. Negative values are inf.");
DEFINE_string(state_channel_name, "CASSIE_STATE_DISPATCHER",
              "The name of the lcm channel that sends Cassie's state");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");
  drake::lcm::DrakeLcm lcm_network("udpm://239.255.76.67:7667?ttl=1");

  DiagramBuilder<double> builder;

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
  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);

  auto input_supervisor_status_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_input_supervisor_status>(
          "INPUT_SUPERVISOR_STATUS", &lcm_network, {TriggerType::kPeriodic},
          FLAGS_pub_rate));

  auto input_supervisor = builder.AddSystem<InputSupervisor>(
      plant, FLAGS_control_channel_name_initial, FLAGS_max_joint_velocity,
      input_supervisor_update_period, input_limit, FLAGS_supervisor_N);
  auto net_command_sender = builder.AddSystem<RobotCommandSender>(plant);
  command_pub_network =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          "NETWORK_CASSIE_INPUT", &lcm_network, {TriggerType::kPeriodic},
          FLAGS_pub_rate));
  builder.Connect(*net_command_sender, *command_pub_network);
  auto command_pub_local =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          "CASSIE_INPUT", &lcm_local, {TriggerType::kForced}));

  ros::init(argc, argv, "torque_controller");
  ros::NodeHandle node_handle;
  signal(SIGINT, SigintHandler);
  auto c3_to_ros = builder.AddSystem<systems::TimestampedVectorToROS>(7);
  // try making this kForced
  auto ros_publisher = builder.AddSystem(
      systems::RosPublisherSystem<std_msgs::Float64MultiArray>::Make(
          "/franka_control/torque_in", &node_handle, 1 / publish_frequency));

  builder.Connect(subvector_passthrough->get_output_port(),
                  c3_to_ros->get_input_port());
  builder.Connect(c3_to_ros->get_output_port(),
                  ros_publisher->get_input_port());

  builder.Connect(input_supervisor->get_output_port_command(),
                  net_command_sender->get_input_port(0));
  builder.Connect(*net_command_sender, *command_pub_local);

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

  auto owned_diagram = builder.Build();
  owned_diagram->set_name("franka_dispatcher_in");

  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &drake_lcm, std::move(diagram), state_receiver, FLAGS_channel, true);

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

  loop.Simulate();

  return 0;
}
}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }
