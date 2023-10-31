#define ROS

#ifdef ROS

#include <signal.h>

#include <memory>
#include <string>

#include <dairlib/lcmt_robot_input.hpp>
#include <drake/common/find_resource.h>
#include <drake/common/yaml/yaml_io.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/lcm/lcm_interface_system.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake/systems/lcm/lcm_subscriber_system.h>
#include <gflags/gflags.h>

#include "examples/franka/parameters/franka_lcm_channels.h"
#include "examples/franka/parameters/franka_sim_params.h"
#include "franka_msgs/FrankaState.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "systems/robot_lcm_systems.h"
#include "systems/ros/franka_ros_lcm_conversions.h"
#include "systems/ros/parameters/franka_ros_channels.h"
#include "systems/ros/ros_publisher_system.h"
#include "systems/ros/ros_subscriber_system.h"
#include "systems/system_utils.h"

using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

using dairlib::systems::LcmToRosTimestampedVector;
using dairlib::systems::RobotInputReceiver;
using dairlib::systems::RobotOutputSender;
using dairlib::systems::RosPublisherSystem;
using dairlib::systems::RosSubscriberSystem;
using dairlib::systems::RosToLcmObjectState;
using dairlib::systems::RosToLcmRobotState;
using dairlib::systems::SubvectorPassThrough;
using dairlib::systems::TimestampedVector;
// using dairlib::systems::ROSToRobotOutputLCM;

// Shutdown ROS gracefully and then exit
void SigintHandler(int sig) {
  ros::shutdown();
  exit(sig);
}

DEFINE_string(lcm_channels,
              "examples/franka/parameters/lcm_channels_simulation.yaml",
              "Filepath containing lcm channels");
DEFINE_string(ros_channels, "systems/ros/parameters/franka_ros_channels.yaml",
              "Filepath containing ROS channels");

namespace dairlib {

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  FrankaLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<FrankaLcmChannels>(FLAGS_lcm_channels);
  FrankaRosChannels ros_channel_params =
      drake::yaml::LoadYamlFile<FrankaRosChannels>(FLAGS_ros_channels);
  FrankaSimParams sim_params = drake::yaml::LoadYamlFile<FrankaSimParams>(
      "examples/franka/parameters/franka_sim_params.yaml");

  ros::init(argc, argv, "run_c3_ros_to_lcm");
  ros::NodeHandle node_handle;

  drake::lcm::DrakeLcm drake_lcm;
  DiagramBuilder<double> builder;

  MultibodyPlant<double> plant(0.0);

  Parser parser(&plant);
  drake::multibody::ModelInstanceIndex franka_index =
      parser.AddModels(drake::FindResourceOrThrow(sim_params.franka_model))[0];

  /* -------------------------------------------------------------------------------------------*/
  /// convert franka joint states to lcm
  auto franka_joint_state_ros_subscriber =
      builder.AddSystem(RosSubscriberSystem<sensor_msgs::JointState>::Make(
          ros_channel_params.franka_state_channel, &node_handle));
  auto ros_to_lcm_robot_state = builder.AddSystem(RosToLcmRobotState::Make(
      plant.num_positions(), plant.num_velocities(), plant.num_actuators()));

  // change this to output correctly (i.e. when ros subscriber gets new message)
  auto robot_output_lcm_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          lcm_channel_params.franka_state_channel, &drake_lcm,
          {drake::systems::TriggerType::kForced}));
  /// connections
  builder.Connect(franka_joint_state_ros_subscriber->get_output_port(),
                  ros_to_lcm_robot_state->get_input_port());
  builder.Connect(ros_to_lcm_robot_state->get_output_port(),
                  robot_output_lcm_publisher->get_input_port());

  /* -------------------------------------------------------------------------------------------*/
  /// convert ball position estimate to lcm
  auto tray_object_state_ros_subscriber =
      builder.AddSystem(RosSubscriberSystem<std_msgs::Float64MultiArray>::Make(
          ros_channel_params.tray_state_channel, &node_handle));
  auto ros_to_lcm_object_state = builder.AddSystem(RosToLcmObjectState::Make());

  // change this to output correctly (i.e. when ros subscriber gets new message)
  auto tray_state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.tray_state_channel, &drake_lcm,
          {drake::systems::TriggerType::kPeriodic}, 0.005));
  /// connections
  builder.Connect(tray_object_state_ros_subscriber->get_output_port(),
                  ros_to_lcm_object_state->get_input_port());
  builder.Connect(ros_to_lcm_object_state->get_output_port(),
                  tray_state_pub->get_input_port());

  /* -------------------------------------------------------------------------------------------*/
  auto robot_input_lcm_subscriber =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_input>(
          lcm_channel_params.osc_channel, &drake_lcm));
  auto robot_input_receiver = builder.AddSystem<RobotInputReceiver>(plant);
  auto robot_input_passthrough = builder.AddSystem<SubvectorPassThrough>(
      robot_input_receiver->get_output_port(0).size(), 0,
      plant.get_actuation_input_port().size());
  auto robot_input_ros_publisher = builder.AddSystem(
      systems::RosPublisherSystem<std_msgs::Float64MultiArray>::Make(
          ros_channel_params.franka_input_channel, &node_handle, .001));
  auto lcm_to_ros_robot_input = builder.AddSystem<LcmToRosTimestampedVector>(7);
  // try making this kForced

  builder.Connect(robot_input_passthrough->get_output_port(),
                  lcm_to_ros_robot_input->get_input_port());
  builder.Connect(lcm_to_ros_robot_input->get_output_port(),
                  robot_input_ros_publisher->get_input_port());

  auto sys = builder.Build();
  // DrawAndSaveDiagramGraph(*sys,
  // "examples/franka_trajectory_following/diagram_run_c3_ros_to_lcm");

  Simulator<double> simulator(*sys);
  simulator.Initialize();
  simulator.set_target_realtime_rate(1.0);

  // figure out what the arguments to this mean
  ros::AsyncSpinner spinner(1);
  spinner.start();
  signal(SIGINT, SigintHandler);
  simulator.AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv); }

#endif