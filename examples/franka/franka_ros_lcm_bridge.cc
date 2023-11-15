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

#include "common/find_resource.h"
#include "examples/franka/parameters/franka_lcm_channels.h"
#include "examples/franka/parameters/franka_sim_params.h"
#include "franka_msgs/FrankaState.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "systems/robot_lcm_systems.h"
#include "systems/ros/franka_ros_lcm_conversions.h"
#include "systems/ros/parameters/franka_ros_channels.h"
#include "systems/ros/ros_publisher_system.h"
#include "systems/ros/ros_subscriber_system.h"
#include "systems/system_utils.h"

using drake::math::RigidTransform;
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
              "examples/franka/parameters/lcm_channels_hardware.yaml",
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

  ros::init(argc, argv, "run_ros_to_lcm");
  ros::NodeHandle node_handle;

  drake::lcm::DrakeLcm drake_lcm("udpm://239.255.76.67:7667?ttl=0");
  DiagramBuilder<double> builder;

  MultibodyPlant<double> plant(0.0);

  Parser parser(&plant);
  auto franka_index =
      parser.AddModels(drake::FindResourceOrThrow(sim_params.franka_model))[0];
  auto tray_index =
      parser.AddModels(FindResourceOrThrow(sim_params.tray_model))[0];
  Eigen::Vector3d franka_origin = Eigen::VectorXd::Zero(3);
  RigidTransform<double> R_X_W = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), franka_origin);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"),
                   R_X_W);
  plant.Finalize();

  /// convert franka joint states to lcm
  auto franka_joint_state_ros_subscriber =
      builder.AddSystem(RosSubscriberSystem<sensor_msgs::JointState>::Make(
          ros_channel_params.franka_state_channel, &node_handle));
  auto ros_to_lcm_robot_state = builder.AddSystem(RosToLcmRobotState::Make(
      plant.num_positions(franka_index), plant.num_velocities(franka_index),
      plant.num_actuators(franka_index)));

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
  auto tray_object_state_ros_subscriber =
      builder.AddSystem(RosSubscriberSystem<nav_msgs::Odometry>::Make(
          ros_channel_params.tray_state_channel, &node_handle));
  auto ros_to_lcm_object_state = builder.AddSystem(
      RosToLcmObjectState::Make(plant, tray_index, "tray"));

  // change this to output correctly (i.e. when ros subscriber gets new message)
  auto tray_state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.tray_state_channel, &drake_lcm,
          {drake::systems::TriggerType::kForced}));
  /// connections
  builder.Connect(tray_object_state_ros_subscriber->get_output_port(),
                  ros_to_lcm_object_state->get_input_port());
  builder.Connect(ros_to_lcm_object_state->get_output_port(),
                  tray_state_pub->get_input_port());

  /* -------------------------------------------------------------------------------------------*/

  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("franka_ros_lcm_bridge"));
  const auto& diagram = *owned_diagram;
  DrawAndSaveDiagramGraph(diagram);
  drake::systems::Simulator<double> simulator(std::move(owned_diagram));
  auto& diagram_context = simulator.get_mutable_context();

  // figure out what the arguments to this mean
  ros::AsyncSpinner spinner(1);
  spinner.start();
  signal(SIGINT, SigintHandler);

  // Wait for the first message.
  drake::log()->info("Waiting for first ROS message from Franka");
  int old_message_count = 0;
  old_message_count =
      franka_joint_state_ros_subscriber->WaitForMessage(old_message_count);

  // Initialize the context based on the first message.
  const double t0 = franka_joint_state_ros_subscriber->message_time();
  diagram_context.SetTime(t0);
  simulator.Initialize();

  drake::log()->info("franka ros-lcm bridge started");

  while (true) {
    old_message_count =
        franka_joint_state_ros_subscriber->WaitForMessage(old_message_count);
    const double time = franka_joint_state_ros_subscriber->message_time();

    // Check if we are very far ahead or behind
    // (likely due to a restart of the driving clock)
    if (time > simulator.get_context().get_time() + 1.0 ||
        time < simulator.get_context().get_time()) {
      std::cout << "Dispatcher time is " << simulator.get_context().get_time()
                << ", but stepping to " << time << std::endl;
      std::cout << "Difference is too large, resetting dispatcher time."
                << std::endl;
      simulator.get_mutable_context().SetTime(time);
      simulator.Initialize();
    }

    simulator.AdvanceTo(time);

    // Force-publish via the diagram
    diagram.CalcForcedUnrestrictedUpdate(diagram_context,
                                         &diagram_context.get_mutable_state());
    diagram.CalcForcedDiscreteVariableUpdate(
        diagram_context, &diagram_context.get_mutable_discrete_state());
    diagram.ForcedPublish(diagram_context);
  }
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv); }
