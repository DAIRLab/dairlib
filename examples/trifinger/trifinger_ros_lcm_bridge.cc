#include <signal.h>

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
#include "examples/trifinger/parameters/trifinger_lcm_channels.h"
#include "examples/trifinger/parameters/trifinger_sim_params.h"
#include "nav_msgs/msg/odometry.hpp"
#include "parameters/trifinger_ros2_channels.h"
#include "rclcpp/rclcpp.hpp"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"
#include "systems/trifinger_ros_lcm_msgs_conversions.h"
#include "tools/drake_ros/drake_ros.h"
#include "tools/drake_ros/ros_interface_system.h"
#include "tools/drake_ros/ros_publisher_system.h"
#include "tools/drake_ros/ros_subscriber_system.h"

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
using dairlib::systems::RosToLcmFingertipsDeltaPosition;
using dairlib::systems::RosToLcmObjectState;
using dairlib::systems::RosToLcmRobotState;
using dairlib::systems::SubvectorPassThrough;
using dairlib::systems::TimestampedVector;

DEFINE_string(lcm_channels,
              "examples/trifinger/parameters/lcm_channels_hardware.yaml",
              "Filepath containing lcm channels");
DEFINE_string(ros2_channels,
              "examples/trifinger/parameters/trifinger_ros2_channels"
              ".yaml",
              "Filepath containing ROS2 channels");
DEFINE_string(sim_parameters,
              "examples/trifinger/parameters/trifinger_sim_params.yaml",
              "Filepath to simulation configs");
DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "How many seconds to run the simulation");

namespace dairlib {

using drake_ros::DrakeRos;
using drake_ros::RosInterfaceSystem;
using drake_ros::RosPublisherSystem;
using drake_ros::RosSubscriberSystem;

// Shutdown ROS gracefully and then exit
void SigintHandler(int sig) {
  drake_ros::shutdown();
  exit(sig);
}

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  TrifingerLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<TrifingerLcmChannels>(FLAGS_lcm_channels);
  TrifingerRos2Channels ros_channel_params =
      drake::yaml::LoadYamlFile<TrifingerRos2Channels>(FLAGS_ros2_channels);
  TrifingerSimParams sim_params =
      drake::yaml::LoadYamlFile<TrifingerSimParams>(FLAGS_sim_parameters);

  // How to replace ros init
  drake_ros::init();

  drake::lcm::DrakeLcm drake_lcm("udpm://239.255.76.67:7667?ttl=0");
  DiagramBuilder<double> builder;

  MultibodyPlant<double> plant(0.0);

  Parser parser(&plant);
  auto trifinger_index = parser.AddModels(
      dairlib::FindResourceOrThrow(sim_params.trifinger_model))[0];
  auto cube_index =
      parser.AddModels(FindResourceOrThrow(sim_params.cube_model))[0];
  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link"),
                   X_WI);
  plant.Finalize();

  /// convert trifinger joint states to lcm
  rclcpp::QoS qos{10};
  auto sys_ros_interface = builder.AddSystem<RosInterfaceSystem>(
      std::make_unique<DrakeRos>("run_ros_lcm_node"));
  auto trifinger_joint_state_ros_subscriber = builder.AddSystem(
      RosSubscriberSystem::Make<trifinger_msgs::msg::TrifingerState>(
          ros_channel_params.trifinger_state_channel, qos,
          sys_ros_interface->get_ros_interface()));
  auto ros_to_lcm_robot_state = builder.AddSystem(
      RosToLcmRobotState::Make(plant.num_positions(trifinger_index),
                               plant.num_velocities(trifinger_index),
                               plant.num_actuators(trifinger_index)));
  auto robot_output_lcm_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          lcm_channel_params.trifinger_state_channel, &drake_lcm,
          {drake::systems::TriggerType::kForced}));
  builder.Connect(trifinger_joint_state_ros_subscriber->get_output_port(),
                  ros_to_lcm_robot_state->get_input_port());
  builder.Connect(ros_to_lcm_robot_state->get_output_port(),
                  robot_output_lcm_publisher->get_input_port());

  /// convert fingertips delta position ros msg to lcm
  auto fingertips_delta_position_ros_subscriber = builder.AddSystem(
      RosSubscriberSystem::Make<std_msgs::msg::Float64MultiArray>(
          ros_channel_params.fingertips_delta_position, qos,
          sys_ros_interface->get_ros_interface()));
  auto ros_to_lcm_delta_pos =
      builder.AddSystem(RosToLcmFingertipsDeltaPosition::Make());
  auto delta_pos_lcm_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_fingertips_delta_position>(
          lcm_channel_params.fingertips_delta_position_channel, &drake_lcm,
          {drake::systems::TriggerType::kForced}));
  builder.Connect(fingertips_delta_position_ros_subscriber->get_output_port(0),
                  ros_to_lcm_delta_pos->get_input_port());
  builder.Connect(ros_to_lcm_delta_pos->get_output_port(),
                  delta_pos_lcm_publisher->get_input_port());

  /// convert cube state ros msgs to lcm
  auto cube_object_state_ros_subscriber =
      builder.AddSystem(RosSubscriberSystem::Make<nav_msgs::msg::Odometry>(
          ros_channel_params.cube_state_channel, qos,
          sys_ros_interface->get_ros_interface()));
  auto ros_to_lcm_object_state =
      builder.AddSystem(RosToLcmObjectState::Make(plant, cube_index, "cube"));

  auto cube_state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.cube_state_channel, &drake_lcm,
          {drake::systems::TriggerType::kForced}));
  /// connections
  builder.Connect(cube_object_state_ros_subscriber->get_output_port(),
                  ros_to_lcm_object_state->get_input_port());
  builder.Connect(ros_to_lcm_object_state->get_output_port(),
                  cube_state_pub->get_input_port());

  /* -------------------------------------------------------------------------------------------*/

  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("trifinger_ros_lcm_bridge"));
  const auto& diagram = *owned_diagram;
  DrawAndSaveDiagramGraph(diagram);
  drake::systems::Simulator<double> simulator(std::move(owned_diagram));
  auto& simulator_context = simulator.get_mutable_context();
  int cur_trifinger_state_msg_count = 0;
  int cur_cube_state_msg_count = 0;
  int cur_delta_pos_msg_count = 0;
  simulator.Initialize();

  constexpr double kStep{0.1};
  while (simulator_context.get_time() < FLAGS_simulation_sec) {
    const double next_time =
        std::min(FLAGS_simulation_sec, simulator_context.get_time() + kStep);
    simulator.AdvanceTo(next_time);

    // ros-driven loop
    int actual_trifinger_state_msg_count =
        trifinger_joint_state_ros_subscriber->GetReceivedMessageCount();
    int actual_cube_state_msg_count =
        cube_object_state_ros_subscriber->GetReceivedMessageCount();
    int actual_delta_pos_msg_count =
        fingertips_delta_position_ros_subscriber->GetReceivedMessageCount();
    if ((actual_trifinger_state_msg_count > cur_trifinger_state_msg_count) ||
        (actual_cube_state_msg_count > cur_cube_state_msg_count) ||
        (actual_delta_pos_msg_count > cur_delta_pos_msg_count)) {
      cur_trifinger_state_msg_count = actual_trifinger_state_msg_count;
      cur_cube_state_msg_count = actual_cube_state_msg_count;
      cur_delta_pos_msg_count = actual_delta_pos_msg_count;

      // Force-publish via the diagram
      diagram.CalcForcedUnrestrictedUpdate(
          simulator_context, &simulator_context.get_mutable_state());
      diagram.CalcForcedDiscreteVariableUpdate(
          simulator_context, &simulator_context.get_mutable_discrete_state());
      diagram.ForcedPublish(simulator_context);
    }
  }
  return 0;
}
}  // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv); }
