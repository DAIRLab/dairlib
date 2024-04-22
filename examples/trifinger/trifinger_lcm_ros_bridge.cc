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
#include "examples/trifinger/parameters/trifinger_lcm_channels.h"
#include "examples/trifinger/parameters/trifinger_sim_params.h"
#include "nav_msgs/msg/odometry.hpp"
#include "parameters/trifinger_ros2_channels.h"
#include "std_msgs/msg/float64_multi_array.h"
#include "systems/framework/lcm_driven_loop.h"
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
using dairlib::systems::RosToLcmObjectState;
using dairlib::systems::RosToLcmRobotState;
using dairlib::systems::SubvectorPassThrough;
using dairlib::systems::TimestampedVector;
using drake_ros::DrakeRos;
using drake_ros::RosPublisherSystem;
using drake_ros::RosSubscriberSystem;

// Shutdown ROS gracefully and then exit
void SigintHandler(int sig) {
  drake_ros::shutdown();
  exit(sig);
}

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

namespace dairlib {

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
  parser.AddModels(dairlib::FindResourceOrThrow(sim_params.trifinger_model))[0];
  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link"),
                   X_WI);
  plant.Finalize();

  rclcpp::QoS qos{10};
  auto sys_ros_interface = builder.AddSystem<drake_ros::RosInterfaceSystem>(
      std::make_unique<DrakeRos>("run_lcm_ros_node"));
  auto robot_input_receiver = builder.AddSystem<RobotInputReceiver>(plant);
  auto lcm_to_ros_robot_input = builder.AddSystem<LcmToRosTimestampedVector>(9);
  auto robot_input_ros_publisher = builder.AddSystem(
      RosPublisherSystem::Make<std_msgs::msg::Float64MultiArray>(
          ros_channel_params.trifinger_input_channel, qos,
          sys_ros_interface->get_ros_interface()));

  builder.Connect(robot_input_receiver->get_output_port(),
                  lcm_to_ros_robot_input->get_input_port());
  builder.Connect(lcm_to_ros_robot_input->get_output_port(),
                  robot_input_ros_publisher->get_input_port());

  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("trifinger_lcm_ros_bridge"));
  const auto& diagram = *owned_diagram;

  systems::LcmDrivenLoop<dairlib::lcmt_robot_input> loop(
      &drake_lcm, std::move(owned_diagram), robot_input_receiver,
      lcm_channel_params.trifinger_input_channel, true);
  loop.Simulate();

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv); }