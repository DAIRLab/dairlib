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

#include "examples/jacktoy/parameters/franka_lcm_channels.h"
#include "examples/jacktoy/parameters/franka_sim_params.h"
#include "franka_msgs/FrankaState.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "systems/framework/lcm_driven_loop.h"
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

// Shutdown ROS gracefully and then exit
void SigintHandler(int sig) {
  ros::shutdown();
  exit(sig);
}

DEFINE_string(lcm_channels,
              "examples/jacktoy/parameters/lcm_channels_hardware.yaml",
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
      "examples/jacktoy/parameters/franka_sim_params.yaml");

  ros::init(argc, argv, "run_lcm_to_ros");
  ros::NodeHandle node_handle;

  drake::lcm::DrakeLcm drake_lcm("udpm://239.255.76.67:7667?ttl=0");
  DiagramBuilder<double> builder;

  MultibodyPlant<double> plant(0.0);

  Parser parser(&plant);
  parser.AddModels(drake::FindResourceOrThrow(sim_params.franka_model))[0];
  Eigen::Vector3d franka_origin = Eigen::VectorXd::Zero(3);
  RigidTransform<double> R_X_W = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), franka_origin);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"),
                   R_X_W);
  plant.Finalize();

  /* -------------------------------------------------------------------------------------------*/
//  auto robot_input_lcm_subscriber =
//      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_input>(
//          lcm_channel_params.franka_input_channel, &drake_lcm));
//  auto robot_input_lcm_echo =
//      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
//          lcm_channel_params.franka_input_echo, &drake_lcm,
//          {drake::systems::TriggerType::kForced}));
  auto robot_input_receiver = builder.AddSystem<RobotInputReceiver>(plant);
  auto lcm_to_ros_robot_input = builder.AddSystem<LcmToRosTimestampedVector>(7);
  auto robot_input_ros_publisher = builder.AddSystem(
      systems::RosPublisherSystem<std_msgs::Float64MultiArray>::Make(
          ros_channel_params.franka_input_channel, &node_handle,
          {drake::systems::TriggerType::kForced}));

//  builder.Connect(robot_input_lcm_subscriber->get_output_port(),
//                  robot_input_receiver->get_input_port());
//  builder.Connect(robot_input_lcm_subscriber->get_output_port(),
//                  robot_input_lcm_echo->get_input_port());
  builder.Connect(robot_input_receiver->get_output_port(),
                  lcm_to_ros_robot_input->get_input_port());
  builder.Connect(lcm_to_ros_robot_input->get_output_port(),
                  robot_input_ros_publisher->get_input_port());

  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("franka_lcm_ros_bridge"));
  const auto& diagram = *owned_diagram;
  DrawAndSaveDiagramGraph(diagram);
//  drake::systems::Simulator<double> simulator(std::move(owned_diagram));
//  auto& diagram_context = simulator.get_mutable_context();

  // figure out what the arguments to this mean
  ros::AsyncSpinner spinner(1);
  spinner.start();
  signal(SIGINT, SigintHandler);

  systems::LcmDrivenLoop<dairlib::lcmt_robot_input> loop(
      &drake_lcm, std::move(owned_diagram), robot_input_receiver,
      lcm_channel_params.franka_input_channel, true);
  DrawAndSaveDiagramGraph(*loop.get_diagram());
  loop.Simulate();

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv); }