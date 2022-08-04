#define ROS

#ifdef ROS

#include <memory>
#include <signal.h>
#include <string>
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include <drake/lcm/drake_lcm.h>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"

#include "systems/ros/ros_subscriber_system.h"
#include "systems/ros/ros_publisher_system.h"
#include "systems/ros/c3_ros_conversions.h"
#include "systems/system_utils.h"

using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

using dairlib::systems::RosSubscriberSystem;
using dairlib::systems::RosPublisherSystem;
using dairlib::systems::ROSToRobotOutputLCM;
using dairlib::systems::ROSToC3LCM;
using dairlib::systems::ROSToBallPositionLCM;

// Shutdown ROS gracefully and then exit
void SigintHandler(int sig) {
  ros::shutdown();
  exit(sig);
}

namespace dairlib {

int DoMain(int argc, char* argv[]){
  ros::init(argc, argv, "run_c3_ros_to_lcm");
  ros::NodeHandle node_handle;

  drake::lcm::DrakeLcm drake_lcm;

  DiagramBuilder<double> builder;

  /* -------------------------------------------------------------------------------------------*/
  /// convert franka joint states to lcm
  auto franka_subscriber =
      builder.AddSystem(RosSubscriberSystem<sensor_msgs::JointState>::Make(
          "/c3/joint_states", &node_handle));
  auto to_robot_output = builder.AddSystem(ROSToRobotOutputLCM::Make(14, 13, 7));  
  
  // change this to output correctly (i.e. when ros subscriber gets new message)
  auto robot_output_pub = builder.AddSystem(
    LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
      "FRANKA_ROS_OUTPUT", &drake_lcm, 
      {drake::systems::TriggerType::kPeriodic}, 0.0005));
  /// connections
  builder.Connect(franka_subscriber->get_output_port(), to_robot_output->get_input_port());
  builder.Connect(to_robot_output->get_output_port(), robot_output_pub->get_input_port());
  
  /* -------------------------------------------------------------------------------------------*/
  /// convert ball position estimate to lcm
  auto ball_subscriber =
      builder.AddSystem(RosSubscriberSystem<std_msgs::Float64MultiArray>::Make(
          "/c3/position_estimate", &node_handle));
  auto to_ball_position = builder.AddSystem(ROSToBallPositionLCM::Make());  
  
  // change this to output correctly (i.e. when ros subscriber gets new message)
  auto ball_position_pub = builder.AddSystem(
    LcmPublisherSystem::Make<dairlib::lcmt_ball_position>(
      "VISION_OUTPUT", &drake_lcm, 
      {drake::systems::TriggerType::kPeriodic}, 0.01)); //100hz is sufficient
  /// connections
  builder.Connect(ball_subscriber->get_output_port(), to_ball_position->get_input_port());
  builder.Connect(to_ball_position->get_output_port(), ball_position_pub->get_input_port());

  /* -------------------------------------------------------------------------------------------*/
  /// convert individual camera measurements to lcm for logging
  int num_cameras = 3;
  std::vector<RosSubscriberSystem<std_msgs::Float64MultiArray>*>
    cam_subscribers(num_cameras, nullptr);
  std::vector<ROSToBallPositionLCM*> ros_lcm_converters(num_cameras, nullptr);
  std::vector<LcmPublisherSystem*> cam_publishers(num_cameras, nullptr);

  for (int i = 0; i < num_cameras; i++){
    // Add systems
    std::string cam_id = std::to_string(i);
    cam_subscribers[i] = 
      builder.AddSystem(RosSubscriberSystem<std_msgs::Float64MultiArray>::Make(
          "/c3/cam" + cam_id, &node_handle));
    ros_lcm_converters[i] = 
      builder.AddSystem(ROSToBallPositionLCM::Make());
    cam_publishers[i] = 
      builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_ball_position>(
        "CAM" + cam_id + "_OUTPUT", &drake_lcm, 
        {drake::systems::TriggerType::kPeriodic}, 0.01));
    
    // Connect systems
    builder.Connect(cam_subscribers[i]->get_output_port(),
      ros_lcm_converters[i]->get_input_port());
    builder.Connect(ros_lcm_converters[i]->get_output_port(),
      cam_publishers[i]->get_input_port());
  }

  /* -------------------------------------------------------------------------------------------*/

  auto sys = builder.Build();
  // DrawAndSaveDiagramGraph(*sys, "examples/franka_trajectory_following/diagram_run_c3_ros_to_lcm");

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

} // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv);}

#endif