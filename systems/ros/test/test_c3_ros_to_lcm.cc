#include <memory>
#include <signal.h>
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include <drake/lcm/drake_lcm.h>
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "franka_msgs/FrankaState.h"

#include "systems/ros/ros_subscriber_system.h"
#include "systems/ros/ros_publisher_system.h"
#include "systems/ros/c3_ros_conversions.h"

using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

using dairlib::systems::RosSubscriberSystem;
using dairlib::systems::RosPublisherSystem;
using dairlib::systems::ROSToRobotOutputLCM;
using dairlib::systems::ROSToC3LCM;
using dairlib::systems::ROSToFrankaStateLCM;
using dairlib::systems::ROSToBallPositionLCM;



// Shutdown ROS gracefully and then exit
void SigintHandler(int sig) {
  ros::shutdown();
  exit(sig);
}

int DoMain(ros::NodeHandle& node_handle) {
  DiagramBuilder<double> builder;
  drake::lcm::DrakeLcm drake_lcm;

  /// TEST ROSToRobotOutputLCM
  auto robot_output_subscriber = 
    builder.AddSystem(RosSubscriberSystem<sensor_msgs::JointState>::Make(
        "/franka_state_controller/joint_states", &node_handle));
  auto to_robot_output = builder.AddSystem(ROSToRobotOutputLCM::Make(14, 13, 7));
  builder.Connect(*robot_output_subscriber, *to_robot_output);
  auto robot_output_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
        "LCM_ROBOT_OUTPUT_TEST", &drake_lcm, 
        {drake::systems::TriggerType::kPeriodic}, 0.25));
  builder.Connect(to_robot_output->get_output_port(0), 
    robot_output_pub->get_input_port(0));
  
  /// TEST ROSToC3LCM
  // if test_c3_drake_to_ros is also running, then this system subscribes
  // to a rostopic that outputs 0, 1, 2, ..., 9
  auto c3_subscriber =
      builder.AddSystem(RosSubscriberSystem<std_msgs::Float64MultiArray>::Make(
          "chatter", &node_handle));
  auto to_c3 = builder.AddSystem(ROSToC3LCM::Make(10,0,0,0));
  builder.Connect(*c3_subscriber, *to_c3);
  auto c3_pub = builder.AddSystem(
  LcmPublisherSystem::Make<dairlib::lcmt_c3>(
    "LCM_C3_TEST", &drake_lcm, 
    {drake::systems::TriggerType::kPeriodic}, 0.25));
  builder.Connect(to_c3->get_output_port(0), 
    c3_pub->get_input_port(0));

  /// TEST ROSToFrankaStateLCM
  // auto franka_state_subscriber = 
  //   builder.AddSystem(RosSubscriberSystem<franka_msgs::FrankaState>::Make(
  //       "/franka_state_controller/franka_states", &node_handle));
  // auto to_franka_state = builder.AddSystem(ROSToFrankaStateLCM::Make());
  // builder.Connect(*franka_state_subscriber, *to_franka_state);
  // auto franka_state_pub = builder.AddSystem(
  //     LcmPublisherSystem::Make<dairlib::lcmt_franka_state>(
  //       "LCM_FRANKA_STATE_TEST", &drake_lcm, 
  //       {drake::systems::TriggerType::kPeriodic}, 0.25));
  // builder.Connect(to_franka_state->get_output_port(0), 
  //   franka_state_pub->get_input_port(0));

  /// TEST ROSToBallPositionLCM
  auto ball_position_subscriber = 
    builder.AddSystem(RosSubscriberSystem<std_msgs::Float64MultiArray>::Make(
        "/c3/position_estimate", &node_handle));
  auto to_ball_position = builder.AddSystem(ROSToBallPositionLCM::Make());
  builder.Connect(*ball_position_subscriber, *to_ball_position);
  auto ball_position_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_ball_position>(
        "LCM_BALL_POSITION_TEST", &drake_lcm, 
        {drake::systems::TriggerType::kPeriodic}, 0.25));
  builder.Connect(to_ball_position->get_output_port(0), 
    ball_position_pub->get_input_port(0));


  // TODO: msg_subsciber doesn't produce an output until a new message
  // appears on the ROS channel which means to_c3 and to_robot_output
  // do no receive meaningful outputs until the ROS topic gets updated
  // Before the first ROS topic update, these classes send some default
  // value, but we should probably find a better way to deal with this.

  auto sys = builder.Build();
  Simulator<double> simulator(*sys);
  simulator.Initialize();
  simulator.set_target_realtime_rate(1.0);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  signal(SIGINT, SigintHandler);

  simulator.AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "test_ros_subscriber_system");
  ros::NodeHandle node_handle;

  return DoMain(node_handle);
}
