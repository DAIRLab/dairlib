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
using dairlib::systems::ROSToBallPositionLCM;



// Shutdown ROS gracefully and then exit
void SigintHandler(int sig) {
  ros::shutdown();
  exit(sig);
}

int DoMain(ros::NodeHandle& node_handle) {
  DiagramBuilder<double> builder;
  drake::lcm::DrakeLcm drake_lcm;

  /// subscribe to joint states
  auto robot_output_subscriber = 
    builder.AddSystem(RosSubscriberSystem<sensor_msgs::JointState>::Make(
        "/c3/joint_states", &node_handle));
  auto to_robot_output = builder.AddSystem(ROSToRobotOutputLCM::Make(14, 13, 7));
  builder.Connect(*robot_output_subscriber, *to_robot_output);

  /// subscribe to ball position
  auto ball_position_subscriber = 
    builder.AddSystem(RosSubscriberSystem<std_msgs::Float64MultiArray>::Make(
        "/c3/position_estimate", &node_handle)); // TODO: confirm topic name
  auto to_ball_position = builder.AddSystem(ROSToBallPositionLCM::Make());
  builder.Connect(*ball_position_subsriber, *to_ball_position);

  /// state estimation block
  int p_filter_length = 2;
  int v_filter_length = 10;
  std::vector<double> p_FIR_values = {0.1, 0.9};
  std::vector<double> v_FIR_values(v_filter_length, 1.0 / v_filter_length);

  auto state_estimator = 
    builder.AddSystem<dairlib::systems::C3StateEstimator>(p_FIR_values, v_FIR_values);
  builder.Connect(to_robot_output->output_port(0), state_estimator.input_port(0));
  builder.Connect(to_ball_position->output_port(0), state_estimator.input_port(1));

  auto sender = builder.AddSystem(ROSToRobotOutputLCM::Make(14, 13, 7));  
  auto robot_output_pub = builder.AddSystem(
    LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
      "STATE_ESTIMATE", &drake_lcm, 
      {drake::systems::TriggerType::kPeriodic}, 0.0005));
  builder.Connect(*state_estimator, *sender);
  builder.Connect(*sender, *robot_output_pub);  

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
  ros::init(argc, argv, "test_state_estimation");
  ros::NodeHandle node_handle;

  return DoMain(node_handle);
}
