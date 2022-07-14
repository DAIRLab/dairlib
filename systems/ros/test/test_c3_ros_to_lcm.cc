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
#include "systems/system_utils.h"

using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

using dairlib::systems::RosSubscriberSystem;
using dairlib::systems::RosPublisherSystem;
using dairlib::systems::ROSToRobotOutputLCM;
using dairlib::systems::ROSToC3LCM;



// Shutdown ROS gracefully and then exit
void SigintHandler(int sig) {
  ros::shutdown();
  exit(sig);
}

int DoMain(ros::NodeHandle& node_handle) {
  DiagramBuilder<double> builder;

  // if test_c3_drake_to_ros is also running, then this system subscribes
  // to a rostopic that outputs 0, 1, 2, ..., 9
  auto msg_subscriber =
      builder.AddSystem(RosSubscriberSystem<std_msgs::Float64MultiArray>::Make(
          "chatter", &node_handle));
  auto to_robot_output = builder.AddSystem(ROSToRobotOutputLCM::Make(4, 3, 3));
  auto to_c3 = builder.AddSystem(ROSToC3LCM::Make(10,0,0,0));

  builder.Connect(msg_subscriber->get_output_port(), to_robot_output->get_input_port());
  builder.Connect(*msg_subscriber, *to_c3);

  // TODO: msg_subsciber doesn't produce an output until a new message
  // appears on the ROS channel which means to_c3 and to_robot_output
  // do no receive meaningful outputs until the ROS topic gets updated
  // Before the first ROS topic update, these classes return some default
  // value, but should probably find a better way to deal with this
  drake::lcm::DrakeLcm drake_lcm;
  auto robot_output_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
        "LCM_ROBOT_OUTPUT_TEST", &drake_lcm, 
        {drake::systems::TriggerType::kPeriodic}, 0.25));
  builder.Connect(to_robot_output->get_output_port(0), 
    robot_output_pub->get_input_port(0));

  auto c3_pub = builder.AddSystem(
  LcmPublisherSystem::Make<dairlib::lcmt_c3>(
    "LCM_C3_TEST", &drake_lcm, 
    {drake::systems::TriggerType::kPeriodic}, 0.25));
  builder.Connect(to_c3->get_output_port(0), c3_pub->get_input_port(0));

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
