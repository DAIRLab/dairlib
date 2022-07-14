#include <memory>
#include <signal.h>
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

#include "systems/ros/ros_subscriber_system.h"
#include "systems/ros/ros_publisher_system.h"
#include "systems/ros/c3_ros_conversions.h"

using drake::systems::DiagramBuilder;
using drake::systems::Simulator;

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
  // to a rostopic that outputs 0, 1, 2, ..., 9, timestampe: 0.0
  auto msg_subscriber =
      builder.AddSystem(RosSubscriberSystem<std_msgs::Float64MultiArray>::Make(
          "chatter", &node_handle));
  auto to_robot_output = builder.AddSystem(ROSToRobotOutputLCM::Make(4, 3, 3));
  auto to_c3 = builder.AddSystem(ROSToC3LCM::Make(10,0,0,0));

  builder.Connect(*msg_subscriber, *to_robot_output);
  builder.Connect(*msg_subscriber, *to_c3);

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
