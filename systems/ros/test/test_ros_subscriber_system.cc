#include <memory>
#include <signal.h>
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "systems/ros/ros_subscriber_system.h"
#include "systems/ros/ros_publisher_system.h"

using drake::systems::DiagramBuilder;
using drake::systems::Simulator;

using dairlib::systems::RosSubscriberSystem;
using dairlib::systems::RosPublisherSystem;

// Shutdown ROS gracefully and then exit
void SigintHandler(int sig) {
  ros::shutdown();
  exit(sig);
}

int DoMain(ros::NodeHandle& node_handle) {
  DiagramBuilder<double> builder;

  auto msg_subscriber =
      builder.AddSystem(RosSubscriberSystem<std_msgs::String>::Make(
          "chatter", &node_handle));

  auto msg_publisher = builder.AddSystem(
      RosPublisherSystem<std_msgs::String>::Make("echo", &node_handle, .25));

  builder.Connect(*msg_subscriber, *msg_publisher);

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
