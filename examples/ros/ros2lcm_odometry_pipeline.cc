#include <signal.h>
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

#include "systems/ros/ros_subscriber_system.h"
#include "systems/ros/ros2lcm_odometry.h"
#include "dairlib/lcmt_ros_odometry.hpp"

using drake::systems::DiagramBuilder;
using drake::systems::Simulator;

using dairlib::systems::RosOdometry2LcmSender;
using dairlib::systems::RosSubscriberSystem;

// Shutdown ROS gracefully and then exit
void SigintHandler(int sig) {
  ros::shutdown();
  exit(sig);
}

int DoMain(ros::NodeHandle& node_handle, std::string rostopic,
           std::string broadcastChannel) {

    const std::string broadcastChannel_ = broadcastChannel;

    DiagramBuilder<double> builder;
    auto msg_subscriber = builder.AddSystem(RosSubscriberSystem<nav_msgs::Odometry>::Make(
        rostopic, &node_handle));


    auto ros2lcm_convert = builder.AddSystem<RosOdometry2LcmSender>();

    auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();
    double publishRate = 1.0/200.0;
    auto lcm_publisher = builder.AddSystem(
        drake::systems::lcm::LcmPublisherSystem::Make<dairlib::lcmt_ros_odometry>(
            broadcastChannel, lcm, publishRate));

    builder.Connect(*msg_subscriber, *ros2lcm_convert);
    builder.Connect(*ros2lcm_convert, *lcm_publisher);

    auto system = builder.Build();
    Simulator<double> simulator(*system);

    simulator.Initialize();
    simulator.set_target_realtime_rate(1.0);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    signal(SIGINT, SigintHandler);

    simulator.StepTo(std::numeric_limits<double>::infinity());

    return 0;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "ros2lcm_pipeline");
    ros::NodeHandle node_handle;

    return DoMain(node_handle, "chatter", "lcmchatter");
}
