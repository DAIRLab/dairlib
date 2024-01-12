#include <memory>
#include <signal.h>
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_value_source.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "systems/ros/ros_publisher_system.h"
#include "systems/ros/ros_interface_system.h"

using drake::Value;
using drake::systems::ConstantValueSource;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;

using dairlib::systems::RosPublisherSystem;
using dairlib::systems::RosInterfaceSystem;

int DoMain() {
  DiagramBuilder<double> builder;

  auto ros = builder.AddSystem<RosInterfaceSystem>("test_ros_publisher_system");

  auto msg_publisher = builder.AddSystem(
      RosPublisherSystem<std_msgs::String>::Make(
          "chatter", ros->node_handle(), .25)
      );

  std_msgs::String msg;
  msg.data = "Hello world!";

  auto msg_source =
      builder.AddSystem(std::make_unique<ConstantValueSource<double>>(
          Value<std_msgs::String>(msg)));

  builder.Connect(msg_source->get_output_port(0),
                  msg_publisher->get_input_port(0));

  auto sys = builder.Build();
  Simulator<double> simulator(*sys);

  simulator.Initialize();
  simulator.set_target_realtime_rate(1.0);

  simulator.AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

int main(int argc, char* argv[]) {
  return DoMain();
}
