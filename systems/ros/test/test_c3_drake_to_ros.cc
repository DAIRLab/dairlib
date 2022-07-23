#include <memory>
#include <signal.h>
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include <drake/systems/primitives/constant_vector_source.h>
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

#include "systems/ros/ros_publisher_system.h"
#include "systems/ros/c3_ros_conversions.h"

using drake::Value;
using drake::systems::ConstantVectorSource;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;

using dairlib::systems::RosPublisherSystem;
using dairlib::systems::TimestampedVectorToROS;
using dairlib::systems::TimestampedVector;

// class for outputing constant timestamped vector
// Note: this class has been hard coded for this test file
class ConstantTimestampedVectorSource : public drake::systems::LeafSystem<double> {
public:
  static std::unique_ptr<ConstantTimestampedVectorSource> Make() {
    return std::make_unique<ConstantTimestampedVectorSource>();
  }

  ConstantTimestampedVectorSource(){
    this->DeclareVectorOutputPort("u, t",
                                  TimestampedVector<double>(10),
                                  &ConstantTimestampedVectorSource::Output);
  }

  void Output(const drake::systems::Context<double>& context, TimestampedVector<double>* output) const {
    Eigen::VectorXd data(10);
    for (int i = 0; i < 10; i++){
      data(i) = 0.5*i;
    }
    output->set_timestamp(0.0);
    output->SetDataVector(data);
  }
};

// Shutdown ROS gracefully and then exit
void SigintHandler(int sig) {
  ros::shutdown();
  exit(sig);
}

int DoMain(ros::NodeHandle& node_handle) {
  DiagramBuilder<double> builder;

  auto drake_to_ros = builder.AddSystem(TimestampedVectorToROS::Make(10));
  auto msg_publisher = builder.AddSystem(
      RosPublisherSystem<std_msgs::Float64MultiArray>::Make("chatter", &node_handle, .25));

  auto msg_source =
      builder.AddSystem(ConstantTimestampedVectorSource::Make());

  builder.Connect(msg_source->get_output_port(0),
                  drake_to_ros->get_input_port(0));
  builder.Connect(drake_to_ros->get_output_port(0),
                  msg_publisher->get_input_port(0));

  auto sys = builder.Build();
  Simulator<double> simulator(*sys);

  simulator.Initialize();
  simulator.set_target_realtime_rate(1.0);

  signal(SIGINT, SigintHandler);

  simulator.AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "test_ros_publisher_system");
  ros::NodeHandle node_handle;

  return DoMain(node_handle);
}
