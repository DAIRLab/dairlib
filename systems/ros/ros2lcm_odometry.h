#include "drake/lcm/drake_lcm.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/basic_vector.h"

#include "dairlib/lcmt_ros_odometry.hpp"

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

using drake::systems::LeafSystem;
using drake::systems::Context;
using drake::systems::InputPort;
using drake::systems::OutputPort;

namespace dairlib {
namespace systems {

// Converts ROS 'nav_msgs/Odometry' msgs to LCM lcmt_ros_odometry msgs.

class RosOdometry2LcmSender : public LeafSystem<double> {
 public:
  RosOdometry2LcmSender();

  const InputPort<double>& get_rosmsg_input_port() const;
  const OutputPort<double>& get_lcmmsg_output_port() const;

 private:
  void CalcLcmFromRos(const Context<double> &context,
                      lcmt_ros_odometry* output) const;

  int rosmsg_input_port_;
  int lcm_output_port_;
  const std::string topic_;
  const std::string channel_;


};

}
}
