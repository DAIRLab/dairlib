#pragma once

#include <string>
#include "ros/ros.h"
// TODO: not to sure what type franka actually publishes
// using this for 
#include "std_msgs/Float64MultiArray.h"

#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "systems/framework/output_vector.h"
#include "systems/framework/timestamped_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"

// TODO: WORK IN PROGRESS

namespace dairlib {
namespace systems {

/// @file This file contains classes that handle conversions
/// between Drake input/output types and ROS for the C3 HW experiments.

/// Receives the output of a TimestampedVector output port, converts it
/// to an array of doubles, and publishes it to a ROS topic.  Note that
/// the output port is a dummy port and simply allows the TimeStamped-
/// Vector to passthrough.
class TimestampedVectorToROS : public drake::systems::LeafSystem<double> {
 public:
  explicit TimestampedVectorToROS(const std::string& topic_name,
          int num_elements, int queue_size=1);

 private:
  // convert the timestamped vector from input port into array of numbers and
  // publish it using the RS node
  void PublishMessage(const drake::systems::Context<double>& context,
                  TimestampedVector<double>* output) const;
  const std::string topic_name_;
  const int num_elements_;
  const int queue_size_;
  int input_port_;

  ros::NodeHandle nh_;
  ros::Publisher publisher_;

};

/// Subscribes to a ROS topic and converts its messages to a Drake
/// OutputVector
// TODO: figure out how to get time accurately
// ROS spin needs to be called outside class (in python interface file)
class ROSToOutputVector : public drake::systems::LeafSystem<double> {
 public:
  // "num" variables should be that of the franka and the ball
  // ex. num_positions should be 14 (7 for franka, 7 for ball)
  explicit ROSToOutputVector(const std::string& topic_name, 
                  int num_positions, int num_velocities, 
                  int num_efforts, int queue_size = 1);

 private:
  void CopyOutput(const drake::systems::Context<double>& context,
                  OutputVector<double>* output) const;
  void ConversionCallback(const std_msgs::Float64MultiArray& msg);
  const std::string topic_name_;
  const int num_positions_;
  const int num_velocities_;
  const int num_efforts_;
  const int queue_size_;
  mutable OutputVector<double> output_;

  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
};


}  // namespace systems
}  // namespace dairlib
