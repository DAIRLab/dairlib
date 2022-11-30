#pragma once

// ROS includes
#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf2_eigen/tf2_eigen.h"

// Drake includes
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib {
namespace systems {

/// Class to convert a robot state, as given by Drake, to the pose of
/// a given Link of the robot
class RobotStateToRosPose : public drake::systems::LeafSystem<double> {
 public:
  RobotStateToRosPose(const drake::multibody::MultibodyPlant<double> &plant,
                      drake::systems::Context<double>* context,
                      const std::string& body);

  const drake::systems::InputPort<double>& get_input_port_state() {
    return this->get_input_port(state_input_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_covariance() {
    return this->get_input_port(covariance_input_port_);
  }
 private:

  drake::systems::InputPortIndex covariance_input_port_;
  drake::systems::InputPortIndex state_input_port_;

  void CopyPose(const drake::systems::Context<double> &context,
                geometry_msgs::PoseWithCovarianceStamped *pose) const;
  const std::string body_name_;
  const drake::multibody::MultibodyPlant<double> &plant_;
  drake::systems::Context<double>* context_;
};

}
}

