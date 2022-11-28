#pragma once

// ROS includes
#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

// Drake includes
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"

// Dairlib includes
#include "systems/framework/output_vector.h"

namespace dairlib {
namespace systems {

/// Class to convert a robot state, as given by Drake, to the pose of
/// a given Link of the robot
class RobotStateToRosPose : public drake::systems::LeafSystem<double> {
 public:
  RobotStateToRosPose(const drake::multibody::MultibodyPlant<double> &plant,
                      drake::systems::Context<double>* context,
                      const std::string& body);
 private:
  void CopyPose(const drake::systems::Context<double> &context,
                geometry_msgs::PoseWithCovarianceStamped *pose) const;
  const std::string body_name_;
  const drake::multibody::MultibodyPlant<double> &plant_;
  drake::systems::Context<double>* context_;
};

}
}

