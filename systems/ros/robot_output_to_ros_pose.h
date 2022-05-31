#pragma once

// ROS includes
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

// Drake includes
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"

// Dairlib includes
#include "systems/framework/output_vector.h"

namespace dairlib {
namespace systems {

class RobotOutputToRosPose : public drake::systems::LeafSystem<double> {
 public:
  RobotOutputToRosPose(const drake::multibody::MultibodyPlant<double> &plant);
 private:
  void CopyPose(const drake::systems::Context<double> &context,
                drake::Value<geometry_msgs::Pose> *pose) const;
  const drake::multibody::MultibodyPlant<double> &plant_;
};

}
}

