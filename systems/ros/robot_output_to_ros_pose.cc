#include "robot_output_to_ros_pose.h"

using drake::multibody::MultibodyPlant;
using dairlib::systems::OutputVector;
using geometry_msgs::Pose;

namespace dairlib {
namespace systems {

RobotOutputToRosPose::RobotOutputToRosPose(const MultibodyPlant<double>& plant)
  : plant_(plant) {

  this->DeclareVectorInputPort("x, u, t",
      OutputVector<double>(plant_.num_positions(),
          plant.num_velocities(), plant.num_actuators()));

  this->DeclareAbstractOutputPort("ros_pose",
      &RobotOutputToRosPose::CopyPose);
}

void RobotOutputToRosPose::CopyPose(
    const drake::systems::Context<double> &context,
    drake::Value<geometry_msgs::Pose> *pose) const {

  auto& pose_msg = pose->get_mutable_value();

  // Copy pose from robot state into pose message here
}

}
}