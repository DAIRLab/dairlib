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

  Pose pose;
  this->DeclareAbstractOutputPort("ros_pose", pose,
      &RobotOutputToRosPose::CopyPose);
}

void RobotOutputToRosPose::CopyPose(
    const drake::systems::Context<double> &context,
    geometry_msgs::Pose *pose) const {
  // Copy pose from robot state into pose message here
}

}
}