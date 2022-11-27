#include "robot_output_to_ros_pose.h"

#include "multibody/multibody_utils.h"

using dairlib::systems::OutputVector;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using geometry_msgs::PoseWithCovarianceStamped;
using Eigen::VectorXd;
using Eigen::MatrixXd;

namespace dairlib {
namespace systems {

RobotOutputToRosPose::RobotOutputToRosPose(const MultibodyPlant<double>& plant,
                                           Context<double>* context,
                                           const std::string& body_name)
  : plant_(plant), context_(context), body_name_(body_name) {

  this->DeclareVectorInputPort("x, u, t",
      OutputVector<double>(plant_.num_positions(),
          plant.num_velocities(), plant.num_actuators()));

  PoseWithCovarianceStamped pose;
  this->DeclareAbstractOutputPort("ros_pose", pose,
      &RobotOutputToRosPose::CopyPose);
}

void RobotOutputToRosPose::CopyPose(
    const Context<double> &context, PoseWithCovarianceStamped *pose) const {

  const auto robot_output = dynamic_cast<const OutputVector<double>*>(
      this->EvalVectorInput(context, 0));
  const VectorXd state = robot_output->GetState();
  multibody::SetPositionsAndVelocitiesIfNew<double>(plant_, state, context_);

  const MatrixXd cov = MatrixXd::Zero(6,6);
  const auto X_WB = plant_.GetBodyByName(body_name_).EvalPoseInWorld(*context_);

  const Eigen::Quaternion<double>& orientation = X_WB.rotation().ToQuaternion();
  const Eigen::Vector3d& position = X_WB.translation();

  pose->header.frame_id = body_name_;
  pose->pose.pose.position.x = position.x();
  pose->pose.pose.position.y = position.y();
  pose->pose.pose.position.z = position.z();
  pose->pose.pose.orientation.x = orientation.x();
  pose->pose.pose.orientation.y = orientation.y();
  pose->pose.pose.orientation.z = orientation.z();
  pose->pose.pose.orientation.w = orientation.w();
}

}
}