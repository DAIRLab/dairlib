#include "robot_state_to_ros_pose.h"

#include "multibody/multibody_utils.h"

using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using geometry_msgs::PoseWithCovarianceStamped;
using Eigen::VectorXd;
using Eigen::MatrixXd;

namespace dairlib {
namespace systems {

RobotStateToRosPose::RobotStateToRosPose(const MultibodyPlant<double>& plant,
                                         Context<double>* context,
                                         const std::string& body_name)
  : plant_(plant), context_(context), body_name_(body_name) {

  state_input_port_ = this->DeclareVectorInputPort(
      "x", plant_.num_positions() + plant.num_velocities())
      .get_index();

  covariance_input_port_ = this->DeclareVectorInputPort("Cov", 36).get_index();

  PoseWithCovarianceStamped pose;
  this->DeclareAbstractOutputPort("ros_pose", pose,
      &RobotStateToRosPose::CopyPose);
}

void RobotStateToRosPose::CopyPose(
    const Context<double> &context, PoseWithCovarianceStamped *pose) const {

  const VectorXd& state =
      this->EvalVectorInput(context, state_input_port_)->get_value();
  const VectorXd& cov =
      this->EvalVectorInput(context, covariance_input_port_)->get_value();

  multibody::SetPositionsAndVelocitiesIfNew<double>(plant_, state, context_);

  const auto X_WB = plant_.GetBodyByName(body_name_).EvalPoseInWorld(*context_);

  pose->header.frame_id = body_name_;
  pose->pose.pose = tf2::toMsg(X_WB.GetAsIsometry3());
  for (int i = 0; i < 36; i++) {
    pose->pose.covariance[i] = cov[i];
  }
}

}
}