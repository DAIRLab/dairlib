#include "rot_space_tracking_data.h"

#include <iostream>

using Eigen::Isometry3d;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;

using std::string;
using std::vector;

using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;

namespace dairlib::systems::controllers {

/**** RotTaskSpaceTrackingData ****/
RotTaskSpaceTrackingData::RotTaskSpaceTrackingData(
    const string& name, const MatrixXd& K_p, const MatrixXd& K_d,
    const MatrixXd& W, const MultibodyPlant<double>& plant)
    : OptionsTrackingData(name, kQuaternionDim, kSpaceDim, K_p, K_d, W, plant) {
  is_rotational_tracking_data_ = true;
}

void RotTaskSpaceTrackingData::AddFrameToTrack(
    const std::string& body_name, const Eigen::Isometry3d& frame_pose) {
  AddStateAndFrameToTrack(-1, body_name, frame_pose);
}
void RotTaskSpaceTrackingData::AddStateAndFrameToTrack(
    int fsm_state, const std::string& body_name,
    const Eigen::Isometry3d& frame_pose) {
  AddFiniteStateToTrack(fsm_state);
  DRAKE_DEMAND(plant_.HasBodyNamed(body_name));
  DRAKE_DEMAND(plant_.HasBodyNamed(body_name));
  body_frames_[fsm_state] =
      &plant_.GetBodyByName(body_name).body_frame();
  frame_poses_[fsm_state] = frame_pose;
}

void RotTaskSpaceTrackingData::UpdateY(const VectorXd& x,
                                       const Context<double>& context) {
  auto transform_mat = plant_.CalcRelativeTransform(
      context, plant_.world_frame(),
      *body_frames_[fsm_state_]);
  Quaterniond y_quat(transform_mat.rotation() *
                     frame_poses_[fsm_state_].linear());
  Eigen::Vector4d y_4d;
  y_4d << y_quat.w(), y_quat.vec();
  y_ = y_4d;
}

void RotTaskSpaceTrackingData::UpdateYError() {
  DRAKE_DEMAND(y_des_.size() == 4);
  Quaterniond y_quat_des(y_des_(0), y_des_(1), y_des_(2), y_des_(3));
  Quaterniond y_quat(y_(0), y_(1), y_(2), y_(3));

  Eigen::AngleAxis<double> angle_axis_diff(y_quat_des * y_quat.inverse());
  error_y_ = angle_axis_diff.angle() * angle_axis_diff.axis();
  if (with_view_frame_) {
    error_y_ = view_frame_rot_T_ * error_y_;
  }
}

void RotTaskSpaceTrackingData::UpdateYdot(
    const VectorXd& x, const Context<double>& context) {
  MatrixXd J_spatial(6, plant_.num_velocities());
  plant_.CalcJacobianSpatialVelocity(
      context, JacobianWrtVariable::kV, *body_frames_[fsm_state_],
      frame_poses_[fsm_state_].translation(), world_, world_,
      &J_spatial);
  ydot_ = J_spatial.block(0, 0, kSpaceDim, J_spatial.cols()) *
          x.tail(plant_.num_velocities());
}

void RotTaskSpaceTrackingData::UpdateYdotError(const Eigen::VectorXd& v_proj) {
  // Transform qdot to w
  Quaterniond y_quat_des(y_des_(0), y_des_(1), y_des_(2), y_des_(3));
  Quaterniond dy_quat_des(ydot_des_(0), ydot_des_(1), ydot_des_(2),
                          ydot_des_(3));
  Vector3d w_des_ = 2 * (dy_quat_des * y_quat_des.conjugate()).vec();
  // Because we transform the error here rather than in the parent
  // options_tracking_data, and because J_y is already transformed in the view
  // frame, we need to undo the transformation on J_y
  error_ydot_ =
      w_des_ - ydot_ - view_frame_rot_T_.transpose() * GetJ() * v_proj;
  if (with_view_frame_) {
    error_ydot_ = view_frame_rot_T_ * error_ydot_;
  }

  ydot_des_ =
      w_des_;  // Overwrite 4d quat_dot with 3d omega. Need this for osc logging
}

void RotTaskSpaceTrackingData::UpdateJ(const VectorXd& x,
                                       const Context<double>& context) {
  MatrixXd J_spatial(6, plant_.num_velocities());
  plant_.CalcJacobianSpatialVelocity(
      context, JacobianWrtVariable::kV, *body_frames_[fsm_state_],
      frame_poses_[fsm_state_].translation(), world_, world_,
      &J_spatial);
  J_ = J_spatial.block(0, 0, kSpaceDim, J_spatial.cols());
}

void RotTaskSpaceTrackingData::UpdateJdotV(
    const VectorXd& x, const Context<double>& context) {
  JdotV_ =
      plant_
          .CalcBiasSpatialAcceleration(context, JacobianWrtVariable::kV,
                                       *body_frames_[fsm_state_],
                                       frame_poses_[fsm_state_].translation(),
                                       world_, world_)
          .rotational();
}

void RotTaskSpaceTrackingData::UpdateYddotDes(double, double) {
  // Convert ddq into angular acceleration
  // See https://physics.stackexchange.com/q/460311
  Quaterniond y_quat_des(y_des_(0), y_des_(1), y_des_(2), y_des_(3));
  Quaterniond yddot_quat_des(yddot_des_(0), yddot_des_(1), yddot_des_(2),
                             yddot_des_(3));
  yddot_des_converted_ = 2 * (yddot_quat_des * y_quat_des.conjugate()).vec();
  if (!idx_zero_feedforward_accel_.empty()) {
    std::cerr << "RotTaskSpaceTrackingData does not support zero feedforward "
                 "acceleration";
  }
  if (ff_accel_multiplier_traj_ != nullptr) {
    std::cerr
        << "RotTaskSpaceTrackingData does not support feedforward multipliers ";
  }
}

void RotTaskSpaceTrackingData::CheckDerivedOscTrackingData() {}
}  // namespace dairlib::systems::controllers
