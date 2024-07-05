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

void RotTaskSpaceTrackingData::UpdateY(
    const VectorXd& x, const Context<double>& context,
    OscTrackingDataState& td_state) const {
  auto transform_mat = plant_.CalcRelativeTransform(
      context, plant_.world_frame(), *body_frames_.at(td_state.fsm_state_));
  Quaterniond y_quat(transform_mat.rotation() *
                     frame_poses_.at(td_state.fsm_state_).linear());
  Eigen::Vector4d y_4d;
  y_4d << y_quat.w(), y_quat.vec();
  td_state.y_ = y_4d;
}

void RotTaskSpaceTrackingData::UpdateYError(OscTrackingDataState& td_state) const {
  DRAKE_DEMAND(td_state.y_des_.size() == 4);
  Quaterniond y_quat_des(td_state.y_des_(0), td_state.y_des_(1), td_state.y_des_(2), td_state.y_des_(3));
  Quaterniond y_quat(td_state.y_(0), td_state.y_(1), td_state.y_(2), td_state.y_(3));

  Eigen::AngleAxis<double> angle_axis_diff(y_quat_des * y_quat.inverse());
  td_state.error_y_ = angle_axis_diff.angle() * angle_axis_diff.axis();
  if (with_view_frame_) {
    td_state.error_y_ = td_state.view_frame_rot_T_ * td_state.error_y_;
  }
}

void RotTaskSpaceTrackingData::UpdateYdot(
    const VectorXd& x, const Context<double>& context, OscTrackingDataState& td_state) const {
  MatrixXd J_spatial(6, plant_.num_velocities());
  plant_.CalcJacobianSpatialVelocity(
      context, JacobianWrtVariable::kV, *body_frames_.at(td_state.fsm_state_),
      frame_poses_.at(td_state.fsm_state_).translation(), world_, world_,
      &J_spatial);
  td_state.ydot_ = J_spatial.block(0, 0, kSpaceDim, J_spatial.cols()) *
          x.tail(plant_.num_velocities());
}

void RotTaskSpaceTrackingData::UpdateYdotError(
    const Eigen::VectorXd& v_proj, OscTrackingDataState& td_state) const {
  DRAKE_DEMAND(td_state.ydot_des_.size() == 3);
  // Because we transform the error here rather than in the parent
  // options_tracking_data, and because J_y is already transformed in the view
  // frame, we need to undo the transformation on J_y
  td_state.error_ydot_ =
      td_state.ydot_des_ - td_state.ydot_ - td_state.view_frame_rot_T_.transpose() * td_state.J_ * v_proj;
  if (with_view_frame_) {
    td_state.error_ydot_ = td_state.view_frame_rot_T_ * td_state.error_ydot_;
  }
}

void RotTaskSpaceTrackingData::UpdateJ(const VectorXd& x,
                                       const Context<double>& context, OscTrackingDataState& td_state) const {
  MatrixXd J_spatial(6, plant_.num_velocities());
  plant_.CalcJacobianSpatialVelocity(
      context, JacobianWrtVariable::kV, *body_frames_.at(td_state.fsm_state_),
      frame_poses_.at(td_state.fsm_state_).translation(), world_, world_,
      &J_spatial);
  td_state.J_ = J_spatial.block(0, 0, kSpaceDim, J_spatial.cols());
}

void RotTaskSpaceTrackingData::UpdateJdotV(
    const VectorXd& x, const Context<double>& context, OscTrackingDataState& td_state) const {
  td_state.JdotV_ = plant_.CalcBiasSpatialAcceleration(
      context, JacobianWrtVariable::kV, *body_frames_.at(td_state.fsm_state_),
      frame_poses_.at(td_state.fsm_state_).translation(), world_, world_
  ).rotational();
}

void RotTaskSpaceTrackingData::UpdateYddotDes(
    double, double, OscTrackingDataState& td_state) const {
  DRAKE_DEMAND(td_state.yddot_des_.size() == 3);
  td_state.yddot_des_converted_ = td_state.yddot_des_;
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
