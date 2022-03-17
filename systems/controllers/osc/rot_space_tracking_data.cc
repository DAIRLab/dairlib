#include "rot_space_tracking_data.h"

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
    const MatrixXd& W, const MultibodyPlant<double>* plant_w_spr,
    const MultibodyPlant<double>* plant_wo_spr)
    : OptionsTrackingData(name, kQuaternionDim, kSpaceDim, K_p, K_d, W,
                          plant_w_spr, plant_wo_spr) {}


void RotTaskSpaceTrackingData::AddFrameToTrack(
    const std::string& body_name, const Eigen::Isometry3d& frame_pose) {
  AddStateAndFrameToTrack(-1, body_name, frame_pose);
}
void RotTaskSpaceTrackingData::AddStateAndFrameToTrack(
    int fsm_state, const std::string& body_name,
    const Eigen::Isometry3d& frame_pose) {
  AddFiniteStateToTrack(fsm_state);
  //  AddPointToTrack(body_name, pt_on_body);
  DRAKE_DEMAND(plant_w_spr_->HasBodyNamed(body_name));
  DRAKE_DEMAND(plant_wo_spr_->HasBodyNamed(body_name));
  body_frames_w_spr_[fsm_state] =
      &plant_w_spr_->GetBodyByName(body_name).body_frame();
  body_frames_wo_spr_[fsm_state] =
      &plant_wo_spr_->GetBodyByName(body_name).body_frame();
  frame_poses_[fsm_state] = frame_pose;
}



void RotTaskSpaceTrackingData::UpdateY(const VectorXd& x_w_spr,
                                       const Context<double>& context_w_spr) {
  auto transform_mat = plant_w_spr_->CalcRelativeTransform(
      context_w_spr, plant_w_spr_->world_frame(),
      *body_frames_w_spr_[fsm_state_]);
  Quaterniond y_quat(transform_mat.rotation() *
                     frame_poses_[fsm_state_].linear());
  Eigen::Vector4d y_4d;
  y_4d << y_quat.w(), y_quat.vec();
  y_ = y_4d;
}

void RotTaskSpaceTrackingData::UpdateYError() {
  DRAKE_DEMAND(y_des_.size() == 4);
  Quaterniond y_quat_des(y_des_(0), y_des_(1), y_des_(2), y_des_(3));
  y_quat_des.normalize();

  Quaterniond y_quat(y_(0), y_(1), y_(2), y_(3));

  // Get relative quaternion (from current to desired)
  Quaterniond relative_qaut = (y_quat_des * y_quat.inverse()).normalized();
  double theta = 2 * acos(relative_qaut.w());
  Vector3d rot_axis = relative_qaut.vec().normalized();
  error_y_ = theta * rot_axis;
}

void RotTaskSpaceTrackingData::UpdateYdot(
    const VectorXd& x_w_spr, const Context<double>& context_w_spr) {
  MatrixXd J_spatial(6, plant_w_spr_->num_velocities());
  plant_w_spr_->CalcJacobianSpatialVelocity(
      context_w_spr, JacobianWrtVariable::kV, *body_frames_w_spr_[fsm_state_],
      frame_poses_[fsm_state_].translation(), *world_w_spr_, *world_w_spr_,
      &J_spatial);
  ydot_ = J_spatial.block(0, 0, kSpaceDim, J_spatial.cols()) *
          x_w_spr.tail(plant_w_spr_->num_velocities());
}

void RotTaskSpaceTrackingData::UpdateYdotError(const Eigen::VectorXd& v_proj) {
  // Transform qdot to w
  Quaterniond y_quat_des(y_des_(0), y_des_(1), y_des_(2), y_des_(3));
  Quaterniond dy_quat_des(ydot_des_(0), ydot_des_(1), ydot_des_(2),
                          ydot_des_(3));
  Vector3d w_des_ = 2 * (dy_quat_des * y_quat_des.conjugate()).vec();
  error_ydot_ = w_des_ - ydot_ - GetJ() * v_proj;

  ydot_des_ =
      w_des_;  // Overwrite 4d quat_dot with 3d omega. Need this for osc logging
}

void RotTaskSpaceTrackingData::UpdateJ(const VectorXd& x_wo_spr,
                                       const Context<double>& context_wo_spr) {
  MatrixXd J_spatial(6, plant_wo_spr_->num_velocities());
  plant_wo_spr_->CalcJacobianSpatialVelocity(
      context_wo_spr, JacobianWrtVariable::kV, *body_frames_wo_spr_[fsm_state_],
      frame_poses_[fsm_state_].translation(), *world_wo_spr_, *world_wo_spr_,
      &J_spatial);
  J_ = J_spatial.block(0, 0, kSpaceDim, J_spatial.cols());
}

void RotTaskSpaceTrackingData::UpdateJdotV(
    const VectorXd& x_wo_spr, const Context<double>& context_wo_spr) {
  JdotV_ =
      plant_wo_spr_
          ->CalcBiasSpatialAcceleration(context_wo_spr, JacobianWrtVariable::kV,
                                       *body_frames_wo_spr_[fsm_state_],
                                       frame_poses_[fsm_state_].translation(),
                                       *world_wo_spr_, *world_wo_spr_)
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
  if (ff_accel_multiplier_ != nullptr) {
    std::cerr << "RotTaskSpaceTrackingData does not support feedforward multipliers ";
  }
}

void RotTaskSpaceTrackingData::CheckDerivedOscTrackingData() {
    if (!body_frames_w_spr_.empty()) {
      body_frames_w_spr_ = body_frames_wo_spr_;
    }
}
}  // namespace dairlib::systems::controllers
