#include "rpy_space_tracking_data.h"

using Eigen::Isometry3d;
using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;

using std::string;
using std::vector;

using drake::math::RollPitchYaw;
using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;

namespace dairlib::systems::controllers {

/**** RpyTaskSpaceTrackingData ****/
RpyTaskSpaceTrackingData::RpyTaskSpaceTrackingData(
    const string& name, const MatrixXd& K_p, const MatrixXd& K_d,
    const MatrixXd& W, const MultibodyPlant<double>& plant_w_spr,
    const MultibodyPlant<double>& plant_wo_spr)
    : OptionsTrackingData(name, kSpaceDim, kSpaceDim, K_p, K_d, W,
                          plant_w_spr, plant_wo_spr) {}


void RpyTaskSpaceTrackingData::AddFrameToTrack(
    const std::string& body_name, const Eigen::Isometry3d& frame_pose) {
  AddStateAndFrameToTrack(-1, body_name, frame_pose);
}

void RpyTaskSpaceTrackingData::AddStateAndFrameToTrack(
    int fsm_state, const std::string& body_name,
    const Eigen::Isometry3d& frame_pose) {
  AddFiniteStateToTrack(fsm_state);
  //  AddPointToTrack(body_name, pt_on_body);
  DRAKE_DEMAND(plant_w_spr_.HasBodyNamed(body_name));
  DRAKE_DEMAND(plant_wo_spr_.HasBodyNamed(body_name));
  body_frames_w_spr_[fsm_state] =
      &plant_w_spr_.GetBodyByName(body_name).body_frame();
  body_frames_wo_spr_[fsm_state] =
      &plant_wo_spr_.GetBodyByName(body_name).body_frame();
  frame_poses_[fsm_state] = frame_pose;
}

void RpyTaskSpaceTrackingData::UpdateYddotDes(double, double) {
  // Convert ddq into angular acceleration
  // See https://physics.stackexchange.com/q/460311
  // Convert ddt_rpy into angular acceleration
  RollPitchYaw<double> rpy_des(y_des_);
  Vector3d M_r_ddot = rpy_des.CalcAngularVelocityInParentFromRpyDt(yddot_des_);
  Matrix3d Mdot = rpy_des.CalcDtMatrixRelatingAngularVelocityInParentToRpyDt(ydot_des_);
  yddot_des_converted_ = M_r_ddot + Mdot * ydot_des_;
}

void RpyTaskSpaceTrackingData::UpdateY(const VectorXd& x_w_spr,
                                       const Context<double>& context_w_spr) {
  auto transform_mat = plant_w_spr_.CalcRelativeTransform(
      context_w_spr, plant_w_spr_.world_frame(),
      *body_frames_w_spr_[fsm_state_]);

  RollPitchYaw<double> rpy(transform_mat.rotation());
  y_ = rpy.vector();
}

void RpyTaskSpaceTrackingData::UpdateYError() {
  DRAKE_DEMAND(y_des_.size() == kSpaceDim);

  RollPitchYaw<double> rpy_des(y_des_);

  // Get relative rotation (from current to desired)
  auto ax_ang = (RollPitchYaw<double>(y_).ToRotationMatrix()
      .InvertAndCompose(rpy_des.ToRotationMatrix())).ToAngleAxis();
  error_y_ = ax_ang.angle() * ax_ang.axis();
}

void RpyTaskSpaceTrackingData::UpdateYdot(
    const VectorXd& x_w_spr, const Context<double>& context_w_spr) {
  MatrixXd J_spatial(6, plant_w_spr_.num_velocities());
  plant_w_spr_.CalcJacobianSpatialVelocity(
      context_w_spr, JacobianWrtVariable::kV, *body_frames_w_spr_[fsm_state_],
      frame_poses_[fsm_state_].translation(), world_w_spr_, world_w_spr_,
      &J_spatial);
  ydot_ = J_spatial.block(0, 0, kSpaceDim, J_spatial.cols()) *
      x_w_spr.tail(plant_w_spr_.num_velocities());
}

void RpyTaskSpaceTrackingData::UpdateYdotError(const Eigen::VectorXd& v_proj) {
  // Transform qdot to w
  RollPitchYaw<double> rpy(y_);
  Vector3d w_des_ = rpy.CalcAngularVelocityInParentFromRpyDt(ydot_des_);
  error_ydot_ = w_des_ - ydot_ - GetJ() * v_proj;

  ydot_des_ =
      w_des_;  // Overwrite 4d quat_dot with 3d omega. Need this for osc logging
}

void RpyTaskSpaceTrackingData::UpdateJ(const VectorXd& x_wo_spr,
                                       const Context<double>& context_wo_spr) {
  MatrixXd J_spatial(6, plant_wo_spr_.num_velocities());
  plant_wo_spr_.CalcJacobianSpatialVelocity(
      context_wo_spr, JacobianWrtVariable::kV, *body_frames_wo_spr_[fsm_state_],
      frame_poses_[fsm_state_].translation(), world_wo_spr_, world_wo_spr_,
      &J_spatial);
  J_ = J_spatial.block(0, 0, kSpaceDim, J_spatial.cols());
}

void RpyTaskSpaceTrackingData::UpdateJdotV(
    const VectorXd& x_wo_spr, const Context<double>& context_wo_spr) {
  JdotV_ =
      plant_wo_spr_
          .CalcBiasSpatialAcceleration(context_wo_spr, JacobianWrtVariable::kV,
                                       *body_frames_wo_spr_[fsm_state_],
                                       frame_poses_[fsm_state_].translation(),
                                       world_wo_spr_, world_wo_spr_)
          .rotational();
}

void RpyTaskSpaceTrackingData::CheckDerivedOscTrackingData() {
  //  if (body_frames_w_spr_ != nullptr) {
  //    body_frames_w_spr_ = body_frames_wo_spr_;
  //  }
}
}  // namespace dairlib::systems::controllers
