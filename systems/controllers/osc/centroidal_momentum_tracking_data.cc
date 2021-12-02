#include "centroidal_momentum_tracking_data.h"
#include "drake/math/roll_pitch_yaw.h"

namespace dairlib::systems::controllers {

using multibody::PinocchioPlant;

using Eigen::MatrixXd;
using Eigen::VectorXd;

using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::math::RollPitchYaw;

CentroidalMomentumTrackingData::CentroidalMomentumTrackingData(
    const std::string& name, const MatrixXd& K_p,
    const MatrixXd& K_d, const MatrixXd& W,
    const drake::multibody::MultibodyPlant<double>& plant_w_spr,
    const drake::multibody::MultibodyPlant<double>& plant_wo_sp,
    const std::string& urdf_w_spr, const std::string& urdf_wo_spr,
    bool angular_only=true) :
    OptionsTrackingData(name,
                        angular_only ? 3 : 6, angular_only ? 3 : 6,
                        K_p, K_d, W, plant_w_spr, plant_wo_sp),
    pinocchio_plant_w_spings_ (PinocchioPlant<double>(0.0, urdf_w_spr)),
    pinocchio_plant_wo_springs_ (PinocchioPlant<double>(0.0, urdf_wo_spr)) {

  angular_only_ = angular_only;
  pinocchio_plant_w_spings_.Finalize();
  pinocchio_plant_wo_springs_.Finalize();
  pin_context_w_springs_ = pinocchio_plant_w_spings_.CreateDefaultContext();
  pin_context_wo_springs_ = pinocchio_plant_wo_springs_.CreateDefaultContext();
}

void CentroidalMomentumTrackingData::UpdateYddotDes(double, double) {
  // Convert ddq into angular acceleration
  // See https://physics.stackexchange.com/q/460311
  // Convert ddt_rpy into angular acceleration
  RollPitchYaw<double> rpy(y_);
  Vector3d M_r_ddot = rpy.CalcAngularVelocityInParentFromRpyDt(yddot_des_);
  Matrix3d Mdot = rpy.CalcDtMatrixRelatingAngularVelocityInParentToRpyDt(ydot_des_);
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

