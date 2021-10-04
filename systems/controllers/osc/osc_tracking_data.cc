#include "systems/controllers/osc/osc_tracking_data.h"

#include <math.h>

#include <algorithm>

#include <drake/multibody/plant/multibody_plant.h>

#include "multibody/multibody_utils.h"
#include "systems/controllers/osc/osc_utils.h"

using std::cout;
using std::endl;

using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using Eigen::Isometry3d;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::string;
using std::vector;

namespace dairlib::systems::controllers {

using multibody::JwrtqdotToJwrtv;
using multibody::makeNameToPositionsMap;
using multibody::makeNameToVelocitiesMap;
using multibody::WToQuatDotMap;

/**** OscTrackingData ****/
OscTrackingData::OscTrackingData(const string& name, int n_y, int n_ydot,
                                 const MatrixXd& K_p, const MatrixXd& K_d,
                                 const MatrixXd& W,
                                 const MultibodyPlant<double>& plant_w_spr,
                                 const MultibodyPlant<double>& plant_wo_spr,
                                 bool use_only_plant_wo_spr_in_evaluation)
    : plant_w_spr_(plant_w_spr),
      plant_wo_spr_(plant_wo_spr),
      world_w_spr_(plant_w_spr_.world_frame()),
      world_wo_spr_(plant_wo_spr_.world_frame()),
      name_(name),
      n_y_(n_y),
      n_ydot_(n_ydot),
      K_p_(K_p),
      K_d_(K_d),
      W_(W),
      use_only_plant_wo_spr_in_evaluation_(
          use_only_plant_wo_spr_in_evaluation) {}

// Update
bool OscTrackingData::Update(
    const VectorXd& x_w_spr, const Context<double>& context_w_spr,
    const VectorXd& x_wo_spr, const Context<double>& context_wo_spr,
    const drake::trajectories::Trajectory<double>& traj, double t,
    double t_since_last_state_switch, int finite_state_machine_state,
    const Eigen::VectorXd& v_proj, bool no_desired_traj) {
  // Update track_at_current_state_
  UpdateTrackingFlag(finite_state_machine_state);

  // Proceed based on the result of track_at_current_state_
  if (track_at_current_state_) {
    if (pre_update_) {
      PreUpdate(x_w_spr, context_w_spr, x_wo_spr, context_wo_spr, traj, t,
                t_since_last_state_switch, finite_state_machine_state, v_proj,
                true);
    }

    // 1. Update feedback output (Calling virtual methods)
    if (use_only_plant_wo_spr_in_evaluation_) {
      UpdateY(x_wo_spr, context_wo_spr);
      UpdateYdot(x_wo_spr, context_wo_spr);
    } else {
      UpdateY(x_w_spr, context_w_spr);
      UpdateYdot(x_w_spr, context_w_spr);
    }

    UpdateJ(x_wo_spr, context_wo_spr);
    UpdateJdotV(x_wo_spr, context_wo_spr);

    // Rotate for view frame
    if (with_view_frame_) {
      Eigen::Matrix3d view_frame_rot_T =
          view_frame_->CalcRotationalMatrix(plant_w_spr_, context_w_spr)
              .transpose();

      y_ = view_frame_rot_T * y_;
      ydot_ = view_frame_rot_T * ydot_;
      J_ = view_frame_rot_T * J_;
      JdotV_ = view_frame_rot_T * JdotV_;
    }

    // Low-pass filtering y_ and ydot_
    if (tau_ > 0) {
      LowPassFiltering(t);
    }

    if (no_desired_traj) {
      return track_at_current_state_;
    }

    // 2. Update desired output
    y_des_ = traj.value(t);
    if (traj.has_derivative()) {
      ydot_des_ = traj.EvalDerivative(t, 1);
      yddot_des_ = traj.EvalDerivative(t, 2);
    }
    // TODO (yangwill): Remove this edge case after EvalDerivative has been
    // implemented for ExponentialPlusPiecewisePolynomial
    else {
      ydot_des_ = traj.MakeDerivative(1)->value(t);
      yddot_des_ = traj.MakeDerivative(2)->value(t);
    }
    UpdateYddotDes();
    // Zero feedforward acceleration
    for (auto idx : idx_zero_feedforward_accel_) {
      yddot_des_converted_(idx) = 0;
    }
    if (ff_accel_multiplier_ != nullptr) {
      yddot_des_converted_ =
          ff_accel_multiplier_->value(t_since_last_state_switch) *
          yddot_des_converted_;
    }

    // 3. Update error
    // Careful: must update y and y_des before calling UpdateYError()
    UpdateYError();
    UpdateYdotError();

    if (impact_invariant_projection_) {
      error_ydot_ -= GetJ() * v_proj;
    }

    // 4. Update command output (desired output with pd control)
    MatrixXd gain_multiplier;
    if (gain_multiplier_ != nullptr) {
      gain_multiplier = gain_multiplier_->value(t_since_last_state_switch);
    } else {
      gain_multiplier = MatrixXd::Identity(n_ydot_, n_ydot_);
    }

    yddot_command_ =
        yddot_des_converted_ +
        gain_multiplier * (K_p_ * (error_y_) + K_d_ * (error_ydot_));
  }
  return track_at_current_state_;
}

void OscTrackingData::UpdateTrackingFlag(int finite_state_machine_state) {
  if (state_.empty()) {
    track_at_current_state_ = true;
    state_idx_ = 0;
    return;
  }

  auto it = find(state_.begin(), state_.end(), finite_state_machine_state);
  state_idx_ = std::distance(state_.begin(), it);
  track_at_current_state_ = it != state_.end();
}

void OscTrackingData::SetTimeVaryingGains(
    const drake::trajectories::Trajectory<double>& gain_multiplier) {
  DRAKE_DEMAND(gain_multiplier.cols() == n_ydot_);
  DRAKE_DEMAND(gain_multiplier.rows() == n_ydot_);
  DRAKE_DEMAND(gain_multiplier.start_time() == 0);
  //  DRAKE_DEMAND(gain_multiplier.end_time() == );
  gain_multiplier_ = &gain_multiplier;
}
void OscTrackingData::SetFeedforwardAccelMultiplier(
    const drake::trajectories::Trajectory<double>& ff_accel_multiplier) {
  DRAKE_DEMAND(ff_accel_multiplier.cols() == n_ydot_);
  DRAKE_DEMAND(ff_accel_multiplier.rows() == n_ydot_);
  DRAKE_DEMAND(ff_accel_multiplier.start_time() == 0);
  //  DRAKE_DEMAND(ff_accel_multiplier.end_time() == );
  ff_accel_multiplier_ = &ff_accel_multiplier;
}

void OscTrackingData::SetLowPassFilter(double tau,
                                       const std::set<int>& element_idx) {
  DRAKE_DEMAND(tau > 0);
  tau_ = tau;

  DRAKE_DEMAND(n_y_ == n_ydot_);  // doesn't support quaternion yet
  if (element_idx.empty()) {
    for (int i = 0; i < n_y_; i++) {
      low_pass_filter_element_idx_.insert(i);
    }
  } else {
    low_pass_filter_element_idx_ = element_idx;
  }
}

void OscTrackingData::LowPassFiltering(double t) {
  if (last_timestamp_ < 0) {
    // Initialize
    filtered_y_ = y_;
    filtered_ydot_ = ydot_;
  } else if (t != last_timestamp_) {
    double dt = t - last_timestamp_;
    double alpha = dt / (dt + tau_);
    filtered_y_ = alpha * y_ + (1 - alpha) * filtered_y_;
    filtered_ydot_ = alpha * ydot_ + (1 - alpha) * filtered_ydot_;
  }

  // Assign filtered values
  for (auto idx : low_pass_filter_element_idx_) {
    y_(idx) = filtered_y_(idx);
    ydot_(idx) = filtered_ydot_(idx);
  }

  // Update timestamp
  last_timestamp_ = t;
}

void OscTrackingData::PrintFeedbackAndDesiredValues(const VectorXd& dv) {
  DRAKE_ASSERT(track_at_current_state_);
  cout << name_ << ":\n";
  cout << "  y = " << y_.transpose() << endl;
  cout << "  y_des = " << y_des_.transpose() << endl;
  cout << "  error_y = " << error_y_.transpose() << endl;
  cout << "  ydot = " << ydot_.transpose() << endl;
  cout << "  ydot_des = " << ydot_des_.transpose() << endl;
  cout << "  error_ydot_ = " << error_ydot_.transpose() << endl;
  cout << "  yddot_des_converted = " << yddot_des_converted_.transpose()
       << endl;
  cout << "  yddot_command = " << yddot_command_.transpose() << endl;
  cout << "  yddot_command_sol = " << (J_ * dv + JdotV_).transpose() << endl;
}

void OscTrackingData::SaveYddotCommandSol(const VectorXd& dv) {
  DRAKE_ASSERT(track_at_current_state_);
  yddot_command_sol_ = J_ * dv + JdotV_;
}

void OscTrackingData::AddState(int state) {
  // Avoid repeated states
  for (auto const& element : state_) {
    DRAKE_DEMAND(element != state);
  }
  state_.push_back(state);
}

// Run this function in OSC constructor to make sure that users constructed
// OscTrackingData correctly.
void OscTrackingData::CheckOscTrackingData(bool no_control_gains) {
  cout << "Checking " << name_ << endl;
  CheckDerivedOscTrackingData();

  // State_ cannot have repeated state
  auto it = std::unique(state_.begin(), state_.end());
  bool all_state_are_different = (it == state_.end());
  DRAKE_DEMAND(all_state_are_different);

  if (no_control_gains) return;
  DRAKE_DEMAND((K_p_.rows() == n_ydot_) && (K_p_.cols() == n_ydot_));
  DRAKE_DEMAND((K_d_.rows() == n_ydot_) && (K_d_.cols() == n_ydot_));
  DRAKE_DEMAND((W_.rows() == n_ydot_) && (W_.cols() == n_ydot_));
}

/**** ComTrackingData ****/
ComTrackingData::ComTrackingData(const string& name, const MatrixXd& K_p,
                                 const MatrixXd& K_d, const MatrixXd& W,
                                 const MultibodyPlant<double>& plant_w_spr,
                                 const MultibodyPlant<double>& plant_wo_spr)
    : OscTrackingData(name, kSpaceDim, kSpaceDim, K_p, K_d, W, plant_w_spr,
                      plant_wo_spr) {}

void ComTrackingData::AddStateToTrack(int state) { AddState(state); }

void ComTrackingData::UpdateYddotDes() { yddot_des_converted_ = yddot_des_; }

void ComTrackingData::UpdateY(const VectorXd& x_w_spr,
                              const Context<double>& context_w_spr) {
  y_ = plant_w_spr_.CalcCenterOfMassPositionInWorld(context_w_spr);
}

void ComTrackingData::UpdateYError() { error_y_ = y_des_ - y_; }

void ComTrackingData::UpdateYdot(const VectorXd& x_w_spr,
                                 const Context<double>& context_w_spr) {
  MatrixXd J_w_spr(kSpaceDim, plant_w_spr_.num_velocities());
  plant_w_spr_.CalcJacobianCenterOfMassTranslationalVelocity(
      context_w_spr, JacobianWrtVariable::kV, world_w_spr_, world_w_spr_,
      &J_w_spr);
  ydot_ = J_w_spr * x_w_spr.tail(plant_w_spr_.num_velocities());
}

void ComTrackingData::UpdateYdotError() { error_ydot_ = ydot_des_ - ydot_; }

void ComTrackingData::UpdateJ(const VectorXd& x_wo_spr,
                              const Context<double>& context_wo_spr) {
  J_ = MatrixXd::Zero(kSpaceDim, plant_wo_spr_.num_velocities());
  plant_wo_spr_.CalcJacobianCenterOfMassTranslationalVelocity(
      context_wo_spr, JacobianWrtVariable::kV, world_w_spr_, world_w_spr_, &J_);
}

void ComTrackingData::UpdateJdotV(const VectorXd& x_wo_spr,
                                  const Context<double>& context_wo_spr) {
  JdotV_ = plant_wo_spr_.CalcBiasCenterOfMassTranslationalAcceleration(
      context_wo_spr, JacobianWrtVariable::kV, world_wo_spr_, world_wo_spr_);
}

void ComTrackingData::CheckDerivedOscTrackingData() {}

/**** TaskSpaceTrackingData ****/
TaskSpaceTrackingData::TaskSpaceTrackingData(
    const string& name, int n_y, int n_ydot, const MatrixXd& K_p,
    const MatrixXd& K_d, const MatrixXd& W,
    const MultibodyPlant<double>& plant_w_spr,
    const MultibodyPlant<double>& plant_wo_spr)
    : OscTrackingData(name, n_y, n_ydot, K_p, K_d, W, plant_w_spr,
                      plant_wo_spr) {}

/**** TransTaskSpaceTrackingData ****/
TransTaskSpaceTrackingData::TransTaskSpaceTrackingData(
    const string& name, const MatrixXd& K_p, const MatrixXd& K_d,
    const MatrixXd& W, const MultibodyPlant<double>& plant_w_spr,
    const MultibodyPlant<double>& plant_wo_spr)
    : TaskSpaceTrackingData(name, kSpaceDim, kSpaceDim, K_p, K_d, W,
                            plant_w_spr, plant_wo_spr) {}

void TransTaskSpaceTrackingData::AddPointToTrack(const std::string& body_name,
                                                 const Vector3d& pt_on_body) {
  DRAKE_DEMAND(plant_w_spr_.HasBodyNamed(body_name));
  DRAKE_DEMAND(plant_wo_spr_.HasBodyNamed(body_name));
  body_index_w_spr_.push_back(plant_w_spr_.GetBodyByName(body_name).index());
  body_index_wo_spr_.push_back(plant_wo_spr_.GetBodyByName(body_name).index());
  body_frames_w_spr_.push_back(
      &plant_w_spr_.GetBodyByName(body_name).body_frame());
  body_frames_wo_spr_.push_back(
      &plant_wo_spr_.GetBodyByName(body_name).body_frame());
  pts_on_body_.push_back(pt_on_body);
}
void TransTaskSpaceTrackingData::AddStateAndPointToTrack(
    int state, const std::string& body_name, const Vector3d& pt_on_body) {
  AddState(state);
  AddPointToTrack(body_name, pt_on_body);
}

void TransTaskSpaceTrackingData::UpdateYddotDes() {
  yddot_des_converted_ = yddot_des_;
}

void TransTaskSpaceTrackingData::UpdateY(const VectorXd& x_w_spr,
                                         const Context<double>& context_w_spr) {
  y_ = Vector3d::Zero();
  plant_w_spr_.CalcPointsPositions(
      context_w_spr, *body_frames_wo_spr_[GetStateIdx()],
      pts_on_body_[GetStateIdx()], world_w_spr_, &y_);
}

void TransTaskSpaceTrackingData::UpdateYError() { error_y_ = y_des_ - y_; }

void TransTaskSpaceTrackingData::UpdateYdot(
    const VectorXd& x_w_spr, const Context<double>& context_w_spr) {
  MatrixXd J(kSpaceDim, plant_w_spr_.num_velocities());
  plant_w_spr_.CalcJacobianTranslationalVelocity(
      context_w_spr, JacobianWrtVariable::kV,
      *body_frames_w_spr_.at(GetStateIdx()), pts_on_body_.at(GetStateIdx()),
      world_w_spr_, world_w_spr_, &J);
  ydot_ = J * x_w_spr.tail(plant_w_spr_.num_velocities());
}

void TransTaskSpaceTrackingData::UpdateYdotError() {
  error_ydot_ = ydot_des_ - ydot_;
}

void TransTaskSpaceTrackingData::UpdateJ(
    const VectorXd& x_wo_spr, const Context<double>& context_wo_spr) {
  J_ = MatrixXd::Zero(kSpaceDim, plant_wo_spr_.num_velocities());
  plant_wo_spr_.CalcJacobianTranslationalVelocity(
      context_wo_spr, JacobianWrtVariable::kV,
      *body_frames_wo_spr_.at(GetStateIdx()), pts_on_body_.at(GetStateIdx()),
      world_wo_spr_, world_wo_spr_, &J_);
}

void TransTaskSpaceTrackingData::UpdateJdotV(
    const VectorXd& x_wo_spr, const Context<double>& context_wo_spr) {
  JdotV_ = plant_wo_spr_.CalcBiasTranslationalAcceleration(
      context_wo_spr, drake::multibody::JacobianWrtVariable::kV,
      *body_frames_wo_spr_.at(GetStateIdx()), pts_on_body_.at(GetStateIdx()),
      world_wo_spr_, world_wo_spr_);
}

void TransTaskSpaceTrackingData::CheckDerivedOscTrackingData() {
  if (body_index_w_spr_.empty()) {
    body_index_w_spr_ = body_index_wo_spr_;
  }
  DRAKE_DEMAND(body_index_w_spr_.size() == pts_on_body_.size());
  DRAKE_DEMAND(body_index_wo_spr_.size() == pts_on_body_.size());
  DRAKE_DEMAND(state_.empty() || (state_.size() == pts_on_body_.size()));
  if (state_.empty()) {
    DRAKE_DEMAND(body_index_w_spr_.size() == 1);
    DRAKE_DEMAND(body_index_wo_spr_.size() == 1);
    DRAKE_DEMAND(pts_on_body_.size() == 1);
  }
}

/**** RelativeTranslationTrackingData ****/
RelativeTranslationTrackingData::RelativeTranslationTrackingData(
    const std::string& name, const Eigen::MatrixXd& K_p,
    const Eigen::MatrixXd& K_d, const Eigen::MatrixXd& W,
    const drake::multibody::MultibodyPlant<double>& plant_w_spr,
    const drake::multibody::MultibodyPlant<double>& plant_wo_spr,
    OscTrackingData& to_frame_data, OscTrackingData& from_frame_data)
    : OscTrackingData(name, kSpaceDim, kSpaceDim, K_p, K_d, W, plant_w_spr,
                      plant_wo_spr),
      to_frame_data_(to_frame_data),
      from_frame_data_(from_frame_data) {
  auto states1 = to_frame_data.GetStates();
  auto states2 = from_frame_data.GetStates();
  DRAKE_DEMAND(states1.size() == states2.size());
  for (int i = 0; i < states1.size(); i++) {
    DRAKE_DEMAND(states1.at(i) == states2.at(i));
    AddState(states1.at(i));
  }

  pre_update_ = true;
}

void RelativeTranslationTrackingData::PreUpdate(
    const VectorXd& x_w_spr, const Context<double>& context_w_spr,
    const VectorXd& x_wo_spr, const Context<double>& context_wo_spr,
    const drake::trajectories::Trajectory<double>& traj, double t,
    double t_since_last_state_switch, int finite_state_machine_state,
    const Eigen::VectorXd& v_proj, bool no_desired_traj) {
  to_frame_data_.Update(x_w_spr, context_w_spr, x_wo_spr, context_wo_spr, traj,
                        t, t_since_last_state_switch,
                        finite_state_machine_state, v_proj, true);
  from_frame_data_.Update(x_w_spr, context_w_spr, x_wo_spr, context_wo_spr,
                          traj, t, t_since_last_state_switch,
                          finite_state_machine_state, v_proj, true);
}

void RelativeTranslationTrackingData::UpdateYddotDes() {
  yddot_des_converted_ = yddot_des_;
}

void RelativeTranslationTrackingData::UpdateY(
    const VectorXd& x_w_spr, const Context<double>& context_w_spr) {
  y_ = to_frame_data_.GetY() - from_frame_data_.GetY();
}

void RelativeTranslationTrackingData::UpdateYError() { error_y_ = y_des_ - y_; }

void RelativeTranslationTrackingData::UpdateYdot(
    const VectorXd& x_w_spr, const Context<double>& context_w_spr) {
  ydot_ = to_frame_data_.GetYdot() - from_frame_data_.GetYdot();
}

void RelativeTranslationTrackingData::UpdateYdotError() {
  error_ydot_ = ydot_des_ - ydot_;
}

void RelativeTranslationTrackingData::UpdateJ(
    const VectorXd& x_wo_spr, const Context<double>& context_wo_spr) {
  J_ = to_frame_data_.GetJ() - from_frame_data_.GetJ();
}

void RelativeTranslationTrackingData::UpdateJdotV(
    const VectorXd& x_wo_spr, const Context<double>& context_wo_spr) {
  JdotV_ = to_frame_data_.GetJdotTimesV() - from_frame_data_.GetJdotTimesV();
}

void RelativeTranslationTrackingData::CheckDerivedOscTrackingData() {
  to_frame_data_.CheckOscTrackingData(true);
  from_frame_data_.CheckOscTrackingData(true);
}

/**** WorldYawOscViewFrame ****/
Eigen::Matrix3d WorldYawOscViewFrame::CalcRotationalMatrix(
    const drake::multibody::MultibodyPlant<double>& plant_w_spr,
    const drake::systems::Context<double>& context_w_spr) const {
  // Get approximated heading angle of pelvis and rotational matrix
  Vector3d body_x_axis =
      plant_w_spr.EvalBodyPoseInWorld(context_w_spr, body_).rotation().col(0);
  double approx_body_yaw = atan2(body_x_axis(1), body_x_axis(0));
  Eigen::MatrixXd rot(3, 3);
  rot << cos(approx_body_yaw), -sin(approx_body_yaw), 0, sin(approx_body_yaw),
      cos(approx_body_yaw), 0, 0, 0, 1;
  return rot;
}

}  // namespace dairlib::systems::controllers
