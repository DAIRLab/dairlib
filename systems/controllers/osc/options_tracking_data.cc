#include "options_tracking_data.h"
#include "common/eigen_utils.h"

#include "drake/common/trajectories/piecewise_quaternion.h"

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

OptionsTrackingData::OptionsTrackingData(
    const string& name, int n_y, int n_ydot, const MatrixXd& K_p,
    const MatrixXd& K_d, const MatrixXd& W,
    const MultibodyPlant<double>& plant)
    : OscTrackingData(name, n_y, n_ydot, K_p, K_d, W, plant) {
  yddot_cmd_lb_ = std::numeric_limits<double>::lowest() * VectorXd::Ones(n_ydot_);
  yddot_cmd_ub_ = std::numeric_limits<double>::max() * VectorXd::Ones(n_ydot_);
}

void OptionsTrackingData::UpdateActual(
    const Eigen::VectorXd& x,
    const drake::systems::Context<double>& context, double t,
    OscTrackingDataState& td_state) const {
  OscTrackingData::UpdateActual(x, context, t, td_state);

  if (with_view_frame_) {
    td_state.view_frame_rot_T_ =
        view_frame_->CalcWorldToFrameRotation(plant_, context);
    if (!is_rotational_tracking_data_) {
      td_state.y_ =td_state.view_frame_rot_T_ * td_state.y_;
      td_state.ydot_ = td_state.view_frame_rot_T_ * td_state.ydot_;
    }
    td_state.J_ = td_state.view_frame_rot_T_ * td_state.J_;
    td_state.JdotV_ = td_state.view_frame_rot_T_ * td_state.JdotV_;
  }

  UpdateFilters(t, td_state);
}

void OptionsTrackingData::UpdateFilters(
    double t, OscTrackingDataState& td_state) const {

  if (tau_ > 0) {
    if (td_state.last_timestamp_ < 0) {
      // Initialize
      td_state.filtered_y_ = td_state.y_;
      td_state.filtered_ydot_ = td_state.ydot_;
    } else if (t != td_state.last_timestamp_) {
      double dt = t - td_state.last_timestamp_;
      double alpha = dt / (dt + tau_);
      if (this->is_rotational_tracking_data_) {  // quaternion
        auto slerp = drake::trajectories::PiecewiseQuaternionSlerp<double>(
            {0, 1},
            {Quaterniond(
                td_state.y_[0], td_state.y_[1], td_state.y_[2], td_state.y_[3]),
             Quaterniond(
                 td_state.filtered_y_[0], td_state.filtered_y_[1],
                 td_state.filtered_y_[2],td_state.filtered_y_[3])});
        td_state.filtered_y_ = slerp.value(1 - alpha);
      } else {
        td_state.filtered_y_ = alpha * td_state.y_ + (1 - alpha) * td_state.filtered_y_;
      }
      td_state.filtered_ydot_ = alpha * td_state.ydot_ + (1 - alpha) * td_state.filtered_ydot_;
    }

    // Assign filtered values
    if (n_y_ == 4) {
      td_state.y_ = td_state.filtered_y_;
      td_state.ydot_ = td_state.filtered_ydot_;
    } else {
      for (auto idx : low_pass_filter_element_idx_) {
        td_state.y_(idx) = td_state.filtered_y_(idx);
        td_state.ydot_(idx) = td_state.filtered_ydot_(idx);
      }
    }
    // Update timestamp
    td_state.last_timestamp_ = t;
  }
}

void OptionsTrackingData::UpdateYError(OscTrackingDataState& td_state) const{
  td_state.error_y_ = td_state.y_des_ - td_state.y_;
}

void OptionsTrackingData::UpdateYdotError(
    const Eigen::VectorXd& v_proj, OscTrackingDataState& td_state) const {
  td_state.error_ydot_ = td_state.ydot_des_ - td_state.ydot_;
  if (impact_invariant_projection_) {
    td_state.error_ydot_ -= td_state.J_ * v_proj;
  } else if (no_derivative_feedback_ && !v_proj.isZero()) {
    td_state.error_ydot_ = VectorXd::Zero(n_ydot_);
  }
}

void OptionsTrackingData::UpdateYddotDes(
    double t, double t_since_state_switch, OscTrackingDataState& td_state) const {
  td_state.yddot_des_converted_ = td_state.yddot_des_;
  for (auto idx : idx_zero_feedforward_accel_) {
    td_state.yddot_des_converted_(idx) = 0;
  }
  if (ff_accel_multiplier_traj_ != nullptr) {
    td_state.yddot_des_converted_ =
        ff_accel_multiplier_traj_->value(t_since_state_switch) *
        td_state.yddot_des_converted_;
  }
}

void OptionsTrackingData::UpdateYddotCmd(
    double t, double t_since_state_switch, OscTrackingDataState& td_state) const {
  // 4. Update command output (desired output with pd control)
  MatrixXd p_gain_multiplier =
      (p_gain_multiplier_traj_ != nullptr)
          ? p_gain_multiplier_traj_->value(t_since_state_switch)
          : MatrixXd::Identity(n_ydot_, n_ydot_);

  MatrixXd d_gain_multiplier =
      (d_gain_multiplier_traj_ != nullptr)
          ? d_gain_multiplier_traj_->value(t_since_state_switch)
          : MatrixXd::Identity(n_ydot_, n_ydot_);

  td_state.yddot_command_ = td_state.yddot_des_converted_ + p_gain_multiplier * K_p_ * td_state.error_y_ +
                   d_gain_multiplier * K_d_ * td_state.error_ydot_;
  td_state.yddot_command_ = eigen_clamp(td_state.yddot_command_, yddot_cmd_lb_, yddot_cmd_ub_);
  UpdateW(t, t_since_state_switch, td_state);
}

void OptionsTrackingData::UpdateW(
    double t, double t_since_state_switch, OscTrackingDataState& td_state) const {
  if (weight_trajectory_ != nullptr) {
    td_state.time_varying_weight_ =
        weight_trajectory_->value(td_state.time_through_trajectory_).row(0)[0] * W_;
  } else {
    td_state.time_varying_weight_ = W_;
  }
}

void OptionsTrackingData::SetLowPassFilter(double tau,
                                           const std::set<int>& element_idx) {
  DRAKE_DEMAND(tau > 0);
  tau_ = tau;

  //  DRAKE_DEMAND(n_y_ == n_ydot_);  // doesn't support quaternion yet
  if (element_idx.empty()) {
    for (int i = 0; i < n_y_; i++) {
      low_pass_filter_element_idx_.insert(i);
    }
  } else {
    low_pass_filter_element_idx_ = element_idx;
  }
}

void OptionsTrackingData::SetTimeVaryingWeights(
    std::shared_ptr<drake::trajectories::Trajectory<double>>
        weight_trajectory) {
  //  DRAKE_DEMAND(weight_trajectory->cols() == n_ydot_);
  //  DRAKE_DEMAND(weight_trajectory->rows() == n_ydot_);
  //  DRAKE_DEMAND(weight_trajectory->start_time() == 0);
  weight_trajectory_ = weight_trajectory;
}

void OptionsTrackingData::SetTimeVaryingPDGainMultiplier(
    std::shared_ptr<drake::trajectories::Trajectory<double>>
        gain_multiplier_trajectory) {
  SetTimeVaryingProportionalGainMultiplier(gain_multiplier_trajectory);
  SetTimeVaryingDerivativeGainMultiplier(gain_multiplier_trajectory);
}

void OptionsTrackingData::SetTimeVaryingProportionalGainMultiplier(
    std::shared_ptr<drake::trajectories::Trajectory<double>>
        gain_multiplier_trajectory) {
  DRAKE_DEMAND(gain_multiplier_trajectory->cols() == n_ydot_);
  DRAKE_DEMAND(gain_multiplier_trajectory->rows() == n_ydot_);
  DRAKE_DEMAND(gain_multiplier_trajectory->start_time() == 0);
  p_gain_multiplier_traj_ = gain_multiplier_trajectory;
}

void OptionsTrackingData::SetTimeVaryingDerivativeGainMultiplier(
    std::shared_ptr<drake::trajectories::Trajectory<double>>
        gain_multiplier_trajectory) {
  DRAKE_DEMAND(gain_multiplier_trajectory->cols() == n_ydot_);
  DRAKE_DEMAND(gain_multiplier_trajectory->rows() == n_ydot_);
  DRAKE_DEMAND(gain_multiplier_trajectory->start_time() == 0);
  d_gain_multiplier_traj_ = gain_multiplier_trajectory;
}

void OptionsTrackingData::SetTimerVaryingFeedForwardAccelMultiplier(
    std::shared_ptr<drake::trajectories::Trajectory<double>>
        ff_accel_multiplier_traj) {
  DRAKE_DEMAND(ff_accel_multiplier_traj->cols() == n_ydot_);
  DRAKE_DEMAND(ff_accel_multiplier_traj->rows() == n_ydot_);
  DRAKE_DEMAND(ff_accel_multiplier_traj->start_time() == 0);
  ff_accel_multiplier_traj_ = ff_accel_multiplier_traj;
}

void OptionsTrackingData::SetCmdAccelerationBounds(Eigen::VectorXd& lb, Eigen::VectorXd& ub){
  DRAKE_DEMAND(lb.size() == n_ydot_);
  DRAKE_DEMAND(ub.size() == n_ydot_);
  yddot_cmd_lb_ = lb;
  yddot_cmd_ub_ = ub;
};

}  // namespace dairlib::systems::controllers
