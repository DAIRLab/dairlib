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
    const MultibodyPlant<double>& plant_w_spr,
    const MultibodyPlant<double>& plant_wo_spr)
    : OscTrackingData(name, n_y, n_ydot, K_p, K_d, W, plant_w_spr,
                      plant_wo_spr) {
  yddot_cmd_lb_ = std::numeric_limits<double>::lowest() * VectorXd::Ones(n_ydot_);
  yddot_cmd_ub_ = std::numeric_limits<double>::max() * VectorXd::Ones(n_ydot_);
}

void OptionsTrackingData::UpdateActual(
    const Eigen::VectorXd& x_w_spr,
    const drake::systems::Context<double>& context_w_spr,
    const Eigen::VectorXd& x_wo_spr,
    const drake::systems::Context<double>& context_wo_spr, double t) {
  OscTrackingData::UpdateActual(x_w_spr, context_w_spr, x_wo_spr,
                                context_wo_spr, t);

  if (with_view_frame_) {
    view_frame_rot_T_ =
        view_frame_->CalcWorldToFrameRotation(plant_w_spr_, context_w_spr);
    if (!is_rotational_tracking_data_) {
      y_ = view_frame_rot_T_ * y_;
      ydot_ = view_frame_rot_T_ * ydot_;
    }
    J_ = view_frame_rot_T_ * J_;
    JdotV_ = view_frame_rot_T_ * JdotV_;
  }

  UpdateFilters(t);
}

void OptionsTrackingData::UpdateFilters(double t) {
  if (tau_ > 0) {
    if (last_timestamp_ < 0) {
      // Initialize
      filtered_y_ = y_;
      filtered_ydot_ = ydot_;
    } else if (t != last_timestamp_) {
      double dt = t - last_timestamp_;
      double alpha = dt / (dt + tau_);
      if (this->is_rotational_tracking_data_) {  // quaternion
        auto slerp = drake::trajectories::PiecewiseQuaternionSlerp<double>(
            {0, 1}, {Quaterniond(y_[0], y_[1], y_[2], y_[3]),
                     Quaterniond(filtered_y_[0], filtered_y_[1], filtered_y_[2],
                                 filtered_y_[3])});
        filtered_y_ = slerp.value(1 - alpha);
      } else {
        filtered_y_ = alpha * y_ + (1 - alpha) * filtered_y_;
      }
      filtered_ydot_ = alpha * ydot_ + (1 - alpha) * filtered_ydot_;
    }

    // Assign filtered values
    if (n_y_ == 4) {
      y_ = filtered_y_;
      ydot_ = filtered_ydot_;
    } else {
      for (auto idx : low_pass_filter_element_idx_) {
        y_(idx) = filtered_y_(idx);
        ydot_(idx) = filtered_ydot_(idx);
      }
    }
    // Update timestamp
    last_timestamp_ = t;
  }
}

void OptionsTrackingData::UpdateYError() { error_y_ = y_des_ - y_; }

void OptionsTrackingData::UpdateYdotError(const Eigen::VectorXd& v_proj) {
  error_ydot_ = ydot_des_ - ydot_;
  if (impact_invariant_projection_) {
    error_ydot_ -= GetJ() * v_proj;
  } else if (no_derivative_feedback_ && !v_proj.isZero()) {
    error_ydot_ = VectorXd::Zero(n_ydot_);
  }
}

void OptionsTrackingData::UpdateYddotDes(double t,
                                         double t_since_state_switch) {
  yddot_des_converted_ = yddot_des_;
  for (auto idx : idx_zero_feedforward_accel_) {
    yddot_des_converted_(idx) = 0;
  }
  if (ff_accel_multiplier_traj_ != nullptr) {
    yddot_des_converted_ =
        ff_accel_multiplier_traj_->value(t_since_state_switch) *
        yddot_des_converted_;
  }
}

void OptionsTrackingData::UpdateYddotCmd(double t,
                                         double t_since_state_switch) {
  // 4. Update command output (desired output with pd control)
  MatrixXd p_gain_multiplier =
      (p_gain_multiplier_traj_ != nullptr)
          ? p_gain_multiplier_traj_->value(t_since_state_switch)
          : MatrixXd::Identity(n_ydot_, n_ydot_);

  MatrixXd d_gain_multiplier =
      (d_gain_multiplier_traj_ != nullptr)
          ? d_gain_multiplier_traj_->value(t_since_state_switch)
          : MatrixXd::Identity(n_ydot_, n_ydot_);

  yddot_command_ = yddot_des_converted_ + p_gain_multiplier * K_p_ * error_y_ +
                   d_gain_multiplier * K_d_ * error_ydot_;
  yddot_command_ = eigen_clamp(yddot_command_, yddot_cmd_lb_, yddot_cmd_ub_);
  UpdateW(t, t_since_state_switch);
}

void OptionsTrackingData::UpdateW(double t, double t_since_state_switch) {
  if (weight_trajectory_ != nullptr) {
    time_varying_weight_ =
        weight_trajectory_->value(time_through_trajectory_).row(0)[0] * W_;
  } else {
    time_varying_weight_ = W_;
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
