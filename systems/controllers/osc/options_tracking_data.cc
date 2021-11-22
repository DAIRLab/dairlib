#include "options_tracking_data.h"

using Eigen::MatrixXd;
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
                      plant_wo_spr) {}

void OptionsTrackingData::UpdateActual(
    const Eigen::VectorXd& x_w_spr,
    const drake::systems::Context<double>& context_w_spr,
    const Eigen::VectorXd& x_wo_spr,
    const drake::systems::Context<double>& context_wo_spr, double t) {
  OscTrackingData::UpdateActual(x_w_spr, context_w_spr, x_wo_spr,
                                context_wo_spr, t);

  if (with_view_frame_) {
    view_frame_rot_T_ =
        view_frame_->CalcWorldToFrameRotation(plant_w_spr_, context_wo_spr);
    y_ = view_frame_rot_T_ * y_;
    ydot_ = view_frame_rot_T_ * ydot_;
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
}

void OptionsTrackingData::UpdateYError() { error_y_ = y_des_ - y_; }

void OptionsTrackingData::UpdateYdotError(const Eigen::VectorXd& v_proj) {
  error_ydot_ = ydot_des_ - ydot_;
  if (impact_invariant_projection_) {
    error_ydot_ -= GetJ() * v_proj;
  }
}

void OptionsTrackingData::UpdateYddotDes(double t,
                                         double t_since_state_switch) {
  yddot_des_converted_ = yddot_des_;
  for (auto idx : idx_zero_feedforward_accel_) {
    yddot_des_converted_(idx) = 0;
  }
  if (ff_accel_multiplier_ != nullptr) {
    yddot_des_converted_ = ff_accel_multiplier_->value(t_since_state_switch) *
                           yddot_des_converted_;
  }
}

void OptionsTrackingData::UpdateYddotCmd(double t,
                                         double t_since_state_switch) {
  // 4. Update command output (desired output with pd control)
  MatrixXd gain_multiplier;
  if (gain_multiplier_ != nullptr) {
    gain_multiplier = gain_multiplier_->value(t_since_state_switch);
  } else {
    gain_multiplier = MatrixXd::Identity(n_ydot_, n_ydot_);
  }

  yddot_command_ = yddot_des_converted_ +
                   gain_multiplier * (K_p_ * (error_y_) + K_d_ * (error_ydot_));
}

void OptionsTrackingData::SetLowPassFilter(double tau,
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

void OptionsTrackingData::SetTimeVaryingGains(
    const drake::trajectories::Trajectory<double>& gain_multiplier) {
  DRAKE_DEMAND(gain_multiplier.cols() == n_ydot_);
  DRAKE_DEMAND(gain_multiplier.rows() == n_ydot_);
  DRAKE_DEMAND(gain_multiplier.start_time() == 0);
  gain_multiplier_ = &gain_multiplier;
}
void OptionsTrackingData::SetFeedforwardAccelMultiplier(
    const drake::trajectories::Trajectory<double>& ff_accel_multiplier) {
  DRAKE_DEMAND(ff_accel_multiplier.cols() == n_ydot_);
  DRAKE_DEMAND(ff_accel_multiplier.rows() == n_ydot_);
  DRAKE_DEMAND(ff_accel_multiplier.start_time() == 0);
  ff_accel_multiplier_ = &ff_accel_multiplier;
}

}  // namespace dairlib::systems::controllers
