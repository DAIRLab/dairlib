#pragma once

#include "multibody/view_frame.h"
#include "osc_tracking_data.h"

namespace dairlib {
namespace systems {
namespace controllers {

/// OptionsTrackingData
class OptionsTrackingData : public OscTrackingData {
 public:
  OptionsTrackingData(
      const std::string& name, int n_y, int n_ydot, const Eigen::MatrixXd& K_p,
      const Eigen::MatrixXd& K_d, const Eigen::MatrixXd& W,
      const drake::multibody::MultibodyPlant<double>& plant_w_spr,
      const drake::multibody::MultibodyPlant<double>& plant_wo_spr);

  // enable the low pass filter
  void SetLowPassFilter(double tau, const std::set<int>& element_idx = {});

  // Additional feature -- multipliers for gains and feedforward acceleration
  // TOOD(yminchen): You can make ratio dictionary so that we have one ratio per
  //  finite state
  void SetTimeVaryingWeights(
      std::shared_ptr<drake::trajectories::Trajectory<double>>
          weight_trajectory);

  void SetTimeVaryingPDGainMultiplier(
      std::shared_ptr<drake::trajectories::Trajectory<double>>
          gain_multiplier_trajectory);

  void SetTimeVaryingProportionalGainMultiplier(
      std::shared_ptr<drake::trajectories::Trajectory<double>>
          gain_multiplier_trajectory);

  void SetTimeVaryingDerivativeGainMultiplier(
      std::shared_ptr<drake::trajectories::Trajectory<double>>
          gain_multiplier_trajectory);

  void SetTimerVaryingFeedForwardAccelMultiplier(
      std::shared_ptr<drake::trajectories::Trajectory<double>>
          ff_accel_multiplier_traj);

  void SetCmdAccelerationBounds(Eigen::VectorXd& lb, Eigen::VectorXd& ub);

  void SetViewFrame(std::shared_ptr<multibody::ViewFrame<double>> view_frame) {
    view_frame_ = view_frame;
    with_view_frame_ = true;
  }

  const Eigen::MatrixXd& GetWeight() const override {
    return time_varying_weight_;
  }

  // disable feedforward acceleration for the components of the task space given
  // by indices
  void DisableFeedforwardAccel(const std::set<int>& indices) {
    idx_zero_feedforward_accel_ = indices;
  };

  // Ignore a joint to ignore in jacobian calculation.
  // State must be added to the tracking data already
  void AddJointAndStateToIgnoreInJacobian(int joint_vel_idx, int fsm_state);

 protected:
  std::shared_ptr<drake::trajectories::Trajectory<double>>
      ff_accel_multiplier_traj_;
  std::shared_ptr<drake::trajectories::Trajectory<double>>
      p_gain_multiplier_traj_;
  std::shared_ptr<drake::trajectories::Trajectory<double>>
      d_gain_multiplier_traj_;
  std::shared_ptr<drake::trajectories::Trajectory<double>> weight_trajectory_;

  Eigen::VectorXd yddot_cmd_lb_;
  Eigen::VectorXd yddot_cmd_ub_;

  std::set<int> idx_zero_feedforward_accel_ = {};
  std::shared_ptr<multibody::ViewFrame<double>> view_frame_;
  Eigen::Matrix3d view_frame_rot_T_;
  bool with_view_frame_ = false;
  bool is_rotational_tracking_data_ = false;

 private:
  // This method is called from the parent class (OscTrackingData) due to C++
  // polymorphism.
  void UpdateActual(const Eigen::VectorXd& x_w_spr,
                    const drake::systems::Context<double>& context_w_spr,
                    const Eigen::VectorXd& x_wo_spr,
                    const drake::systems::Context<double>& context_wo_spr,
                    double t) override;

  // We don't override the following methods (leave them to children classes):
  //   UpdateY
  //   UpdateYdot
  //   UpdateJ
  //   UpdateJdotV

  // Override the error method
  void UpdateYError() override;
  void UpdateYdotError(const Eigen::VectorXd& v_proj) override;
  void UpdateYddotDes(double t, double t_since_state_switch) override;
  void UpdateYddotCmd(double t, double t_since_state_switch) override;
  void UpdateW(double t, double t_since_state_switch);

  void UpdateFilters(double t);

  // Members of low-pass filter
  Eigen::VectorXd filtered_y_;
  Eigen::VectorXd filtered_ydot_;
  Eigen::MatrixXd time_varying_weight_;
  double tau_ = -1;
  std::set<int> low_pass_filter_element_idx_;
  double last_timestamp_ = -1;
};

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib