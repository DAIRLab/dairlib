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
  void SetTimeVaryingGains(
      std::shared_ptr<drake::trajectories::Trajectory<double>> gain_multiplier);
  void SetTimeVaryingWeights(
      std::shared_ptr<drake::trajectories::Trajectory<double>> weight_trajectory);

  void SetFeedforwardAccelMultiplier(
      const drake::trajectories::Trajectory<double>& ff_accel_multiplier);
  const drake::trajectories::Trajectory<double>* ff_accel_multiplier_ = nullptr;

  const Eigen::MatrixXd& GetWeight() const override { return time_varying_weight_; }

  // Additional feature -- ViewFrame
  const multibody::ViewFrame<double>* view_frame_;
  Eigen::Matrix3d view_frame_rot_T_;
  void SetViewFrame(const multibody::ViewFrame<double>& view_frame) {
    view_frame_ = &view_frame;
    with_view_frame_ = true;
  }

  // Additional feature -- disable feedforward acceleration
  std::set<int> idx_zero_feedforward_accel_ = {};
  void DisableFeedforwardAccel(const std::set<int>& indices) {
    idx_zero_feedforward_accel_ = indices;
  };

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

  std::shared_ptr<drake::trajectories::Trajectory<double>> gain_multiplier_;
  std::shared_ptr<drake::trajectories::Trajectory<double>> weight_trajectory_;

  void UpdateFilters(double t);

  bool with_view_frame_ = false;

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