#include "impact_invariant_tracking_data.h"

namespace dairlib {
namespace systems {
namespace controllers {

/// OscViewFrame is used to rotate the frame for translational task space
/// tracking data (including ComTrackingData, TransTaskSpaceTrackingData,
/// RelativeTranslationTrackingData).
/// One use case: we can set the swing foot tracking gains in the local frame
/// instead of the global frame.
class OscViewFrame {
 public:
  OscViewFrame(const drake::multibody::Body<double>& body) : body_(body) {}
  virtual Eigen::Matrix3d CalcRotationalMatrix(
      const drake::multibody::MultibodyPlant<double>& plant_w_spr,
      const drake::systems::Context<double>& context_w_spr) const = 0;

 protected:
  const drake::multibody::Body<double>& body_;
};

class WorldYawOscViewFrame : public OscViewFrame {
 public:
  WorldYawOscViewFrame(const drake::multibody::Body<double>& body)
      : OscViewFrame(body) {}

  Eigen::Matrix3d CalcRotationalMatrix(
      const drake::multibody::MultibodyPlant<double>& plant_w_spr,
      const drake::systems::Context<double>& context_w_spr) const override;
};

/// OptionsTrackingData
class OptionsTrackingData : public ImpactInvariantTrackingData {
 public:
  OptionsTrackingData(
      const std::string& name, int n_y, int n_ydot, const Eigen::MatrixXd& K_p,
      const Eigen::MatrixXd& K_d, const Eigen::MatrixXd& W,
      const drake::multibody::MultibodyPlant<double>& plant_w_spr,
      const drake::multibody::MultibodyPlant<double>& plant_wo_spr);

  // Additional feature -- multipliers for gains and feedforward acceleration
  // TOOD(yminchen): You can make ratio dictionary so that we have one ratio per
  //  finite state
  void SetTimeVaryingGains(
      const drake::trajectories::Trajectory<double>& gain_multiplier);
  const drake::trajectories::Trajectory<double>* gain_multiplier_ = nullptr;
  void SetFeedforwardAccelMultiplier(
      const drake::trajectories::Trajectory<double>& ff_accel_multiplier);
  const drake::trajectories::Trajectory<double>* ff_accel_multiplier_ = nullptr;

  // Additional feature -- OscViewFrame
  const OscViewFrame* view_frame_;
  Eigen::Matrix3d view_frame_rot_T_;
  void SetViewFrame(const OscViewFrame& view_frame) {
    view_frame_ = &view_frame;
    with_view_frame_ = true;
  }

  // Additional feature -- disable feedforward acceleration
  std::set<int> idx_zero_feedforward_accel_ = {};
  void DisableFeedforwardAccel(const std::set<int>& indices) {
    idx_zero_feedforward_accel_ = indices;
  };

 private:
  void UpdateActual(
      const Eigen::VectorXd& x_wo_spr,
      const drake::systems::Context<double>& context_wo_spr) override;
  void UpdateYError() override;
  void UpdateYdotError() override;
  void UpdateYddotDes(double t) override;

  bool with_view_frame_ = false;
};

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib