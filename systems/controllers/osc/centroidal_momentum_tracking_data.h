#pragma once

#include "options_tracking_data.h"
#include "multibody/pinocchio_plant.h"

namespace dairlib {
namespace systems {
namespace controllers {
/// TransTaskSpaceTrackingData is used when we want to track a trajectory
/// (translational position) in the task space.

/// AddPointToTrack() should be called to specify what is the point that
/// follows the desired trajectory.

/// If users want to track the trajectory only in some states of the finite
/// state machine, they should use AddStateAndPointToTrack().
/// Also, at most one point (of the body) can follow the desired trajectory, so
/// state_ elements can not repeat, and the length of state_ must be the same as
/// pt_on_body_'s if state_ is not empty.
/// This also means that AddPointToTrack and AddStateAndPointToTrack cannot be
/// called one after another for the same TrackingData.
class CentroidalMomentumTrackingData final : public OptionsTrackingData {
 public:
  CentroidalMomentumTrackingData(
      const std::string& name, const Eigen::MatrixXd& K_p,
      const Eigen::MatrixXd& W,
      const drake::multibody::MultibodyPlant<double>& plant_w_spr,
      const drake::multibody::MultibodyPlant<double>& plant_wo_sp,
      const std::string& urdf_w_spr, const std::string& urdf_wo_spr,
      bool angular_only);

 private:

  void UpdateActual(
      const Eigen::VectorXd& x_w_spr,
      const drake::systems::Context<double>& context_w_spr,
      const Eigen::VectorXd& x_wo_spr,
      const drake::systems::Context<double>& context_wo_spr,
      double t) override;

  void UpdateYError() final;
  void UpdateYdotError(const Eigen::VectorXd& v_proj) final;
  void UpdateYddotDes(double t, double t_since_state_switch) final;
  void UpdateYddotCmd(double t, double t_since_state_switch) final;
  void UpdateY(const Eigen::VectorXd& x_w_spr,
              const drake::systems::Context<double>& context_w_spr) final;
  void UpdateYdot(const Eigen::VectorXd& x_w_spr,
                  const drake::systems::Context<double>& context_w_spr) final;
  void UpdateJ(const Eigen::VectorXd& x_w_spr,
               const drake::systems::Context<double>& context_w_spr) final;
  void UpdateJdotV(const Eigen::VectorXd& x_w_spr,
                   const drake::systems::Context<double>& context_w_spr) final;

  void CheckDerivedOscTrackingData() final;

  multibody::PinocchioPlant<double> pinocchio_plant_w_spings_;
  multibody::PinocchioPlant<double> pinocchio_plant_wo_springs_;
  bool angular_only_;
  std::unique_ptr<drake::systems::Context<double>> pin_context_w_springs_;
  std::unique_ptr<drake::systems::Context<double>>  pin_context_wo_springs_;
};

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib