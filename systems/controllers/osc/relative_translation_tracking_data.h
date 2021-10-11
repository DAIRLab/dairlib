#pragma once

#include "options_tracking_data.h"

namespace dairlib {
namespace systems {
namespace controllers {
/// RelativeTranslationTrackingData is used when we want to track a trajectory
/// of a relative translational position in the task space.
/// We built this class on top of two other OscTrackingData's. The user have to
/// provide `to_frame_data` and `from_frame_data`.
/// The resulting relative pos/vel/accel are
///    position(to_frame_data) - position(from_frame_data),
///    velocity(to_frame_data) - velocity(from_frame_data),
///    acceleration(to_frame_data) - acceleration(from_frame_data).

/// WARNING: this doesn't work for the rotational position.

/// Developer notes: the current implementation is not the cleanest, since we
/// have to add a PreUpdate() to make this work.
class RelativeTranslationTrackingData final : public OptionsTrackingData {
 public:
  RelativeTranslationTrackingData(
      const std::string& name, const Eigen::MatrixXd& K_p,
      const Eigen::MatrixXd& K_d, const Eigen::MatrixXd& W,
      const drake::multibody::MultibodyPlant<double>& plant_w_spr,
      const drake::multibody::MultibodyPlant<double>& plant_wo_spr,
      OptionsTrackingData* to_frame_data, OptionsTrackingData* from_frame_data);

  void Update(const Eigen::VectorXd& x_w_spr,
              const drake::systems::Context<double>& context_w_spr,
              const Eigen::VectorXd& x_wo_spr,
              const drake::systems::Context<double>& context_wo_spr,
              const drake::trajectories::Trajectory<double>& traj, double t,
              double t_gait_cycle, int fsm_state,
              const Eigen::VectorXd& v_proj) final;

 private:
  void UpdateY(const Eigen::VectorXd& x_wo_spr,
               const drake::systems::Context<double>& context_wo_spr) final;
  void UpdateYdot(const Eigen::VectorXd& x_wo_spr,
                  const drake::systems::Context<double>& context_wo_spr) final;
  void UpdateJ(const Eigen::VectorXd& x_wo_spr,
               const drake::systems::Context<double>& context_wo_spr) final;
  void UpdateJdotV(const Eigen::VectorXd& x_wo_spr,
                   const drake::systems::Context<double>& context_wo_spr) final;

  void CheckDerivedOscTrackingData() final;

  OptionsTrackingData* to_frame_data_;
  OptionsTrackingData* from_frame_data_;
};

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib