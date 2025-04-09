#pragma once

#include "options_tracking_data.h"

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
class TransTaskSpaceTrackingData final : public OptionsTrackingData {
 public:
  TransTaskSpaceTrackingData(
      const std::string& name, const Eigen::MatrixXd& K_p,
      const Eigen::MatrixXd& K_d, const Eigen::MatrixXd& W,
      const drake::multibody::MultibodyPlant<double>& plant_w_spr,
      const drake::multibody::MultibodyPlant<double>& plant_wo_sp);
  void AddPointToTrack(
      const std::string& body_name,
      const Eigen::Vector3d& pt_on_body = Eigen::Vector3d::Zero());
  void AddStateAndPointToTrack(
      int state, const std::string& body_name,
      const Eigen::Vector3d& pt_on_body = Eigen::Vector3d::Zero());

 protected:
  std::unordered_map<int, const drake::multibody::BodyFrame<double>*>
      body_frames_w_spr_;
  std::unordered_map<int, const drake::multibody::BodyFrame<double>*>
      body_frames_wo_spr_;

 private:
  void UpdateY(const Eigen::VectorXd& x_w_spr,
               const drake::systems::Context<double>& context_w_spr) final;
  void UpdateYdot(const Eigen::VectorXd& x_w_spr,
                  const drake::systems::Context<double>& context_w_spr) final;
  void UpdateJ(const Eigen::VectorXd& x_wo_spr,
               const drake::systems::Context<double>& context_wo_spr) final;
  void UpdateJdotV(const Eigen::VectorXd& x_wo_spr,
                   const drake::systems::Context<double>& context_wo_spr) final;

  void CheckDerivedOscTrackingData() final;

  // `pt_on_body` is the position w.r.t. the origin of the body
  std::unordered_map<int, Eigen::Vector3d> pts_on_body_;
};

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib