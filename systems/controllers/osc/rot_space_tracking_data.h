#pragma once

#include "options_tracking_data.h"

namespace dairlib {
namespace systems {
namespace controllers {

/// RotTaskSpaceTrackingData is used when we want to track a trajectory
/// (rotational position) in the task space. The desired position must be
/// expressed in quaternion (a 4d vector).

/// AddFrameToTrack() should be called to specify what is the frame that
/// follows the desired trajectory

/// If users want to track the trajectory only in some states of the finite
/// state machine, they should use AddStateAndFrameToTrack().
/// Also, at most one point (of the body) can follow the desired trajectory, so
/// state_ elements can not repeat, and the length of state_ must be the same as
/// frame_pose_'s if state_ is not empty.
/// This also means that AddFrameToTrack and AddStateAndFrameToTrack cannot be
/// called one after another for the same TrackingData.
class RotTaskSpaceTrackingData final : public OptionsTrackingData {
 public:
  RotTaskSpaceTrackingData(
      const std::string& name, const Eigen::MatrixXd& K_p,
      const Eigen::MatrixXd& K_d, const Eigen::MatrixXd& W,
      const drake::multibody::MultibodyPlant<double>& plant_w_spr,
      const drake::multibody::MultibodyPlant<double>& plant_wo_spr);

  void AddFrameToTrack(
      const std::string& body_name,
      const Eigen::Isometry3d& frame_pose = Eigen::Isometry3d::Identity());
  void AddStateAndFrameToTrack(
      int state, const std::string& body_name,
      const Eigen::Isometry3d& frame_pose = Eigen::Isometry3d::Identity());

 protected:
  std::unordered_map<int, const drake::multibody::BodyFrame<double>*>
      body_frames_w_spr_;
  std::unordered_map<int, const drake::multibody::BodyFrame<double>*>
      body_frames_wo_spr_;

 private:
  void UpdateY(const Eigen::VectorXd& x_w_spr,
               const drake::systems::Context<double>& context_w_spr) final;
  void UpdateYError() final;
  void UpdateYdot(const Eigen::VectorXd& x_w_spr,
                  const drake::systems::Context<double>& context_w_spr) final;
  void UpdateYdotError(const Eigen::VectorXd& v_proj) final;
  void UpdateJ(const Eigen::VectorXd& x_wo_spr,
               const drake::systems::Context<double>& context_wo_spr) final;
  void UpdateJdotV(const Eigen::VectorXd& x_wo_spr,
                   const drake::systems::Context<double>& context_wo_spr) final;
  void UpdateYddotDes(double t, double t_since_state_switch) override;
  void CheckDerivedOscTrackingData() final;

  // frame_pose_ represents the pose of the frame (w.r.t. the body's frame)
  // which follows the desired rotation.
  std::unordered_map<int, Eigen::Isometry3d> frame_poses_;
};

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib