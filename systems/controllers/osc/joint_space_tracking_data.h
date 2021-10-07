#pragma once

#include "options_tracking_data.h"

namespace dairlib {
namespace systems {
namespace controllers {
/// JointSpaceTrackingData is used when we want to track a trajectory
/// in the joint space.

/// AddJointToTrack() should be called to specify which joint to track.
/// Note that one instance of `JointSpaceTrackingData` allows to track 1 joint.

/// If users want to track the trajectory only in some states of the finite
/// state machine, they should use AddStateAndJointToTrack().
/// Also, at most one point (of the body) can follow the desired trajectory, so
/// state_ elements can not repeat, and the length of state_ must be the same as
/// joint_idx's if state_ is not empty.
/// This also means that AddJointToTrack and AddStateAndJointToTrack cannot be
/// called one after another for the same TrackingData.
class JointSpaceTrackingData final : public OptionsTrackingData {
 public:
  JointSpaceTrackingData(
      const std::string& name, const Eigen::MatrixXd& K_p,
      const Eigen::MatrixXd& K_d, const Eigen::MatrixXd& W,
      const drake::multibody::MultibodyPlant<double>& plant_w_spr,
      const drake::multibody::MultibodyPlant<double>& plant_wo_spr);

  // For single joint
  void AddJointToTrack(const std::string& joint_pos_name,
                       const std::string& joint_vel_name);
  void AddStateAndJointToTrack(int state, const std::string& joint_pos_name,
                               const std::string& joint_vel_name);
  // For multi joints
  void AddJointsToTrack(const std::vector<std::string>& joint_pos_names,
                        const std::vector<std::string>& joint_vel_names);
  void AddStateAndJointsToTrack(
      int state, const std::vector<std::string>& joint_pos_names,
      const std::vector<std::string>& joint_vel_names);

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

  // `joint_pos_idx_wo_spr` is the index of the joint position
  // `joint_vel_idx_wo_spr` is the index of the joint velocity
  std::unordered_map<int, std::vector<int>> joint_pos_idx_w_spr_;
  std::unordered_map<int, std::vector<int>> joint_vel_idx_w_spr_;
  std::unordered_map<int, std::vector<int>> joint_pos_idx_wo_spr_;
  std::unordered_map<int, std::vector<int>> joint_vel_idx_wo_spr_;
};

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib