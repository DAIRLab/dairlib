#pragma once

#include "cassie_acom_function.h"
#include "systems/controllers/osc/options_tracking_data.h"

namespace dairlib {
namespace systems {
namespace controllers {

template <typename T>
drake::MatrixX<T> E_from_Quat(const drake::VectorX<T>& Q);
template <typename T>
drake::MatrixX<T> R_from_Quat(const drake::VectorX<T>& Q);
template <typename T>
drake::MatrixX<T> EvalJOmegaBaseAcomEwrtAcom(const drake::VectorX<T>& q);
template <typename T>
drake::MatrixX<T> EvalJOmegaWorldAcomEwrtWorld(const drake::VectorX<T>& q);

Eigen::MatrixXd MapWToQuatDot(const Eigen::Vector4d& Q);

/// AcomTrackingData is a variation of RotTaskSpaceTrackingData that tracks the
/// angular center of mass instead of the orientation of a body. The desired
/// position must be expressed in quaternion (a 4d vector).

/// AddFrameToTrack() should be called to specify what is the frame that
/// follows the desired trajectory

/// If users want to track the trajectory only in some states of the finite
/// state machine, they should use AddStateAndFrameToTrack().
/// Also, at most one point (of the body) can follow the desired trajectory, so
/// state_ elements can not repeat, and the length of state_ must be the same as
/// frame_pose_'s if state_ is not empty.
/// This also means that AddFrameToTrack and AddStateAndFrameToTrack cannot be
/// called one after another for the same TrackingData.
class CassieAcomTrackingData final : public OptionsTrackingData {
 public:
  CassieAcomTrackingData(
      const std::string& name, const Eigen::MatrixXd& K_p,
      const Eigen::MatrixXd& K_d, const Eigen::MatrixXd& W,
      const drake::multibody::MultibodyPlant<double>& plant);

  void AddStateToTrack(
      int state, const Eigen::Isometry3d& frame_pose = Eigen::Isometry3d::Identity());

 protected:

 private:
  void UpdateY(const Eigen::VectorXd& x,
               const drake::systems::Context<double>& context,
               OscTrackingDataState& tracking_data_state) const final;
  void UpdateYError(OscTrackingDataState& tracking_data_state) const final;
  void UpdateYdot(const Eigen::VectorXd& x,
                  const drake::systems::Context<double>& context,
                  OscTrackingDataState& tracking_data_state) const final;
  void UpdateYdotError(const Eigen::VectorXd& v_proj,
                       OscTrackingDataState& tracking_data_state) const final;
  void UpdateJ(const Eigen::VectorXd& x,
               const drake::systems::Context<double>& context,
               OscTrackingDataState& tracking_data_state) const final;
  void UpdateJdotV(const Eigen::VectorXd& x,
                   const drake::systems::Context<double>& context,
                   OscTrackingDataState& tracking_data_state) const final;
  void UpdateYddotDes(double t, double t_since_state_switch,
                      OscTrackingDataState& tracking_data_state) const override;
  void CheckDerivedOscTrackingData() final;

  // frame_pose_ represents the pose of the frame (w.r.t. the body's frame)
  // which follows the desired rotation.
  std::unordered_map<int, Eigen::Isometry3d> frame_poses_;
};

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib