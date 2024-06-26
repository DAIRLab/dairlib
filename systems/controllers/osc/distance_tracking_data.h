#pragma once

#include "options_tracking_data.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

namespace dairlib {
namespace systems {
namespace controllers {
/// DistanceTrackingData is used when we want to track the distance between two
/// translational tracking data.
///
/// We built this class on top of two other OscTrackingData's. The user have to
/// provide `to_frame_data` and `from_frame_data`.
/// The resulting relative pos is
///    ||position(to_frame_data) - position(from_frame_data)||_2,
///
/// Developer notes: the current implementation is not the cleanest, since we
/// have to add a PreUpdate() to make this work.
class DistanceTrackingData final : public OptionsTrackingData {
 public:
  DistanceTrackingData(
      const std::string& name, const Eigen::MatrixXd& K_p,
      const Eigen::MatrixXd& K_d, const Eigen::MatrixXd& W,
      const drake::multibody::MultibodyPlant<double>& plant,
      OptionsTrackingData* to_frame_data, OptionsTrackingData* from_frame_data);

  void Update(const Eigen::VectorXd& x,
              const drake::systems::Context<double>& context,
              const drake::trajectories::Trajectory<double>& traj, double t,
              double t_gait_cycle, int fsm_state,
              const Eigen::VectorXd& v_proj,
              OscTrackingDataState& tracking_data_state) const final;

 private:
  void UpdateY(const Eigen::VectorXd& x,
               const drake::systems::Context<double>& context,
               OscTrackingDataState& tracking_data_state) const final;
  void UpdateYdot(const Eigen::VectorXd& x,
                  const drake::systems::Context<double>& context,
                  OscTrackingDataState& tracking_data_state) const final;
  void UpdateJ(const Eigen::VectorXd& x,
               const drake::systems::Context<double>& context,
               OscTrackingDataState& tracking_data_state) const final;
  void UpdateJdotV(const Eigen::VectorXd& x,
                   const drake::systems::Context<double>& context,
                   OscTrackingDataState& tracking_data_state) const final;

  void CheckDerivedOscTrackingData() final;

  OptionsTrackingData* to_frame_data_;
  OptionsTrackingData* from_frame_data_;

  drake::trajectories::PiecewisePolynomial<double> dummy_traj_{Eigen::Vector3d::Zero()};

  // Used to update to and from tracking data,
  // but all persistent state is stored in external state
  mutable OscTrackingDataState to_state_;
  mutable OscTrackingDataState from_state_;
};

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib