#pragma once
#include "systems/controllers/osc/options_tracking_data.h"

namespace dairlib {
namespace systems {
namespace controllers {
/// ComTrackingData is used when we want to track center of mass trajectory.
class ComTrackingData final : public OptionsTrackingData {
 public:
  ComTrackingData(const std::string& name, const Eigen::MatrixXd& K_p,
                  const Eigen::MatrixXd& K_d, const Eigen::MatrixXd& W,
                  const drake::multibody::MultibodyPlant<double>& plant);

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

  void CheckDerivedOscTrackingData() final {}
};

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib