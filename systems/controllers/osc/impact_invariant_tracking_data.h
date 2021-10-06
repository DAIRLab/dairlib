#pragma once

#include "osc_tracking_data_new.h"

namespace dairlib {
namespace systems {
namespace controllers {

/// ImpactInvariantTrackingData
class ImpactInvariantTrackingData : public OscTrackingData {
 public:
  ImpactInvariantTrackingData(
      const std::string& name, int n_y, int n_ydot, const Eigen::MatrixXd& K_p,
      const Eigen::MatrixXd& K_d, const Eigen::MatrixXd& W,
      const drake::multibody::MultibodyPlant<double>& plant_w_spr,
      const drake::multibody::MultibodyPlant<double>& plant_wo_spr);

  // enable the low pass filter
  void SetLowPassFilter(double tau, const std::set<int>& element_idx = {});



 protected:
  void UpdateActual(
      const Eigen::VectorXd& x_w_spr,
      const drake::systems::Context<double>& context_w_spr) override;
  void UpdateFilters(double t);
  void UpdateYdotError(const Eigen::VectorXd& v_proj) override;

 private:

};

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib