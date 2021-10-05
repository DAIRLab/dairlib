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

  // Set whether or not to use the impact invariant projection
  void SetImpactInvariantProjection(bool use_impact_invariant_projection) {
    impact_invariant_projection_ = use_impact_invariant_projection;
  }

 protected:
  void UpdateActual(
      const Eigen::VectorXd& x_w_spr,
      const drake::systems::Context<double>& context_w_spr) override;
  void UpdateFilters(double t);
  void UpdateYdotError() override;

 private:
  bool impact_invariant_projection_ = false;
  Eigen::VectorXd v_proj_;

  // Members of low-pass filter
  Eigen::VectorXd filtered_y_;
  Eigen::VectorXd filtered_ydot_;
  double tau_ = -1;
  std::set<int> low_pass_filter_element_idx_;
  double last_timestamp_ = -1;
};

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib