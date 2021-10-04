#include "impact_invariant_tracking_data.h"

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::string;
using std::vector;

using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;

namespace dairlib::systems::controllers {

ImpactInvariantTrackingData::ImpactInvariantTrackingData(
    const string& name, int n_y, int n_ydot, const MatrixXd& K_p,
    const MatrixXd& K_d, const MatrixXd& W,
    const MultibodyPlant<double>& plant_w_spr,
    const MultibodyPlant<double>& plant_wo_spr)
    : OscTrackingData(name, n_y, n_ydot_, K_p, K_d, W, plant_w_spr,
                      plant_wo_spr) {}

void ImpactInvariantTrackingData::UpdateActual(
    const Eigen::VectorXd& x_wo_spr,
    const drake::systems::Context<double>& context_wo_spr) {
  OscTrackingData::UpdateActual(x_wo_spr, context_wo_spr);
  UpdateFilters(context_wo_spr.get_time());
}

void ImpactInvariantTrackingData::UpdateFilters(double t) {
  double dt = t - last_timestamp_;
  double alpha = dt / (dt + tau_);
  filtered_y_ = alpha * y_ + (1 - alpha) * filtered_y_;
  filtered_ydot_ = alpha * ydot_ + (1 - alpha) * filtered_ydot_;

  // Assign filtered values
  for (auto idx : low_pass_filter_element_idx_) {
    y_(idx) = filtered_y_(idx);
    ydot_(idx) = filtered_ydot_(idx);
  }
}

void ImpactInvariantTrackingData::UpdateYdotError() {
  error_ydot_ = ydot_des_ - ydot_;
  if (impact_invariant_projection_) {
    error_ydot_ -= GetJ() * v_proj_;
  }
}

void ImpactInvariantTrackingData::SetLowPassFilter(
    double tau, const std::set<int>& element_idx) {
  DRAKE_DEMAND(tau > 0);
  tau_ = tau;

  DRAKE_DEMAND(n_y_ == n_ydot_);  // doesn't support quaternion yet
  if (element_idx.empty()) {
    for (int i = 0; i < n_y_; i++) {
      low_pass_filter_element_idx_.insert(i);
    }
  } else {
    low_pass_filter_element_idx_ = element_idx;
  }
}

}  // namespace dairlib::systems::controllers