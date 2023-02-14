#pragma once

#include "systems/framework/output_vector.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

/// OutputVectorFilter implements a first order vector-valued linear
/// lowpass filter.  Since the control loops we use generally do not have
/// a fixed sampling rate, we implement this filter using timestamps
/// to achieve the  desired time constant / cutoff frequency

using drake::systems::Context;
using drake::systems::LeafSystem;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace dairlib::systems {

/// Filter system -
/// tau is the cutoff frequency for each index listed in filter idxs
/// n_y is the length of the base vector that will be filtered
/// filter_idxs is the indexes of the input vector to be filtered
/// Note: If filter_idxs is not supplied, then the length of tau must be n_y
class OutputVectorFilter : public LeafSystem<double> {
 public:
  // For cutoff frequency w_c (in rad/s), tau = 1/w_c,
  // For cutoff frequency f in Hz, tau = 1/(2*pi*f)
  OutputVectorFilter(const drake::multibody::MultibodyPlant<double>& plant,
                     const std::vector<double>& tau,
                     std::optional<std::vector<int>> filter_idxs);

 private:
  void CopyFilterValues(const drake::systems::Context<double>& context,
                        OutputVector<double>* y) const;

  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  std::vector<int> filter_idxs_;
  const int n_y_filt_;
  const std::vector<double>& tau_;
  int prev_val_idx_;
  int prev_time_idx_;
};

namespace filtering_utils {

struct CascadedFilter {
  Eigen::MatrixXd A_;
  Eigen::VectorXd B_;
  Eigen::VectorXd UpdateFilter(const Eigen::VectorXd& x, double u) const {
    return A_ * x + B_ * u;
  };
  static double GetFilterOutput(const Eigen::VectorXd& x) {
    return x.tail(1)(0);
  }
};

CascadedFilter butter(int order, double w_c);
inline CascadedFilter butter(int order, double f_s, double f_c) {
  DRAKE_DEMAND(f_s > f_c);
  return butter(order, 0.5 * f_c / f_s);
}
}

class OutputVectorButterworthFilter : public LeafSystem<double> {
 public:
  OutputVectorButterworthFilter(
      const drake::multibody::MultibodyPlant<double>& plant,
      int order,
      double sampling_frequency,
      const std::vector<double>& w_c,
      std::optional<std::vector<int>> filter_idxs);

 private:
  const int order_;
  drake::systems::DiscreteStateIndex filter_state_idx_;
  std::vector<int> filter_idxs_;
  std::unordered_map<int, filtering_utils::CascadedFilter> index_to_filter_map_;
  void CopyFilterValues(const drake::systems::Context<double>& context,
                        OutputVector<double>* y) const;

  drake::systems::EventStatus UnrestrictedUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const;

};
}  // namespace dairlib::systems
