#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "systems/framework/output_vector.h"

/// TimestampedLowpassFilter implements a first order vector-valued linear
/// lowpass filter.  Since the control loops we use generally do not have 
/// a fixed sampling rate, we implement this filter using timestamped vectors
/// to achieve the  desired time constant / cutoff frequency


using Eigen::VectorXd;
using Eigen::MatrixXd;
using drake::systems::LeafSystem;
using drake::systems::Context;

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
  OutputVectorFilter(
      const drake::multibody::MultibodyPlant<double>& plant,
      const std::vector<double>& tau,
      std::optional<std::vector<int>> filter_idxs);

 private:
  void CopyFilterValues(const drake::systems::Context<double> &context,
                  OutputVector<double> *y) const;

  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  std::vector<int> filter_idxs_;
  const int n_y_filt_;
  const std::vector<double>& tau_; // time constant
  int prev_val_idx_;
  int prev_time_idx_;
};
}
