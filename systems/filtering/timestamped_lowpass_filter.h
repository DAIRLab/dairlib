#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/timestamped_vector.h"

/// TimestampedLowpassFilter implements a first order vector-valued linear 
/// lowpass filter.  Since the control loops we use generally do not have 
/// a fixed sampling rate, we implement this filter using timestamped vectors
/// to achieve the  desired time constant

using Eigen::VectorXd;
using Eigen::MatrixXd;
using drake::systems::LeafSystem;
using drake::systems::Context;

namespace dairlib {

class TimestampedLowpassFilter : public LeafSystem<double> {
 public:
  TimestampedLowpassFilter(double tau, int n_states);

 private:
  void CalcFilter(const drake::systems::Context<double> &context,
                  systems::TimestampedVector<double> *y) const;

  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  const double tau_; // time constant
  int prev_val_idx_;
  int prev_time_idx_;
};
}