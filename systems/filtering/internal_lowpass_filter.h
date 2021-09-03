//
// Created by brian on 9/3/21.
//

#pragma once
#include <Eigen/Dense>

using Eigen::VectorXd;

/// Linear discrete lowpass filter to be used when filtering is needed inside a LeafSystem

namespace dairlib {

class InternalLowpassFilter {
 public:
  InternalLowpassFilter(double tau, int n_states);
  VectorXd filter(const double timestamp, const VectorXd& x);

 private:
  const double tau_;
  double prev_time_;
  VectorXd prev_y_;
};
}
