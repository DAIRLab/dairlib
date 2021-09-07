#pragma once
#include <Eigen/Dense>

using Eigen::VectorXd;

/// Linear discrete lowpass filter to be used when filtering is needed inside a LeafSystem

namespace dairlib {

class InternalLowPassFilter {
 public:
  InternalLowPassFilter(double tau, int n_y);
  VectorXd filter(const double timestamp, const VectorXd& y);

 private:
  const double tau_;
  double prev_time_;
  VectorXd prev_y_;
};
}
