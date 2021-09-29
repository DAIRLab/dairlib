#pragma once
#include <Eigen/Dense>

using Eigen::VectorXd;

/// Linear discrete lowpass filter to be used when filtering is needed inside a LeafSystem

namespace dairlib {

class InternalLowPassFilter {
 public:

  // For cutoff frequency w_c (in rad/s), tau = 1/w_c, 
  // For cutoff frequency f in Hz, tau = 1/(2*pi*f) 
  InternalLowPassFilter(double tau, int n_y);
  VectorXd filter(const double timestamp, const VectorXd& y);

  double time() {return prev_time_;}
  VectorXd y() {return prev_y_;}

 private:
  const double tau_;
  double prev_time_;
  VectorXd prev_y_;
};
}
