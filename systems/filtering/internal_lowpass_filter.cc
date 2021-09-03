#include "internal_lowpass_filter.h"

using Eigen::VectorXd;

namespace dairlib {

InternalLowpassFilter::InternalLowpassFilter(double tau, int n_states) : tau_(tau) {
  prev_y_ = VectorXd::Zero(n_states);
  prev_time_ = -1;
}


VectorXd InternalLowpassFilter::filter(double timestamp, const VectorXd& x){
  if (prev_time_ < 0) {
    prev_time_ = timestamp;
    prev_y_ = x;
    return prev_y_;
  }

  double dt = timestamp - prev_time_;
  double alpha = dt / (dt + tau_);

  prev_y_ = alpha * x + (1.0 - alpha) *  prev_y_;
  prev_time_ = timestamp;

  return prev_y_;
}

}