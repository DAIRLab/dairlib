#include "internal_lowpass_filter.h"

using Eigen::VectorXd;

namespace dairlib {

InternalLowpassFilter::InternalLowpassFilter(double tau, int n_y) : tau_(tau) {
  prev_y_ = VectorXd::Zero(n_y);
  prev_time_ = -1;
}


VectorXd InternalLowpassFilter::filter(double timestamp, const VectorXd& y){
  if (prev_time_ < 0) {
    prev_time_ = timestamp;
    prev_y_ = y;
    return prev_y_;
  }

  double dt = timestamp - prev_time_;
  double alpha = dt / (dt + tau_);

  prev_y_ = alpha * y + (1.0 - alpha) *  prev_y_;
  prev_time_ = timestamp;

  return prev_y_;
}

}