#include "discrete_time_filter.h"

#include "drake/common/drake_assert.h"

using Eigen::VectorXd;
namespace dairlib {

void DiscreteTimeFilter::Update(Eigen::VectorXd value) {}
void DiscreteTimeFilter::Reset() {}

FirstOrderLowPassFilter::FirstOrderLowPassFilter(double alpha, int vector_size)
    : alpha_(alpha) {
  DRAKE_DEMAND(alpha <= 1);
  DRAKE_DEMAND(alpha >= 0);
  y_.resize(vector_size);
  y_.setZero();
}

void FirstOrderLowPassFilter::Reset() { y_.setZero(); }

void FirstOrderLowPassFilter::Update(VectorXd value) {
  DRAKE_DEMAND(value.size() == y_.size());
  y_ = alpha_ * value + (1 - alpha_) * y_;
}

void FirstOrderLowPassFilter::UpdateParameters(double alpha) {
  DRAKE_DEMAND(alpha <= 1);
  DRAKE_DEMAND(alpha >= 0);
  alpha_ = alpha;
}

ButterworthFilter::ButterworthFilter(VectorXd& a, VectorXd& b,
                                     int vector_size) {
  data_size_ = vector_size;
  this->nb_ = b.size();
  this->na_ = a.size();
  this->b_ = b;
  this->a_ = a;
  this->y_.resize(vector_size, na_);
  this->y_.setZero();
  this->x_.resize(vector_size, nb_);
  this->x_.setZero();
}

void ButterworthFilter::Reset() {
  y_.setZero();
  x_.setZero();
}

void ButterworthFilter::Update(VectorXd value) {
  DRAKE_DEMAND(value.size() == data_size_);
  // Modified code from Jenna Reher's smoothing.cpp

  // Shift the data back and append the raw
  for (long i = nb_ - 1; i > 0; i--) {
    x_.col(i) = x_.col(i - 1);
  }
  for (long i = na_ - 1; i > 0; i--) {
    y_.col(i) = y_.col(i - 1);
  }
  x_.col(0) = value;

  // Get the solution -- MATLAB pseudocode...
  // a(1)*y(n) = b(1)*x(n) + b(2)*x(n-1) + ... + b(nb+1)*x(n-nb)
  //                         - a(2)*y(n-1) - ... - a(na+1)*y(n-na)
  VectorXd y = VectorXd::Zero(data_size_);
  for (long i = 0; i < nb_; i++) {
    y += b_(i) * x_.col(i);
  }
  for (long i = 1; i < na_; i++) {
    y -= a_(i) * y_.col(i);
  }

  // Multiply solution... a(0) should be 1 from MATLAB
  y *= a_(0);
  y_.col(0) = y;
}

void ButterworthFilter::UpdateParameters(Eigen::VectorXd &b, Eigen::VectorXd &a) {
  DRAKE_DEMAND(b.size() == nb_);
  DRAKE_DEMAND(a.size() == na_);
  this->b_ = b;
  this->a_ = a;
}

}  // namespace dairlib