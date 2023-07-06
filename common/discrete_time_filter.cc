#include "discrete_time_filter.h"

#include "drake/common/drake_assert.h"

using Eigen::VectorXd;
namespace dairlib {

void DiscreteTimeFilter::Update(Eigen::VectorXd value){
}
void DiscreteTimeFilter::Reset(){
}

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

}  // namespace dairlib