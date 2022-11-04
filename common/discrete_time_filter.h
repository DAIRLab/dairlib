#pragma once

#include <Eigen/Core>

namespace dairlib {

class DiscreteTimeFilter {
 public:
  DiscreteTimeFilter() = default;
  virtual ~DiscreteTimeFilter() = default;

  virtual void Reset();
  virtual void Update(Eigen::VectorXd);
  Eigen::VectorXd Value() const {
    return y_;
  }

 protected:
  Eigen::VectorXd y_;

 private:
};

class FirstOrderLowPassFilter : public DiscreteTimeFilter {
 public:
  FirstOrderLowPassFilter(double alpha, int vector_size);
  void Reset() override;
  void Update(Eigen::VectorXd) override;
  void UpdateParameters(double alpha);

 private:
  double alpha_;
};

class ButterworthFilter : public DiscreteTimeFilter {};

}  // namespace dairlib