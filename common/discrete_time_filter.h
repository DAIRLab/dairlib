#pragma once

#include <Eigen/Core>

namespace dairlib {

/// Abstract class that serves as the base structure of any discrete time filter
/// that would exist better outside of the drake LeafSystem framework.
/// Any discrete time filter that inherits from this class is responsible for
/// calling the Update() function. Therefore, a equivalent "frequency" may not
/// exist.
class DiscreteTimeFilter {
 public:
  DiscreteTimeFilter() = default;
  virtual ~DiscreteTimeFilter() = default;

  virtual void Reset();
  virtual void Update(Eigen::VectorXd);
  Eigen::VectorXd Value() const { return y_; }

 protected:
  Eigen::VectorXd y_;

 private:
};

/// DiscreteTimeFilter with update law:
/// y(t+1) = alpha * z(t+1) + (1 - alpha) * y(t), where z(t+1) is the update
/// value. alpha \in [0, 1], where alpha = 1 is equivalent to no filter.
class FirstOrderLowPassFilter : public DiscreteTimeFilter {
 public:
  FirstOrderLowPassFilter(double alpha, int vector_size);
  void Reset() override;
  void Update(Eigen::VectorXd) override;
  void UpdateParameters(double alpha);

 private:
  double alpha_;
};

class ButterworthFilter : public DiscreteTimeFilter {
 public:
  ButterworthFilter(Eigen::VectorXd& a, Eigen::VectorXd& b, int vector_size);
  void Reset() override;
  void Update(Eigen::VectorXd) override;
  void UpdateParameters(Eigen::VectorXd &b, Eigen::VectorXd &a);

 private:
  int nb_;
  int na_;
  int data_size_;
  Eigen::MatrixXd y_; // Array of previously filtered values
  Eigen::MatrixXd x_; // Array of raw values
  Eigen::VectorXd b_; // filter coefficient
  Eigen::VectorXd a_; // filter coefficient
};

}  // namespace dairlib