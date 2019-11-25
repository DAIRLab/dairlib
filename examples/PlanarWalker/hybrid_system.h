#pragma once
#include <Eigen/Dense>

template <typename T>
class HybridSystem {
 public:
  HybridSystem(int num_states, int num_inputs, int num_reset_inputs)
      : num_states_(num_states),
        num_inputs_(num_inputs),
        num_reset_inputs_(num_reset_inputs) {}

  virtual Eigen::Matrix<T, Eigen::Dynamic, 1> dynamics(
      double time, Eigen::Matrix<T, Eigen::Dynamic, 1> x,
      Eigen::Matrix<T, Eigen::Dynamic, 1> u) const;

  virtual Eigen::Matrix<T, Eigen::Dynamic, 1> reset(
      double time, Eigen::Matrix<T, Eigen::Dynamic, 1> x,
      Eigen::Matrix<T, Eigen::Dynamic, 1> u) const;

  virtual T inputLimits(Eigen::Matrix<T, Eigen::Dynamic, 1> x,
                           Eigen::Matrix<T, Eigen::Dynamic, 1> u) const;

  virtual T resetInputLimits(Eigen::Matrix<T, Eigen::Dynamic, 1> x,
                                Eigen::Matrix<T, Eigen::Dynamic, 1> s) const;

 protected:
  int num_states_;
  int num_inputs_;
  int num_reset_inputs_;
};

