#pragma once

#include <iostream>
#include "math.h"
#include <Eigen/Dense>
#include "drake/math/autodiff_gradient.h"
#include "drake/common/eigen_types.h"

#include "drake/common/drake_assert.h"

using Eigen::Matrix;
using Eigen::Dynamic;

using std::cout;
using std::endl;
using Eigen::VectorXd;
using Eigen::MatrixXd;

using drake::MatrixX;
using drake::VectorX;
using drake::AutoDiffXd;
using drake::math::DiscardGradient;


namespace dairlib {
namespace goldilocks_models {

class DynamicsExpression {
 public:
  explicit DynamicsExpression(int n_sDDot, int n_feature_sDDot,
                              int robot_option);
  explicit DynamicsExpression(int n_sDDot, int n_feature_sDDot, MatrixXd B_tau,
                              int robot_option);
  DynamicsExpression() {}  // Default constructor

  int getDimFeature();

  template <typename U, typename V>
  V getExpression(const U & theta, const V & s, const V & ds,
                  const V & tau) const;

  template <typename T>
  T getFeature(const T & s, const T & ds) const;

 private:
  int n_feature_sDDot_;
  int n_sDDot_;
  MatrixXd B_tau_;
  int robot_option_;
};

}  // namespace goldilocks_models
}  // namespace dairlib

