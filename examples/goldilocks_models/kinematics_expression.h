#pragma once

#include <iostream>
#include <string>
#include <vector>
#include "math.h"
#include <Eigen/Dense>
#include "drake/math/autodiff_gradient.h"
#include "drake/common/eigen_types.h"

#include "multibody/multibody_utils.h"

#include "drake/common/drake_assert.h"

using Eigen::Matrix;
using Eigen::Dynamic;

using std::cout;
using std::endl;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

using drake::MatrixX;
using drake::VectorX;
using drake::AutoDiffXd;
using drake::math::DiscardGradient;

using drake::multibody::MultibodyPlant;

namespace dairlib {
namespace goldilocks_models {

template <typename T>
class KinematicsExpression {
 public:
  explicit KinematicsExpression(int n_s, int n_feature,
                                int robot_option);
  explicit KinematicsExpression(int n_s, int n_feature,
                                const MultibodyPlant<T> * plant,
                                int robot_option);
  KinematicsExpression() {}  // Default constructor

  int getDimFeature();

  template <typename U, typename V>
  VectorX<T> getExpression(const U & theta, const V & q) const;
  template <typename U, typename V>
  VectorX<T> getExpressionDot(const U & theta, const V & q, const V & v) const;

  template <typename U>
  VectorX<U> getFeature(const VectorX<U> & q) const;
  template <typename U>
  VectorX<U> getFeatureDot(const VectorX<U> & q, const VectorX<U> & v) const;

  void setModelDimension(int n_s_new) {n_s_ = n_s_new;};

 private:
  const MultibodyPlant<T> * plant_;
  std::unique_ptr<drake::systems::Context<T>> context_;
  int n_feature_;
  int n_q_;
  int n_s_;
  int robot_option_;

  // robot0
  const std::vector<std::string> leg_link_names_{
    "left_upper_leg_mass", "left_lower_leg_mass",
    "right_upper_leg_mass", "right_lower_leg_mass"};
  Vector3d mass_disp_ = Vector3d(0, 0, -0.25);
  Vector3d foot_disp_ = Vector3d(0, 0, -0.5);

  // robot1
  VectorX<T> pt_front_contact_ = VectorX<T>::Zero(3);
  VectorX<T> pt_rear_contact_ = VectorX<T>::Zero(3);
  VectorX<T> mid_disp_ = VectorX<T>::Zero(3);
};

}  // namespace goldilocks_models
}  // namespace dairlib

