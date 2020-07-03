#pragma once

#include "systems/goldilocks_models/file_utils.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

#include <iostream>
#include <string>
#include <Eigen/Dense>

using drake::trajectories::PiecewisePolynomial;
using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::shared_ptr;
using std::string;
using std::to_string;
using std::vector;

namespace dairlib {
namespace goldilocks_models {

class RomData {
 public:
  RomData(int n_y, int n_tau, int n_feature_y, int n_feature_yddot);
  RomData(int n_y, int n_tau, int n_feature_y, int n_feature_yddot,
          const MatrixXd& B_tau, const VectorXd& theta_y,
          const VectorXd& theta_yddot);

  void CheckDataConsistency() const;

  // Getters
  int n_y() const { return n_y_; };
  int n_yddot() const { return n_y_; };
  int n_tau() const { return n_tau_; };
  int n_feature_y() const { return n_feature_y_; };
  int n_feature_yddot() const { return n_feature_yddot_; };
  const MatrixXd& B() const { return B_tau_; };
  const VectorXd& theta_y() const { return theta_y_; };
  const VectorXd& theta_yddot() const { return theta_yddot_; };
  VectorXd theta() const {
    VectorXd ret(theta_y_.size() + theta_yddot_.size());
    ret << theta_y_, theta_yddot_;
    return ret;
  };
  int n_theta_y() const { return theta_y_.size(); };
  int n_theta_yddot() const { return theta_yddot_.size(); };
  int n_theta() const { return theta_y_.size() + theta_yddot_.size(); };

  // Setters
  void set_n_y(int n_y) { n_y_ = n_y; };
  void set_n_yddot(int n_yddot) { n_yddot_ = n_yddot; };
  void set_n_tau(int n_tau) { n_tau_ = n_tau; };
  void set_n_feature_y(int n_feature_y) { n_feature_y_ = n_feature_y; };
  void set_n_feature_yddot(int n_feature_yddot) {
    n_feature_yddot_ = n_feature_yddot;
  };
  void SetB(const MatrixXd& B_tau);
  void SetThetaY(const VectorXd& theta_y);
  void SetThetaYddot(const VectorXd& theta_yddot);
  void SetTheta(const VectorXd& theta);

 private:
  int n_y_;
  int n_yddot_;
  int n_tau_;
  int n_feature_y_;
  int n_feature_yddot_;

  MatrixXd B_tau_;
  VectorXd theta_y_;
  VectorXd theta_yddot_;
};

}  // namespace goldilocks_models
}  // namespace dairlib
