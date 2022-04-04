#pragma once
#include <Eigen/Dense>

/// Implements a Kalman Filter for a Linear Time Invariant (LTI) system with
/// a variable rate controller by re-discretizing the continuous dynamics at
/// each step

namespace dairlib::systems {

class LinearKalmanFilter {
 public:
  LinearKalmanFilter(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C,
                     Eigen::MatrixXd Q, Eigen::MatrixXd R);
  Eigen::VectorXd getState();

 private:
  double t;
  Eigen::MatrixXd P;
  Eigen::VectorXd xhat;
};

}
