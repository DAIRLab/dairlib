#pragma once
#include "linear_kalman_filter.h"

/// Kalman filter specialization with step-to-step dynamics.
/// Representing a linear system xdot = Ax during stance
/// With reset map Ax + Bu

namespace dairlib::systems {

struct S2SKalmanFilterData: KalmanFilterData {
  Eigen::MatrixXd G;
};

class S2SKalmanFilter : public LinearKalmanFilter {
 public:
  S2SKalmanFilter(const S2SKalmanFilterData& sys) :
                  LinearKalmanFilter(sys) {};
 private:
  void Predict(const S2SKalmanFilterData& sys,
      const Eigen::VectorXd& u, double t) {
    double dt  = t - t_;
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(nx_, nx_) + dt * sys.A;
    if (u.transpose() * u == 0) {
      x_ = A*x_;
    } else {
      x_ = (A + sys.G)*x_ + sys.B*u;
    }
    P_ = A*P_*A.transpose() + sys.Q;
  }
};
}