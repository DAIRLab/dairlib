#pragma once
#include <Eigen/Dense>

/// Implements a Kalman Filter for a Linear Time Invariant (LTI) system with
/// a variable rate controller by re-discretizing the continuous dynamics at
/// each step

namespace dairlib::systems {

struct KalmanFilterData {
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::MatrixXd C;
  Eigen::MatrixXd Q;
  Eigen::MatrixXd R;
};

class LinearKalmanFilter {
 public:
  LinearKalmanFilter(const KalmanFilterData& sys);

  void Initialize(double t, Eigen::VectorXd x, Eigen::MatrixXd P);
  Eigen::VectorXd Update(const KalmanFilterData& sys,
                         const Eigen::VectorXd& u,
                         const Eigen::VectorXd& y, double t);

 protected:
  void Predict(const KalmanFilterData& sys, const Eigen::VectorXd& u, double t);
  void Correct(const KalmanFilterData& sys, const Eigen::VectorXd& y);
  void Rollover(double t){t_ = t;};

  int nx_;
  int nu_;
  int ny_;
  double t_ = 0;
  Eigen::MatrixXd P_;
  Eigen::VectorXd x_;
  double rate_ = .001;
};

}
