#pragma once
#include <Eigen/Dense>

/// Implements a Kalman Filter for a Linear Time Invariant (LTI) system with
/// a variable rate controller by re-discretizing the continuous dynamics at
/// each step

namespace dairlib::systems {

/// parameters for the continuous LTI system
/// \dot{x} = Ax + Bu + w
/// y = Cx + v
/// w is guassian white noise w/ covariance Q
/// v is guassian white noise w/ covariance R
struct KalmanFilterData {
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::MatrixXd C;
  Eigen::MatrixXd Q;
  Eigen::MatrixXd R;
};

class LinearKalmanFilter {
 public:
  /// @param sys The system to kalman filter
  /// @param rate The approximate loop time in seconds - used hueristically to
  /// determine when/if to reinitialize the filter
  /// @param missed_dts_before_reset The number of skipped loops before the
  /// filter is assumed wrong and should be reinitialized. Used in conjunction
  /// with rate
  LinearKalmanFilter(const KalmanFilterData& sys, double rate = .001,
      int missed_dts_before_reset = 100);

  void Initialize(double t, Eigen::VectorXd x, Eigen::MatrixXd P);
  Eigen::VectorXd Update(const KalmanFilterData& sys,
                         const Eigen::VectorXd& u,
                         const Eigen::VectorXd& y, double t);
  [[nodiscard]] Eigen::VectorXd x() const { return x_; };

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
  double rate_;
  int missed_dts_before_reset_;

};
}
