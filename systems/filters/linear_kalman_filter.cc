#include "linear_kalman_filter.h"
#include "drake/common/drake_assert.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace dairlib::systems {

LinearKalmanFilter::LinearKalmanFilter(const KalmanFilterData& sys, double rate,
                                       int missed_dts_before_reset) :

    nx_(sys.A.cols()), nu_(sys.B.cols()), ny_(sys.C.rows()), rate_(rate),
    missed_dts_before_reset_(missed_dts_before_reset){
  DRAKE_DEMAND(sys.A.rows() == nx_);
  DRAKE_DEMAND(sys.B.rows() == nx_);
  DRAKE_DEMAND(sys.C.cols() == nx_);
  DRAKE_DEMAND(sys.Q.rows() == nx_ && sys.Q.cols() == nx_);
  DRAKE_DEMAND(sys.R.rows() == ny_ && sys.R.cols() == ny_);

  Initialize(0, VectorXd::Zero(nx_), sys.Q);
}

void LinearKalmanFilter::Initialize(double t, VectorXd x, MatrixXd P) {
  t_ = t;
  x_ = x;
  P_ = P;
}

void LinearKalmanFilter::Predict(
    const KalmanFilterData& sys, const VectorXd& u, double t) {
  double dt  = t - t_;

  if (dt > missed_dts_before_reset_* rate_) {
    dt = rate_;
    Initialize(t, VectorXd::Zero(nx_), sys.Q);
  }

  Eigen::MatrixXd A = MatrixXd::Identity(nx_, nx_) + dt * sys.A;
  Eigen::MatrixXd B = dt * sys.B;

  x_ = A*x_ + B*u;
  P_ = A*P_*A.transpose() + sys.Q;
}

void LinearKalmanFilter::Correct(
    const KalmanFilterData& sys, const VectorXd& y) {
  // Calculate Kalman Gain
  MatrixXd K = P_*sys.C.transpose()*(
      sys.C*P_ * sys.C.transpose() + sys.R).inverse();
  // Update state estimate
  x_ += K * (y - sys.C * x_);
  // Update covariance estimate
  MatrixXd H = MatrixXd::Identity(nx_, nx_) - K*sys.C;
  P_ = H*P_*H.transpose() + K*sys.R*K.transpose();
}

VectorXd LinearKalmanFilter::Update(const KalmanFilterData& sys,
    const Eigen::VectorXd &u, const Eigen::VectorXd &y, double t) {
  Predict(sys, u, t);
  Correct(sys, y);
  Rollover(t);
  return x_;
}

}