#include <iostream>
#include "mpc_periodic_residual_manager.h"

using Eigen::MatrixXd;
using Eigen::Matrix;
using Eigen::VectorXd;
using Eigen::Vector3d;

namespace dairlib::systems {

MpcPeriodicResidualManager::MpcPeriodicResidualManager(
    int nknots, const Eigen::MatrixXd &Aref, const Eigen::MatrixXd &Bref,
    const Eigen::VectorXd &bref) {
  for (int i = 0; i < nknots; i++) {
    buf_.push_back({Matrix<double, 12, 15>::Zero(),
                    Matrix<double, 12, 4>::Zero(),
                    Matrix<double, 12, 1>::Zero()});
  }
  n_ = nknots;
}

void MpcPeriodicResidualManager::AddResidualToDynamics(
    const residual_dynamics &res1, const residual_dynamics& res2,
    drake::EigenPtr<MatrixXd> A,
    drake::EigenPtr<MatrixXd> B, drake::EigenPtr<MatrixXd> b) {
  *A = res1.A + res2.A;
  *B = res1.B + res2.B;
  *b = res1.b + res2.b;
}

void MpcPeriodicResidualManager::SetResidualForCurrentKnot(
    const residual_dynamics &dyn) {
  buf_.at(idx_).A = dyn.A;
  buf_.at(idx_).B = dyn.B;
  buf_.at(idx_).b = dyn.b;
  std::cout << "Setting residual " << std::to_string(idx_) << "\n";
}

residual_dynamics MpcPeriodicResidualManager::GetAverageResidualForNextTwoKnots(
    int i) {
  residual_dynamics r1 = GetResidualForKnotFromCurrent(i);
  residual_dynamics r2 = GetResidualForKnotFromCurrent(i+1);
  return residual_dynamics {0.5*(r1.A + r2.A), 0.5*(r1.B + r2.B),
                            0.5*(r1.b+r2.b)};
}

residual_dynamics MpcPeriodicResidualManager::GetResidualForKnotFromCurrent(int i) {
  int idx = (idx_ + i) % (n_ - idx_);
  std::cout << "returning residual " << std::to_string(idx) << "\n";
  return GetResidualForKnot(idx);
}

}