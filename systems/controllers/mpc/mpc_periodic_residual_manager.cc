#include "mpc_periodic_residual_manager.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;

namespace dairlib::systems {

MpcPeriodicResidualManager::MpcPeriodicResidualManager(
    int nknots, const Eigen::MatrixXd &Aref, const Eigen::MatrixXd &Bref,
    const Eigen::VectorXd &bref) {
  for (int i = 0; i < nknots; i++) {
    buf_.push_back({MatrixXd::Zero(Aref.rows(), Aref.cols()),
                    MatrixXd::Zero(Bref.rows(), Bref.cols()),
                    VectorXd::Zero(bref.size())});
  }
}

void MpcPeriodicResidualManager::AddResidualToDynamics(
    const residual_dynamics &res, drake::EigenPtr<MatrixXd> A,
    drake::EigenPtr<MatrixXd> B, drake::EigenPtr<MatrixXd> b,
    bool stance, const Vector3d& pos = Vector3d::Zero()) {

  if (stance) {
    *A = *A + res.A.block(0, 0, res.A.rows(), res.A.cols() - 3);
    *b = *b + res.b + res.A.block(0, res.A.cols() - 3, res.A.rows(), 3) * pos;
  } else {
    *A = *A + res.A;
    *b = *b + res.b;
  }
  *B = *B + res.B;
}

void MpcPeriodicResidualManager::SetResidualForCurrentKnot(
    const residual_dynamics &dyn) {
  buf_.at(idx_).A = dyn.A;
  buf_.at(idx_).B = dyn.B;
  buf_.at(idx_).b = dyn.b;
}

residual_dynamics MpcPeriodicResidualManager::GetResidualForKnotFromCurrent(int i) {
  int idx = (idx_ + i) % i;
  return GetResidualForKnot(idx);
}

}