#include "solvers/lcs.h"

#include <functional>
#include <iostream>

#include <optional>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/moby_lcp_solver.h"
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

namespace dairlib {
namespace solvers {

LCS::LCS(const vector<MatrixXd>& A, const vector<MatrixXd>& B,
         const vector<MatrixXd>& D, const vector<VectorXd>& d,
         const vector<MatrixXd>& E, const vector<MatrixXd>& F,
         const vector<MatrixXd>& H, const vector<VectorXd>& c, double dt,
         bool rescale)
    : A_(A),
      B_(B),
      D_(D),
      d_(d),
      E_(E),
      F_(F),
      H_(H),
      c_(c),
      N_(A_.size()),
      dt_(dt),
      n_(A_[0].rows()),
      m_(D_[0].cols()),
      k_(H_[0].cols()) {
  if (rescale) {
    Rescale();
  } else {
    AnDn_.resize(N_);
    for (int i = 0; i < N_; ++i) {
      AnDn_.at(i) = 1;
    }
  }
}

LCS::LCS(const vector<MatrixXd>& A, const vector<MatrixXd>& B,
         const vector<MatrixXd>& D, const vector<VectorXd>& d,
         const vector<MatrixXd>& E, const vector<MatrixXd>& F,
         const vector<MatrixXd>& H, const vector<VectorXd>& c,
         std::optional<vector<MatrixXd>> K, double dt, bool rescale)
    : LCS(A, B, D, d, E, F, H, c, dt, rescale) {
  if (K.has_value()) {
    K_ = K.value();
  }
}

LCS::LCS(const MatrixXd& A, const MatrixXd& B, const MatrixXd& D,
         const VectorXd& d, const MatrixXd& E, const MatrixXd& F,
         const MatrixXd& H, const VectorXd& c, const int& N, double dt,
         bool rescale)
    : LCS(vector<MatrixXd>(N, A), vector<MatrixXd>(N, B),
          vector<MatrixXd>(N, D), vector<VectorXd>(N, d),
          vector<MatrixXd>(N, E), vector<MatrixXd>(N, F),
          vector<MatrixXd>(N, H), vector<VectorXd>(N, c), dt, rescale) {}

LCS::LCS(const MatrixXd& A, const MatrixXd& B, const MatrixXd& D,
         const VectorXd& d, const MatrixXd& E, const MatrixXd& F,
         const MatrixXd& H, const VectorXd& c, std::optional<MatrixXd> K,
         const int& N, double dt, bool rescale)
    : LCS(A, B, D, d, E, F, H, c, N, dt, rescale) {
  if (K.has_value()) {
    K_ = vector<MatrixXd>(N, K.value());
  }
}

LCS::LCS(const LCS& lcs)
    : A_(lcs.A_),
      B_(lcs.B_),
      D_(lcs.D_),
      d_(lcs.d_),
      E_(lcs.E_),
      F_(lcs.F_),
      H_(lcs.H_),
      c_(lcs.c_),
      K_(lcs.K_),
      AnDn_(lcs.AnDn_),
      W_x_(lcs.W_x_),
      W_l_(lcs.W_l_),
      W_u_(lcs.W_u_),
      w_(lcs.w_),
      has_tangent_linearization_(lcs.has_tangent_linearization_),
      N_(lcs.N_),
      dt_(lcs.dt_),
      n_(lcs.n_),
      m_(lcs.m_),
      k_(lcs.k_) {}

LCS& LCS::operator=(const LCS& lcs) {
  N_ = lcs.N_;
  dt_ = lcs.dt_;
  n_ = lcs.n_;
  m_ = lcs.m_;
  k_ = lcs.k_;
  A_ = lcs.A_;
  B_ = lcs.B_;
  D_ = lcs.D_;
  d_ = lcs.d_;
  E_ = lcs.E_;
  F_ = lcs.F_;
  H_ = lcs.H_;
  c_ = lcs.c_;
  K_ = lcs.K_;
  AnDn_ = lcs.AnDn_;
  W_x_ = lcs.W_x_;
  W_l_ = lcs.W_l_;
  W_u_ = lcs.W_u_;
  w_ = lcs.w_;
  has_tangent_linearization_ = lcs.has_tangent_linearization_;
  return *this;
}

const VectorXd LCS::Simulate(VectorXd& x_init, VectorXd& input) {
  VectorXd x_final;
  // calculate force
  drake::solvers::MobyLCPSolver<double> LCPSolver;
  VectorXd force;

  auto flag = LCPSolver.SolveLcpLemke(
      F_[0], E_[0] * x_init + c_[0] + H_[0] * input, &force);
  // update
  x_final = A_[0] * x_init + B_[0] * input + D_[0] * force + d_[0];
  return x_final;
}

void LCS::Rescale() {
  // Scaling dynamics matrices constraint for better numerics
  // This only scales lambda
  AnDn_.resize(N_);
  for (int i = 0; i < N_; ++i) {
    double Dn;
    if (!K_.has_value()) {
      Dn = D_.at(i).norm();
    } else {
      Dn = (D_.at(i) * K_.value().at(i)).norm();
    }
    double An = A_[i].norm();
    AnDn_.at(i) = An / Dn;
    D_.at(i) *= AnDn_.at(i);
    E_.at(i) /= AnDn_.at(i);
    c_.at(i) /= AnDn_.at(i);
    H_.at(i) /= AnDn_.at(i);
  }
}

}  // namespace solvers
}  // namespace dairlib
