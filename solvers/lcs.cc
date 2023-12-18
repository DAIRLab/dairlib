#include "solvers/lcs.h"

#include <iostream>

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
         const vector<MatrixXd>& H, const vector<VectorXd>& c, double dt)
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
      k_(H_[0].cols()) {}

LCS::LCS(const MatrixXd& A, const MatrixXd& B, const MatrixXd& D,
         const VectorXd& d, const MatrixXd& E, const MatrixXd& F,
         const MatrixXd& H, const VectorXd& c, const int& N, double dt)
    : LCS(vector<MatrixXd>(N, A), vector<MatrixXd>(N, B),
          vector<MatrixXd>(N, D), vector<VectorXd>(N, d),
          vector<MatrixXd>(N, E), vector<MatrixXd>(N, F),
          vector<MatrixXd>(N, H), vector<VectorXd>(N, c), dt) {}

LCS::LCS(const LCS& lcs)
    : N_(lcs.N_), dt_(lcs.dt_), n_(lcs.n_), m_(lcs.m_), k_(lcs.k_) {
  A_.resize(N_);
  B_.resize(N_);
  D_.resize(N_);
  d_.resize(N_);
  E_.resize(N_);
  F_.resize(N_);
  H_.resize(N_);
  c_.resize(N_);
  for (int i = 0; i < lcs.N_; ++i) {
    A_.at(i) = lcs.A_.at(i);
    B_.at(i) = lcs.B_.at(i);
    D_.at(i) = lcs.D_.at(i);
    d_.at(i) = lcs.d_.at(i);
    E_.at(i) = lcs.E_.at(i);
    F_.at(i) = lcs.F_.at(i);
    H_.at(i) = lcs.H_.at(i);
    c_.at(i) = lcs.c_.at(i);
  }
}

LCS& LCS::operator=(const LCS& lcs) {
  N_ = lcs.N_;
  dt_ = lcs.dt_;
  n_ = lcs.n_;
  m_ = lcs.m_;
  k_ = lcs.k_;
  A_.resize(N_);
  B_.resize(N_);
  D_.resize(N_);
  d_.resize(N_);
  E_.resize(N_);
  F_.resize(N_);
  H_.resize(N_);
  c_.resize(N_);
  for (int i = 0; i < lcs.N_; ++i) {
    A_.at(i) = lcs.A_.at(i);
    B_.at(i) = lcs.B_.at(i);
    D_.at(i) = lcs.D_.at(i);
    d_.at(i) = lcs.d_.at(i);
    E_.at(i) = lcs.E_.at(i);
    F_.at(i) = lcs.F_.at(i);
    H_.at(i) = lcs.H_.at(i);
    c_.at(i) = lcs.c_.at(i);
  }
}

const VectorXd LCS::Simulate(VectorXd& x_init, VectorXd& input) {
  VectorXd x_final;
  // calculate force
  drake::solvers::MobyLCPSolver<double> LCPSolver;
  VectorXd force;

  auto flag = LCPSolver.SolveLcpLemkeRegularized(F_[0], E_[0] * x_init + c_[0] + H_[0] * input,
                          &force);


  // update
  x_final = A_[0] * x_init + B_[0] * input + D_[0] * force + d_[0];

  if (flag == 0){
        std::cout << "LCS SOLVE FAILED" << std::endl;
  }


  return x_final;
}

}  // namespace solvers
}  // namespace dairlib
