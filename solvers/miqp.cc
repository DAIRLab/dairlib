#include "solvers/miqp.h"
#include <chrono>

#include <omp.h>

#include "solvers/lcs.h"

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/moby_lcp_solver.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/solve.h"

#include "drake/solvers/branch_and_bound.h"
//#include "solvers/optimization_utils.h"
#include "drake/solvers/gurobi_solver.h"

#include <iostream>

namespace dairlib {
namespace solvers {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
using drake::solvers::GurobiSolver;

using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::SolutionResult;
using drake::solvers::SolverOptions;

using drake::solvers::OsqpSolver;
using drake::solvers::OsqpSolverDetails;
using drake::solvers::Solve;

MIQP::MIQP(const LCS& LCS, const vector<MatrixXd>& Q, const vector<MatrixXd>& R,
       const vector<MatrixXd>& G, const vector<MatrixXd>& U,
       const vector<VectorXd>& xdesired, const C3Options& options, double scaling,
       const std::vector<Eigen::VectorXd>& warm_start_delta,
       const std::vector<Eigen::VectorXd>& warm_start_binary,
       const std::vector<Eigen::VectorXd>& warm_start_x,
       const std::vector<Eigen::VectorXd>& warm_start_lambda,
       const std::vector<Eigen::VectorXd>& warm_start_u,
       bool warm_start)
    : A_(LCS.A_),
      B_(LCS.B_),
      D_(LCS.D_),
      d_(LCS.d_),
      E_(LCS.E_),
      F_(LCS.F_),
      H_(LCS.H_),
      c_(LCS.c_),
      Q_(Q),
      R_(R),
      U_(U),
      G_(G),
      xdesired_(xdesired),
      options_(options),
      scaling_(scaling),
      N_((LCS.A_).size()),
      n_((LCS.A_)[0].cols()),
      m_((LCS.D_)[0].cols()),
      k_((LCS.B_)[0].cols()),
      hflag_(H_[0].isZero(0)),
      prog_(MathematicalProgram()),
      OSQPoptions_(SolverOptions()),
      osqp_(GurobiSolver()){


  x_ = vector<drake::solvers::VectorXDecisionVariable>();
  u_ = vector<drake::solvers::VectorXDecisionVariable>();
  lambda_ = vector<drake::solvers::VectorXDecisionVariable>();
  binary_ = vector<drake::solvers::VectorXDecisionVariable>();

  for (int i = 0; i < N_ + 1; i++) {
    x_.push_back(prog_.NewContinuousVariables(n_, "x" + std::to_string(i)));
    if (i < N_) {
      u_.push_back(prog_.NewContinuousVariables(k_, "k" + std::to_string(i)));
      lambda_.push_back(
          prog_.NewContinuousVariables(m_, "lambda" + std::to_string(i)));
      binary_.push_back(
          prog_.NewBinaryVariables(m_, "binary" + std::to_string(i)));
    }
  }

  MatrixXd LinEq(n_, 2 * n_ + k_ + m_);
  LinEq.block(0, n_ + k_ + m_, n_, n_) = -1 * MatrixXd::Identity(n_, n_);

  for (int i = 0; i < N_; i++) {
    LinEq.block(0, 0, n_, n_) = A_.at(i);
    LinEq.block(0, n_, n_, k_) = B_.at(i);
    LinEq.block(0, n_ + k_, n_, m_) = D_.at(i);

    prog_.AddLinearEqualityConstraint(
        LinEq, -d_.at(i), {x_.at(i), u_.at(i), lambda_.at(i), x_.at(i + 1)});
  }

  ///add lambda > 0
  for (int i = 0; i < N_; i++) {
    prog_.AddLinearConstraint(
        MatrixXd::Identity(m_, m_), VectorXd::Zero(m_), 100000000*VectorXd::Ones(m_), lambda_.at(i));
  }

  double M = 100;

  MatrixXd Mcons1(m_, 2 * m_);
  Mcons1 << M*MatrixXd::Identity(m_, m_), -MatrixXd::Identity(m_, m_);
  ///add M b - lambda > 0
  for (int i = 0; i < N_; i++) {
    prog_.AddLinearConstraint(
        Mcons1, VectorXd::Zero(m_), 100000000*VectorXd::Ones(m_), {binary_.at(i), lambda_.at(i) });
  }

  MatrixXd Mcons2(m_, n_ + m_ + k_);
  Mcons2 << E_.at(0), F_.at(0), H_.at(0);
  ///add Ex + F lam + Hu > - c
  for (int i = 0; i < N_; i++) {
    prog_.AddLinearConstraint(
        Mcons2, -c_.at(0), 1000000000*VectorXd::Ones(m_), {x_.at(i), lambda_.at(i), u_.at(i)});
  }


  ///add M Ones - c >= M binary + Ex + F lambda + Hu
  MatrixXd Mcons3(m_, n_ + 2*m_ + k_);
  Mcons3 << M*MatrixXd::Identity(m_, m_), E_.at(0), F_.at(0), H_.at(0);
  for (int i = 0; i < N_; i++) {
    prog_.AddLinearConstraint(
        Mcons3, -100000000*VectorXd::Ones(m_) , M*VectorXd::Ones(m_) -c_.at(0), {binary_.at(i),x_.at(i), lambda_.at(i), u_.at(i)});
  }




  //prog_.AddQuadraticCost(MatrixXd::Identity(19,19), VectorXd::Zero(19), x_.at(1), 0);
  for (int i = 0; i < N_ + 1; i++) {
    prog_.AddQuadraticCost(Q_.at(i) * 2, -2 * Q_.at(i) * xdesired_.at(i), x_.at(i), 1);
    if (i < N_) {
      prog_.AddQuadraticCost(R_.at(i) * 2, VectorXd::Zero(k_), u_.at(i), 1);
    }
  }

}

VectorXd MIQP::Solve(VectorXd& x0, vector<VectorXd>& delta, vector<VectorXd>& w) {
  vector<MatrixXd> Gv = G_;
  VectorXd z;

  vector<VectorXd> WD(N_, VectorXd::Zero(n_ + m_ + k_));
  for (int i = 0; i < N_; i++) {
    WD.at(i) = delta.at(i) - w.at(i);

    //////CHECKING STUFF
    //std::cout << "WD" << i << WD[i] << std::endl;
  }


  vector<VectorXd> zfin = SolveQP(x0, Gv, WD);

  z = zfin[0];


  return z.segment(n_ + m_, k_);
}

vector<VectorXd> MIQP::SolveQP(VectorXd& x0, vector<MatrixXd>& G,
                             vector<VectorXd>& WD) {

  for (auto& constraint : constraints_) {
    prog_.RemoveConstraint(constraint);
  }
  constraints_.clear();

  constraints_.push_back(prog_.AddLinearConstraint(x_[0] == x0));


//  if (hflag_ == 1) {
//    drake::solvers::MobyLCPSolver<double> LCPSolver;
//    VectorXd lambda0;
//    LCPSolver.SolveLcpLemke(F_[0], E_[0] * x0 + c_[0], &lambda0);
//    constraints_.push_back(prog_.AddLinearConstraint(lambda_[0] == lambda0));
//  }

//  for (auto& cost : costs_) {
//    prog_.RemoveCost(cost);
//  }
//  costs_.clear();


//  for (int i = 0; i < N_ + 1; i++) {
//    if (i < N_) {
//      costs_.push_back(prog_.AddQuadraticCost(
//          G.at(i).block(0, 0, n_, n_) * 2,
//          -2 * G.at(i).block(0, 0, n_, n_) * WD.at(i).segment(0, n_), x_.at(i),
//          1));
//      costs_.push_back(prog_.AddQuadraticCost(
//          G.at(i).block(n_, n_, m_, m_) * 2,
//          -2 * G.at(i).block(n_, n_, m_, m_) * WD.at(i).segment(n_, m_),
//          lambda_.at(i), 1));
//      costs_.push_back(
//          prog_.AddQuadraticCost(G.at(i).block(n_ + m_, n_ + m_, k_, k_) * 2,
//                                 -2 * G.at(i).block(n_ + m_, n_ + m_, k_, k_) *
//                                     WD.at(i).segment(n_ + m_, k_),
//                                 u_.at(i), 1));
//    }
//  }

  //prog_.SetProgram   env_.set("OutputFlag", "0");
//  prog_.SetSolverOption(GurobiSolver::id(), "LogToConsole", 1);
//  prog_.SetSolverOption(GurobiSolver::id(), "OutputFlag", 1);
  //prog_.SetSolverOption(GurobiSolver::id(), "MIPGap", 0);

  //prog_.SetSolverOption(GurobiSolver::id(), "NumericFocus", 2);
//  prog_.SetSolverOption(GurobiSolver::id(), "FeasibilityTol", 1e-6);
//  prog_.SetSolverOption(GurobiSolver::id(), "IntFeasTol", 1e-6);
//  prog_.SetSolverOption(GurobiSolver::id(), "OptimalityTol", 1e-6);


  //prog_.SetSolverOption(GurobiSolver::id(), "PreQLinearize", 0);




  MathematicalProgramResult result = osqp_.Solve(prog_);


  DRAKE_DEMAND(result.is_success());
  VectorXd xSol = result.GetSolution(x_[0]);
  vector<VectorXd> zz(N_, VectorXd::Zero(n_ + m_ + k_));


  for (int i = 0; i < N_; i++) {
    zz.at(i).segment(0, n_) = result.GetSolution(x_[i]);
    zz.at(i).segment(n_, m_) = result.GetSolution(lambda_[i]);
    zz.at(i).segment(n_ + m_, k_) = result.GetSolution(u_[i]);
  }




  return zz;
}

}  // namespace solvers
}  // namespace dairlib
