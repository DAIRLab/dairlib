#include "solvers/c3.h"

#include <chrono>
#include <iostream>

#include <Eigen/Core>
#include <omp.h>

#include "solvers/lcs.h"

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/moby_lcp_solver.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/solve.h"

namespace dairlib {
namespace solvers {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::SolutionResult;
using drake::solvers::SolverOptions;

using drake::solvers::OsqpSolver;
using drake::solvers::OsqpSolverDetails;
using drake::solvers::Solve;

C3::CostMatrices::CostMatrices(const std::vector<Eigen::MatrixXd>& Q,
                               const std::vector<Eigen::MatrixXd>& R,
                               const std::vector<Eigen::MatrixXd>& G,
                               const std::vector<Eigen::MatrixXd>& U) {
  this->Q = Q;
  this->R = R;
  this->G = G;
  this->U = U;
}

C3::C3(const LCS& lcs, const C3::CostMatrices& costs,
       const vector<VectorXd>& x_des, const C3Options& options)
    : warm_start_(options.warm_start),
      N_((lcs.A_).size()),
      n_((lcs.A_)[0].cols()),
      m_((lcs.D_)[0].cols()),
      k_((lcs.B_)[0].cols()),
      A_(lcs.A_),
      B_(lcs.B_),
      D_(lcs.D_),
      d_(lcs.d_),
      E_(lcs.E_),
      F_(lcs.F_),
      H_(lcs.H_),
      c_(lcs.c_),
      Q_(costs.Q),
      R_(costs.R),
      U_(costs.U),
      G_(costs.G),
      x_desired_(x_des),
      options_(options),
      h_is_zero_(lcs.H_[0].isZero(0)),
      osqp_(OsqpSolver()),
      prog_(MathematicalProgram()) {
  if (warm_start_) {
    warm_start_delta_.resize(options_.admm_iter + 1);
    warm_start_binary_.resize(options_.admm_iter + 1);
    warm_start_x_.resize(options_.admm_iter + 1);
    warm_start_lambda_.resize(options_.admm_iter + 1);
    warm_start_u_.resize(options_.admm_iter + 1);
    for (size_t iter = 0; iter < options_.admm_iter + 1; ++iter) {
      warm_start_delta_[iter].resize(N_);
      for (size_t i = 0; i < N_; i++) {
        warm_start_delta_[iter][i] = VectorXd::Zero(n_ + m_ + k_);
      }
      warm_start_binary_[iter].resize(N_);
      for (size_t i = 0; i < N_; i++) {
        warm_start_binary_[iter][i] = VectorXd::Zero(m_);
      }
      warm_start_x_[iter].resize(N_ + 1);
      for (size_t i = 0; i < N_ + 1; i++) {
        warm_start_x_[iter][i] = VectorXd::Zero(n_);
      }
      warm_start_lambda_[iter].resize(N_);
      for (size_t i = 0; i < N_; i++) {
        warm_start_lambda_[iter][i] = VectorXd::Zero(m_);
      }
      warm_start_u_[iter].resize(N_);
      for (size_t i = 0; i < N_; i++) {
        warm_start_u_[iter][i] = VectorXd::Zero(k_);
      }
    }
  }

  auto Dn = lcs.D_.at(0).norm();
  auto An = lcs.A_.at(0).norm();
  AnDn_ = An / Dn;

  for (int i = 0; i < N_; ++i) {
    D_.at(i) *= AnDn_;
    E_.at(i) /= AnDn_;
    c_.at(i) /= AnDn_;
    H_.at(i) /= AnDn_;
  }

  x_ = vector<drake::solvers::VectorXDecisionVariable>();
  u_ = vector<drake::solvers::VectorXDecisionVariable>();
  lambda_ = vector<drake::solvers::VectorXDecisionVariable>();

  z_sol_ = std::make_unique<std::vector<VectorXd>>();
  x_sol_ = std::make_unique<std::vector<VectorXd>>();
  lambda_sol_ = std::make_unique<std::vector<VectorXd>>();
  u_sol_ = std::make_unique<std::vector<VectorXd>>();
  w_sol_ = std::make_unique<std::vector<VectorXd>>();
  delta_sol_ = std::make_unique<std::vector<VectorXd>>();
  for (int i = 0; i < N_; ++i) {
    z_sol_->push_back(Eigen::VectorXd::Zero(n_ + m_ + k_));
    x_sol_->push_back(Eigen::VectorXd::Zero(n_));
    lambda_sol_->push_back(Eigen::VectorXd::Zero(m_));
    u_sol_->push_back(Eigen::VectorXd::Zero(k_));
    w_sol_->push_back(Eigen::VectorXd::Zero(n_ + m_ + k_));
    delta_sol_->push_back(Eigen::VectorXd::Zero(n_ + m_ + k_));
  }

  for (int i = 0; i < N_ + 1; i++) {
    x_.push_back(prog_.NewContinuousVariables(n_, "x" + std::to_string(i)));
    if (i < N_) {
      u_.push_back(prog_.NewContinuousVariables(k_, "k" + std::to_string(i)));
      lambda_.push_back(
          prog_.NewContinuousVariables(m_, "lambda" + std::to_string(i)));
    }
  }

  MatrixXd LinEq(n_, 2 * n_ + m_ + k_);
  LinEq.block(0, n_ + m_ + k_, n_, n_) = -1 * MatrixXd::Identity(n_, n_);
  dynamics_constraints_.resize(N_);
  target_cost_.resize(N_ + 1);
  for (int i = 0; i < N_; i++) {
    LinEq.block(0, 0, n_, n_) = lcs.A_.at(i);
    LinEq.block(0, n_, n_, m_) = lcs.D_.at(i);
    LinEq.block(0, n_ + m_, n_, k_) = lcs.B_.at(i);

    dynamics_constraints_[i] =
        prog_
            .AddLinearEqualityConstraint(
                LinEq, -lcs.d_.at(i),
                {x_.at(i), lambda_.at(i), u_.at(i), x_.at(i + 1)})
            .evaluator()
            .get();
    prog_.AddLinearConstraint(lambda_.at(i) >= VectorXd::Zero(m_));
  }
  input_costs_.resize(N_);
  for (int i = 0; i < N_ + 1; i++) {
    target_cost_[i] =
        prog_
            .AddQuadraticCost(2 * Q_.at(i), -2 * Q_.at(i) * x_desired_.at(i),
                              x_.at(i), 1)
            .evaluator()
            .get();
    if (i < N_) {
      input_costs_[i] =
          prog_.AddQuadraticCost(2 * R_.at(i), VectorXd::Zero(k_), u_.at(i), 1)
              .evaluator();
    }
  }
}

void C3::UpdateLCS(const LCS& lcs) {
  A_ = lcs.A_;
  B_ = lcs.B_;
  D_ = lcs.D_;
  d_ = lcs.d_;
  E_ = lcs.E_;
  F_ = lcs.F_;
  H_ = lcs.H_;
  c_ = lcs.c_;
  dt_ = lcs.dt_;
  W_x_ = lcs.W_x_;
  W_l_ = lcs.W_l_;
  W_u_ = lcs.W_u_;
  w_ = lcs.w_;
  h_is_zero_ = H_[0].isZero(0);

  MatrixXd LinEq = MatrixXd::Zero(n_, 2 * n_ + m_ + k_);
  LinEq.block(0, n_ + k_ + m_, n_, n_) = -1 * MatrixXd::Identity(n_, n_);
  auto Dn = D_[0].norm();
  auto An = A_[0].norm();
  AnDn_ = An / Dn;

  for (int i = 0; i < N_; ++i) {
    D_.at(i) *= AnDn_;
    E_.at(i) /= AnDn_;
    c_.at(i) /= AnDn_;
    H_.at(i) /= AnDn_;
  }

  for (int i = 0; i < N_; i++) {
    LinEq.block(0, 0, n_, n_) = A_.at(i);
    LinEq.block(0, n_, n_, m_) = D_.at(i);
    LinEq.block(0, n_ + m_, n_, k_) = B_.at(i);

    dynamics_constraints_[i]->UpdateCoefficients(LinEq, -lcs.d_.at(i));
  }
}

void C3::UpdateTarget(const std::vector<Eigen::VectorXd>& x_des) {
  x_desired_ = x_des;
  for (int i = 0; i < N_ + 1; i++) {
    target_cost_[i]->UpdateCoefficients(2 * Q_.at(i),
                                        -2 * Q_.at(i) * x_desired_.at(i));
  }
}

void C3::Solve(const VectorXd& x0) {
  auto start = std::chrono::high_resolution_clock::now();

  VectorXd delta_init = VectorXd::Zero(n_ + m_ + k_);
  if (options_.delta_option == 1) {
    delta_init.head(n_) = x0;
  }
  std::vector<VectorXd> delta(N_, delta_init);
  std::vector<VectorXd> w(N_, VectorXd::Zero(n_ + m_ + k_));
  vector<MatrixXd> Gv = G_;

//  for (int i = 0; i < N_; ++i) {
//    input_costs_[i]->UpdateCoefficients(2 * R_.at(i),
//                                        -2 * R_.at(i) * u_sol_->at(i));
//  }

  for (int iter = 0; iter < options_.admm_iter; iter++) {
    ADMMStep(x0, &delta, &w, &Gv, iter);
  }

  vector<VectorXd> WD(N_, VectorXd::Zero(n_ + m_ + k_));
  for (int i = 0; i < N_; i++) {
    WD.at(i) = delta.at(i) - w.at(i);
  }

  vector<VectorXd> zfin = SolveQP(x0, Gv, WD, options_.admm_iter, true);

  *w_sol_ = w;
  *delta_sol_ = delta;

  if (!options_.end_on_qp_step) {
    *z_sol_ = delta;
    z_sol_->at(0).segment(0, n_) = x0;
    x_sol_->at(0) = x0;
    for (int i = 1; i < N_; ++i) {
      z_sol_->at(i).segment(0, n_) =
          A_.at(i - 1) * x_sol_->at(i - 1) + B_.at(i - 1) * u_sol_->at(i - 1) +
          D_.at(i - 1) * lambda_sol_->at(i - 1) + d_.at(i - 1);
    }
  }

  for (int i = 0; i < N_; ++i) {
    lambda_sol_->at(i) *= AnDn_;
    z_sol_->at(i).segment(n_, m_) *= AnDn_;
  }

  auto finish = std::chrono::high_resolution_clock::now();
  auto elapsed = finish - start;
  solve_time_ =
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() /
      1e6;
}

void C3::ADMMStep(const VectorXd& x0, vector<VectorXd>* delta,
                  vector<VectorXd>* w, vector<MatrixXd>* Gv,
                  int admm_iteration) {
  vector<VectorXd> WD(N_, VectorXd::Zero(n_ + m_ + k_));

  for (int i = 0; i < N_; i++) {
    WD.at(i) = delta->at(i) - w->at(i);
  }

  vector<VectorXd> z = SolveQP(x0, *Gv, WD, admm_iteration, true);

  vector<VectorXd> ZW(N_, VectorXd::Zero(n_ + m_ + k_));
  for (int i = 0; i < N_; i++) {
    ZW[i] = w->at(i) + z[i];
  }

  if (U_[0].isZero(0)) {
    *delta = SolveProjection(*Gv, ZW, admm_iteration);
  } else {
    *delta = SolveProjection(U_, ZW, admm_iteration);
  }
//  std::cout << "z after projection: " << ZW[0].transpose() << std::endl;
//  std::cout << "delta after projection: " << delta->at(0).transpose() << std::endl;
//  std::cout << "z after projection: " << ZW[1].transpose() << std::endl;
//  std::cout << "delta after projection: " << delta->at(1).transpose() << std::endl;

  for (int i = 0; i < N_; i++) {
    w->at(i) = w->at(i) + z[i] - delta->at(i);
    w->at(i) = w->at(i) / options_.rho_scale;
    Gv->at(i) = Gv->at(i) * options_.rho_scale;
  }
}

vector<VectorXd> C3::SolveQP(const VectorXd& x0, const vector<MatrixXd>& G,
                             const vector<VectorXd>& WD, int admm_iteration,
                             bool is_final_solve) {
  for (auto& constraint : constraints_) {
    prog_.RemoveConstraint(constraint);
  }
  constraints_.clear();
  constraints_.push_back(prog_.AddLinearConstraint(x_[0] == x0));

  if (h_is_zero_ == 1) {  // No dependence on u, so just simulate passive system
    drake::solvers::MobyLCPSolver<double> LCPSolver;
    VectorXd lambda0;
    LCPSolver.SolveLcpLemke(F_[0], E_[0] * x0 + c_[0], &lambda0);
    constraints_.push_back(prog_.AddLinearConstraint(lambda_[0] == lambda0));
  }

  for (auto& cost : costs_) {
    prog_.RemoveCost(cost);
  }
  costs_.clear();

  for (int i = 0; i < N_ + 1; i++) {
    if (i < N_) {
      costs_.push_back(prog_.AddQuadraticCost(
          2 * G.at(i).block(0, 0, n_, n_),
          -2 * G.at(i).block(0, 0, n_, n_) * WD.at(i).segment(0, n_), x_.at(i),
          1));
      costs_.push_back(prog_.AddQuadraticCost(
          2 * G.at(i).block(n_, n_, m_, m_),
          -2 * G.at(i).block(n_, n_, m_, m_) * WD.at(i).segment(n_, m_),
          lambda_.at(i), 1));
      costs_.push_back(
          prog_.AddQuadraticCost(2 * G.at(i).block(n_ + m_, n_ + m_, k_, k_),
                                 -2 * G.at(i).block(n_ + m_, n_ + m_, k_, k_) *
                                     WD.at(i).segment(n_ + m_, k_),
                                 u_.at(i), 1));
    }
  }

  //  /// initialize decision variables to warm start
  if (warm_start_) {
    int index = solve_time_ / dt_;
    double weight = (solve_time_ - index * dt_) / dt_;
    for (int i = 0; i < N_ - 1; i++) {
      prog_.SetInitialGuess(x_[i],
                            (1 - weight) * warm_start_x_[admm_iteration][i] +
                                weight * warm_start_x_[admm_iteration][i + 1]);
      prog_.SetInitialGuess(
          lambda_[i], (1 - weight) * warm_start_lambda_[admm_iteration][i] +
                          weight * warm_start_lambda_[admm_iteration][i + 1]);
      prog_.SetInitialGuess(u_[i],
                            (1 - weight) * warm_start_u_[admm_iteration][i] +
                                weight * warm_start_u_[admm_iteration][i + 1]);
    }
    prog_.SetInitialGuess(x_[0], x0);
    prog_.SetInitialGuess(x_[N_], warm_start_x_[admm_iteration][N_]);
  }

  MathematicalProgramResult result = osqp_.Solve(prog_);

  if (result.is_success()) {
    for (int i = 0; i < N_; i++) {
      if (is_final_solve) {
        x_sol_->at(i) = result.GetSolution(x_[i]);
        lambda_sol_->at(i) = result.GetSolution(lambda_[i]);
        u_sol_->at(i) = result.GetSolution(u_[i]);
      }
      z_sol_->at(i).segment(0, n_) = result.GetSolution(x_[i]);
      z_sol_->at(i).segment(n_, m_) = result.GetSolution(lambda_[i]);
      z_sol_->at(i).segment(n_ + m_, k_) = result.GetSolution(u_[i]);

      if (warm_start_) {
        // update warm start parameters
        warm_start_x_[admm_iteration][i] = result.GetSolution(x_[i]);
        warm_start_lambda_[admm_iteration][i] = result.GetSolution(lambda_[i]);
        warm_start_u_[admm_iteration][i] = result.GetSolution(u_[i]);
      }
    }
    if (warm_start_) {
      warm_start_x_[admm_iteration][N_] = result.GetSolution(x_[N_]);
    }
  }

  return *z_sol_;
}

vector<VectorXd> C3::SolveProjection(const vector<MatrixXd>& U,
                                     vector<VectorXd>& WZ, int admm_iteration) {
  vector<VectorXd> deltaProj(N_, VectorXd::Zero(n_ + m_ + k_));
  int i;

  if (options_.num_threads > 0) {
    omp_set_dynamic(0);  // Explicitly disable dynamic teams
    omp_set_num_threads(options_.num_threads);  // Set number of threads
    omp_set_max_active_levels(1);
  }

#pragma omp parallel for num_threads(options_.num_threads)
  for (i = 0; i < N_; i++) {
    if (options_.use_robust_formulation &&
        admm_iteration ==
            (options_.admm_iter - 1)) {  // only on the last iteration
      if (warm_start_) {
        if (i == N_ - 1) {
          deltaProj[i] = SolveRobustSingleProjection(
              U[i], WZ[i], E_[i], F_[i], H_[i], c_[i], W_x_, W_l_, W_u_, w_,
              admm_iteration, -1);
        } else {
          deltaProj[i] = SolveRobustSingleProjection(
              U[i], WZ[i], E_[i], F_[i], H_[i], c_[i], W_x_, W_l_, W_u_, w_,
              admm_iteration, i);
        }
      } else {
        deltaProj[i] = SolveRobustSingleProjection(
            U[i], WZ[i], E_[i], F_[i], H_[i], c_[i], W_x_, W_l_, W_u_, w_,
            admm_iteration, -1);
      }
    } else {
      if (warm_start_) {
        if (i == N_ - 1) {
          deltaProj[i] = SolveSingleProjection(U[i], WZ[i], E_[i], F_[i], H_[i],
                                               c_[i], admm_iteration, -1);
        } else {
          deltaProj[i] = SolveSingleProjection(U[i], WZ[i], E_[i], F_[i], H_[i],
                                               c_[i], admm_iteration, i);
        }
      } else {
        deltaProj[i] = SolveSingleProjection(U[i], WZ[i], E_[i], F_[i], H_[i],
                                             c_[i], admm_iteration, -1);
      }
    }
  }

  return deltaProj;
}

void C3::AddLinearConstraint(Eigen::RowVectorXd& A, double lower_bound,
                             double upper_bound, int constraint) {
  if (constraint == 1) {
    for (int i = 1; i < N_; i++) {
      user_constraints_.push_back(
          prog_.AddLinearConstraint(A, lower_bound, upper_bound, x_.at(i)));
    }
  }

  if (constraint == 2) {
    for (int i = 0; i < N_; i++) {
      user_constraints_.push_back(
          prog_.AddLinearConstraint(A, lower_bound, upper_bound, u_.at(i)));
    }
  }

  if (constraint == 3) {
    for (int i = 0; i < N_; i++) {
      user_constraints_.push_back(prog_.AddLinearConstraint(
          A, lower_bound, upper_bound, lambda_.at(i)));
    }
  }
}

void C3::RemoveConstraints() {
  for (auto& userconstraint : user_constraints_) {
    prog_.RemoveConstraint(userconstraint);
  }
  user_constraints_.clear();
}

}  // namespace solvers
}  // namespace dairlib
