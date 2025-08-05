#include "solvers/base_c3.h"

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

C3Base::CostMatrices::CostMatrices(const std::vector<Eigen::MatrixXd>& Q,
                                   const std::vector<Eigen::MatrixXd>& R,
                                   const std::vector<Eigen::MatrixXd>& G,
                                   const std::vector<Eigen::MatrixXd>& U) {
  this->Q = Q;
  this->R = R;
  this->G = G;
  this->U = U;
}

C3Base::C3Base(const LCS& lcs, const C3Base::CostMatrices& costs,
               const vector<VectorXd>& x_des, const C3Options& options,
               const int z_size)
    : warm_start_(options.warm_start),
      lcs_(lcs),
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
      prog_(MathematicalProgram()),
      z_size_(z_size) {
  ScaleLCS();
  if (warm_start_) {
    InitializeWarmStarts();
  }
  InitializeOptimizationVariables();
  InitializeDynamicsConstraints();
  InitializeStateAndInputCosts();
}

C3Base::C3Base(const LCS& lcs, const C3Base::CostMatrices& costs,
               const std::vector<Eigen::VectorXd>& x_des,
               const C3Options& options)
    : C3Base(lcs, costs, x_des, options,
             lcs.A_[0].cols() + lcs.D_[0].cols() + lcs.B_[0].cols()) {}

void C3Base::ScaleLCS() {
  DRAKE_DEMAND(lcs_.D_.at(0).norm() > 0);
  auto Dn = lcs_.D_.at(0).norm();
  auto An = lcs_.A_.at(0).norm();
  AnDn_ = An / Dn;
  for (int i = 0; i < N_; ++i) {
    D_.at(i) *= AnDn_;
    E_.at(i) /= AnDn_;
    c_.at(i) /= AnDn_;
    H_.at(i) /= AnDn_;
  }
}

void C3Base::InitializeWarmStarts() {
  warm_start_delta_.resize(options_.admm_iter + 1);
  warm_start_binary_.resize(options_.admm_iter + 1);
  warm_start_x_.resize(options_.admm_iter + 1);
  warm_start_lambda_.resize(options_.admm_iter + 1);
  warm_start_u_.resize(options_.admm_iter + 1);
  for (size_t iter = 0; iter < options_.admm_iter + 1; ++iter) {
    warm_start_delta_[iter].resize(N_);
    for (size_t i = 0; i < N_; i++) {
      warm_start_delta_[iter][i] = VectorXd::Zero(z_size_);
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

void C3Base::InitializeOptimizationVariables() {
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
    z_sol_->push_back(Eigen::VectorXd::Zero(z_size_));
    x_sol_->push_back(Eigen::VectorXd::Zero(n_));
    lambda_sol_->push_back(Eigen::VectorXd::Zero(m_));
    u_sol_->push_back(Eigen::VectorXd::Zero(k_));
    w_sol_->push_back(Eigen::VectorXd::Zero(z_size_));
    delta_sol_->push_back(Eigen::VectorXd::Zero(z_size_));
  }

  for (int i = 0; i < N_ + 1; i++) {
    x_.push_back(prog_.NewContinuousVariables(n_, "x" + std::to_string(i)));
    if (i < N_) {
      u_.push_back(prog_.NewContinuousVariables(k_, "k" + std::to_string(i)));
      lambda_.push_back(
          prog_.NewContinuousVariables(m_, "lambda" + std::to_string(i)));
    }
  }
}

void C3Base::InitializeDynamicsConstraints() {
  MatrixXd LinEq(n_, 2 * n_ + m_ + k_);
  LinEq.block(0, n_ + m_ + k_, n_, n_) = -1 * MatrixXd::Identity(n_, n_);
  dynamics_constraints_.resize(N_);
  for (int i = 0; i < N_; i++) {
    LinEq.block(0, 0, n_, n_) = A_.at(i);
    LinEq.block(0, n_, n_, m_) = D_.at(i);
    LinEq.block(0, n_ + m_, n_, k_) = B_.at(i);

    dynamics_constraints_[i] =
        prog_
            .AddLinearEqualityConstraint(
                LinEq, -lcs_.d_.at(i),
                {x_.at(i), lambda_.at(i), u_.at(i), x_.at(i + 1)})
            .evaluator()
            .get();
  }
}

void C3Base::InitializeStateAndInputCosts() {
  target_cost_.resize(N_ + 1);
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

void C3Base::UpdateCostMatrices(const C3Base::CostMatrices& costs) {
  DRAKE_DEMAND(costs.Q.size() == N_ + 1);
  DRAKE_DEMAND(costs.R.size() == N_);
  DRAKE_DEMAND(costs.U.size() == N_);
  DRAKE_DEMAND(costs.G.size() == N_);
  DRAKE_DEMAND(costs.Q[0].rows() == n_);
  DRAKE_DEMAND(costs.Q[0].cols() == n_);
  DRAKE_DEMAND(costs.R[0].rows() == k_);
  DRAKE_DEMAND(costs.R[0].cols() == k_);
  DRAKE_DEMAND(costs.U[0].rows() == z_size_);
  DRAKE_DEMAND(costs.U[0].cols() == z_size_);
  DRAKE_DEMAND(costs.G[0].rows() == z_size_);
  DRAKE_DEMAND(costs.G[0].cols() == z_size_);
  Q_ = costs.Q;
  R_ = costs.R;
  U_ = costs.U;
  G_ = costs.G;
}

void C3Base::UpdateLCS(const LCS& lcs) {
  DRAKE_DEMAND(lcs.A_.size() == N_);
  DRAKE_DEMAND(lcs.A_[0].rows() == n_);
  DRAKE_DEMAND(lcs.A_[0].cols() == n_);
  DRAKE_DEMAND(lcs.D_[0].cols() == m_);
  DRAKE_DEMAND(lcs.B_[0].cols() == k_);
  // Update the stored LCS object.
  lcs_ = lcs;
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

  ScaleLCS();
  UpdateDynamicsConstraints();
}

void C3Base::UpdateDynamicsConstraints() {
  MatrixXd LinEq = MatrixXd::Zero(n_, n_ + n_ + m_ + k_);
  LinEq.block(0, n_ + m_ + k_, n_, n_) = -1 * MatrixXd::Identity(n_, n_);

  for (int i = 0; i < N_; i++) {
    LinEq.block(0, 0, n_, n_) = A_.at(i);
    LinEq.block(0, n_, n_, m_) = D_.at(i);
    LinEq.block(0, n_ + m_, n_, k_) = B_.at(i);
    dynamics_constraints_[i]->UpdateCoefficients(LinEq, -d_.at(i));
  }
}

void C3Base::UpdateCostLCS(const LCS& lcs_for_cost) {
  DRAKE_DEMAND(lcs_for_cost.A_.size() == N_);
  DRAKE_DEMAND(lcs_for_cost.A_[0].rows() == n_);
  DRAKE_DEMAND(lcs_for_cost.A_[0].cols() == n_);
  DRAKE_DEMAND(lcs_for_cost.B_[0].cols() == k_);
  // Update the stored LCS object.
  if (!lcs_for_cost_) {
    lcs_for_cost_ = std::make_unique<LCS>(lcs_for_cost);
  } else {
    *lcs_for_cost_ = lcs_for_cost;
  }
}

void C3Base::UpdateTarget(const std::vector<Eigen::VectorXd>& x_des) {
  x_desired_ = x_des;
  for (int i = 0; i < N_ + 1; i++) {
    target_cost_[i]->UpdateCoefficients(2 * Q_.at(i),
                                        -2 * Q_.at(i) * x_desired_.at(i));
  }
}

void C3Base::Solve(const VectorXd& x0, bool verbose) {
  auto start = std::chrono::high_resolution_clock::now();

  VectorXd delta_init = VectorXd::Zero(z_size_);
  if (options_.delta_option == 1) {
    delta_init.head(n_) = x0;
  }
  std::vector<VectorXd> delta(N_, delta_init);
  std::vector<VectorXd> w(N_, VectorXd::Zero(z_size_));
  vector<MatrixXd> Gv = G_;

  for (int i = 0; i < N_; ++i) {
    if (options_.penalize_changes_in_u_across_solves) {
      // Penalize deviation from previous input solution:  input cost is
      // (u-u_prev)' * R * (u-u_prev).
      input_costs_[i]->UpdateCoefficients(2 * R_.at(i),
                                          -2 * R_.at(i) * u_sol_->at(i));
    } else {
      // Penalize inputs:  input cost is u' * R * u.
      input_costs_[i]->UpdateCoefficients(2 * R_.at(i),
                                          Eigen::VectorXd::Zero(k_));
    }
  }

  for (int iter = 0; iter < options_.admm_iter; iter++) {
    ADMMStep(x0, &delta, &w, &Gv, iter, verbose);
  }

  vector<VectorXd> WD(N_, VectorXd::Zero(z_size_));
  for (int i = 0; i < N_; i++) {
    WD.at(i) = delta.at(i) - w.at(i);
  }

  vector<VectorXd> zfin = SolveQP(x0, Gv, WD, options_.admm_iter, true);

  if (verbose) {
    std::cout << "x0: " << x0.transpose() << std::endl;
    std::cout << "Final ADMM Iteration: " << options_.admm_iter << std::endl;
    // Make matrix versions of variables for more compact printing.
    Eigen::MatrixXd verbose_delta = Eigen::MatrixXd::Zero(z_size_, N_);
    Eigen::MatrixXd verbose_w = Eigen::MatrixXd::Zero(z_size_, N_);
    Eigen::MatrixXd verbose_zfin = Eigen::MatrixXd::Zero(z_size_, N_);
    Eigen::MatrixXd verbose_zsol = Eigen::MatrixXd::Zero(z_size_, N_);
    for (int i = 0; i < N_; i++) {
      verbose_delta.col(i) = delta[i];
      verbose_w.col(i) = w[i];
      verbose_zfin.col(i) = zfin_[i];
      verbose_zsol.col(i) = z_sol_->at(i);
    }
    std::cout << "zfin: \n" << verbose_zfin << std::endl;
    std::cout << "zsol: \n" << verbose_zsol << std::endl;
    std::cout << "delta: \n" << verbose_delta << std::endl;
    std::cout << "w: \n" << verbose_w << std::endl;
  }

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

  zfin_ = zfin;
  auto finish = std::chrono::high_resolution_clock::now();
  auto elapsed = finish - start;
  solve_time_ =
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() /
      1e6;
}

void C3Base::ADMMStep(const VectorXd& x0, vector<VectorXd>* delta,
                      vector<VectorXd>* w, vector<MatrixXd>* Gv,
                      int admm_iteration, bool verbose) {
  vector<VectorXd> WD(N_, VectorXd::Zero(z_size_));

  for (int i = 0; i < N_; i++) {
    WD.at(i) = delta->at(i) - w->at(i);
  }

  vector<VectorXd> z = SolveQP(x0, *Gv, WD, admm_iteration, true);
  if (verbose) {
    std::cout << "SolveQP Iteration: " << admm_iteration << std::endl;
    Eigen::MatrixXd verbose_z = Eigen::MatrixXd::Zero(z_size_, N_);
    for (int i = 0; i < N_; i++) {
      verbose_z.col(i) = z[i];
    }
    std::cout << "z: \n" << verbose_z << std::endl;
  }

  vector<VectorXd> ZW(N_, VectorXd::Zero(z_size_));
  for (int i = 0; i < N_; i++) {
    ZW[i] = w->at(i) + z[i];
  }

  if (U_[0].isZero(0)) {
    *delta = SolveProjection(*Gv, ZW, admm_iteration);

  } else {
    *delta = SolveProjection(U_, ZW, admm_iteration);
  }
  if (verbose) {
    std::cout << "ADMM Iteration: " << admm_iteration << std::endl;
    Eigen::MatrixXd verbose_delta = Eigen::MatrixXd::Zero(z_size_, N_);
    for (int i = 0; i < N_; i++) {
      verbose_delta.col(i) = delta->at(i);
    }
    std::cout << "delta: \n" << verbose_delta << std::endl;
  }

  for (int i = 0; i < N_; i++) {
    w->at(i) = w->at(i) + z[i] - delta->at(i);
    w->at(i) = w->at(i) / options_.rho_scale;
    Gv->at(i) = Gv->at(i) * options_.rho_scale;
  }
}

void C3Base::AddAugmentedCostsQPStep(const vector<MatrixXd>& G,
                                     const vector<VectorXd>& WD) {
  for (int i = 0; i < N_; i++) {
    costs_.push_back(prog_.AddQuadraticCost(
        2 * G.at(i).block(n_, n_, m_, m_),
        -2 * G.at(i).block(n_, n_, m_, m_) * WD.at(i).segment(n_, m_),
        lambda_.at(i), 1));
    costs_.push_back(prog_.AddQuadraticCost(
        2 * G.at(i).block(0, 0, n_, n_),
        -2 * G.at(i).block(0, 0, n_, n_) * WD.at(i).segment(0, n_), x_.at(i),
        1));
    costs_.push_back(
        prog_.AddQuadraticCost(2 * G.at(i).block(n_ + m_, n_ + m_, k_, k_),
                               -2 * G.at(i).block(n_ + m_, n_ + m_, k_, k_) *
                                   WD.at(i).segment(n_ + m_, k_),
                               u_.at(i), 1));
  }
}

void C3Base::SetInitialGuessQPStep(const Eigen::VectorXd& x0,
                                   int admm_iteration) {
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
}

void C3Base::ExtractQPSolution(
    const drake::solvers::MathematicalProgramResult& result, int admm_iteration,
    bool is_final_solve) {
  for (int i = 0; i < N_; i++) {
    if (is_final_solve) {
      x_sol_->at(i) = result.GetSolution(x_[i]);
      lambda_sol_->at(i) = result.GetSolution(lambda_[i]);
      u_sol_->at(i) = result.GetSolution(u_[i]);
    }
    z_sol_->at(i).segment(0, n_) = result.GetSolution(x_[i]);
    z_sol_->at(i).segment(n_, m_) = result.GetSolution(lambda_[i]);
    z_sol_->at(i).segment(n_ + m_, k_) = result.GetSolution(u_[i]);
  }
}

void C3Base::UpdateWarmStarts(
    const drake::solvers::MathematicalProgramResult& result,
    int admm_iteration) {
  for (int i = 0; i < N_; i++) {
    warm_start_x_[admm_iteration][i] = result.GetSolution(x_[i]);
    warm_start_lambda_[admm_iteration][i] = result.GetSolution(lambda_[i]);
    warm_start_u_[admm_iteration][i] = result.GetSolution(u_[i]);
  }
  warm_start_x_[admm_iteration][N_] = result.GetSolution(x_[N_]);
}

vector<VectorXd> C3Base::SolveQP(const VectorXd& x0, const vector<MatrixXd>& G,
                                 const vector<VectorXd>& WD, int admm_iteration,
                                 bool is_final_solve) {
  // Remove initial state and initial force constraint from previous solve
  for (auto& constraint : constraints_) {
    prog_.RemoveConstraint(constraint);
  }
  constraints_.clear();

  // Add initial state constraint
  constraints_.push_back(prog_.AddLinearConstraint(x_[0] == x0));

  // H matrix does not depend on u, so just simulate passive system
  if (h_is_zero_ == 1) {
    drake::solvers::MobyLCPSolver<double> LCPSolver;
    VectorXd lambda0;
    LCPSolver.SolveLcpLemkeRegularized(F_[0], E_[0] * x0 + c_[0], &lambda0);
    constraints_.push_back(prog_.AddLinearConstraint(lambda_[0] == lambda0));
  }

  // Remove augmented costs from previous solve
  for (auto& cost : costs_) {
    prog_.RemoveCost(cost);
  }
  costs_.clear();

  AddAugmentedCostsQPStep(G, WD);

  SetInitialGuessQPStep(x0, admm_iteration);
  MathematicalProgramResult result = osqp_.Solve(prog_);

  if (result.is_success()) {
    if (warm_start_) {
      UpdateWarmStarts(result, admm_iteration);
    }
    ExtractQPSolution(result, admm_iteration, is_final_solve);
    return *z_sol_;
  } else {
    throw std::runtime_error("CAUTION: QP Step in C3 did not succeed");
  }
}

vector<VectorXd> C3Base::SolveProjection(const vector<MatrixXd>& U,
                                         vector<VectorXd>& WZ,
                                         int admm_iteration) {
  vector<VectorXd> deltaProj(N_, VectorXd::Zero(z_size_));
  int i;

  if (options_.num_threads > 0) {
    omp_set_dynamic(0);  // Explicitly disable dynamic teams
    omp_set_num_threads(options_.num_threads);  // Set number of threads
    // TODO: A newer version of OpenMP likely uses omp_set_max_active_levels
    // instead of omp_set_nested. Consider updating this after testing.
    omp_set_nested(1);  // Enable nested parallelism (important for
                        // sampling_c3_controller)
  }

#pragma omp parallel for num_threads( \
        options_.num_threads) if (use_parallelization_in_projection_)
  for (i = 0; i < N_; i++) {
    if (warm_start_) {
      if (i == N_ - 1) {
        deltaProj[i] = SolveSingleProjection(U[i], WZ[i], E_[i], F_[i], H_[i],
                                             c_[i], admm_iteration, -1);
      } else {
        deltaProj[i] = SolveSingleProjection(U[i], WZ[i], E_[i], F_[i], H_[i],
                                             c_[i], admm_iteration, i + 1);
      }
    } else {
      deltaProj[i] = SolveSingleProjection(U[i], WZ[i], E_[i], F_[i], H_[i],
                                           c_[i], admm_iteration, -1);
    }
  }
  return deltaProj;
}

void C3Base::AddLinearConstraint(Eigen::RowVectorXd& A, double lower_bound,
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

void C3Base::RemoveUserConstraints() {
  for (auto& userconstraint : user_constraints_) {
    prog_.RemoveConstraint(userconstraint);
  }
  user_constraints_.clear();
}

}  // namespace solvers
}  // namespace dairlib
