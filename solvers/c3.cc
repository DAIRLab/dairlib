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
    LinEq.block(0, 0, n_, n_) = A_.at(i);
    LinEq.block(0, n_, n_, m_) = D_.at(i);
    LinEq.block(0, n_ + m_, n_, k_) = B_.at(i);

    dynamics_constraints_[i] =
        prog_
            .AddLinearEqualityConstraint(
                LinEq, -lcs.d_.at(i),
                {x_.at(i), lambda_.at(i), u_.at(i), x_.at(i + 1)})
            .evaluator()
            .get();
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

void C3::UpdateCostMatrices(const C3::CostMatrices& costs) {
  DRAKE_DEMAND(costs.Q.size() == N_ + 1);
  DRAKE_DEMAND(costs.R.size() == N_);
  DRAKE_DEMAND(costs.U.size() == N_);
  DRAKE_DEMAND(costs.G.size() == N_);
  DRAKE_DEMAND(costs.Q[0].rows() == n_);
  DRAKE_DEMAND(costs.Q[0].cols() == n_);
  DRAKE_DEMAND(costs.R[0].rows() == k_);
  DRAKE_DEMAND(costs.R[0].cols() == k_);
  DRAKE_DEMAND(costs.U[0].rows() == n_ + m_ + k_);
  DRAKE_DEMAND(costs.U[0].cols() == n_ + m_ + k_);
  DRAKE_DEMAND(costs.G[0].rows() == n_ + m_ + k_);
  DRAKE_DEMAND(costs.G[0].cols() == n_ + m_ + k_);
  Q_ = costs.Q;
  R_ = costs.R;
  U_ = costs.U;
  G_ = costs.G;
}

void C3::UpdateLCS(const LCS& lcs) {
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

  MatrixXd LinEq = MatrixXd::Zero(n_, 2 * n_ + m_ + k_);
  LinEq.block(0, n_ + k_ + m_, n_, n_) = -1 * MatrixXd::Identity(n_, n_);

  auto Dn = D_.at(0).norm();
  auto An = A_.at(0).norm();
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

void C3::UpdateCostLCS(const LCS& lcs_for_cost) {
  DRAKE_DEMAND(lcs_for_cost.A_.size() == N_);
  DRAKE_DEMAND(lcs_for_cost.A_[0].rows() == n_);
  DRAKE_DEMAND(lcs_for_cost.A_[0].cols() == n_);
  DRAKE_DEMAND(lcs_for_cost.B_[0].cols() == k_);
  // Update the stored LCS object.
  if (!lcs_for_cost_) {
    lcs_for_cost_ = std::make_unique<LCS>(lcs_for_cost);
  }else{
    *lcs_for_cost_ = lcs_for_cost;
  }
}

void C3::UpdateTarget(const std::vector<Eigen::VectorXd>& x_des) {
  x_desired_ = x_des;
  for (int i = 0; i < N_ + 1; i++) {
    target_cost_[i]->UpdateCoefficients(2 * Q_.at(i),
                                        -2 * Q_.at(i) * x_desired_.at(i));
  }
}

void C3::Solve(const VectorXd& x0, bool verbose) {
  auto start = std::chrono::high_resolution_clock::now();

  VectorXd delta_init = VectorXd::Zero(n_ + m_ + k_);
  if (options_.delta_option == 1) {
    delta_init.head(n_) = x0;
  }
  std::vector<VectorXd> delta(N_, delta_init);
  std::vector<VectorXd> w(N_, VectorXd::Zero(n_ + m_ + k_));
  vector<MatrixXd> Gv = G_;

  for (int i = 0; i < N_; ++i) {
    if (options_.penalize_changes_in_u_across_solves) {
      // Penalize deviation from previous input solution:  input cost is
      // (u-u_prev)' * R * (u-u_prev).
      input_costs_[i]->UpdateCoefficients(2 * R_.at(i),
                                          -2 * R_.at(i) * u_sol_->at(i));
    }
    else{
      // Penalize inputs:  input cost is u' * R * u.
      input_costs_[i]->UpdateCoefficients(2 * R_.at(i),
                                          Eigen::VectorXd::Zero(k_));   
    }
  }

  for (int iter = 0; iter < options_.admm_iter; iter++) {
    ADMMStep(x0, &delta, &w, &Gv, iter, verbose);
  }

  vector<VectorXd> WD(N_, VectorXd::Zero(n_ + m_ + k_));
  for (int i = 0; i < N_; i++) {
    WD.at(i) = delta.at(i) - w.at(i);
  }

  vector<VectorXd> zfin = SolveQP(x0, Gv, WD, options_.admm_iter, true);

  if (verbose) {
    std::cout << "x0: " << x0.transpose() << std::endl;
    std::cout << "Final ADMM Iteration: " << options_.admm_iter << std::endl;
    // Make matrix versions of variables for more compact printing.
    Eigen::MatrixXd verbose_delta = Eigen::MatrixXd::Zero(n_ + m_ + k_, N_);
    Eigen::MatrixXd verbose_w = Eigen::MatrixXd::Zero(n_ + m_ + k_, N_);
    Eigen::MatrixXd verbose_zfin = Eigen::MatrixXd::Zero(n_ + m_ + k_, N_);
    Eigen::MatrixXd verbose_zsol = Eigen::MatrixXd::Zero(n_ + m_ + k_, N_);
    for(int i = 0; i < N_; i++) {
      verbose_delta.col(i) = delta[i];
      verbose_w.col(i) = w[i];
      verbose_zfin.col(i) = zfin_[i];
      verbose_zsol.col(i) = z_sol_->at(i);
    }
    std::cout<<"zfin: \n"<<verbose_zfin<<std::endl;
    std::cout<<"zsol: \n"<<verbose_zsol<<std::endl;
    std::cout<<"delta: \n"<<verbose_delta<<std::endl;
    std::cout<<"w: \n"<<verbose_w<<std::endl;
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

// This function relies on the previously computed zfin_ from Solve.
std::pair<double,std::vector<Eigen::VectorXd>> C3::CalcCost(
    C3CostComputationType cost_type,
    double Kp_for_ee_pd_rollout,
    double Kd_for_ee_pd_rollout,
    bool force_tracking_disabled,
    bool print_cost_breakdown,
    bool verbose) const {
  vector<VectorXd> UU(N_, VectorXd::Zero(k_));
  std::vector<Eigen::VectorXd> XX(N_+1, VectorXd::Zero(n_)); 

  // Simulate the dynamics from the planned inputs.
  if (cost_type == C3CostComputationType::kSimLCS) {
    XX[0] = zfin_[0].segment(0, n_);
    for (int i = 0; i < N_; i++) {
      UU[i] = zfin_[i].segment(n_ + m_, k_);
      if (lcs_for_cost_) {
        XX[i+1] = lcs_for_cost_->Simulate(XX[i], UU[i]);
      }
      else {
        XX[i+1] = lcs_.Simulate(XX[i], UU[i]);
      }
    }
  }

  // Use the C3 plan.
  else if (cost_type == C3CostComputationType::kUseC3Plan) {
    for (int i = 0; i < N_; i++) {
      UU[i] = zfin_[i].segment(n_ + m_, k_);
      XX[i] = zfin_[i].segment(0, n_);
      if (i == N_-1) {
        if (lcs_for_cost_) {
          XX[i+1] = lcs_for_cost_->Simulate(XX[i], UU[i]);
        }
        else {
          XX[i+1] = lcs_.Simulate(XX[i], UU[i]);
        }
      }
    }
  }

  // Simulate the dynamics from the planned inputs only for the object; use the
  // planned EE trajectory.
  else if (cost_type == C3CostComputationType::kSimLCSReplaceC3EEPlan) {
    // Simulate the object trajectory.
    XX[0] = zfin_[0].segment(0, n_);
    for (int i = 0; i < N_; i++) {
      UU[i] = zfin_[i].segment(n_ + m_, k_);
      if (lcs_for_cost_) {
        XX[i+1] = lcs_for_cost_->Simulate(XX[i], UU[i]);
      }
      else {
        XX[i+1] = lcs_.Simulate(XX[i], UU[i]);
      }
    }
    // Replace ee traj with those from zfin_.
    for (int i = 0; i < N_; i++) {
      XX[i].segment(0,3) = zfin_[i].segment(0,3);
      if (i == N_-1) {
        if (lcs_for_cost_) {
          XX[i+1].segment(0,3) = 
            lcs_for_cost_->Simulate(XX[i], UU[i]).segment(0,3);
        }
        else {
          XX[i+1].segment(0,3) = lcs_.Simulate(XX[i], UU[i]).segment(0,3);
        }
      }
    }
  }

  // Try to emulate the real cost of the system associated not only applying the
  // planned u but also the u associated with tracking the position plan over
  // time.
  else if (cost_type == C3CostComputationType::kSimImpedance) {
    std::tie(XX, UU) = SimulatePDControl(Kp_for_ee_pd_rollout, 
                                         Kd_for_ee_pd_rollout,
                                         force_tracking_disabled, 
                                         verbose);
  }

  // The same as the previous cost type except the EE states are replaced with
  // the plan from C3 at the end.
  else if (cost_type == C3CostComputationType::kSimImpedanceReplaceC3EEPlan) {
    std::tie(XX, UU) = SimulatePDControl(Kp_for_ee_pd_rollout, 
                                         Kd_for_ee_pd_rollout,
                                         force_tracking_disabled, 
                                         verbose);

    // Replace the end effector position and velocity plans with the ones from
    // the C3 plan.
    for(int i = 0; i < N_; i++) {
      XX[i].segment(0,3) = zfin_[i].segment(0,3);
      XX[i].segment(10,3) = zfin_[i].segment(10,3);
      if (i == N_-1) {
        XX[i+1].segment(0,3) = zfin_[i].segment(0,3);
        XX[i+1].segment(10,3) = zfin_[i].segment(10,3);
      }
    }
  }

  // The same as the previous cost type except only object terms contribute to
  // the final cost. 
  else if (cost_type == C3CostComputationType::kSimImpedanceObjectCostOnly) {
    std::tie(XX, UU) = SimulatePDControl(Kp_for_ee_pd_rollout, 
                                         Kd_for_ee_pd_rollout,
                                         force_tracking_disabled, 
                                         verbose);
  }

  // Declare Q_eff and R_eff as the Q and R to use for cost computation.
  std::vector<Eigen::MatrixXd> Q_eff = Q_;
  std::vector<Eigen::MatrixXd> R_eff = R_;

  // Calculate the cost over the N+1 time steps.
  double cost = 0;

  //used only for verbose mode printouts
  double error_contrib_ee_pos = 0;
  double error_contrib_obj_orientation = 0;
  double error_contrib_obj_pos = 0;
  double error_contrib_ee_vel = 0;
  double error_contrib_obj_ang_vel = 0;
  double error_contrib_obj_vel = 0;

  double cost_contrib_ee_pos = 0;
  double cost_contrib_obj_orientation = 0;
  double cost_contrib_obj_pos = 0;
  double cost_contrib_ee_vel = 0;
  double cost_contrib_obj_ang_vel = 0;
  double cost_contrib_obj_vel = 0;

  // Calculate the error and cost contributions for each state.
  for (int i = 0; i < N_; i++) {
    //ee_pos
    error_contrib_ee_pos += 
      (XX[i].segment(0,3) - x_desired_[i].segment(0,3)).transpose() *
        (XX[i].segment(0,3) - x_desired_[i].segment(0,3));
    cost_contrib_ee_pos += 
      (XX[i].segment(0,3) - x_desired_[i].segment(0,3)).transpose() *
        Q_eff.at(i).block(0,0,3,3)*(XX[i].segment(0,3) - 
          x_desired_[i].segment(0,3));
    //obj_orientation
    error_contrib_obj_orientation += 
      (XX[i].segment(3,4) - x_desired_[i].segment(3,4)).transpose() * 
        (XX[i].segment(3,4) - x_desired_[i].segment(3,4));
    cost_contrib_obj_orientation += 
      (XX[i].segment(3,4) - x_desired_[i].segment(3,4)).transpose() *
        Q_eff.at(i).block(3,3,4,4) * 
          (XX[i].segment(3,4) - x_desired_[i].segment(3,4));
    //obj_pos
    error_contrib_obj_pos += 
      (XX[i].segment(7,3) - x_desired_[i].segment(7,3)).transpose() * 
        (XX[i].segment(7,3) - x_desired_[i].segment(7,3));
    cost_contrib_obj_pos += 
      (XX[i].segment(7,3) - x_desired_[i].segment(7,3)).transpose() *
        Q_eff.at(i).block(7,7,3,3)*(XX[i].segment(7,3) - 
          x_desired_[i].segment(7,3));
    //ee_vel
    error_contrib_ee_vel += 
      (XX[i].segment(10,3) - x_desired_[i].segment(10,3)).transpose() * 
        (XX[i].segment(10,3) - x_desired_[i].segment(10,3));
    cost_contrib_ee_vel += 
      (XX[i].segment(10,3) - x_desired_[i].segment(10,3)).transpose() *
        Q_eff.at(i).block(10,10,3,3) * (XX[i].segment(10,3) - 
          x_desired_[i].segment(10,3));
    //obj_ang_vel
    error_contrib_obj_ang_vel += 
      (XX[i].segment(13,3) - x_desired_[i].segment(13,3)).transpose() * 
        (XX[i].segment(13,3) - x_desired_[i].segment(13,3));
    cost_contrib_obj_ang_vel += 
      (XX[i].segment(13,3) - x_desired_[i].segment(13,3)).transpose() *
        Q_eff.at(i).block(13,13,3,3)*(XX[i].segment(13,3) - 
          x_desired_[i].segment(13,3));
    //obj_vel
    error_contrib_obj_vel += 
      (XX[i].segment(16,3) - x_desired_[i].segment(16,3)).transpose() * 
        (XX[i].segment(16,3) - x_desired_[i].segment(16,3));
    cost_contrib_obj_vel += 
      (XX[i].segment(16,3) - x_desired_[i].segment(16,3)).transpose() *
        Q_eff.at(i).block(16,16,3,3)*(XX[i].segment(16,3) - 
          x_desired_[i].segment(16,3));

    cost = cost + (XX[i] - x_desired_[i]).transpose() * 
            Q_eff.at(i)*(XX[i] - x_desired_[i]) + 
              UU[i].transpose()*R_eff.at(i)*UU[i];
  }

  // Handle the N_th state.
  cost = cost + (XX[N_] - x_desired_[N_]).transpose()*Q_eff.at(N_)*(
    XX[N_] - x_desired_[N_]);

  error_contrib_ee_pos += 
    (XX[N_].segment(0,3) - x_desired_[N_].segment(0,3)).transpose() * 
      (XX[N_].segment(0,3) - x_desired_[N_].segment(0,3));
  error_contrib_obj_orientation += 
    (XX[N_].segment(3,4) - x_desired_[N_].segment(3,4)).transpose() * 
      (XX[N_].segment(3,4) - x_desired_[N_].segment(3,4));
  error_contrib_obj_pos += 
    (XX[N_].segment(7,3) - x_desired_[N_].segment(7,3)).transpose() * 
      (XX[N_].segment(7,3) - x_desired_[N_].segment(7,3));
  error_contrib_ee_vel += 
    (XX[N_].segment(10,3) - x_desired_[N_].segment(10,3)).transpose() * 
      (XX[N_].segment(10,3) - x_desired_[N_].segment(10,3));
  error_contrib_obj_ang_vel += 
    (XX[N_].segment(13,3) - x_desired_[N_].segment(13,3)).transpose() * 
      (XX[N_].segment(13,3) - x_desired_[N_].segment(13,3));
  error_contrib_obj_vel += 
    (XX[N_].segment(16,3) - x_desired_[N_].segment(16,3)).transpose() * 
      (XX[N_].segment(16,3) - x_desired_[N_].segment(16,3));

  cost_contrib_ee_pos += 
    (XX[N_].segment(0,3) - x_desired_[N_].segment(0,3)).transpose() *
      Q_eff.at(N_).block(0,0,3,3) * 
        (XX[N_].segment(0,3) - x_desired_[N_].segment(0,3));
  cost_contrib_obj_orientation += 
    (XX[N_].segment(3,4) - x_desired_[N_].segment(3,4)).transpose() *
      Q_eff.at(N_).block(3,3,4,4)*(XX[N_].segment(3,4) - 
        x_desired_[N_].segment(3,4));
  cost_contrib_obj_pos += 
    (XX[N_].segment(7,3) - x_desired_[N_].segment(7,3)).transpose() *
      Q_eff.at(N_).block(7,7,3,3)*(XX[N_].segment(7,3) - 
        x_desired_[N_].segment(7,3));
  cost_contrib_ee_vel += 
    (XX[N_].segment(10,3) - x_desired_[N_].segment(10,3)).transpose() *
      Q_eff.at(N_).block(10,10,3,3)*(XX[N_].segment(10,3) - 
        x_desired_[N_].segment(10,3));
  cost_contrib_obj_ang_vel += 
    (XX[N_].segment(13,3) - x_desired_[N_].segment(13,3)).transpose() *
      Q_eff.at(N_).block(13,13,3,3)*(XX[N_].segment(13,3) - 
        x_desired_[N_].segment(13,3));
  cost_contrib_obj_vel += 
    (XX[N_].segment(16,3) - x_desired_[N_].segment(16,3)).transpose() *
      Q_eff.at(N_).block(16,16,3,3)*(XX[N_].segment(16,3) - 
        x_desired_[N_].segment(16,3));

  if (cost_type == C3CostComputationType::kSimImpedanceObjectCostOnly) {
    cost = cost_contrib_obj_pos + cost_contrib_obj_orientation +
      cost_contrib_obj_vel + cost_contrib_obj_ang_vel;
  }

  if (verbose || print_cost_breakdown) {
    std::cout<<"Error breakdown"<<std::endl;
    std::cout<<"\t total error contribution from x_ee: "<<
      error_contrib_ee_pos<<std::endl;
    std::cout<<"\t total error contribution from q_obj: "<<
      error_contrib_obj_orientation<<std::endl;
    std::cout<<"\t total error contribution from x_obj: "<<
      error_contrib_obj_pos<<std::endl;
    std::cout<<"\t total error contribution from v_ee: "<<
      error_contrib_ee_vel<<std::endl;
    std::cout<<"\t total error contribution from w_obj: "<<
      error_contrib_obj_ang_vel<<std::endl;
    std::cout<<"\t total error contribution from v_obj: "<<
      error_contrib_obj_vel<<std::endl;

    std::cout<<"\nCOST BREAKDOWN"<<std::endl;
    std::cout<<"\t total cost contribution from x_ee: "<<
      cost_contrib_ee_pos<<std::endl;
    std::cout<<"\t total cost contribution from q_obj: "<<
      cost_contrib_obj_orientation<<std::endl;
    std::cout<<"\t total cost contribution from x_obj: "<<
      cost_contrib_obj_pos<<std::endl;
    std::cout<<"\t total cost contribution from v_ee: "<<
      cost_contrib_ee_vel<<std::endl;
    std::cout<<"\t total cost contribution from w_obj: "<<
      cost_contrib_obj_ang_vel<<std::endl;
    std::cout<<"\t total cost contribution from v_obj: "<<
      cost_contrib_obj_vel<<std::endl;

    std::cout<<"\t total cost is: "<< cost <<std::endl;
    std::cout<<"\t total cost object terms only is : "<< 
      cost_contrib_obj_pos + cost_contrib_obj_orientation +
      cost_contrib_obj_vel + cost_contrib_obj_ang_vel << std::endl;
    std::cout<<"\n\n";
  }

  // Return the cost and associated state trajectory.
  std::pair <double, std::vector<VectorXd>> ret (cost, XX);
  return ret;
}

std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>>
  C3::SimulatePDControl(
    double Kp_for_ee_pd_rollout, double Kd_for_ee_pd_rollout,
    bool force_tracking_disabled, bool verbose) const
  {
    if (verbose) {
      std::cout<<"\nSIMULATING PD CONTROL"<<std::endl;
    }

    // Obtain the solutions from C3.
    vector<VectorXd> UU(N_, VectorXd::Zero(k_));
    std::vector<Eigen::VectorXd> XX(N_+1, VectorXd::Zero(n_));
    for (int i = 0; i < N_; i++) {
      UU[i] = zfin_[i].segment(n_ + m_, k_);
      XX[i] = zfin_[i].segment(0, n_);
      if (i == N_-1) {
        if (lcs_for_cost_) {
          XX[i+1] = lcs_for_cost_->Simulate(XX[i], UU[i]);
        }
        else{
          XX[i+1] = lcs_.Simulate(XX[i], UU[i]);
        }
      }
    }
    
    // Set the PD gains for the emulated tracking controller.
    Eigen::MatrixXd Kp = Kp_for_ee_pd_rollout*Eigen::MatrixXd::Identity(3,3);
    Eigen::MatrixXd Kd = Kd_for_ee_pd_rollout*Eigen::MatrixXd::Identity(3,3);

    // Obtain modified solutions for the PD controller.
    std::vector<Eigen::VectorXd> UU_new(N_, VectorXd::Zero(k_));
    std::vector<Eigen::VectorXd> XX_new(N_+1, VectorXd::Zero(n_));

    XX_new[0] = zfin_[0].segment(0, n_);
    // This will just be the original u from zfin_[0] for the first time step.
    // if the radio input is true, then the u will only emulate position
    // tracking using the PD controller.
    for (int i = 0; i < N_; i++) {
        if (force_tracking_disabled) {
          UU_new[i] =  Kp*(XX[i].segment(0, 3) - XX_new[i].segment(0, 3)) + 
                       Kd*(XX[i].segment(10, 3) - XX_new[i].segment(10, 3));
        }
        else{
          UU_new[i] = UU[i] + 
            Kp*(XX[i].segment(0, 3) - XX_new[i].segment(0, 3)) + 
            Kd*(XX[i].segment(10, 3) - XX_new[i].segment(10, 3));
        }
        if (lcs_for_cost_) {

          if (verbose) {
            std::cout<<"simulated step "<<i+1<<std::endl;
          }
          XX_new[i+1] = lcs_for_cost_->Simulate(XX_new[i], UU_new[i], verbose);
        }
        else{
          XX_new[i+1] = lcs_.Simulate(XX_new[i], UU_new[i]);
        }
    }
    return {XX_new, UU_new};
}

void C3::ADMMStep(const VectorXd& x0, vector<VectorXd>* delta,
                  vector<VectorXd>* w, vector<MatrixXd>* Gv,
                  int admm_iteration, bool verbose) {
  vector<VectorXd> WD(N_, VectorXd::Zero(n_ + m_ + k_));

  for (int i = 0; i < N_; i++) {
    WD.at(i) = delta->at(i) - w->at(i);
  }

  vector<VectorXd> z = SolveQP(x0, *Gv, WD, admm_iteration, true);
  if (verbose) {
    std::cout << "SolveQP Iteration: " << admm_iteration << std::endl;
    Eigen::MatrixXd verbose_z = Eigen::MatrixXd::Zero(n_ + m_ + k_, N_);
    for(int i = 0; i < N_; i++) {
      verbose_z.col(i) = z[i];
    }
    std::cout<<"z: \n"<<verbose_z<<std::endl;
  }

  vector<VectorXd> ZW(N_, VectorXd::Zero(n_ + m_ + k_));
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
    Eigen::MatrixXd verbose_delta = Eigen::MatrixXd::Zero(n_ + m_ + k_, N_);
    for(int i = 0; i < N_; i++) {
      verbose_delta.col(i) = delta->at(i);
    }
    std::cout<<"delta: \n"<<verbose_delta<<std::endl;
  }

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
    LCPSolver.SolveLcpLemkeRegularized(F_[0], E_[0] * x0 + c_[0], &lambda0);
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
  else {
    std::cout<<"CAUTION: C3 QP solve did not succeed" << std::endl;
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
    // TODO: A newer version of OpenMP likely uses omp_set_max_active_levels
    // instead of omp_set_nested. Consider updating this after testing.
    omp_set_nested(1);  // Enable nested parallelism (important for
                        // sampling_c3_controller)
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
              admm_iteration, i + 1);
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
                                               c_[i], admm_iteration, i + 1);
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
