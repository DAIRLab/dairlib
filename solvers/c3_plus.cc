#include "solvers/c3_plus.h"

#include <vector>

#include <Eigen/Dense>

#include "solvers/c3_options.h"
#include "solvers/lcs.h"

namespace dairlib {
namespace solvers {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

C3Plus::C3Plus(const LCS& LCS, const CostMatrices& costs,
               const vector<VectorXd>& xdesired, const C3Options& options)
    : BaseC3(LCS, costs, xdesired, options, [](const class LCS& lcs) {
        return (lcs.A_)[0].cols() + 2 * (lcs.D_)[0].cols() + (lcs.B_)[0].cols();
      }) {
  InitializeEtaAsOptimizationVariables();
  AddEtaEqualityConstraints();

  // Disable parallelization for C3+ because of the overhead cost
  use_parallelization_in_projection_ = false;
}

void C3Plus::UpdateLCS(const LCS& lcs) {
  BaseC3::UpdateLCS(lcs);
  MatrixXd EtaLinEq(m_, n_ + 2 * m_ + k_);
  EtaLinEq.block(0, n_ + m_ + k_, m_, m_) = -1 * MatrixXd::Identity(m_, m_);
  for (int i = 0; i < N_; ++i) {
    EtaLinEq.block(0, 0, m_, n_) = E_.at(i);
    EtaLinEq.block(0, n_, m_, m_) = F_.at(i);
    EtaLinEq.block(0, n_ + m_, m_, k_) = H_.at(i);

    eta_constraints_[i]->UpdateCoefficients(EtaLinEq, -c_.at(i));
  }
}

void C3Plus::InitializeEtaAsOptimizationVariables() {
  eta_ = vector<drake::solvers::VectorXDecisionVariable>();
  eta_sol_ = std::make_unique<std::vector<VectorXd>>();
  for (int i = 0; i < N_; ++i) {
    eta_sol_->push_back(Eigen::VectorXd::Zero(m_));
    eta_.push_back(prog_.NewContinuousVariables(m_, "eta" + std::to_string(i)));
  }
}

void C3Plus::AddEtaEqualityConstraints() {
  MatrixXd EtaLinEq(m_, n_ + 2 * m_ + k_);
  EtaLinEq.block(0, n_ + m_ + k_, m_, m_) = -1 * MatrixXd::Identity(m_, m_);
  eta_constraints_.resize(N_);
  for (int i = 0; i < N_; ++i) {
    EtaLinEq.block(0, 0, m_, n_) = E_.at(i);
    EtaLinEq.block(0, n_, m_, m_) = F_.at(i);
    EtaLinEq.block(0, n_ + m_, m_, k_) = H_.at(i);

    eta_constraints_[i] =
        prog_
            .AddLinearEqualityConstraint(
                EtaLinEq, -c_.at(i),
                {x_.at(i), lambda_.at(i), u_.at(i), eta_.at(i)})
            .evaluator()
            .get();
  }
}

void C3Plus::AddMatchingCostsQPStep(const std::vector<Eigen::MatrixXd>& G,
                                    const std::vector<Eigen::VectorXd>& WD) {
  for (int i = 0; i < N_; i++) {
    costs_.push_back(prog_.AddQuadraticCost(
        2 * G.at(i).block(n_, n_, m_, m_),
        -2 * G.at(i).block(n_, n_, m_, m_) * WD.at(i).segment(n_, m_),
        lambda_.at(i), 1));
    costs_.push_back(prog_.AddQuadraticCost(
        2 * G.at(i).block(n_ + m_ + k_, n_ + m_ + k_, m_, m_),
        -2 * G.at(i).block(n_ + m_ + k_, n_ + m_ + k_, m_, m_) *
            WD.at(i).segment(n_ + m_ + k_, m_),
        eta_.at(i), 1));
  }
}

void C3Plus::ExtractQPSolution(
    const drake::solvers::MathematicalProgramResult& result, int admm_iteration,
    bool is_final_solve) {
  BaseC3::ExtractQPSolution(result, admm_iteration, is_final_solve);
  for (int i = 0; i < N_; i++) {
    if (is_final_solve) {
      eta_sol_->at(i) = result.GetSolution(eta_[i]);
    }
    z_sol_->at(i).segment(n_ + m_ + k_, m_) = result.GetSolution(eta_[i]);
  }
}

void C3Plus::UpdateWarmStarts(
    const drake::solvers::MathematicalProgramResult& result,
    int admm_iteration) {
  BaseC3::UpdateWarmStarts(result, admm_iteration);
}

VectorXd C3Plus::SolveSingleProjection(const MatrixXd& U,
                                       const VectorXd& delta_c,
                                       const MatrixXd& E, const MatrixXd& F,
                                       const MatrixXd& H, const VectorXd& c,
                                       const int admm_iteration,
                                       const int& warm_start_index) {
  VectorXd delta_proj = delta_c;

  // Handle complementarity constraints for each lambda-eta pair
  for (int i = 0; i < m_; ++i) {
    double w_eta = std::abs(U(n_ + m_ + k_ + i, n_ + m_ + k_ + i));
    double w_lambda = std::abs(U(n_ + i, n_ + i));

    double lambda_val = delta_c(n_ + i);
    double eta_val = delta_c(n_ + m_ + k_ + i);

    if (lambda_val <= 0) {
      delta_proj(n_ + i) = 0;
      delta_proj(n_ + m_ + k_ + i) = std::max(0.0, eta_val);
    } else {
      if (eta_val <= 0) {
        delta_proj(n_ + i) = lambda_val;
        delta_proj(n_ + m_ + k_ + i) = 0;
      } else {
        // If point (lambda, eta) is above the slope sqrt(w_lambda/w_eta), set
        // lambda to 0 and keep eta Otherwise, set lambda to lambda and set eta
        // to 0
        if (eta_val * std::sqrt(w_eta) > lambda_val * std::sqrt(w_lambda)) {
          delta_proj(n_ + i) = 0;
          delta_proj(n_ + m_ + k_ + i) = eta_val;
        } else {
          delta_proj(n_ + i) = lambda_val;
          delta_proj(n_ + m_ + k_ + i) = 0;
        }
      }
    }
  }

  return delta_proj;
}

VectorXd C3Plus::SolveRobustSingleProjection(
    const MatrixXd& U, const VectorXd& delta_c, const MatrixXd& E,
    const MatrixXd& F, const MatrixXd& H, const VectorXd& c,
    const Eigen::MatrixXd& W_x, const Eigen::MatrixXd& W_l,
    const Eigen::MatrixXd& W_u, const Eigen::VectorXd& w,
    const int admm_iteration, const int& warm_start_index) {
  return delta_c;
}
}  // namespace solvers
}  // namespace dairlib
