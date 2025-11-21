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
    : C3Base(LCS, costs, xdesired, options,
             LCS.A_[0].cols() + 2 * LCS.D_[0].cols() + LCS.B_[0].cols()) {
  // Initialize eta as optimization variables
  eta_ = vector<drake::solvers::VectorXDecisionVariable>();
  eta_sol_ = std::make_unique<std::vector<VectorXd>>();
  for (int i = 0; i < N_; ++i) {
    eta_sol_->push_back(Eigen::VectorXd::Zero(m_));
    eta_.push_back(prog_.NewContinuousVariables(m_, "eta" + std::to_string(i)));
  }

  // Add eta equality constraints η = E * x + F * λ + H * u + c
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

  // Disable parallelization for C3+ because of the overhead cost
  use_parallelization_in_projection_ = false;
}

void C3Plus::UpdateLCS(const LCS& lcs) {
  C3Base::UpdateLCS(lcs);

  // Update eta equality constraints with new LCS
  MatrixXd EtaLinEq(m_, n_ + 2 * m_ + k_);
  EtaLinEq.block(0, n_ + m_ + k_, m_, m_) = -1 * MatrixXd::Identity(m_, m_);
  for (int i = 0; i < N_; ++i) {
    EtaLinEq.block(0, 0, m_, n_) = E_.at(i);
    EtaLinEq.block(0, n_, m_, m_) = F_.at(i);
    EtaLinEq.block(0, n_ + m_, m_, k_) = H_.at(i);
    eta_constraints_[i]->UpdateCoefficients(EtaLinEq, -c_.at(i));
  }
}

void C3Plus::AddAugmentedCostsQPStep(const std::vector<Eigen::MatrixXd>& G,
                                     const std::vector<Eigen::VectorXd>& WD,
                                     const std::vector<Eigen::VectorXd>& delta,
                                     bool is_final_solve) {
  int large_coeff = 1000;
  if (is_final_solve) {
    std::vector<Eigen::MatrixXd> last_qp_G = G;
    for (int i = 0; i < N_; ++i) {
      for (int j = 4; j < 8; ++j) {  // TODO @bibit don't hardcode this
        if (delta.at(i)[n_ + j] == 0) {
          last_qp_G.at(i).block(n_ + j, n_ + j, 1, 1) *= large_coeff;
        } else {
          last_qp_G.at(i).block(n_ + m_ + k_ + j, n_ + m_ + k_ + j, 1, 1) *=
              large_coeff;
        }
      }
    }

    for (int i = 0; i < N_; i++) {
      costs_.push_back(
          prog_.AddQuadraticCost(2 * last_qp_G.at(i).block(n_, n_, m_, m_),
                                 -2 * last_qp_G.at(i).block(n_, n_, m_, m_) *
                                     delta.at(i).segment(n_, m_),
                                 lambda_.at(i), 1));
      costs_.push_back(prog_.AddQuadraticCost(
          2 * last_qp_G.at(i).block(n_ + m_ + k_, n_ + m_ + k_, m_, m_),
          -2 * last_qp_G.at(i).block(n_ + m_ + k_, n_ + m_ + k_, m_, m_) *
              delta.at(i).segment(n_ + m_ + k_, m_),
          eta_.at(i), 1));
    }
  } else {
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
}

void C3Plus::ExtractQPSolution(
    const drake::solvers::MathematicalProgramResult& result, int admm_iteration,
    bool is_final_solve) {
  C3Base::ExtractQPSolution(result, admm_iteration, is_final_solve);
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
  C3Base::UpdateWarmStarts(result, admm_iteration);
}

VectorXd C3Plus::SolveSingleProjection(const MatrixXd& U,
                                       const VectorXd& delta_c,
                                       const MatrixXd& E, const MatrixXd& F,
                                       const MatrixXd& H, const VectorXd& c,
                                       const int admm_iteration,
                                       const int& warm_start_index) {
  VectorXd delta_proj = delta_c;

  // Extract the weight vectors for lambda and eta from the diagonal of the cost
  // matrix U.
  VectorXd w_eta_vec = U.block(n_ + m_ + k_, n_ + m_ + k_, m_, m_).diagonal();
  VectorXd w_lambda_vec = U.block(n_, n_, m_, m_).diagonal();

  // Throw an error if any weights are negative.
  if (w_eta_vec.minCoeff() < 0 || w_lambda_vec.minCoeff() < 0) {
    throw std::runtime_error(
        "Negative weights in the cost matrix U are not allowed.");
  }

  VectorXd lambda_c = delta_c.segment(n_, m_);
  VectorXd eta_c = delta_c.segment(n_ + m_ + k_, m_);

  // Set the smaller of lambda and eta to zero
  Eigen::Array<bool, Eigen::Dynamic, 1> eta_larger =
      eta_c.array() * w_eta_vec.array().sqrt() >
      lambda_c.array() * w_lambda_vec.array().sqrt();

  delta_proj.segment(n_, m_) = eta_larger.select(VectorXd::Zero(m_), lambda_c);
  delta_proj.segment(n_ + m_ + k_, m_) =
      eta_larger.select(eta_c, VectorXd::Zero(m_));

  // Clip lambda and eta at 0
  delta_proj.segment(n_, m_) = delta_proj.segment(n_, m_).cwiseMax(0);
  delta_proj.segment(n_ + m_ + k_, m_) =
      delta_proj.segment(n_ + m_ + k_, m_).cwiseMax(0);

  return delta_proj;
}
}  // namespace solvers
}  // namespace dairlib
