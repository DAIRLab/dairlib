#pragma once

#include <vector>

#include <Eigen/Dense>

#include "solvers/base_c3.h"
#include "solvers/c3_options.h"
#include "solvers/lcs.h"

namespace dairlib {
namespace solvers {

// C3+ is a variant of C3 that introduces a new slack variable, η, and a new
// equality constraint, η = E * x + F * λ + H * u + c, to the QP step.
// This allows us to solve the projection step in a more efficient way (without
// solving any optimization problem).
// The z and delta vector now have the following structure [x, λ, u, η].
//
// In C3+ QP step, we solve the following problem:
//
//   min_{x, u, λ, η}   g_x ||x - x_des||² + g_u ||u||² +
//                               g_λ ||λ - λ₀||² + g_η ||η - η₀||²
//           s.t.       x_next = A * x + B * u + D * λ + d
//                      η = E * x + F * λ + H * u + c
//                      u_min ≤ u ≤ u_max
//                      x_min ≤ x ≤ x_max
//
// In C3+ projection step, we aim to solve the following problem
//
//   min_{λ, η}   w_λ ||λ - λ₀||² + w_η ||η - η₀||²
//   s.t.        0 ≤ λ ⊥ η ≥ 0
//
// where λ₀ and η₀ are the values of λ and η obtained from the QP step,
// respectively. The solution to this problem is the projection of (λ₀, η₀)
// onto the feasible set defined by the complementarity condition (i.e., λᵢ ηᵢ
// = 0 for all i, with λ ≥ 0 and η ≥ 0).
//
// To get the solution, we can simply do the following steps:
// 1. If η₀ > sqrt(w_λ/w_η) * λ₀, then λ = 0, else η = 0
// 2. [λ, η] = max(0, [λ, η])
class C3Plus final : public C3Base {
 public:
  C3Plus(const LCS& LCS, const CostMatrices& costs,
         const std::vector<Eigen::VectorXd>& xdesired,
         const C3Options& options);
  ~C3Plus() override = default;

  Eigen::VectorXd SolveSingleProjection(
      const Eigen::MatrixXd& U, const Eigen::VectorXd& delta_c,
      const Eigen::MatrixXd& E, const Eigen::MatrixXd& F,
      const Eigen::MatrixXd& H, const Eigen::VectorXd& c,
      const int admm_iteration, const int& warm_start_index = -1) override;
// This is used for adding a non-negative constraint on lambda and eta for the target
// group of contacts. This only been used by C3+
  void AddNonnegativityConstraintsOnLambdaEta(
      const std::vector<int>& resolve_contacts_to_lists,
      const std::vector<int>& resolve_as_planar_contacts_list,
      const std::vector<int>& add_nonnegative_constraints_on_lambdaeta_list,
      int num_friction_directions) override;

 private:
  void UpdateLCS(const LCS& lcs) override;
  void AddAugmentedCostsQPStep(const std::vector<Eigen::MatrixXd>& G,
                               const std::vector<Eigen::VectorXd>& WD) override;
  void ExtractQPSolution(
      const drake::solvers::MathematicalProgramResult& result,
      int admm_iteration, bool is_final_solve) override;
  void UpdateWarmStarts(const drake::solvers::MathematicalProgramResult& result,
                        int admm_iteration) override;
  std::vector<drake::solvers::VectorXDecisionVariable> eta_;
  std::unique_ptr<std::vector<Eigen::VectorXd>> eta_sol_;

  // Store the following constraint η = E * x + F * λ + H * u + c
  std::vector<drake::solvers::LinearEqualityConstraint*> eta_constraints_;
};

}  // namespace solvers
}  // namespace dairlib
