#pragma once

#include <vector>
#include <Eigen/Dense>
#include "solvers/c3.h"
#include "solvers/lcs.h"
#include "solvers/c3_options.h"

namespace dairlib {
namespace solvers {

class C3NextGen final : public C3 {
 public:
  /// Default constructor for time-varying LCS
  C3NextGen(const LCS& LCS, const CostMatrices& costs,
       const std::vector<Eigen::VectorXd>& xdesired,
       const C3Options& options);

  ~C3NextGen() override = default;

  // In C4 projection step, we aim to solve the following problem
  //
  //   min_{λ, η}   w_λ ||λ - λ₀||² + w_η ||η - η₀||²
  //   s.t.        0 ≤ λ ⊥ η ≥ 0
  //
  // where λ₀ and η₀ are the values of λ and η obtained from the QP step, respectively.
  // The solution to this problem is the projection of (λ₀, η₀) onto the feasible set
  // defined by the complementarity condition (i.e., λᵢ ηᵢ = 0 for all i, with
  // λ ≥ 0 and η ≥ 0).
  //
  // To get the solution, we can simply perform if-else to handle the following cases:
  //
  // 1. λ₀ <= 0 and η₀ > 0, then λ = 0 and η = η₀
  // 2. λ₀ <= 0 and η₀ <= 0 then λ = 0 and η = 0
  // 3. λ₀ > 0 and η₀ <= 0, then λ = λ₀ and η = 0
  // 4. λ₀ > 0, η₀ > 0, and η₀ > sqrt(w_λ/w_η) * λ₀, then λ = 0 and η = η₀
  // 5. λ₀ > 0, η₀ > 0, and η₀ <= sqrt(w_λ/w_η) * λ₀, then λ = λ₀ and η = 0
  Eigen::VectorXd SolveSingleProjection(const Eigen::MatrixXd& U,
                                        const Eigen::VectorXd& delta_c,
                                        const Eigen::MatrixXd& E,
                                        const Eigen::MatrixXd& F,
                                        const Eigen::MatrixXd& H,
                                        const Eigen::VectorXd& c,
                                        const int admm_iteration,
                                        const int& warm_start_index = -1) override;
  /// Robust projection method
  Eigen::VectorXd SolveRobustSingleProjection(
      const Eigen::MatrixXd& U, const Eigen::VectorXd& delta_c,
      const Eigen::MatrixXd& E, const Eigen::MatrixXd& F,
      const Eigen::MatrixXd& H, const Eigen::VectorXd& c,
      const Eigen::MatrixXd& W_x, const Eigen::MatrixXd& W_l,
      const Eigen::MatrixXd& W_u, const Eigen::VectorXd& w,
      const int admm_iteration, const int& warm_start_index = -1) override;
  std::vector<Eigen::VectorXd> GetWarmStartDelta() const;
  std::vector<Eigen::VectorXd> GetWarmStartBinary() const;

};

}  // namespace solvers
}  // namespace dairlib