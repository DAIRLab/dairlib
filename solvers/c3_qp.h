#pragma once

#include <vector>

#include <Eigen/Dense>

#include "gurobi_c++.h"
#include "solvers/c3.h"
#include "solvers/lcs.h"

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/moby_lcp_solver.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/solve.h"

#include "solvers/c3_options.h"
#include "solvers/fast_osqp_solver.h"


namespace dairlib {
namespace solvers {

class C3QP final : public C3 {
 public:
  /// Default constructor for time-varying LCS
  C3QP(const LCS& LCS, const CostMatrices& costs,
       const std::vector<Eigen::VectorXd>& xdesired,
       const C3Options& options);

  ~C3QP() override = default;

  /// Virtual projection method
  Eigen::VectorXd SolveSingleProjection(const Eigen::MatrixXd& U,
                                        const Eigen::VectorXd& delta_c,
                                        const Eigen::MatrixXd& E,
                                        const Eigen::MatrixXd& F,
                                        const Eigen::MatrixXd& H,
                                        const Eigen::VectorXd& c,
                                        const int admm_iteration,
                                        const int& warm_start_index = -1) override;
  std::vector<Eigen::VectorXd> GetWarmStartDelta() const;
  std::vector<Eigen::VectorXd> GetWarmStartBinary() const;

};

}  // namespace solvers
}  // namespace dairlib
