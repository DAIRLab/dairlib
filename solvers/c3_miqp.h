#pragma once

#include <vector>

#include <Eigen/Dense>

#include "gurobi_c++.h"
#include "solvers/c3.h"
#include "solvers/lcs.h"

namespace dairlib {
namespace solvers {

class C3MIQP final : public C3 {
 public:
  /// Default constructor for time-varying LCS
  C3MIQP(const LCS& LCS, const CostMatrices& costs,
         const std::vector<Eigen::VectorXd>& xdesired, const C3Options& options);

  ~C3MIQP() override = default;

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

 private:
  GRBEnv env_;
};

}  // namespace solvers
}  // namespace dairlib
