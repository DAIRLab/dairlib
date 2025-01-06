#include "solvers/c3_miqp.h"

namespace dairlib {
namespace solvers {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

C3MIQP::C3MIQP(const LCS& LCS, const CostMatrices& costs,
               const vector<VectorXd>& xdesired, const C3Options& options)
    : C3(LCS, costs, xdesired, options), M_(options.M) {
  throw std::runtime_error(
      "The Gurobi bindings were not compiled. You'll need to use a different "
      "projection method. To compile with Gurobi, follow the instruction "
      "listed in install/README");
}

VectorXd C3MIQP::SolveSingleProjection(const MatrixXd& U,
                                       const VectorXd& delta_c,
                                       const MatrixXd& E, const MatrixXd& F,
                                       const MatrixXd& H, const VectorXd& c,
                                       const int admm_iteration,
                                       const int& warm_start_index) {
  throw std::runtime_error(
      "The Gurobi bindings were not compiled.  You'll need to use a different "
      "projection method.");
}

std::vector<Eigen::VectorXd> C3MIQP::GetWarmStartDelta() const {
  throw std::runtime_error(
      "The Gurobi bindings were not compiled.  You'll need to use a different "
      "projection method.");
}

std::vector<Eigen::VectorXd> C3MIQP::GetWarmStartBinary() const {
  throw std::runtime_error(
      "The Gurobi bindings were not compiled.  You'll need to use a different "
      "projection method.");
}

}  // namespace solvers
}  // namespace dairlib
