#include "solvers/c3_nextgen.h"

#include <vector>

#include <Eigen/Dense>

#include "solvers/c3_options.h"
#include "solvers/lcs.h"

namespace dairlib {
namespace solvers {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

C3NextGen::C3NextGen(const LCS& LCS, const CostMatrices& costs,
           const vector<VectorXd>& xdesired, const C3Options& options)
    : C3(LCS, costs, xdesired, options) {}

VectorXd C3NextGen::SolveSingleProjection(const MatrixXd& U, const VectorXd& delta_c,
                                     const MatrixXd& E, const MatrixXd& F,
                                     const MatrixXd& H, const VectorXd& c,
                                     const int admm_iteration,
                                     const int& warm_start_index) {
  VectorXd delta_proj = delta_c;

  // Handle complementarity constraints for each lambda-eta pair
  for (int i = 0; i < m_; ++i) {
    double u1 =
        std::abs(U(n_ + m_ + k_ + i, n_ + m_ + k_ + i));
    double u2 = std::abs(U(n_ + i, n_ + i));

    double lambda_val = delta_c(n_ + i);
    double eta_val = delta_c(n_ + m_ + k_ + i);

    if (lambda_val < 0) {
      delta_proj(n_ + i) = 0;
      delta_proj(n_ + m_ + k_ + i) = std::max(0.0, eta_val);
    }
    else {
      if (eta_val < 0) {
        delta_proj(n_ + i) = lambda_val;
        delta_proj(n_ + m_ + k_ + i) = 0;
      } else {
        // If point (lambda, eta) is above the slope sqrt(u1/u2), set lambda to 0 and keep eta
        // Otherwise, set lambda to lambda and set eta to 0
        if (eta_val * std::sqrt(u2) > lambda_val * std::sqrt(u1)) {
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

VectorXd C3NextGen::SolveRobustSingleProjection(const MatrixXd& U, const VectorXd& delta_c, const MatrixXd& E,
                                             const MatrixXd& F, const MatrixXd& H, const VectorXd& c,
                                             const Eigen::MatrixXd& W_x, const Eigen::MatrixXd& W_l,
                                             const Eigen::MatrixXd& W_u, const Eigen::VectorXd& w,
                                             const int admm_iteration, const int& warm_start_index) {
  return delta_c;
}

std::vector<Eigen::VectorXd> C3NextGen::GetWarmStartDelta() const {
  return warm_start_delta_[0];
}

std::vector<Eigen::VectorXd> C3NextGen::GetWarmStartBinary() const {
  return warm_start_binary_[0];
}

}  // namespace solvers
}  // namespace dairlib