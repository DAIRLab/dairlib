#include "solvers/c3_nextgen.h"

#include <cfenv>
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
  // Initialize result vector with input values
  VectorXd delta_proj = delta_c;

  // Set epsilon for floating point comparisons
  const double EPSILON = 1e-8;

  // Set rounding mode to nearest
  std::fesetround(FE_TONEAREST);

  // Handle complementarity constraints for each lambda-gamma pair
  for (int i = 0; i < m_; ++i) {
    // Calculate ratio of U matrix elements for scaling with more stable
    // computation
    double u_ratio = 0.0;
    double u1 =
        std::abs(U(n_ + m_ + k_ + i, n_ + m_ + k_ + i));
    double u2 = std::abs(U(n_ + i, n_ + i));

    if (u2 < EPSILON) {
      throw std::runtime_error("Numerical instability detected: u2 is very "
                               "close to zero in SolveSingleProjection");
    }
    u_ratio = std::sqrt(u1 / u2);
    // print u_ratio for debugging
    // std::cout << "u_ratio: " << u_ratio << std::endl;

    // Get current lambda and gamma values
    double lambda_val = delta_c(n_ + i);
    double gamma_val = delta_c(n_ + m_ + k_ + i);

    // Case 1: lambda < -EPSILON
    // In this case, we always set lambda to 0 and keep gamma if it's positive
    if (lambda_val < -EPSILON) {
      delta_proj(n_ + i) = 0;
      delta_proj(n_ + m_ + k_ + i) = std::max(EPSILON, gamma_val);
    }
    // Case 2: -EPSILON ≤ lambda ≤ EPSILON (lambda is very close to zero)
    else if (std::abs(lambda_val) <= EPSILON) {
      if (gamma_val < -EPSILON) {
        // If gamma is negative, set lambda to EPSILON and gamma to 0
        delta_proj(n_ + i) = EPSILON;
        delta_proj(n_ + m_ + k_ + i) = 0;
      } else if (std::abs(gamma_val) <= EPSILON) {
        // If both are close to zero, set both to 0
        delta_proj(n_ + i) = 0;
        delta_proj(n_ + m_ + k_ + i) = 0;
      } else {
        // If gamma is positive, set lambda to 0 and keep gamma
        delta_proj(n_ + i) = 0;
        delta_proj(n_ + m_ + k_ + i) = gamma_val;
      }
    }
    // Case 3: lambda > EPSILON
    else {
      if (gamma_val < -EPSILON) {
        // If gamma is negative, keep lambda and set gamma to 0
        delta_proj(n_ + i) = lambda_val;
        delta_proj(n_ + m_ + k_ + i) = 0;
      } else if (std::abs(gamma_val) <= EPSILON) {
        // If gamma is close to zero, keep lambda and set gamma to 0
        delta_proj(n_ + i) = lambda_val;
        delta_proj(n_ + m_ + k_ + i) = 0;
      } else {
        // If gamma is positive, compare with u_ratio * lambda
        if (gamma_val > u_ratio * lambda_val + EPSILON) {
          // If gamma is significantly larger, keep lambda and set gamma to 0
          delta_proj(n_ + i) = 0;
          delta_proj(n_ + m_ + k_ + i) = gamma_val;
        } else {
          // Otherwise, set lambda to 0 and keep gamma
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