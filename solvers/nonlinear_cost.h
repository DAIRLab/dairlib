#pragma once

#include <string>
#include <unordered_map>
#include "drake/common/symbolic/expression.h"
#include "drake/solvers/cost.h"

namespace dairlib {
namespace solvers {

/// Abstract class for nonlinear cost that
/// manages evaluation of functions and numerical differentiation
///
/// Subclasses should implement the method EvaluateCost
template <typename T>
class NonlinearCost : public drake::solvers::Cost {
 public:
  /// Constructor
  /// @param num_vars number of input variables
  /// @param description (default blank)
  /// @param eps step size for numerical gradient (default 1e-8)
  NonlinearCost(int num_vars, const std::string& description = "",
                double eps = 1e-8);

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& x,
              drake::AutoDiffVecXd* y) const override;

  void DoEval(
      const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>&,
      drake::VectorX<drake::symbolic::Expression>*) const override;

  virtual void EvaluateCost(const Eigen::Ref<const drake::VectorX<T>>& x,
                            drake::VectorX<T>* y) const = 0;

 private:
  double eps_;
};

}  // namespace solvers
}  // namespace dairlib
