#pragma once

#include <string>
#include <unordered_map>
#include "drake/common/symbolic/expression.h"
#include "drake/solvers/constraint.h"

namespace dairlib {
namespace solvers {

/// Abstract class for nonlinear constraints that manages 
/// manages evaluation of functions and numerical differentiation
/// 
/// Subclasses should implement the method EvaluateConstraint
template <typename T>
class NonlinearConstraint : public drake::solvers::Constraint {
 public:
  /// Constructor
  /// @param num_constraints length of constraint vector
  /// @param num_vars number of input variables
  /// @param lb lower bound
  /// @param ub lower bound
  /// @param description (default blank)
  /// @param eps step size for numerical gradient (default 1e-7)
  NonlinearConstraint(int num_constraints, int num_vars,
      const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
      const std::string& description = "", double eps=1e-7);

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& x,
              drake::AutoDiffVecXd* y) const override;

  void DoEval(
      const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>&,
      drake::VectorX<drake::symbolic::Expression>*) const override;

  void SetConstraintScaling(const std::unordered_map<int, double>& map);

  virtual void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                                  drake::VectorX<T>* y) const = 0;

 private:
  template <typename U>
  void ScaleConstraint(drake::VectorX<U>* y) const;
  std::unordered_map<int, double> constraint_scaling_;
  double eps_;
};

}  // namespace solvers
}  // namespace dairlib
