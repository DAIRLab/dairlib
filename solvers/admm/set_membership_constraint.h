#pragma once

#include "solvers/nonlinear_constraint.h"

namespace dairlib::solvers {

/*!
 * Generic set-membership constraint class for the constraint x \in X.
 * Subclasses should implement EvaluateConstraint so that the output is set to
 * within the bounds for feasible points.
 *
 * We are mostly interested in implementing the ProjectToFeasibleSet
 * function, which will be used in ADMM.
 */
class SetMembershipConstraint : public NonlinearConstraint<double> {
 public:
  SetMembershipConstraint(int num_constraints, int num_vars,
                      const Eigen::VectorXd& lb, const Eigen::VectorXd& ub);

  void EvaluateConstraint(
      const Eigen::Ref<const drake::VectorX<double>>& x,
      drake::VectorX<double>* y) const override;

  virtual void ProjectToFeasibleSet(
      const Eigen::Ref<const drake::VectorX<double>>& x,
      drake::VectorX<double>* y) const = 0;

  // return a pointer to the previously projected to convex component
  // associated with this constraint
  virtual
  std::tuple<Eigen::MatrixXd, Eigen::VectorXd, Eigen::VectorXd>
  CalcClosestConvexRestrictionToQP(
          const Eigen::Ref<const drake::VectorX<double>>& x) const = 0;

};

}