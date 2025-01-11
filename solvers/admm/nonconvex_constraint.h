#pragma once

#include "solvers/nonlinear_constraint.h"

namespace dairlib::solvers {

class NonconvexConstraint : public NonlinearConstraint<double> {
 public:
  NonconvexConstraint(int num_constraints, int num_vars,
                      const Eigen::VectorXd& lb, const Eigen::VectorXd& ub);

  virtual void EvaluateConstraint(
      const Eigen::Ref<const drake::VectorX<double>>& x,
      drake::VectorX<double>* y) const = 0;

  virtual void ProjectToFeasibleSet(
      const Eigen::Ref<const drake::VectorX<double>>& x,
      drake::VectorX<double>* y);

};

}