#pragma once

#include "solvers/nonlinear_constraint.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"


namespace dairlib::systems::controllers {

class NonlinearPendulumDynamicsConstraint : public solvers::NonlinearConstraint<drake::AutoDiffXd> {
 public:
  /*!
   * trapezoidal collocation with the variable order
   * x0, x1, u0, u1, t_interval
   * @param num_intervals
   */
  NonlinearPendulumDynamicsConstraint(int num_intervals, double mass);

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<drake::AutoDiffXd>>& x,
                          drake::VectorX<drake::AutoDiffXd>* y) const override;

  double num_intervals_;
  double m_;
};

}