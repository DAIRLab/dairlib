#pragma once

#include <string>

#include "examples/goldilocks_models/reduced_order_models.h"
#include "multibody/multibody_utils.h"
#include "solvers/nonlinear_constraint.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"

namespace dairlib {
namespace goldilocks_models {
namespace planning {

class KinematicsConstraintOnlyPos
    : public solvers::NonlinearConstraint<double> {
 public:
  KinematicsConstraintOnlyPos(
      const ReducedOrderModel& rom,
      const drake::multibody::MultibodyPlant<double>& plant,
      const std::string& description = "");

 private:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& rq,
                          drake::VectorX<double>* value) const override;

  const ReducedOrderModel& rom_;
  const drake::multibody::MultibodyPlant<double>& plant_;
  std::unique_ptr<drake::systems::Context<double>> context_;
  int n_y_;
  int n_q_;
};
}  // namespace planning
}  // namespace goldilocks_models
}  // namespace dairlib
