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

class DynamicsConstraint : public solvers::NonlinearConstraint<double> {
 public:
  DynamicsConstraint(const ReducedOrderModel& rom,
                     const std::set<int>& idx_constant_rom_vel,
                     const std::string& description = "rom_dyn_constraint");

 private:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& x,
                          drake::VectorX<double>* y) const override;

  // z = [y; ydot]
  // Calculate the dynamics for z
  drake::VectorX<double> g(const drake::VectorX<double>& z,
                           const drake::VectorX<double>& tau) const;

  const ReducedOrderModel& rom_;
  int n_y_;
  int n_z_;
  int n_tau_;

  const std::set<int>& idx_constant_rom_vel_;
};
}  // namespace planning
}  // namespace goldilocks_models
}  // namespace dairlib
