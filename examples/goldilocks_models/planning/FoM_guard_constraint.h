#pragma once

#include <string>

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

// Constraints are
//   contact 1 vertical pos = 0
//   contact 1 vertical vel <= 0  ( = 0 if zero impact)
//   contact 2 vertical pos = 0
//   contact 2 vertical vel <= 0  ( = 0 if zero impact)
//   ...
class FomGuardConstraint : public solvers::NonlinearConstraint<double> {
 public:
  FomGuardConstraint(
      const drake::multibody::MultibodyPlant<double>& plant,
      const std::vector<std::pair<const Eigen::Vector3d,
                                  const drake::multibody::Frame<double>&>>&
          swing_foot_contacts,
      const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
      const std::string& description = "fom_guard_constraint");

 private:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& x,
                          drake::VectorX<double>* y) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::multibody::BodyFrame<double>& world_;
  std::unique_ptr<drake::systems::Context<double>> context_;
  const std::vector<
      std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>>&
      swing_foot_contacts_;
};
}  // namespace planning
}  // namespace goldilocks_models
}  // namespace dairlib
