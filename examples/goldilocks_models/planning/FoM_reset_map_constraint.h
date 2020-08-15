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

// Impose two constraint s
//   1. discrete map
//   2, zero foot velocity at post impact
// Input order: x_pre_impact, v_post_impact, impulse
class FomResetMapConstraint : public solvers::NonlinearConstraint<double> {
 public:
  FomResetMapConstraint(
      const drake::multibody::MultibodyPlant<double>& plant,
      const std::vector<std::pair<const Eigen::Vector3d,
                                  const drake::multibody::Frame<double>&>>&
          impact_foot_contacts,
      const std::string& description = "");

 private:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& x,
                          drake::VectorX<double>* y) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::multibody::BodyFrame<double>& world_;
  std::unique_ptr<drake::systems::Context<double>> context_;
  const std::vector<
      std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>>&
      impact_foot_contacts_;

  int n_q_;
  int n_v_;
  int n_lambda_;
};
}  // namespace planning
}  // namespace goldilocks_models
}  // namespace dairlib
