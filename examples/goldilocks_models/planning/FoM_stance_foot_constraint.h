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
//   stance foot pos (start of the mode) = stance foot pos (end of the mode)
// The constraint dimension is 3 * (# of contact points)
class FomStanceFootPosConstraint : public solvers::NonlinearConstraint<double> {
 public:
  FomStanceFootPosConstraint(
      const drake::multibody::MultibodyPlant<double>& plant,
      const std::vector<std::pair<const Eigen::Vector3d,
                                  const drake::multibody::Frame<double>&>>&
          stance_foot_contacts,
      const std::string& description = "fom_stance_ft_pos_constraint");

 private:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& x,
                          drake::VectorX<double>* y) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::multibody::BodyFrame<double>& world_;
  std::unique_ptr<drake::systems::Context<double>> context_;
  const std::vector<
      std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>>&
      stance_foot_contacts_;

  int n_q_;
};

// Constraints are
//   stance foot vel = 0
class FomStanceFootVelConstraint : public solvers::NonlinearConstraint<double> {
 public:
  FomStanceFootVelConstraint(
      const drake::multibody::MultibodyPlant<double>& plant,
      const std::vector<std::pair<const Eigen::Vector3d,
                                  const drake::multibody::Frame<double>&>>&
          stance_foot_contacts,
      const std::string& description = "fom_stance_ft_vel_constraint");

 private:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& x,
                          drake::VectorX<double>* y) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::multibody::BodyFrame<double>& world_;
  std::unique_ptr<drake::systems::Context<double>> context_;
  const std::vector<
      std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>>&
      stance_foot_contacts_;

  int n_q_;
  int n_x_;
  int n_c_;
};

}  // namespace planning
}  // namespace goldilocks_models
}  // namespace dairlib
