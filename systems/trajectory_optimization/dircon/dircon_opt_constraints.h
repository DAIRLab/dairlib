#pragma once

#include <string>
#include <unordered_map>
#include <vector>
#include "solvers/nonlinear_constraint.h"
#include "systems/trajectory_optimization/dircon/dircon_mode.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic.h"
#include "drake/solvers/constraint.h"
#include "drake/systems/trajectory_optimization/multiple_shooting.h"

namespace dairlib {
namespace systems {
namespace trajectory_optimization {

/// Unit-norm quaternion constraint
template <typename T>
class QuaternionNormConstraint : public solvers::NonlinearConstraint<T> {
 public:
  QuaternionNormConstraint();
  ~QuaternionNormConstraint() override = default;

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                          drake::VectorX<T>* y) const override;
};

/// Implements the direct collocation constraints for a first-order hold on
/// the input and a cubic polynomial representation of the state trajectories.
/// This class is based on the similar constraint used by DirectCollocation,
/// but incorporates the effect of constraint forces
template <typename T>
class DirconCollocationConstraint : public solvers::NonlinearConstraint<T> {
 public:

 public:
  /// Requires two context pointers to be pasesd as arguments, one for each
  /// knot point. The constraint will create its own pointer for the collocation
  /// point context.
  DirconCollocationConstraint(const drake::multibody::MultibodyPlant<T>& plant,
      const multibody::KinematicEvaluatorSet<T>& evaluators,
      drake::systems::Context<T>* context_0,
      drake::systems::Context<T>* context_1,
      int mode_index, int knot_index);

 public:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                          drake::VectorX<T>* y) const override;

 private:
  const drake::multibody::MultibodyPlant<T>& plant_;
  const multibody::KinematicEvaluatorSet<T>& evaluators_;
  drake::systems::Context<T>* context_0_;
  drake::systems::Context<T>* context_1_;
  std::unique_ptr<drake::systems::Context<T>> context_col_;
  const std::vector<int> quat_start_indices_;
  int n_x_;
  int n_u_;
  int n_l_;
};

/// Implements the impact constraint used by Dircon on mode transitions
///     M(q) * (v_+ - v_-) = J(q)^T Lambda
/// Inputs to this constraint are pre-impact state, x_0, the impulse Lambda,
/// and the post-impact velocity v_1
template <typename T>
class DirconImpactConstraint : public solvers::NonlinearConstraint<T> {
 public:
  DirconImpactConstraint(
      const drake::multibody::MultibodyPlant<T>& plant,
      const multibody::KinematicEvaluatorSet<T>& evaluators,
      drake::systems::Context<T>* context,
      std::string description);

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                          drake::VectorX<T>* y) const override;

 private:
  const drake::multibody::MultibodyPlant<T>& plant_;
  const multibody::KinematicEvaluatorSet<T>& evaluators_;
  drake::systems::Context<T>* context_;
  const int n_x_;
  const int n_l_;
};


}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace dairlib
