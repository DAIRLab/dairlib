#pragma once

#include "multibody/kinematic/kinematic_evaluator.h"
#include "solvers/nonlinear_constraint.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib {
namespace multibody {

template <typename T>
class KinematicPositionConstraint : public solvers::NonlinearConstraint<T> {
 public:
  /// This constructor takes a shared_ptr<Context> as an argument to share
  /// cached kinematic/dynamic computation within the ccontext
  KinematicPositionConstraint(
      const drake::multibody::MultibodyPlant<T>& plant,
      const std::vector<KinematicEvaluator<T>>& evaluators,
      std::shared_ptr<drake::systems::Context<T>> context,
      const std::string& description = "");

  /// This constructor will build its own shared_ptr<Context>
  KinematicPositionConstraint(
      const drake::multibody::MultibodyPlant<T>& plant,
      const std::vector<KinematicEvaluator<T>>& evaluators,
      const std::string& description = "");

  using drake::solvers::Constraint::set_bounds;
  using drake::solvers::EvaluatorBase::set_num_outputs;

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                                  drake::VectorX<T>* y) const = 0;

 private:
  const drake::multibody::MultibodyPlant<T>& plant_;
  const std::vector<KinematicEvaluator<T>>& evaluators_;
  std::shared_ptr<drake::systems::Context<T>> context_;
};

}  // namespace multibody
}  // namespace dairlib
