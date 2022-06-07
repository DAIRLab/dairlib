#include "multibody/kinematic/kinematic_constraints.h"
#include "multibody/multibody_utils.h"

namespace dairlib {
namespace multibody {

using Eigen::VectorXd;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::VectorX;
using solvers::NonlinearConstraint;

///
///  KinematicPositionConstraint
///
template <typename T>
KinematicPositionConstraint<T>::KinematicPositionConstraint(
    const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators,
    Context<T>* context, const std::string& description)
    : KinematicPositionConstraint<T>(plant, evaluators,
          VectorXd::Zero(evaluators.count_active()),
          VectorXd::Zero(evaluators.count_active()),
          std::set<int>(), context, description) {}

template <typename T>
KinematicPositionConstraint<T>::KinematicPositionConstraint(
    const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators,
    const VectorXd& lb, const VectorXd& ub,
    const std::set<int>& full_constraint_relative,
    Context<T>* context, const std::string& description)
    : NonlinearConstraint<T>(evaluators.count_active(),
          plant.num_positions() + full_constraint_relative.size(),
          lb, ub, description),
      plant_(plant), 
      evaluators_(evaluators),
      full_constraint_relative_(full_constraint_relative) {
  // Create a new context if one was not provided
  if (context == nullptr) {
    owned_context_ = plant_.CreateDefaultContext();
    context_ = owned_context_.get();
  } else {
    context_ = context;
  }
}

template <typename T>
void KinematicPositionConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>>& vars, VectorX<T>* y) const {
  const auto& q = vars.head(plant_.num_positions());
  const auto& alpha = vars.tail(full_constraint_relative_.size());

  SetPositionsIfNew<T>(plant_, q, context_);

  *y = evaluators_.EvalActive(*context_);

  // Add relative offsets, looping through the list of relative constraints
  auto it = full_constraint_relative_.begin();
  for (uint i = 0; i < full_constraint_relative_.size(); i++) {
    (*y)(*it) += alpha(i);
    it++;
  }
}

///
///  KinematicVelocityConstraint
///
template <typename T>
KinematicVelocityConstraint<T>::KinematicVelocityConstraint(
    const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators,
    Context<T>* context, const std::string& description)
    : KinematicVelocityConstraint<T>(plant, evaluators,
          VectorXd::Zero(evaluators.count_active()),
          VectorXd::Zero(evaluators.count_active()),
          context, description) {}

template <typename T>
KinematicVelocityConstraint<T>::KinematicVelocityConstraint(
    const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators,
    const VectorXd& lb, const VectorXd& ub,
    Context<T>* context, const std::string& description)
    : NonlinearConstraint<T>(evaluators.count_active(),
          plant.num_positions() + plant.num_velocities(),
          lb, ub, description),
      plant_(plant), 
      evaluators_(evaluators) {
  // Create a new context if one was not provided
  if (context == nullptr) {
    owned_context_ = plant_.CreateDefaultContext();
    context_ = owned_context_.get();
  } else {
    context_ = context;
  }
}

template <typename T>
void KinematicVelocityConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y) const {
  SetPositionsAndVelocitiesIfNew<T>(plant_, x, context_);

  *y = evaluators_.EvalActiveTimeDerivative(*context_);
}

///
///  KinematicAccelerationConstraint
///
template <typename T>
KinematicAccelerationConstraint<T>::KinematicAccelerationConstraint(
    const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators,
    Context<T>* context, const std::string& description)
    : KinematicAccelerationConstraint<T>(plant, evaluators,
          VectorXd::Zero(evaluators.count_active()),
          VectorXd::Zero(evaluators.count_active()),
          context, description) {}

template <typename T>
KinematicAccelerationConstraint<T>::KinematicAccelerationConstraint(
    const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators,
    const VectorXd& lb, const VectorXd& ub,
    Context<T>* context, const std::string& description)
    : NonlinearConstraint<T>(evaluators.count_active(),
          plant.num_positions() + plant.num_velocities() + plant.num_actuators()
              + evaluators.count_full(),
          lb, ub, description),
      plant_(plant), 
      evaluators_(evaluators) {
  // Create a new context if one was not provided
  if (context == nullptr) {
    owned_context_ = plant_.CreateDefaultContext();
    context_ = owned_context_.get();
  } else {
    context_ = context;
  }
}

template <typename T>
void KinematicAccelerationConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>>& vars, VectorX<T>* y) const {
  const auto& x = vars.head(plant_.num_positions() + plant_.num_velocities());
  const auto& u = vars.segment(plant_.num_positions() + plant_.num_velocities(),
      plant_.num_actuators());
  const auto& lambda = vars.tail(evaluators_.count_full());
  multibody::SetContext<T>(plant_, x, u, context_);

  *y = evaluators_.EvalActiveSecondTimeDerivative(context_, lambda);
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::KinematicPositionConstraint)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::KinematicVelocityConstraint)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::KinematicAccelerationConstraint)

}  // namespace multibody
}  // namespace dairlib
