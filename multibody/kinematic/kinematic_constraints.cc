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
          std::vector<bool>(), context, description) {}

template <typename T>
KinematicPositionConstraint<T>::KinematicPositionConstraint(
    const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators,
    const VectorXd& lb, const VectorXd& ub,
    const std::vector<bool>& full_constraint_relative,
    Context<T>* context, const std::string& description)
    : NonlinearConstraint<T>(evaluators.count_active(),
          plant.num_positions() + 
              std::count(full_constraint_relative.begin(),
                         full_constraint_relative.end(), true),
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

  // Build a map from alpha relative variables to indices
  // relative_map_[i] = j means alpha_i corresponds to constraint_j
  int relative_count = 0;
  for (int i = 0; i < evaluators_.count_active(); i++) {
    if (full_constraint_relative.at(i) && evaluators_.is_active(i)) {
      relative_map_[relative_count] = i;
    }
  }
}

template <typename T>
void KinematicPositionConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>>& vars, VectorX<T>* y) const {
  const auto& q = vars.head(plant_.num_positions());
  const auto& alpha = vars.tail(relative_map_.size());

  plant_.SetPositions(context_, q);

  *y = evaluators_.EvalActive(*context_);

  for(const auto& [alpha_index, constraint_index] : relative_map_) {
    (*y)(constraint_index) -= alpha(alpha_index);
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
  plant_.SetPositionsAndVelocities(context_, x);

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
  multibody::setContext<T>(plant_, x, u, context_);

  *y = evaluators_.EvalActiveSecondTimeDerivative(*context_, lambda);
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::KinematicPositionConstraint)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::KinematicVelocityConstraint)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::KinematicAccelerationConstraint)

}  // namespace multibody
}  // namespace dairlib
