#include "multibody/multibody_solvers.h"
#include "multibody/multibody_utils.h"

namespace dairlib {
namespace multibody {

using Eigen::VectorXd;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::MatrixX;
using drake::VectorX;
using solvers::NonlinearConstraint;

template <typename T>
KinematicPositionConstraint<T>::KinematicPositionConstraint(
    const MultibodyPlant<T>& plant,
    const std::vector<KinematicEvaluator<T>>& evaluators,
    std::shared_ptr<Context<T>> context, const std::string& description)
    : NonlinearConstraint<T>(KinematicEvaluator<T>::CountActive(evaluators),
          plant.num_positions(),
          VectorXd::Zero(KinematicEvaluator<T>::CountActive(evaluators)),
          VectorXd::Zero(KinematicEvaluator<T>::CountActive(evaluators)),
          description),
      plant_(plant), 
      evaluators_(evaluators),
      context_(context) {}

template <typename T>
KinematicPositionConstraint<T>::KinematicPositionConstraint(
    const MultibodyPlant<T>& plant,
    const std::vector<KinematicEvaluator<T>>& evaluators,
    const std::string& description) 
    : KinematicPositionConstraint<T>(plant, evaluators,
      std::shared_ptr<Context<T>>(plant_.CreateDefaultContext().release()),
      description) {}

template <typename T>
void KinematicPositionConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>>& q, VectorX<T>* y) const {
  // Verifying the size of the input vector
  DRAKE_DEMAND(q.size() == plant_.num_positions());

  plant_.SetPositions(context_.get(), q);

  
  y->resize(this->num_constraints());
  int ind = 0;
  for (const auto& e : evaluators_) {
    y->segment(ind, e.num_active()) = e.EvalActive(*context_);
    ind += e.num_active();
  }
}

template <typename T>
FixedPointConstraint<T>::FixedPointConstraint(
    const MultibodyPlant<T>& plant,
    const std::vector<KinematicEvaluator<T>>& evaluators,
    std::shared_ptr<Context<T>> context, const std::string& description)
    : NonlinearConstraint<T>(plant.num_positions() + plant.num_velocities(),
          plant.num_positions() + plant.num_velocities() + plant.num_actuators()
          + KinematicEvaluator<T>::CountFull(evaluators),
          VectorXd::Zero(plant.num_positions() + plant.num_velocities()),
          VectorXd::Zero(plant.num_positions() + plant.num_velocities()),
          description),
      plant_(plant), 
      evaluators_(evaluators),
      context_(context) {}

template <typename T>
FixedPointConstraint<T>::FixedPointConstraint(
    const MultibodyPlant<T>& plant,
    const std::vector<KinematicEvaluator<T>>& evaluators,
    const std::string& description) 
    : FixedPointConstraint<T>(plant, evaluators,
      std::shared_ptr<Context<T>>(plant_.CreateDefaultContext().release()),
      description) {}

template <typename T>
void FixedPointConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>>& input, VectorX<T>* y) const {

  auto u = input.segment(plant_.num_positions() + plant_.num_velocities(),
      plant_.num_actuators());
  auto lambda = input.segment(plant_.num_positions() + plant_.num_velocities()
      + plant_.num_actuators(), KinematicEvaluator<T>::CountFull(evaluators_));

  // input order is x, u, lambda
  setContext(plant_,
      input.head(plant_.num_positions() + plant_.num_velocities()),
      u, context_.get());
  
  // Build kinematic Jacobian
  MatrixX<T> J(lambda.size(), plant_.num_velocities());
  int ind = 0;
  for (const auto& e : evaluators_) {
    J.block(ind, 0, e.length(), plant_.num_velocities()) =
        e.EvalFullJacobian(*context_);
    ind += e.length();
  }

  // right_hand_side is the right hand side of the system's equations:
  // M*vdot -J^T*f = right_hand_side.
  // BiasTerm is C(q,v) in manipulator equations
  VectorX<T> generalized_forces(plant_.num_velocities());
  plant_.CalcBiasTerm(*context_, &generalized_forces);

  generalized_forces = -generalized_forces +
      plant_.MakeActuationMatrix() * u +
      plant_.CalcGravityGeneralizedForces(*context_) +
      J.transpose() * lambda;
  *y = generalized_forces;
}

}  // namespace multibody
}  // namespace dairlib

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::KinematicPositionConstraint)
