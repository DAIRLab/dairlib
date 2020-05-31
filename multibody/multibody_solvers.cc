#include "multibody/multibody_solvers.h"
#include "multibody/multibody_utils.h"

namespace dairlib {
namespace multibody {

using Eigen::VectorXd;
using drake::multibody::MultibodyPlant;
using drake::solvers::Binding;
using drake::solvers::Constraint;
using drake::solvers::VectorXDecisionVariable;
using drake::systems::Context;
using drake::MatrixX;
using drake::VectorX;
using solvers::NonlinearConstraint;

template <typename T>
MultibodyProgram<T>::MultibodyProgram(const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators)
    : drake::solvers::MathematicalProgram(),
    plant_(plant),
    evaluators_(evaluators),
    context_(std::shared_ptr<Context<T>>(
        plant_.CreateDefaultContext().release())) {}

template <typename T>
VectorXDecisionVariable MultibodyProgram<T>::AddPositionVariables() {
  DRAKE_DEMAND(q_.size() == 0);
  q_ = NewContinuousVariables(plant_.num_positions(), "q");
  return q_;
}

template <typename T>
VectorXDecisionVariable MultibodyProgram<T>::AddInputVariables() {
  DRAKE_DEMAND(u_.size() == 0);
  u_ = NewContinuousVariables(plant_.num_actuators(), "u");
  return u_;
}

template <typename T>
VectorXDecisionVariable MultibodyProgram<T>::AddConstraintForceVariables() {
  DRAKE_DEMAND(lambda_.size() == 0);
  lambda_ = NewContinuousVariables(evaluators_.count_full(), "lambda");
  return lambda_;
}

template <typename T>
Binding<Constraint> MultibodyProgram<T>::AddKinematicConstraint(
    const VectorXDecisionVariable& q) {
  auto constraint = std::make_shared<KinematicPositionConstraint<T>>(
        plant_, evaluators_, context_);
  return AddConstraint(constraint, q);
}

template <typename T>
Binding<Constraint> MultibodyProgram<T>::AddFixedPointConstraint(
    const VectorXDecisionVariable& q, const VectorXDecisionVariable& u,
    const VectorXDecisionVariable& lambda) {
  DRAKE_DEMAND(lambda.size() == evaluators_.count_full());
  auto constraint = std::make_shared<FixedPointConstraint<T>>(
        plant_, evaluators_, context_);
  return AddConstraint(constraint, {q, u, lambda});
}

template <typename T>
void MultibodyProgram<T>::AddJointLimitConstraints(VectorXDecisionVariable q) {
  for (int i = 0; i < plant_.num_joints(); i++) {
    const auto& joint = plant_.get_joint(drake::multibody::JointIndex(i));
    if (joint.num_positions() > 0) {
      auto q_joint = q.segment(joint.position_start(), joint.num_positions());
      AddConstraint(q_joint <= joint.position_upper_limits());
      AddConstraint(q_joint >= joint.position_lower_limits()); 
  }
  }
}

template <typename T>
KinematicPositionConstraint<T>::KinematicPositionConstraint(
    const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators,
    std::shared_ptr<Context<T>> context, const std::string& description)
    : NonlinearConstraint<T>(evaluators.count_active(), plant.num_positions(),
          VectorXd::Zero(evaluators.count_active()),
          VectorXd::Zero(evaluators.count_active()),
          description),
      plant_(plant), 
      evaluators_(evaluators),
      context_(context) {}

template <typename T>
KinematicPositionConstraint<T>::KinematicPositionConstraint(
    const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators,
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

  *y = evaluators_.EvalActive(*context_);
}

template <typename T>
FixedPointConstraint<T>::FixedPointConstraint(
    const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators,
    std::shared_ptr<Context<T>> context, const std::string& description)
    : NonlinearConstraint<T>(plant.num_velocities(),
          plant.num_positions() + plant.num_actuators()
          + evaluators.count_full(),
          VectorXd::Zero(plant.num_velocities()),
          VectorXd::Zero(plant.num_velocities()),
          description),
      plant_(plant), 
      evaluators_(evaluators),
      context_(context) {}

template <typename T>
FixedPointConstraint<T>::FixedPointConstraint(
    const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators,
    const std::string& description) 
    : FixedPointConstraint<T>(plant, evaluators,
      std::shared_ptr<Context<T>>(plant_.CreateDefaultContext().release()),
      description) {}

template <typename T>
void FixedPointConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>>& input, VectorX<T>* y) const {

  auto u = input.segment(plant_.num_positions(), plant_.num_actuators());
  auto lambda = input.segment(plant_.num_positions() + plant_.num_actuators(),
      evaluators_.count_full());
  VectorX<T> x(plant_.num_positions() + plant_.num_velocities());
  x << input.head(plant_.num_positions()),
      VectorX<T>::Zero(plant_.num_velocities());
  // input order is x, u, lambda
  setContext<T>(plant_, x, u, context_.get());
  
  // Evaluate kinematic Jacobian
  auto J = evaluators_.EvalFullJacobian(*context_);

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
    class ::dairlib::multibody::MultibodyProgram)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::KinematicPositionConstraint)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::FixedPointConstraint)
