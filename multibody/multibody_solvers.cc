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
MultibodyProgram<T>::MultibodyProgram(const MultibodyPlant<T>& plant)
    : drake::solvers::MathematicalProgram(),
    plant_(plant),
    context_(std::shared_ptr<Context<T>>(
        plant_.CreateDefaultContext().release())) {}

template <typename T>
VectorXDecisionVariable MultibodyProgram<T>::AddPositionVariables() {
  return NewContinuousVariables(plant_.num_positions(), "q");
}

template <typename T>
VectorXDecisionVariable MultibodyProgram<T>::AddInputVariables() {
  return NewContinuousVariables(plant_.num_actuators(), "u");
}

template <typename T>
VectorXDecisionVariable MultibodyProgram<T>::AddConstraintForceVariables(
    const KinematicEvaluatorSet<T>& evaluators) {
  return NewContinuousVariables(evaluators.count_full(), "lambda");;
}

template <typename T>
Binding<Constraint> MultibodyProgram<T>::AddKinematicConstraint(
    const KinematicEvaluatorSet<T>& evaluators,
    const VectorXDecisionVariable& q,
    std::shared_ptr<Context<T>> local_context) {
  DRAKE_DEMAND(q.size() == plant_.num_positions());
  auto constraint = std::make_shared<KinematicPositionConstraint<T>>(
        plant_, evaluators, local_context);
  return AddConstraint(constraint, q);
}

template <typename T>
Binding<Constraint> MultibodyProgram<T>::AddKinematicConstraint(
    const KinematicEvaluatorSet<T>& evaluators,
    const VectorXDecisionVariable& q) {
  AddKinematicConstraint(evaluators, q, context_);
}

template <typename T>
Binding<Constraint> MultibodyProgram<T>::AddFixedPointConstraint(
    const KinematicEvaluatorSet<T>& evaluators,
    const VectorXDecisionVariable& q, const VectorXDecisionVariable& u,
    const VectorXDecisionVariable& lambda,
    std::shared_ptr<Context<T>> local_context) {
  DRAKE_DEMAND(q.size() == plant_.num_positions());
  DRAKE_DEMAND(u.size() == plant_.num_actuators());
  DRAKE_DEMAND(lambda.size() == evaluators.count_full());
  auto constraint = std::make_shared<FixedPointConstraint<T>>(
        plant_, evaluators, local_context);
  return AddConstraint(constraint, {q, u, lambda});
}

template <typename T>
Binding<Constraint> MultibodyProgram<T>::AddFixedPointConstraint(
    const KinematicEvaluatorSet<T>& evaluators,
    const VectorXDecisionVariable& q, const VectorXDecisionVariable& u,
    const VectorXDecisionVariable& lambda) {
  AddFixedPointConstraint(evaluators, q, u, lambda, context_);
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

  *y = evaluators_.CalcMassMatrixTimesVDot(*context_, lambda);  
}

}  // namespace multibody
}  // namespace dairlib

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::MultibodyProgram)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::KinematicPositionConstraint)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::FixedPointConstraint)
