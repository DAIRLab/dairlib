#pragma once

#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "solvers/nonlinear_constraint.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/mathematical_program.h"

namespace dairlib {
namespace multibody {

template <typename T>
class MultibodyProgram : public drake::solvers::MathematicalProgram {
 public:
  MultibodyProgram(
      const drake::multibody::MultibodyPlant<T>& plant,
      const KinematicEvaluatorSet<T>& evaluators);

  /// Adds and returns position decision variables. Can only be called once
  drake::solvers::VectorXDecisionVariable AddPositionVariables();

  /// Adds and returns actuation input decision variables.
  /// Can only be called once
  drake::solvers::VectorXDecisionVariable AddInputVariables();

  /// Adds and returns constraint force decision variables.
  /// Can only be called once
  drake::solvers::VectorXDecisionVariable AddConstraintForceVariables();

  void AddJointLimitConstraints(drake::solvers::VectorXDecisionVariable q);

  /// Adds a kinematic constraint for the associated KinematicEvaluators
  /// Decision variables q here are required as an input to avoid calling
  /// this method out of order with AddPositionVariables()
  drake::solvers::Binding<drake::solvers::Constraint> AddKinematicConstraint(
      const drake::solvers::VectorXDecisionVariable& q);

  /// Adds a fixed point constraint, including the associated constraint
  /// forces. Decision variables q,u here are required as inputs to avoid 
  /// calling this method out of order. lambda is optional, since some systems
  /// will not have constraint forces. The length of lambda is checked.
  drake::solvers::Binding<drake::solvers::Constraint> AddFixedPointConstraint(
      const drake::solvers::VectorXDecisionVariable& q,
      const drake::solvers::VectorXDecisionVariable& u,
      const drake::solvers::VectorXDecisionVariable& lambda =
          drake::solvers::VectorXDecisionVariable(0));

 private:
  const drake::multibody::MultibodyPlant<T>& plant_;
  const KinematicEvaluatorSet<T>& evaluators_;
  std::shared_ptr<drake::systems::Context<T>> context_;

  drake::solvers::VectorXDecisionVariable q_;
  drake::solvers::VectorXDecisionVariable u_;
  drake::solvers::VectorXDecisionVariable lambda_;
};

template <typename T>
class KinematicPositionConstraint : public solvers::NonlinearConstraint<T> {
 public:
  /// This constructor takes a shared_ptr<Context> as an argument to share
  /// cached kinematic/dynamic computation within the context
  KinematicPositionConstraint(
      const drake::multibody::MultibodyPlant<T>& plant,
      const KinematicEvaluatorSet<T>& evaluators,
      std::shared_ptr<drake::systems::Context<T>> context,
      const std::string& description = "");

  /// This constructor will build its own shared_ptr<Context>
  KinematicPositionConstraint(
      const drake::multibody::MultibodyPlant<T>& plant,
      const KinematicEvaluatorSet<T>& evaluators,
      const std::string& description = "kinematic position");

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                                  drake::VectorX<T>* y) const;

 private:
  const drake::multibody::MultibodyPlant<T>& plant_;
  const KinematicEvaluatorSet<T>& evaluators_;
  std::shared_ptr<drake::systems::Context<T>> context_;
};

template <typename T>
class FixedPointConstraint : public solvers::NonlinearConstraint<T> {
 public:
  /// This constructor takes a shared_ptr<Context> as an argument to share
  /// cached kinematic/dynamic computation within the context
  FixedPointConstraint(
      const drake::multibody::MultibodyPlant<T>& plant,
      const KinematicEvaluatorSet<T>& evaluators,
      std::shared_ptr<drake::systems::Context<T>> context,
      const std::string& description = "");

  /// This constructor will build its own shared_ptr<Context>
  FixedPointConstraint(
      const drake::multibody::MultibodyPlant<T>& plant,
      const KinematicEvaluatorSet<T>& evaluators,
      const std::string& description = "fixed point");

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                                  drake::VectorX<T>* y) const;

 private:
  const drake::multibody::MultibodyPlant<T>& plant_;
  const KinematicEvaluatorSet<T>& evaluators_;
  std::shared_ptr<drake::systems::Context<T>> context_;
};

}  // namespace multibody
}  // namespace dairlib
