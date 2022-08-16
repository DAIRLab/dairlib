#pragma once

#include "multibody/kinematic/kinematic_constraints.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "solvers/nonlinear_constraint.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/mathematical_program.h"

namespace dairlib {
namespace multibody {

/// Specialization of MathematicalProgram to focus on MultiBodyPlant-based
/// calculations. Currently supports easy addition of q, u, force variables
/// and the addition of kinematic and fixed point constraints. Other features,
/// including those that use velocity, could be added at a future point.
/// For ease of use, this class also maintains a Context object that can be 
/// shared with constraints to leverage constraint caching. However, methods
/// allow the user to provide their own Context when instantiating a constraint.
/// This is to support use cases where multiple state variables might be
/// created, and thus multiple caches required for efficiency.
///
/// A basic implementation for a fixed point will look like
/// KinematicEvaluatorSet evaluators = ...
/// auto program = MultibodyProgram<double>(plant);
/// auto q = program.AddPositionVariables();
/// auto u = program.AddInputVariables();
/// auto lambda = program.AddConstraintForceVariables(evaluators);
/// program.AddKinematicPositionConstraint(evaluators, q);
/// program.AddFixedPointConstraint(evaluators, q, u, lambda);
/// <Solve program and extract solution>
template <typename T>
class MultibodyProgram : public drake::solvers::MathematicalProgram {
 public:
  MultibodyProgram(
      const drake::multibody::MultibodyPlant<T>& plant);

  /// Adds and returns position decision variables.
  drake::solvers::VectorXDecisionVariable AddPositionVariables();

  /// Adds and returns velocity decision variables.
  drake::solvers::VectorXDecisionVariable AddVelocityVariables();

  /// Adds and returns actuation input decision variables.
  drake::solvers::VectorXDecisionVariable AddInputVariables();

  /// Adds and returns constraint force decision variables associated with
  /// the given evaluators (full)
  drake::solvers::VectorXDecisionVariable AddConstraintForceVariables(
            const KinematicEvaluatorSet<T>& evaluators);

  void AddJointLimitConstraints(drake::solvers::VectorXDecisionVariable q);

  /// Adds a the kinematic constraint phi(q) = 0 for the associated
  /// KinematicEvaluators. Decision variables q here are required as an input
  /// to avoid calling this method out of order with AddPositionVariables()
  drake::solvers::Binding<drake::solvers::Constraint>
  AddKinematicPositionConstraint(
      const KinematicEvaluatorSet<T>& evaluators,
      const drake::solvers::VectorXDecisionVariable& q,
      drake::systems::Context<T>* local_context);

  /// See above. Uses the internal context_ (recommended for most use cases)
  drake::solvers::Binding<drake::solvers::Constraint>
  AddKinematicPositionConstraint(
      const KinematicEvaluatorSet<T>& evaluators,
      const drake::solvers::VectorXDecisionVariable& q);

  /// Adds a the kinematic constraint d/dt(phi(q)) = J(q)v = 0 for the
  /// associated KinematicEvaluators, and the decision variables q and v
  drake::solvers::Binding<drake::solvers::Constraint>
  AddKinematicVelocityConstraint(
      const KinematicEvaluatorSet<T>& evaluators,
      const drake::solvers::VectorXDecisionVariable& q,
      const drake::solvers::VectorXDecisionVariable& v,
      drake::systems::Context<T>* local_context);

  drake::solvers::Binding<drake::solvers::Constraint>
  AddKinematicVelocityConstraint(
      const KinematicEvaluatorSet<T>& evaluators,
      const drake::solvers::VectorXDecisionVariable& q,
      const drake::solvers::VectorXDecisionVariable& v);

  /// Adds the kinematic constraint
  /// d^2/dt^2(phi(q)) = Jdot(q, v)v + J vdot(q, v, u, lambda) = 0 for the
  /// associated KinematicEvaluators and the decision variables
  /// q, v, u, and lambda.
  drake::solvers::Binding<drake::solvers::Constraint>
  AddKinematicAccelerationConstraint(
      const KinematicEvaluatorSet<T>& evaluators,
      const drake::solvers::VectorXDecisionVariable& q,
      const drake::solvers::VectorXDecisionVariable& v,
      const drake::solvers::VectorXDecisionVariable& u,
      const drake::solvers::VectorXDecisionVariable& lambda,
      drake::systems::Context<T>* local_context);

  drake::solvers::Binding<drake::solvers::Constraint>
  AddKinematicAccelerationConstraint(
      const KinematicEvaluatorSet<T>& evaluators,
      const drake::solvers::VectorXDecisionVariable& q,
      const drake::solvers::VectorXDecisionVariable& v,
      const drake::solvers::VectorXDecisionVariable& u,
      const drake::solvers::VectorXDecisionVariable& lambda);

  /// Adds a holonomic kinematic constraint phi(q) = 0, and, enforces it at the
  /// at the position, velocity, and acceleration level.
  /// Returns a std::vector of constraint bindings, one for each derivative.
  /// For details on what each of these constraints mean, see
  /// AddKinematic[Position, Velocity, Acceleration]Constraint above.
  std::vector<drake::solvers::Binding<drake::solvers::Constraint>>
  AddHolonomicConstraint(const KinematicEvaluatorSet<T>& evaluators,
                         const drake::solvers::VectorXDecisionVariable& q,
                         const drake::solvers::VectorXDecisionVariable& v,
                         const drake::solvers::VectorXDecisionVariable& u,
                         const drake::solvers::VectorXDecisionVariable& lambda,
                         drake::systems::Context<T>* local_context);

  /// See above. Uses the internal context_ (recommended for most cases)
  std::vector<drake::solvers::Binding<drake::solvers::Constraint>>
  AddHolonomicConstraint(const KinematicEvaluatorSet<T>& evaluators,
                         const drake::solvers::VectorXDecisionVariable& q,
                         const drake::solvers::VectorXDecisionVariable& v,
                         const drake::solvers::VectorXDecisionVariable& u,
                         const drake::solvers::VectorXDecisionVariable& lambda);

  /// Adds a fixed point constraint, including the associated constraint
  /// forces. The length of lambda is checked against
  /// the full set from evaluators
  drake::solvers::Binding<drake::solvers::Constraint> AddFixedPointConstraint(
      const KinematicEvaluatorSet<T>& evaluators,
      const drake::solvers::VectorXDecisionVariable& q,
      const drake::solvers::VectorXDecisionVariable& u,
      const drake::solvers::VectorXDecisionVariable& lambda,
      drake::systems::Context<T>* local_context);

  /// See above. Uses the internal context_ (recommended for most use cases)
  drake::solvers::Binding<drake::solvers::Constraint> AddFixedPointConstraint(
      const KinematicEvaluatorSet<T>& evaluators,
      const drake::solvers::VectorXDecisionVariable& q,
      const drake::solvers::VectorXDecisionVariable& u,
      const drake::solvers::VectorXDecisionVariable& lambda);

  drake::systems::Context<T>* get_context() {return context_.get();};

 private:
  const drake::multibody::MultibodyPlant<T>& plant_;
  std::unique_ptr<drake::systems::Context<T>> context_;
};

template <typename T>
class FixedPointConstraint : public solvers::NonlinearConstraint<T> {
 public:
  /// This constructor takes a shared_ptr<Context> as an argument to share
  /// cached kinematic/dynamic computation within the context
  /// If a context pointer is not provided, will create a new context.
  FixedPointConstraint(
      const drake::multibody::MultibodyPlant<T>& plant,
      const KinematicEvaluatorSet<T>& evaluators,
      drake::systems::Context<T>* context = nullptr,
      const std::string& description = "fixed point");

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                                  drake::VectorX<T>* y) const;

 private:
  const drake::multibody::MultibodyPlant<T>& plant_;
  const KinematicEvaluatorSet<T>& evaluators_;
  drake::systems::Context<T>* context_;
  std::unique_ptr<drake::systems::Context<T>> owned_context_;
};

}  // namespace multibody
}  // namespace dairlib
