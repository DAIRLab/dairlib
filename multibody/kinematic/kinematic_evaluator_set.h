#pragma once

#include "multibody/kinematic/kinematic_evaluator.h"

namespace dairlib {
namespace multibody {

/// Simple class that maintains a vector pointers to KinematicEvaluator
/// objects. Provides a basic API for counting and accumulating evaluations
/// and their Jacobians.
template <typename T>
class KinematicEvaluatorSet {
 public:
  KinematicEvaluatorSet();

  /// Evaluates phi(q), limited only to active rows
  drake::VectorX<T> EvalActive(
      const drake::systems::Context<T>& context) const;

  /// Evaluates the time-derivative, d/dt phi(q), limited only to active rows
  drake::VectorX<T> EvalActiveTimeDerivative(
      const drake::systems::Context<T>& context) const;

  /// Evaluates the constraint Jacobian w.r.t. velocity v (not qdot)
  ///  limited only to active rows
  drake::MatrixX<T> EvalActiveJacobian(
      const drake::systems::Context<T>& context) const;

  /// Evaluates Jdot * v, useful for computing second derivative,
  ///  which would be d^2 phi/dt^2 = J * vdot + Jdot * v
  ///  limited only to active rows
  drake::VectorX<T> EvalActiveJacobianDotTimesV(
      const drake::systems::Context<T>& context) const;

  /// Evaluates the time-derivative, d/dt phi(q)
  drake::VectorX<T> EvalFullTimeDerivative(
      const drake::systems::Context<T>& context) const;

  /// Evaluates, phi(q), including inactive rows
  drake::VectorX<T> EvalFull(
      const drake::systems::Context<T>& context) const;

  /// Evaluates the Jacobian w.r.t. velocity v (not qdot)
  drake::MatrixX<T> EvalFullJacobian(
      const drake::systems::Context<T>& context) const;

  /// Evaluates Jdot * v, useful for computing constraint second derivative,
  drake::VectorX<T> EvalFullJacobianDotTimesV(
      const drake::systems::Context<T>& context) const;

  /// Determines the list of evaluators contained in the union with another set
  /// Specifically, `index` is in the returned vector if
  /// other.evaluators_.at(index) is an element of other.evaluators, as judged
  /// by a comparison of the KinematicEvaluator<T>* pointers.
  ///
  /// Again, note that this is an index set into the other object, not self.
  std::vector<int> FindUnion(KinematicEvaluatorSet<T> other);

  /// Compute M(q) * d/dt v, given the state, control inputs and constraint
  /// forces. Forces are associated with the active kinematic elements.
  /// @param context
  /// @param lambda constraint forces, applied via
  ///   evaluators.EvalActiveJacobian().transpose() * lambda
  drake::VectorX<T> CalcMassMatrixTimesVDot(
      const drake::systems::Context<T>& context,
      const drake::VectorX<T>& lambda) const;

  /// Computes vdot given the state, control inputs and constraint
  /// forces. Similar to CalcMassMatrixTimesVDot, but uses inv(M)
  /// and includes qdot. Forces are associated with the active kinematic
  /// elements.
  /// @param context
  /// @param lambda constraint forces, applied via
  ///   evaluators.EvalActiveJacobian().transpose() * lambda
  drake::VectorX<T> CalcTimeDerivatives(
      const drake::systems::Context<T>& context,
      const drake::VectorX<T>& lambda) const;

  /// Computes vdot given the state and control inputs.
  /// Solves for the constraint forces using the active kinematic elements.
  /// Similar to CalcTimeDerivatives(context, evaluators, lambda), but solves
  /// for the forces to satisfy ddot phi = -kp*phi - kd*phidot
  /// @param context
  /// @param kp A position gain for constraint stabilization. Default = 0
  /// @param kp A velocity gain for constraint stabilization. Default = 0
  drake::VectorX<T> CalcTimeDerivatives(
      const drake::systems::Context<T>& context, double kp = 0,
      double kd = 0) const;

  /// Gets the starting index into phi_full of the specified evaluator
  int evaluator_full_start(int index) const;

  /// Gets the starting index into phi_active of the specified evaluator
  int evaluator_active_start(int index) const;

  KinematicEvaluator<T>* get_evaluator(int index) {
    return evaluators_.at(index);
  };

  /// Adds an evaluator to the end of the list, returning the associated index
  int add_evaluator(KinematicEvaluator<T>* e);

  /// Count the total number of active constraints
  int count_active() const;

  /// Count the total number of constraints
  int count_full() const;

  int num_evaluators() const { return evaluators_.size(); };

 private:
  std::vector<KinematicEvaluator<T>*> evaluators_;
};

}  // namespace multibody
}  // namespace dairlib
