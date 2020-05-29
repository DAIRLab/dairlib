#pragma once

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"

namespace dairlib {
namespace multibody {

/// Virtual class to represent arbitrary kinematic evaluations
/// Evaluations are defined by some function phi(q). Implementations
/// must generate phi(q), J(q) (the Jacboian w.r.t. velocities v), 
/// d/dt phi(q), and [d/dt J] * v
///
/// Evaluations call also be classified as "active" or "inactive." Inactive
/// evaluations can still be evaluated via the EvalAllXXX methods, but will
/// not be included in EvalActiveXXX. The purpose of this is to be able to
/// not include certain evaluations when using these methods to generate
/// optimization constraint (for instance, not constraining x-y position) of
/// a contact point. However, these terms might still be important for inclusion
/// via constraint forces. The active/inactive distinction is also useful for
/// avoiding redundant constraints, but still permitting their constraint
/// forces (via J'*lambda)
///
/// For efficiency, maintains a boolean all_active_ to determine whether all
/// constraints are active. If true, can skip the slicing stage.
template <typename T>
class KinematicEvaluator {
 public:
  // Disabling copy construction and assignment
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(KinematicEvaluator)

  explicit KinematicEvaluator(const drake::multibody::MultibodyPlant<T>& plant,
      int length);  

  /// Evaluates phi(q), limited only to active rows
  drake::VectorX<T> EvalActive(
      const drake::systems::Context<T>& context) const;

  /// Evaluates the time-derivative, d/dt phi(q), limited only to active rows
  drake::VectorX<T> EvalActiveTimeDerivative(
      const drake::systems::Context<T>& context) const;

  /// Evaluates the constraint Jacobian w.r.t. velocity v (not qdot)
  ///  limited only to active rows
  /// TODO (posa): could add an option to compute w.r.t. q
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
  virtual drake::VectorX<T> EvalFull(
      const drake::systems::Context<T>& context) const = 0;

  /// Evaluates the Jacobian w.r.t. velocity v (not qdot)
  /// TODO (posa): could add an option to compute w.r.t. q
  virtual drake::MatrixX<T> EvalFullJacobian(
      const drake::systems::Context<T>& context) const = 0;

  /// Evaluates Jdot * v, useful for computing constraint second derivative,
  ///  which would be d^2 phi/dt^2 = J * vdot + Jdot * v
  virtual drake::VectorX<T> EvalFullJacobianDotTimesV(
      const drake::systems::Context<T>& context) const = 0;

  void set_active_inds(std::vector<int> active_inds);

  int length() const {
    return length_;
  }

  int num_active() const {
    return num_active_;
  }

  const drake::multibody::MultibodyPlant<T>& plant() const {
    return plant_;
  }

  static int CountActive(const std::vector<KinematicEvaluator<T>>& evaluators);
  static int CountFull(const std::vector<KinematicEvaluator<T>>& evaluators);

 private:
  const drake::multibody::MultibodyPlant<T>& plant_;
  int num_active_;
  int length_;
  std::vector<int> active_inds_;
  bool all_active_;
};

}  // namespace multibody
}  // namespace dairlib
