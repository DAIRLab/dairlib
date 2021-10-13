#pragma once

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/constraint.h"
#include "drake/systems/framework/context.h"

namespace dairlib {
namespace multibody {

/// Virtual class to represent arbitrary kinematic evaluations
/// Evaluations are defined by some function phi(q). Implementations
/// must generate phi(q), J(q) (the Jacobian w.r.t. velocities v),
/// and [d/dt J] * v
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
/// For efficiency, maintains a boolean all_active_default_order_ to
/// determine whether all constraints are active in the original order {0,1,...}
/// If true, can skip the slicing stage.
template <typename T>
class KinematicEvaluator {
 public:
  explicit KinematicEvaluator(const drake::multibody::MultibodyPlant<T>& plant,
      int length);

  virtual ~KinematicEvaluator() = default;

  /// Evaluates phi(q), limited only to active rows
  drake::VectorX<T> EvalActive(const drake::systems::Context<T>& context) const;

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
  virtual void EvalFullJacobian(const drake::systems::Context<T>& context,
                                drake::EigenPtr<drake::MatrixX<T>> J) const = 0;

  /// Evaluates the Jacobian w.r.t. velocity v
  drake::MatrixX<T> EvalFullJacobian(
      const drake::systems::Context<T>& context) const;

  /// Evaluates Jdot * v, useful for computing constraint second derivative,
  ///  which would be d^2 phi/dt^2 = J * vdot + Jdot * v
  virtual drake::VectorX<T> EvalFullJacobianDotTimesV(
      const drake::systems::Context<T>& context) const = 0;

  void set_active_inds(std::vector<int> active_inds);

  const std::vector<int>& active_inds() const;

  int num_full() const { return length_; }

  int num_active() const { return num_active_; }

  const drake::multibody::MultibodyPlant<T>& plant() const { return plant_; }

  const std::vector<int>& active_inds() { return active_inds_; };

  bool is_active(int index) const;

  /// Convert an index, relative to the full index set, to be an active index.
  /// Returns -1 if the given index is not active
  int full_index_to_active_index(int full_index) const;

  /// Create friction cone constraints on the force variables (associated with
  /// the full Jacobian). Subclasses which might be associated with frictional
  /// contact should implement this method.
  /// This method returns a vector of constraints, should the friction
  /// constraint be more naturally represented by mixed constraint types.
  virtual std::vector<std::shared_ptr<drake::solvers::Constraint>>
  CreateConicFrictionConstraints() const {
    return std::vector<std::shared_ptr<drake::solvers::Constraint>>();
  };

  /// Create friction cone constraints on the force variables (associated with
  /// the full Jacobian). Subclasses which might be associated with frictional
  /// contact should implement this method.
  /// This method returns a vector of constraints, should the friction
  /// constraint be more naturally represented by mixed constraint types.
  virtual std::vector<std::shared_ptr<drake::solvers::Constraint>>
  CreateLinearFrictionConstraints(int num_faces = 8) const {
    return std::vector<std::shared_ptr<drake::solvers::Constraint>>();
  };

  void set_mu(double mu) { mu_ = mu; };

  double mu() const { return mu_; };

 private:
  const drake::multibody::MultibodyPlant<T>& plant_;
  int num_active_;
  int length_;
  std::vector<int> active_inds_;
  bool all_active_default_order_;
  double mu_ = 0;
};

}  // namespace multibody
}  // namespace dairlib
