#pragma once
#include <set>
#include <unordered_map>
#include <vector>

#include "multibody/kinematic/kinematic_evaluator_set.h"

namespace dairlib {
namespace systems {
namespace trajectory_optimization {

/// Specifies which constraints should be evaluated. The primary reason to use
/// anything other than kAll (default), is to avoid redundant constraints.
/// For instance, fully constraining the first state, x(0) = x_0, would leave
/// position and velocity constraints on x_0 redundant.
enum class KinematicConstraintType {
  kAll = 3,
  kAccelAndVel = 2,
  kAccelOnly = 1
};

/// DirconMode is the primary class for defining constrained collocation
/// (DIRCON) trajectory optimization problems. Each DirconMode object refers
/// to a single hybrid mode of the optimization.
template <typename T>
class DirconMode {
 public:
  /// @param evaluators The set of constraints, active and full.
  /// @param num_knotpoints The total number of knot points.
  /// @param min_T The minimum total duration of the mode (default 0)
  /// @param max_T The maximum total duration of the mode (default inf)
  /// @param force_regularization A scalar c, such that
  ///   /sum_i c||force_i||^2 is added (summed over knotpoints) as a
  ///   regularization cost
  explicit DirconMode(const multibody::KinematicEvaluatorSet<T>& evaluators,
                      int num_knotpoints, double min_T = 0,
                      double max_T = std::numeric_limits<double>::infinity(),
                      double force_regularization = 1.0e-10);

  /// Identify a constraint as being relative, e.g. constrained to be constant
  /// but not necessarily zero (via a new slack variable). Identifies the
  /// constraint by its full index within a specific KinematicEvaluator
  void MakeConstraintRelative(int evaluator_index, int constraint_index);

  /// Return the set of relative constraints (constrained to be constant but
  /// not necessarily zero)
  const std::set<int>& relative_constraints() const {
    return relative_constraints_;
  };

  /// Declare that quaternion constraints at the given knotpoint_index are not
  /// to be included. This is useful if the quaternion configuration is already
  /// constrained via another mechanism.
  void SkipQuaternionConstraint(int knotpoint_index);

  bool IsSkipQuaternionConstraint(int knotpoint_index) const;

  /// Count the number of relative constraints
  int num_relative_constraints() const { return relative_constraints_.size(); };

  /// Set a specific knotpoint in time to use a specific constraint type, either
  /// kAll, kAccelAndVel, kAccelOnly. By default, all constraints (kAll) are
  /// included
  void set_constraint_type(int knotpoint_index, KinematicConstraintType type);

  /// Get the constraint type for a specific knotpoint in time, either
  /// kAll, kAccelAndVel, kAccelOnly. By default, all constraints (kAll) are
  /// included
  KinematicConstraintType get_constraint_type(int knotpoint_index) const;

  /// Set the impact constraint, for the start of this mode, to use a scale
  /// factor
  void SetImpactScale(int velocity_index, double scale);

  /// Set the dynamics constraint, for all collocation points, to use a scale
  /// factor
  void SetDynamicsScale(int state_index, double scale);

  /// Set the kinematics constraint, phi(q) = 0, to use a scale factor.
  /// Constraint is indexed by its evaluator, and a constraint_index within that
  /// evaluator.
  void SetKinPositionScale(int evaluator_index, int constraint_index,
                           double scale);

  /// Set the velocity constraint, d/dt phi(q) = 0, to use a scale factor.
  /// Constraint is indexed by its evaluator, and a constraint_index within that
  /// evaluator.
  void SetKinVelocityScale(int evaluator_index, int constraint_index,
                           double scale);

  /// Set the acceleration constraint, d^2/dt^2 phi(q) = 0, to use a scale
  /// factor. Constraint is indexed by its evaluator, and a constraint_index
  /// within that evaluator.
  void SetKinAccelerationScale(int evaluator_index, int constraint_index,
                               double scale);

  /// See SetImpactScale(velocity_index, scale). Sets multiple indices to use
  /// the same scale.
  void SetImpactScale(std::vector<int> velocity_indices, double scale);

  /// See SetDynamicsScale(state_index, scale). Sets multiple indices to use
  /// the same scale.
  void SetDynamicsScale(std::vector<int> state_indices, double scale);

  /// See SetKinPositionScale(evaluator_index, constraint_index, scale).
  /// Sets multiple indices to use the same scale. All constraint_indices within
  /// all evaluator_indices will use the given scale. This is designed for cases
  /// where multiple KinematicEvaluators share similar structure, and one wishes
  /// to give all the same scale factor.
  void SetKinPositionScale(std::vector<int> evaluator_indices,
                           std::vector<int> constraint_indices, double scale);

  /// See SetKinVelocityScale(evaluator_index, constraint_index, scale).
  /// Sets multiple indices to use the same scale. All constraint_indices within
  /// all evaluator_indices will use the given scale. This is designed for cases
  /// where multiple KinematicEvaluators share similar structure, and one wishes
  /// to give all the same scale factor.
  void SetKinVelocityScale(std::vector<int> evaluator_indices,
                           std::vector<int> constraint_indices, double scale);

  /// See SetKinAccelerationScale(evaluator_index, constraint_index, scale).
  /// Sets multiple indices to use the same scale. All constraint_indices within
  /// all evaluator_indices will use the given scale. This is designed for cases
  /// where multiple KinematicEvaluators share similar structure, and one wishes
  /// to give all the same scale factor.
  void SetKinAccelerationScale(std::vector<int> evaluator_indices,
                               std::vector<int> constraint_indices,
                               double scale);

  /// Gets the map of non-unitary scales for the dynamics constraints
  const std::unordered_map<int, double>& GetDynamicsScale() const {
    return dynamics_scale_;
  };

  /// Gets the map of non-unitary scales for the impact constraints
  const std::unordered_map<int, double>& GetImpactScale() const {
    return impact_scale_;
  };

  /// Gets the map of non-unitary scales for the kinematic position constraints
  /// Note, unlike SetKinPositionScale, indices in the map refer to the complete
  /// KinematicEvaluatorSet, not a specific KinematicEvaluator.
  const std::unordered_map<int, double>& GetKinPositionScale() const {
    return kin_position_scale_;
  };

  /// Gets the map of non-unitary scales for the kinematic velocity constraints
  /// Note, unlike GetKinVelocityScale, indices in the map refer to the complete
  /// KinematicEvaluatorSet, not a specific KinematicEvaluator.
  const std::unordered_map<int, double>& GetKinVelocityScale() const {
    return kin_velocity_scale_;
  };

  /// Gets the map of non-unitary scales for the kinematic acceleration
  /// constraints. Note, unlike GetKinAccelerationScale, indices in the map
  /// refer to the complete KinematicEvaluatorSet, not a specific
  /// KinematicEvaluator.
  const std::unordered_map<int, double>& GetKinAccelerationScale() const {
    return kin_acceleration_scale_;
  };

  /// Get the cost on force regularization.
  double get_force_regularization() const { return force_regularization_; };

  const multibody::KinematicEvaluatorSet<T>& evaluators() const {
    return evaluators_;
  };

  int num_knotpoints() const { return num_knotpoints_; };

  double min_T() const { return min_T_; };

  double max_T() const { return max_T_; };

  const drake::multibody::MultibodyPlant<T>& plant() const {
    return evaluators_.plant();
  };

 private:
  void SetKinScale(std::unordered_map<int, double>* scale_map,
                   int evaluator_index, int constraint_index, double scale);

  const multibody::KinematicEvaluatorSet<T>& evaluators_;
  const int num_knotpoints_;
  const double min_T_;
  const double max_T_;
  const double force_regularization_;
  std::set<int> relative_constraints_;
  std::set<int> skip_quaternion_;

  // Manually-set constraint types, organized by index. See set_constraint_type.
  std::unordered_map<int, KinematicConstraintType> reduced_constraints_;

  // Constraint scaling
  std::unordered_map<int, double> dynamics_scale_;
  std::unordered_map<int, double> impact_scale_;
  std::unordered_map<int, double> kin_position_scale_;
  std::unordered_map<int, double> kin_velocity_scale_;
  std::unordered_map<int, double> kin_acceleration_scale_;
};

template <typename T>
class DirconModeSequence {
 public:
  /// Initialize an empty mode sequence
  DirconModeSequence(const drake::multibody::MultibodyPlant<T>& plant);

  /// Initialize a single mode sequence
  DirconModeSequence(DirconMode<T>* mode);

  void AddMode(DirconMode<T>* mode);

  int count_knotpoints() const;

  int num_modes() const { return modes_.size(); };

  const drake::multibody::MultibodyPlant<T>& plant() const { return plant_; };

  const DirconMode<T>& mode(int index) const { return *modes_.at(index); };

 private:
  const drake::multibody::MultibodyPlant<T>& plant_;
  std::vector<DirconMode<T>*> modes_;
};

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace dairlib
