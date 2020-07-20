#pragma once

#include <set>

#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "solvers/nonlinear_constraint.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib {
namespace multibody {

/// A constraint class to wrap the position component of a KinematicEvaluatorSet
/// 
/// In its most basic form, this constraint is of the form
///    lb <= phi(q) <= ub
/// corresponding to the __active__ constraints only.
///
/// Constraints can also be identified as "relative", where indices are
/// supplied with respect to the full constraint list. This is to eliminate
/// the need for the user to adjust indices depending on which constraints are
/// marked active/full.
///
/// Relative constraints are shifted by the value of some additional decision
/// variable alpha. The constraint is then
///  lb <= phi(q) - alpha <= ub
///
/// The decision variables for this constraint are q and alpha
template <typename T>
class KinematicPositionConstraint : public solvers::NonlinearConstraint<T> {
 public:
  /// This constructor takes a shared_ptr<Context> as an argument to share
  /// cached kinematic/dynamic computation within the context.
  /// If a context pointer is not provided, will create a new context.
  /// This is the simplest form of the construtor, where the lower and upper
  /// bounds are both zero.
  KinematicPositionConstraint(
      const drake::multibody::MultibodyPlant<T>& plant,
      const KinematicEvaluatorSet<T>& evaluators,
      drake::systems::Context<T>* context = nullptr,
      const std::string& description = "kinematic_position");

  /// This constructor takes a shared_ptr<Context> as an argument to share
  /// cached kinematic/dynamic computation within the context.
  /// If a context pointer is not provided, will create a new context.
  /// 
  /// This constructor takes three additional arguments: lower and upper bounds,
  /// and a vector of booleans identifying whether the ith __full__ constraint
  /// is relative.
  KinematicPositionConstraint(
      const drake::multibody::MultibodyPlant<T>& plant,
      const KinematicEvaluatorSet<T>& evaluators,
      const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
      const std::set<int>& full_constraint_relative = {},
      drake::systems::Context<T>* context = nullptr,
      const std::string& description = "kinematic_position");

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                                  drake::VectorX<T>* y) const;

 private:
  const drake::multibody::MultibodyPlant<T>& plant_;
  const KinematicEvaluatorSet<T>& evaluators_;
  drake::systems::Context<T>* context_;
  std::unique_ptr<drake::systems::Context<T>> owned_context_;
  std::set<int> full_constraint_relative_;
};

/// A constraint class to wrap the velocity component of a KinematicEvaluatorSet
/// 
/// This constraint is of the form
///    lb <= d/dt phi(q) <= ub
/// corresponding to the __active__ constraints only.
///
/// The decision variables for this constraint are q and v
///
/// Unlike KinematicPositionConstraint, there is no need for the relative flag.
template <typename T>
class KinematicVelocityConstraint : public solvers::NonlinearConstraint<T> {
 public:
  /// This constructor takes a shared_ptr<Context> as an argument to share
  /// cached kinematic/dynamic computation within the context.
  /// If a context pointer is not provided, will create a new context.
  /// This is the simplest form of the construtor, where the lower and upper
  /// bounds are both zero.
  KinematicVelocityConstraint(
      const drake::multibody::MultibodyPlant<T>& plant,
      const KinematicEvaluatorSet<T>& evaluators,
      drake::systems::Context<T>* context = nullptr,
      const std::string& description = "kinematic_velocity");

  /// This constructor takes a shared_ptr<Context> as an argument to share
  /// cached kinematic/dynamic computation within the context.
  /// If a context pointer is not provided, will create a new context.
  /// 
  /// This constructor takes additional arguments: lower and upper bounds.
  KinematicVelocityConstraint(
      const drake::multibody::MultibodyPlant<T>& plant,
      const KinematicEvaluatorSet<T>& evaluators,
      const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
      drake::systems::Context<T>* context = nullptr,
      const std::string& description = "kinematic_velocity");

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                                  drake::VectorX<T>* y) const;

 private:
  const drake::multibody::MultibodyPlant<T>& plant_;
  const KinematicEvaluatorSet<T>& evaluators_;
  drake::systems::Context<T>* context_;
  std::unique_ptr<drake::systems::Context<T>> owned_context_;
};

/// A constraint class to wrap the acceleration component of a
/// KinematicEvaluatorSet
/// 
/// This constraint is of the form
///    lb <= d^2/dt^2 phi(q) <= u
/// corresponding to the __active__ constraints only.
///
/// To calculate acceleration, the decision variables for this constraint are
/// q, v, u and constraint force lambda (corresponding to the full set).
///
/// Unlike KinematicPositionConstraint, there is no need for the relative flag.
template <typename T>
class KinematicAccelerationConstraint : public solvers::NonlinearConstraint<T> {
 public:
  /// This constructor takes a shared_ptr<Context> as an argument to share
  /// cached kinematic/dynamic computation within the context.
  /// If a context pointer is not provided, will create a new context.
  /// This is the simplest form of the construtor, where the lower and upper
  /// bounds are both zero.
  KinematicAccelerationConstraint(
      const drake::multibody::MultibodyPlant<T>& plant,
      const KinematicEvaluatorSet<T>& evaluators,
      drake::systems::Context<T>* context = nullptr,
      const std::string& description = "kinematic_acceleration");

  /// This constructor takes a shared_ptr<Context> as an argument to share
  /// cached kinematic/dynamic computation within the context.
  /// If a context pointer is not provided, will create a new context.
  /// 
  /// This constructor takes additional arguments: lower and upper bounds.
  KinematicAccelerationConstraint(
      const drake::multibody::MultibodyPlant<T>& plant,
      const KinematicEvaluatorSet<T>& evaluators,
      const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
      drake::systems::Context<T>* context = nullptr,
      const std::string& description = "kinematic_acceleration");

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
