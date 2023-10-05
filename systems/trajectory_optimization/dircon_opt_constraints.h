#pragma once

#include <string>
#include <unordered_map>
#include <vector>
#include <memory.h>
#include "solvers/nonlinear_constraint.h"
#include "systems/trajectory_optimization/dircon_kinematic_data.h"
#include "systems/trajectory_optimization/dircon_kinematic_data_set.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic/expression.h"
#include "drake/solvers/constraint.h"
#include "drake/planning/trajectory_optimization/multiple_shooting.h"

namespace dairlib {
namespace systems {
namespace trajectory_optimization {

enum DirconKinConstraintType { kAll = 3, kAccelAndVel = 2, kAccelOnly = 1 };

/// Unit-norm quaternion constraint
template <typename T>
class QuaternionNormConstraint : public solvers::NonlinearConstraint<T> {
 public:
  QuaternionNormConstraint();
  ~QuaternionNormConstraint() override = default;

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                          drake::VectorX<T>* y) const override;
};

/// Implements the direct collocation constraints for a first-order hold on
/// the input and a cubic polynomial representation of the state trajectories.
/// This class is based on the similar constraint used by DirectCollocation,
/// but incorporates the effect of constraint forces

template <typename T>
class DirconDynamicConstraint : public solvers::NonlinearConstraint<T> {
 public:
  //  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DirconDynamicConstraint)

 public:
  DirconDynamicConstraint(const drake::multibody::MultibodyPlant<T>& plant,
                          DirconKinematicDataSet<T>& constraints,
                          bool is_quaternion = false);

  ~DirconDynamicConstraint() override = default;

  int num_states() const { return num_states_; }
  int num_inputs() const { return num_inputs_; }
  int num_kinematic_constraints_wo_skipping() const {
    return num_kinematic_constraints_wo_skipping_;
  }

  int num_quat_slack() const { return num_quat_slack_; }

 public:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                          drake::VectorX<T>* y) const override;

 private:
  // num_quat_slack is the dimension of the slack variable for the constraint
  // of unit norm quaternion (of the floating base)
  // It's 1 if the MBP is in quaternion floating-base. It's 0 otherwise.
  DirconDynamicConstraint(const drake::multibody::MultibodyPlant<T>& plant,
                          DirconKinematicDataSet<T>& constraints,
                          int num_positions, int num_velocities, int num_inputs,
                          int num_kinematic_constraints_wo_skipping,
                          int num_quat_slack);

  const drake::multibody::MultibodyPlant<T>& plant_;
  DirconKinematicDataSet<T>* constraints_;
  const int num_states_{0};
  const int num_inputs_{0};
  const int num_kinematic_constraints_wo_skipping_{0};
  const int num_positions_{0};
  const int num_velocities_{0};
  const int num_quat_slack_{0};
  std::unique_ptr<drake::systems::Context<T>> context_;
};

/// Implements the kinematic constraints used by Dircon
/// For constraints given by c(q), enforces the three constraints
///   c(q), d/dt c(q), d^2/dt^2 c(q)
///   at the position, velocity, and constraint levels
/// When used with states that are fully constrained, i.e. if q_0 or x_0
/// is fully specified, this class contains the option DirconKinConstraintType
/// to only include a subset of the three constraints above, avoiding
/// redundancy.
///
/// Constraints may also be specified as relative, where rather than c(q)=0,
/// we have the constriant c(q)=constant. The constant value is a then new
/// optimization decision variable.
template <typename T>
class DirconKinematicConstraint : public solvers::NonlinearConstraint<T> {
 public:
  /// Constructor. Defaults the relative constraints to be all false
  /// @param plant the MultibodyPlant
  /// @param DirconKinematicDataSet the set of kinematic constraints
  /// @param type the constraint type. All (default), accel and vel, accel only.
  DirconKinematicConstraint(
      const drake::multibody::MultibodyPlant<T>& plant,
      DirconKinematicDataSet<T>& constraint_data,
      DirconKinConstraintType type = DirconKinConstraintType::kAll);
  /// Constructor
  /// @param plant the MultibodyPlant
  /// @param DirconKinematicDataSet the set of kinematic constraints
  /// @param is_constraint_relative vector of booleans
  /// @param type the constraint type. All (default), accel and vel, accel only.
  DirconKinematicConstraint(
      const drake::multibody::MultibodyPlant<T>& plant,
      DirconKinematicDataSet<T>& constraint_data,
      std::vector<bool> is_constraint_relative,
      DirconKinConstraintType type = DirconKinConstraintType::kAll);

  ~DirconKinematicConstraint() override = default;

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                          drake::VectorX<T>* y) const override;

 private:
  DirconKinematicConstraint(const drake::multibody::MultibodyPlant<T>& plant,
                            DirconKinematicDataSet<T>& constraint_data,
                            std::vector<bool> is_constraint_relative,
                            DirconKinConstraintType type, int num_positions,
                            int num_velocities, int num_inputs,
                            int num_kinematic_constraints,
                            int num_kinematic_constraints_wo_skipping);

  const drake::multibody::MultibodyPlant<T>& plant_;
  DirconKinematicDataSet<T>* constraints_;

  const int num_states_{0};
  const int num_inputs_{0};
  const int num_kinematic_constraints_{0};
  const int num_kinematic_constraints_wo_skipping_{0};
  const int num_positions_{0};
  const int num_velocities_{0};
  const DirconKinConstraintType type_{kAll};
  const std::vector<bool> is_constraint_relative_;
  const int n_relative_;
  Eigen::MatrixXd relative_map_;
  std::unique_ptr<drake::systems::Context<T>> context_;
};

/// Helper method to add a DirconDynamicConstraint to the @p prog,
/// ensuring that the order of variables in the binding matches the order
/// expected by the constraint.
// Note: The order of arguments is a compromise between GSG and the desire to
// match the AddConstraint interfaces in MathematicalProgram.
//
// mposa: I don't think this function is actually being used, and I'm not sure
// what it does
template <typename T>
drake::solvers::Binding<drake::solvers::Constraint> AddDirconConstraint(
    std::shared_ptr<DirconDynamicConstraint<T>> constraint,
    const Eigen::Ref<const drake::solvers::VectorXDecisionVariable>& timestep,
    const Eigen::Ref<const drake::solvers::VectorXDecisionVariable>& state,
    const Eigen::Ref<const drake::solvers::VectorXDecisionVariable>& next_state,
    const Eigen::Ref<const drake::solvers::VectorXDecisionVariable>& input,
    const Eigen::Ref<const drake::solvers::VectorXDecisionVariable>& next_input,
    const Eigen::Ref<const drake::solvers::VectorXDecisionVariable>& force,
    const Eigen::Ref<const drake::solvers::VectorXDecisionVariable>& next_force,
    const Eigen::Ref<const drake::solvers::VectorXDecisionVariable>&
        collocation_force,
    const Eigen::Ref<const drake::solvers::VectorXDecisionVariable>&
        collocation_position_slack,
    drake::solvers::MathematicalProgram* prog);

/// Implements the hybrid impact constraints used by Dircon
/// Enforces the impact constraint that vp = vm + M^{-1}*J^T*Lambda
template <typename T>
class DirconImpactConstraint : public solvers::NonlinearConstraint<T> {
 public:
  /// @param plant the MultibodyPlant
  /// @param DirconKinematicDataSet the set of kinematic constraints
  DirconImpactConstraint(const drake::multibody::MultibodyPlant<T>& plant,
                         DirconKinematicDataSet<T>& constraint_data);

  ~DirconImpactConstraint() override = default;

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                          drake::VectorX<T>* y) const override;

 private:
  DirconImpactConstraint(const drake::multibody::MultibodyPlant<T>& plant,
                         DirconKinematicDataSet<T>& constraint_data,
                         int num_positions, int num_velocities,
                         int num_kinematic_constraints_wo_skipping);

  const drake::multibody::MultibodyPlant<T>& plant_;
  DirconKinematicDataSet<T>* constraints_;

  const int num_states_{0};
  const int num_kinematic_constraints_wo_skipping_{0};
  const int num_positions_{0};
  const int num_velocities_{0};
  std::unique_ptr<drake::systems::Context<T>> context_;
};

// Position constraint of a point in the directions `dir` with respect to the
// world, where the point is specified by body_name and point_wrt_body.
// Each row of `dir` is the direction in which you want to constrain the point,
// and each row of lb/ub is the corresponding lower/upper bound.
// To clarify, the # of constraints = `dir.rows()` = `lb.size()` = `ub.size()`
template <typename T>
class PointPositionConstraint : public solvers::NonlinearConstraint<T> {
 public:
  PointPositionConstraint(const drake::multibody::MultibodyPlant<T>& plant,
                          const std::string& body_name,
                          const Eigen::Vector3d& point_wrt_body,
                          const Eigen::Matrix<double, Eigen::Dynamic, 3>& dir,
                          const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
                          const std::string& description = "");
  // A constructor that fix the 3D position of a point wrt the world
  PointPositionConstraint(
      const drake::multibody::MultibodyPlant<T>& plant,
      const std::string& body_name, const Eigen::Vector3d& point_wrt_body,
      const Eigen::Vector3d& fix_pos = Eigen::Vector3d::Zero());
  ~PointPositionConstraint() override = default;

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                          drake::VectorX<T>* y) const override;

 private:
  const drake::multibody::MultibodyPlant<T>& plant_;
  const drake::multibody::Body<T>& body_;
  const drake::Vector3<T> point_wrt_body_;
  const Eigen::Matrix<T, Eigen::Dynamic, 3> dir_;
  std::unique_ptr<drake::systems::Context<T>> context_;
};

// Velocity constraint of a point in the directions `dir` with respect to the
// world, where the point is specified by body_name and point_wrt_body.
// Each row of `dir` is the direction in which you want to constrain the point,
// and each row of lb/ub is the corresponding lower/upper bound.
// To clarify, the # of constraints = `dir.rows()` = `lb.size()` = `ub.size()`
template <typename T>
class PointVelocityConstraint : public solvers::NonlinearConstraint<T> {
 public:
  PointVelocityConstraint(const drake::multibody::MultibodyPlant<T>& plant,
                          const std::string& body_name,
                          const Eigen::Vector3d& point_wrt_body,
                          const Eigen::Matrix<double, Eigen::Dynamic, 3>& dir,
                          const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
                          const std::string& description = "");
  // A constructor that fix the 3D velocity of a point wrt the world
  PointVelocityConstraint(
      const drake::multibody::MultibodyPlant<T>& plant,
      const std::string& body_name, const Eigen::Vector3d& point_wrt_body,
      const Eigen::Vector3d& fix_pos = Eigen::Vector3d::Zero());
  ~PointVelocityConstraint() override = default;

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                          drake::VectorX<T>* y) const override;

 private:
  const drake::multibody::MultibodyPlant<T>& plant_;
  const drake::multibody::Body<T>& body_;
  const drake::Vector3<T> point_wrt_body_;
  const Eigen::Matrix<T, Eigen::Dynamic, 3> dir_;
  std::unique_ptr<drake::systems::Context<T>> context_;
};

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace dairlib
