#pragma once

#include <memory.h>
#include <string>
#include <vector>
#include "drake/common/drake_copyable.h"
#include "drake/solvers/constraint.h"
#include "drake/common/symbolic.h"
#include "drake/systems/trajectory_optimization/multiple_shooting.h"
#include "attic/systems/trajectory_optimization/dircon_kinematic_data.h"
#include "attic/systems/trajectory_optimization/dircon_kinematic_data_set.h"

namespace dairlib {
namespace systems {
namespace trajectory_optimization {

/// Helper class for all dircon constraints
/// manages evaluation of functions and numerical gradients
template <typename T>
class DirconAbstractConstraint : public drake::solvers::Constraint {
 public:
  DirconAbstractConstraint(int num_constraints, int num_vars,
                           const Eigen::VectorXd& lb,
                           const Eigen::VectorXd& ub,
                           const std::string& description = "");

 public:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& x,
              drake::AutoDiffVecXd* y) const override;

  void DoEval(
      const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>&,
      drake::VectorX<drake::symbolic::Expression>*) const override;

  virtual void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
              drake::VectorX<T>* y) const = 0;
};

enum DirconKinConstraintType { kAll = 3, kAccelAndVel = 2, kAccelOnly = 1 };

/// Implements the direct collocation constraints for a first-order hold on
/// the input and a cubic polynomial representation of the state trajectories.
/// This class is based on the similar constraint used by DirectCollocation,
/// but incorporates the effect of constraint forces

template <typename T>
class DirconDynamicConstraint : public DirconAbstractConstraint<T> {
 public:
//  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DirconDynamicConstraint)

 public:
  DirconDynamicConstraint(const RigidBodyTree<double>& tree,
                          DirconKinematicDataSet<T>& constraints);

  ~DirconDynamicConstraint() override = default;

  int num_states() const { return num_states_; }
  int num_inputs() const { return num_inputs_; }
  int num_kinematic_constraints() const { return num_kinematic_constraints_; }

 public:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                          drake::VectorX<T>* y) const override;

 private:
  DirconDynamicConstraint(const RigidBodyTree<double>& tree,
                          DirconKinematicDataSet<T>& constraints,
                          int num_positions, int num_velocities, int num_inputs,
                          int num_kinematic_constraints);

  const RigidBodyTree<double>* tree_;
  DirconKinematicDataSet<T>* constraints_;
  const int num_states_{0};
  const int num_inputs_{0};
  const int num_kinematic_constraints_{0};
  const int num_positions_{0};
  const int num_velocities_{0};
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
class DirconKinematicConstraint : public DirconAbstractConstraint<T> {
 public:
  /// Constructor. Defaults the relative constraints to be all false
  /// @param tree the RigidBodyTree
  /// @param DirconKinematicDataSet the set of kinematic constraints
  /// @param type the constraint type. All (default), accel and vel, accel only.
  DirconKinematicConstraint(const RigidBodyTree<double>& tree,
    DirconKinematicDataSet<T>& constraint_data,
    DirconKinConstraintType type = DirconKinConstraintType::kAll);
  /// Constructor
  /// @param tree the RigidBodyTree
  /// @param DirconKinematicDataSet the set of kinematic constraints
  /// @param is_constraint_relative vector of booleans
  /// @param type the constraint type. All (default), accel and vel, accel only.
  DirconKinematicConstraint(const RigidBodyTree<double>& tree,
    DirconKinematicDataSet<T>& constraint_data,
    std::vector<bool> is_constraint_relative,
    DirconKinConstraintType type = DirconKinConstraintType::kAll);

  ~DirconKinematicConstraint() override = default;

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                          drake::VectorX<T>* y) const override;

 private:
  DirconKinematicConstraint(const RigidBodyTree<double>& tree,
                            DirconKinematicDataSet<T>& constraint_data,
                            std::vector<bool> is_constraint_relative,
                            DirconKinConstraintType type, int num_positions,
                            int num_velocities, int num_inputs,
                            int num_kinematic_constraints);


  const RigidBodyTree<double>* tree_;
  DirconKinematicDataSet<T>* constraints_;

  const int num_states_{0};
  const int num_inputs_{0};
  const int num_kinematic_constraints_{0};
  const int num_positions_{0};
  const int num_velocities_{0};
  const DirconKinConstraintType type_{kAll};
  const std::vector<bool> is_constraint_relative_;
  const int n_relative_;
  Eigen::MatrixXd relative_map_;
};

/// Helper method to add a DirconDynamicConstraint to the @p prog,
/// ensuring that the order of variables in the binding matches the order
/// expected by the constraint.
// Note: The order of arguments is a compromise between GSG and the desire to
// match the AddConstraint interfaces in MathematicalProgram.
//
// mposa: I don't think this function is actually being used, and I'm not sure what it does
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
    const Eigen::Ref<const drake::solvers::VectorXDecisionVariable>& collocation_force,
    const Eigen::Ref<const drake::solvers::VectorXDecisionVariable>& collocation_position_slack,
    drake::solvers::MathematicalProgram* prog);


/// Implements the hybrid impact constraints used by Dircon
/// Enforces the impact constraint that vp = vm + M^{-1}*J^T*Lambda
template <typename T>
class DirconImpactConstraint : public DirconAbstractConstraint<T> {
 public:
  /// @param tree the RigidBodyTree
  /// @param DirconKinematicDataSet the set of kinematic constraints
  DirconImpactConstraint(const RigidBodyTree<double>& tree,
                         DirconKinematicDataSet<T>& constraint_data);

  ~DirconImpactConstraint() override = default;

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                          drake::VectorX<T>* y) const override;

 private:
  DirconImpactConstraint(const RigidBodyTree<double>& tree,
                         DirconKinematicDataSet<T>& constraint_data,
                         int num_positions, int num_velocities,
                         int num_kinematic_constraints);


  const RigidBodyTree<double>* tree_;
  DirconKinematicDataSet<T>* constraints_;

  const int num_states_{0};
  const int num_kinematic_constraints_{0};
  const int num_positions_{0};
  const int num_velocities_{0};
};

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace dairlib
