#pragma once

#include <memory.h>
#include "dircon_kinematic_data.h"
#include "dircon_kinematic_data_set.h"
#include "drake/common/drake_copyable.h"
#include "drake/solvers/constraint.h"
#include "drake/common/symbolic.h"
#include "drake/systems/trajectory_optimization/multiple_shooting.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
  
enum DirconKinConstraintType { kAll = 3, kAccelAndVel = 2, kAccelOnly = 1 };

/// Implements the direct collocation constraints for a first-order hold on
/// the input and a cubic polynomial representation of the state trajectories.
///
/// Note that the DirectCollocation implementation allocates only ONE of
/// these constraints, but binds that constraint multiple times (with
/// different decision variables, along the trajectory).

class DirconDynamicConstraint : public solvers::Constraint {
 public:
//  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DirconDynamicConstraint)

 public:
  DirconDynamicConstraint(const RigidBodyTree<double>& tree, DirconKinematicDataSet<AutoDiffXd>& constraints);

  ~DirconDynamicConstraint() override = default;

  int num_states() const { return num_states_; }
  int num_inputs() const { return num_inputs_; }
  int num_kinematic_constraints() const { return num_kinematic_constraints_; }

 public:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd& y) const override;

 private:
  DirconDynamicConstraint(const RigidBodyTree<double>& tree, DirconKinematicDataSet<AutoDiffXd>& constraints,
    int num_positions, int num_velocities, int num_inputs, int num_kinematic_constraints);

  const RigidBodyTree<double>* tree_;
  DirconKinematicDataSet<AutoDiffXd>* constraints_;

  const int num_positions_{0};
  const int num_velocities_{0};
  const int num_states_{0};
  const int num_inputs_{0};
  const int num_kinematic_constraints_{0};
};

class DirconKinematicConstraint : public solvers::Constraint{

   public:
  DirconKinematicConstraint(const RigidBodyTree<double>& tree, DirconKinematicDataSet<AutoDiffXd>& constraint_data);
  DirconKinematicConstraint(const RigidBodyTree<double>& tree, DirconKinematicDataSet<AutoDiffXd>& constraint_data, DirconKinConstraintType type);

  ~DirconKinematicConstraint() override = default;

 protected:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd& y) const override;

 private:
  DirconKinematicConstraint(const RigidBodyTree<double>& tree, DirconKinematicDataSet<AutoDiffXd>& constraint_data, DirconKinConstraintType type,
    int num_positions, int num_velocities, int num_inputs, int num_kinematic_constraints);


  const RigidBodyTree<double>* tree_;
  DirconKinematicDataSet<AutoDiffXd>* constraints_;

  const int num_positions_{0};
  const int num_velocities_{0};
  const int num_states_{0};
  const int num_inputs_{0};
  const int num_kinematic_constraints_{0};
  const DirconKinConstraintType type_{kAll};
};

}
}
}