#pragma once

#include <memory.h>
#include "dircon_options.h"
#include "dircon_kinematic_data.h"
#include "dircon_kinematic_data_set.h"
#include "drake/common/drake_copyable.h"
#include "drake/solvers/constraint.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/trajectory_optimization/multiple_shooting.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {

using trajectories::PiecewisePolynomial;

/// DirectCollocation implements the approach to trajectory optimization as
/// described in
///   C. R. Hargraves and S. W. Paris. Direct trajectory optimization using
///    nonlinear programming and collocation. J Guidance, 10(4):338-342,
///    July-August 1987.
/// It assumes a first-order hold on the input trajectory and a cubic spline
/// representation of the state trajectory, and adds dynamic constraints (and
/// running costs) to the midpoints as well as the knot points in order to
/// achieve a 3rd order integration accuracy.

class Dircon : public MultipleShooting {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Dircon)

  /// Constructs the %MathematicalProgram% and adds the collocation constraints.
  ///
  /// @param system A dynamical system to be used in the dynamic constraints.
  ///    This system must support System::ToAutoDiffXd.
  ///    Note that this is aliased for the lifetime of this object.
  /// @param context Required to describe any parameters of the system.  The
  ///    values of the state in this context do not have any effect.  This
  ///    context will also be "cloned" by the optimization; changes to the
  ///    context after calling this method will NOT impact the trajectory
  ///    optimization.
  /// @param num_time_samples The number of knot points in the trajectory.
  /// @param minimum_timestep Minimum spacing between sample times.
  /// @param maximum_timestep Maximum spacing between sample times.
  Dircon(const RigidBodyTree<double>& tree, int num_time_samples, double minimum_timestep, double maximum_timestep,
    DirconKinematicDataSet<AutoDiffXd>& constraints, DirconOptions options);

  // NOTE: The fixed timestep constructor, which would avoid adding h as
  // decision variables, has been removed since it complicates the API and code.
  // Unlike other trajectory optimization transcriptions, direct collocation
  // will not be a convex optimization even if the sample times are fixed, so
  // there is little advantage to actually removing the variables.  Setting
  // minimum_timestep == maximum_timestep should be essentially just as good.

  ~Dircon() override {}

  /// Get the input trajectory at the solution as a
  /// %PiecewisePolynomialTrajectory%.
  PiecewisePolynomial<double> ReconstructInputTrajectory()
  const override;

  /// Get the state trajectory at the solution as a
  /// %PiecewisePolynomialTrajectory%.
  PiecewisePolynomial<double> ReconstructStateTrajectory()
  const override;

  void SetInitialTrajectory(const PiecewisePolynomial<double>& traj_init_u, const PiecewisePolynomial<double>& traj_init_x,
                            const PiecewisePolynomial<double>& traj_init_l, const PiecewisePolynomial<double>& traj_init_lc,
                            const PiecewisePolynomial<double>& traj_init_vc);

  int num_kinematic_constraints() const { return num_kinematic_constraints_; }

  const solvers::VectorXDecisionVariable& force_vars() const { return force_vars_; }

  const solvers::VectorXDecisionVariable& collocation_force_vars() const { return collocation_force_vars_; }

  const solvers::VectorXDecisionVariable& collocation_slack_vars() const { return collocation_slack_vars_; }

  Eigen::VectorBlock<const solvers::VectorXDecisionVariable> force(
      int index) const {
    DRAKE_DEMAND(index >= 0 && index < N());
    return force_vars_.segment(index * num_kinematic_constraints_, num_kinematic_constraints_);
  }

 private:
  // Implements a running cost at all timesteps using trapezoidal integration.
  const RigidBodyTree<double>* tree_;
  DirconKinematicDataSet<AutoDiffXd>* constraints_;
  void DoAddRunningCost(const symbolic::Expression& e) override;
  const solvers::VectorXDecisionVariable force_vars_;
  const solvers::VectorXDecisionVariable collocation_force_vars_;
  const solvers::VectorXDecisionVariable collocation_slack_vars_;
  const int num_kinematic_constraints_;
};


class DirconKinematicConstraint : public solvers::Constraint{

   public:
  DirconKinematicConstraint(const RigidBodyTree<double>& tree, DirconKinematicDataSet<AutoDiffXd>& constraint_data);

  ~DirconKinematicConstraint() override = default;

 protected:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd& y) const override;

 private:
  DirconKinematicConstraint(const RigidBodyTree<double>& tree, DirconKinematicDataSet<AutoDiffXd>& constraint_data,
    int num_positions, int num_velocities, int num_inputs, int num_kinematic_constraints);


  const RigidBodyTree<double>* tree_;
  DirconKinematicDataSet<AutoDiffXd>* constraints_;

  const int num_positions_{0};
  const int num_velocities_{0};
  const int num_states_{0};
  const int num_inputs_{0};
  const int num_kinematic_constraints_{0};
};

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

/// Helper method to add a DirconDynamicConstraint to the @p prog,
/// ensuring that the order of variables in the binding matches the order
/// expected by the constraint.
// Note: The order of arguments is a compromise between GSG and the desire to
// match the AddConstraint interfaces in MathematicalProgram.

solvers::Binding<solvers::Constraint> AddDirconConstraint(
    std::shared_ptr<DirconDynamicConstraint> constraint,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& timestep,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& state,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& next_state,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& input,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& next_input,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& force,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& next_force,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& collocation_force,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& collocation_position_slack,
    solvers::MathematicalProgram* prog);

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake