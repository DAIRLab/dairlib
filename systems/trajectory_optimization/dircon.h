#pragma once

#include <memory.h>
#include "dircon_opt_constraints.h"
#include "dircon_options.h"
#include "dircon_kinematic_data.h"
#include "dircon_kinematic_data_set.h"
#include "drake/common/drake_copyable.h"
#include "drake/solvers/constraint.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/trajectory_optimization/multiple_shooting.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/symbolic.h"

namespace dairlib {
namespace systems {
namespace trajectory_optimization {

/// DIRCON implements the approach to trajectory optimization as
/// described in
///   Michael Posa, Scott Kuindersma, Russ Tedrake. "Optimization and
///   Stabilization of Trajectories for Constrained Dynamical Systems." ICRA,
///   2016.
/// It assumes a first-order hold on the input trajectory and a cubic spline
/// representation of the state trajectory, and adds dynamic constraints (and
/// running costs) to the midpoints as well as the knot points in order to
/// achieve a 3rd order integration accuracy.
/// DIRCON addresses kinematic constraints by incorporating constraint forces
/// and corresponding acceleration, velocity, and position constraints.

template <typename T>
class Dircon:public drake::systems::trajectory_optimization::MultipleShooting {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Dircon)

  /// Constructs the %MathematicalProgram% and adds the collocation constraints.
  ///
  /// @param tree The RigidBodyTree describing the plant and kinematics
  /// @param num_time_samples The number of knot points in the trajectory.
  /// @param minimum_timestep Minimum spacing between sample times.
  /// @param maximum_timestep Maximum spacing between sample times.
  /// @param constraints The set of kinematic constraints that must be enforced
  /// @param opttions (see DirconOptions)
  Dircon(const RigidBodyTree<double>& tree, int num_time_samples,
         double minimum_timestep, double maximum_timestep,
         DirconKinematicDataSet<T>& constraints, DirconOptions options);

  ~Dircon() override {}

  /// Get the input trajectory at the solution as a
  /// %PiecewisePolynomialTrajectory%.
  drake::trajectories::PiecewisePolynomial<double> ReconstructInputTrajectory()
  const override;

  /// Get the state trajectory at the solution as a
  /// %PiecewisePolynomialTrajectory%.
  drake::trajectories::PiecewisePolynomial<double> ReconstructStateTrajectory()
  const override;

  /// Set the initial guess
  /// @param traj_init_u control input u
  /// @param traj_init_x stat ex
  /// @param traj_init_l contact forces lambda (interpreted at knot points)
  /// @param traj_init_lc contact forces (interpreted at collocation points)
  /// @param traj_init_vc velocity constraint slack variables at collocation pts
  void SetInitialTrajectory(
      const drake::trajectories::PiecewisePolynomial<double>& traj_init_u,
      const drake::trajectories::PiecewisePolynomial<double>& traj_init_x,
      const drake::trajectories::PiecewisePolynomial<double>& traj_init_l,
      const drake::trajectories::PiecewisePolynomial<double>& traj_init_lc,
      const drake::trajectories::PiecewisePolynomial<double>& traj_init_vc);



  int num_kinematic_constraints() const { return num_kinematic_constraints_; }

  const drake::solvers::VectorXDecisionVariable& force_vars() const {
    return force_vars_;
  }

  const drake::solvers::VectorXDecisionVariable& offset_vars() const {
    return offset_vars_;
  }

  const drake::solvers::VectorXDecisionVariable& collocation_force_vars()
      const {
    return collocation_force_vars_;
  }

  const drake::solvers::VectorXDecisionVariable& collocation_slack_vars()
      const {
    return collocation_slack_vars_;
  }

  Eigen::VectorBlock<const drake::solvers::VectorXDecisionVariable> force(
      int index) const {
    DRAKE_DEMAND(index >= 0 && index < N());
    return force_vars_.segment(index * num_kinematic_constraints_,
                              num_kinematic_constraints_);
  }

 private:
  // Implements a running cost at all timesteps using trapezoidal integration.
  const int num_kinematic_constraints_;
  const RigidBodyTree<double>* tree_;
  DirconKinematicDataSet<T>* constraints_;
  void DoAddRunningCost(const drake::symbolic::Expression& e) override;
  const drake::solvers::VectorXDecisionVariable force_vars_;
  const drake::solvers::VectorXDecisionVariable collocation_force_vars_;
  const drake::solvers::VectorXDecisionVariable collocation_slack_vars_;
  const drake::solvers::VectorXDecisionVariable offset_vars_;
};

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace dairlib
