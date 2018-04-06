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



namespace drake {
namespace systems {
namespace trajectory_optimization {

using std::vector;
using trajectories::PiecewisePolynomial;

/// DIRCCON implements the approach to trajectory optimization as
/// described in
///   Michael Posa, Scott Kuindersma, Russ Tedrake. "Optimization and Stabilization
////  of Trajectories for Constrained Dynamical Systems." ICRA, 2016.
/// It assumes a first-order hold on the input trajectory and a cubic spline
/// representation of the state trajectory, and adds dynamic constraints (and
/// running costs) to the midpoints as well as the knot points in order to
/// achieve a 3rd order integration accuracy.
/// DIRCON addresses kinematic constraints by incorporating constraint forces
/// and corresponding acceleration, velocity, and position constraints.

template <typename T>
class HybridDircon : public MultipleShooting {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HybridDircon)

  /// Constructs the %MathematicalProgram% and adds the collocation constraints.
  ///
  /// @param tree The RigidBodyTree describing the plant and kinematics
  /// @param num_time_samples The number of knot points in the trajectory.
  /// @param minimum_timestep Minimum spacing between sample times.
  /// @param maximum_timestep Maximum spacing between sample times.
  /// @param constraints The set of kinematic constraints that must be enforced
  /// @param opttions a set of options for the optimization program (see DirconOptions)
  HybridDircon(const RigidBodyTree<double>& tree, vector<int> num_time_samples, vector<double> minimum_timestep,
    vector<double> maximum_timestep, vector<DirconKinematicDataSet<T>*> constraints, vector<DirconOptions> options);

  ~HybridDircon() override {}

  /// Get the input trajectory at the solution as a
  /// %PiecewisePolynomialTrajectory%.
  PiecewisePolynomial<double> ReconstructInputTrajectory()
  const override;

  /// Get the state trajectory at the solution as a
  /// %PiecewisePolynomialTrajectory%.
  PiecewisePolynomial<double> ReconstructStateTrajectory()
  const override;

  /// Set the initial guess
  /// @param traj_init_u control input u
  /// @param traj_init_x stat ex
  /// @param traj_init_l contact forces lambda (interpreted at knot points)
  /// @param traj_init_lc contact forces lambda_collocation (interpretted at collocation points)
  /// @param traj_init_vc velocity constrait slack variables at collocation points
  void SetInitialTrajectory(int mode, const PiecewisePolynomial<double>& traj_init_u, const PiecewisePolynomial<double>& traj_init_x,
                            const PiecewisePolynomial<double>& traj_init_l, const PiecewisePolynomial<double>& traj_init_lc,
                            const PiecewisePolynomial<double>& traj_init_vc);



  int num_kinematic_constraints(int mode) const { return num_kinematic_constraints_[mode]; }

  const solvers::VectorXDecisionVariable& force_vars(int mode) const { return force_vars_[mode]; }

  const solvers::VectorXDecisionVariable& offset_vars(int mode) const { return offset_vars_[mode]; }

  const solvers::VectorXDecisionVariable& collocation_force_vars(int mode) const { return collocation_force_vars_[mode]; }

  const solvers::VectorXDecisionVariable& collocation_slack_vars(int mode) const { return collocation_slack_vars_[mode]; }

  Eigen::VectorBlock<const solvers::VectorXDecisionVariable> force(int mode, int index) const {
    DRAKE_DEMAND(index >= 0 && index < N());
    return force_vars_[mode].segment(index * num_kinematic_constraints_[mode], num_kinematic_constraints_[mode]);
  }

 private:
  // Implements a running cost at all timesteps using trapezoidal integration.
  const RigidBodyTree<double>* tree_;
  const int num_modes_;
  const vector<int> mode_lengths_;
  vector<int> mode_start_;
  vector<DirconKinematicDataSet<T>*> constraints_;
  void DoAddRunningCost(const symbolic::Expression& e) override;
  vector<solvers::VectorXDecisionVariable> force_vars_;
  vector<solvers::VectorXDecisionVariable> collocation_force_vars_;
  vector<solvers::VectorXDecisionVariable> collocation_slack_vars_;
  vector<solvers::VectorXDecisionVariable> offset_vars_;
  vector<int> num_kinematic_constraints_;
};

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake