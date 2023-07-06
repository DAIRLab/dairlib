#pragma once

#include <vector>

#include <memory.h>

#include "multibody/multipose_visualizer.h"
#include "systems/trajectory_optimization/dircon_kinematic_data.h"
#include "systems/trajectory_optimization/dircon_kinematic_data_set.h"
#include "systems/trajectory_optimization/dircon_opt_constraints.h"
#include "systems/trajectory_optimization/dircon_options.h"

#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic/expression.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/solvers/constraint.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"
#include "drake/planning/trajectory_optimization/multiple_shooting.h"

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
class HybridDircon
    : public drake::planning::trajectory_optimization::MultipleShooting {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HybridDircon)

  /// Constructs the %MathematicalProgram% and adds the collocation constraints.
  ///
  /// @param plant The MultibodyPlant describing the plant and kinematics
  /// @param num_time_samples The number of knot points in the trajectory.
  /// @param minimum_timestep Minimum spacing between sample times.
  /// @param maximum_timestep Maximum spacing between sample times.
  /// @param constraints The set of kinematic constraints that must be enforced
  /// @param opttions (see DirconOptions)
  HybridDircon(const drake::multibody::MultibodyPlant<T>& plant,
               std::vector<int> num_time_samples,
               std::vector<double> minimum_timestep,
               std::vector<double> maximum_timestep,
               std::vector<DirconKinematicDataSet<T>*> constraints,
               std::vector<DirconOptions> options);

  ~HybridDircon() override {}

  /// Returns a vector of matrices containing the state and derivative values at
  /// each breakpoint at the solution for each mode of the trajectory.
  void GetStateAndDerivativeSamples(
      const drake::solvers::MathematicalProgramResult& result,
      std::vector<Eigen::MatrixXd>* state_samples,
      std::vector<Eigen::MatrixXd>* derivative_samples,
      std::vector<Eigen::VectorXd>* state_breaks) const;

  /// Get the input trajectory at the solution as a
  /// %drake::trajectories::PiecewisePolynomialTrajectory%.
  drake::trajectories::PiecewisePolynomial<double> ReconstructInputTrajectory(
      const drake::solvers::MathematicalProgramResult& result) const override;

  /// Get the state trajectory at the solution as a
  /// %drake::trajectories::PiecewisePolynomialTrajectory%.
  drake::trajectories::PiecewisePolynomial<double> ReconstructStateTrajectory(
      const drake::solvers::MathematicalProgramResult& result) const override;

  /// Adds a visualization callback that will visualize knot points
  /// without transparency. Cannot be called twice
  /// @param model_name The path of a URDF/SDF model name for visualization
  /// @param poses_per_mode Regulates how many knot points are visualized. A
  ///   vector containing the nubmer of poses to show per mode. This is in
  ///   addition to the start/end poses of every mode! The total number of poses
  ///   is therefore [sum(poses_per_mode) + num_modes + 1]
  /// @param alpha A transparency scaler for all poses except the first and last
  /// @param weld_frame_to_world The name of a frame to weld to the world frame
  ///   when parsing the model. Defaults to blank, which will not perform a weld
  void CreateVisualizationCallback(std::string model_file,
                                   std::vector<unsigned int> poses_per_mode,
                                   double alpha = 1,
                                   std::string weld_frame_to_world = "");

  /// See CreateVisualizationCallback(std::string model_file,
  ///    std::vector<unsigned int> poses_per_mode,
  ///    std::string weld_frame_to_world)
  ///
  /// Creates a callback using a single pose count parameter, num_poses
  /// Evenly divides the poses among the different modes, weighting by number
  /// of frames in that mode. Since start/end poses per mdoe are required, must
  /// have num_poses >= num_modes + 1
  void CreateVisualizationCallback(std::string model_file,
                                   unsigned int num_poses, double alpha = 1,
                                   std::string weld_frame_to_world = "");

  /// See CreateVisualizationCallback(std::string model_file,
  ///    unsigned int poses_per_mode,
  ///    std::string weld_frame_to_world)
  ///
  /// Creates a visualization callback that shows all knot points.
  void CreateVisualizationCallback(std::string model_file, double alpha = 1,
                                   std::string weld_frame_to_world = "");

  /// Set the initial guess for the force variables for a specific mode
  /// @param mode the mode index
  /// @param traj_init_l contact forces lambda (interpreted at knot points)
  /// @param traj_init_lc contact forces (interpretted at collocation points)
  /// @param traj_init_vc velocity constraint slack variables (at collocation)
  void SetInitialForceTrajectory(
      int mode,
      const drake::trajectories::PiecewisePolynomial<double>& traj_init_l,
      const drake::trajectories::PiecewisePolynomial<double>& traj_init_lc,
      const drake::trajectories::PiecewisePolynomial<double>& traj_init_vc);

  int num_kinematic_constraints(int mode) const {
    return num_kinematic_constraints_[mode];
  }
  int num_kinematic_constraints_wo_skipping(int mode) const {
    return num_kinematic_constraints_wo_skipping_[mode];
  }

  int num_modes() const { return num_modes_; }

  /// Get the number of knotpoints in a specified mode
  int mode_length(int mode_index) const { return mode_lengths()[mode_index]; };

  std::vector<int> mode_lengths() const { return mode_lengths_; }

  const drake::solvers::VectorXDecisionVariable& force_vars(int mode) const {
    return force_vars_[mode];
  }

  const drake::solvers::VectorXDecisionVariable& offset_vars(int mode) const {
    return offset_vars_[mode];
  }

  const drake::solvers::VectorXDecisionVariable& collocation_force_vars(
      int mode) const {
    return collocation_force_vars_[mode];
  }

  const drake::solvers::VectorXDecisionVariable& collocation_slack_vars(
      int mode) const {
    return collocation_slack_vars_[mode];
  }

  const drake::solvers::VectorXDecisionVariable& quaternion_slack_vars(
      int mode) const {
    return quaternion_slack_vars_[mode];
  }

  const drake::solvers::VectorXDecisionVariable& v_post_impact_vars() const {
    return v_post_impact_vars_;
  }

  const drake::solvers::VectorXDecisionVariable& impulse_vars(int mode) const {
    return impulse_vars_[mode];
  }

  const Eigen::VectorBlock<const drake::solvers::VectorXDecisionVariable>
  v_post_impact_vars_by_mode(int mode) const;

  /// Get the state decision variables given a mode and a time_index
  /// (time_index is w.r.t that particular mode). This will use the
  ///  v_post_impact_vars_ if needed. Otherwise, it just returns the standard
  /// x_vars element
  drake::solvers::VectorXDecisionVariable state_vars_by_mode(
      int mode, int time_index) const;

  Eigen::VectorBlock<const drake::solvers::VectorXDecisionVariable> force(
      int mode, int index) const {
    DRAKE_DEMAND(index < mode_lengths_[mode]);
    return force_vars_[mode].segment(
        index * num_kinematic_constraints_wo_skipping_[mode],
        num_kinematic_constraints_wo_skipping_[mode]);
  }
  Eigen::VectorBlock<const drake::solvers::VectorXDecisionVariable>
  collocation_force(int mode, int index) const {
    DRAKE_DEMAND(index < mode_lengths_[mode] - 1);
    return collocation_force_vars_[mode].segment(
        index * num_kinematic_constraints_wo_skipping_[mode],
        num_kinematic_constraints_wo_skipping_[mode]);
  }

  drake::VectorX<drake::symbolic::Expression> SubstitutePlaceholderVariables(
      const drake::VectorX<drake::symbolic::Expression>& f,
      int interval_index) const;

  /// Substitute the velocity variables in the expression with
  /// v_post_impact_vars_ for the corresponding mode
  /// It is up to the user to make sure this is used only on the first knotpoint
  drake::symbolic::Expression SubstitutePostImpactVelocityVariables(
      const drake::symbolic::Expression& e, int mode) const;

  using drake::planning::trajectory_optimization::MultipleShooting::N;
  using drake::planning::trajectory_optimization::MultipleShooting::
  SubstitutePlaceholderVariables;

  void ScaleTimeVariables(double scale);
  void ScaleQuaternionSlackVariables(double scale);
  void ScaleStateVariable(int idx, double scale);
  void ScaleInputVariable(int idx, double scale);
  void ScaleForceVariable(int mode, int idx, double scale);
  void ScaleImpulseVariable(int mode, int idx, double scale);
  void ScaleKinConstraintSlackVariable(int mode, int idx, double scale);
  void ScaleStateVariables(std::vector<int> idx_list, double scale);
  void ScaleInputVariables(std::vector<int> idx_list, double scale);
  void ScaleForceVariables(int mode, std::vector<int> idx_list, double scale);
  void ScaleImpulseVariables(int mode, std::vector<int> idx_list, double scale);
  void ScaleKinConstraintSlackVariables(int mode, std::vector<int> idx_list,
                                        double scale);

 private:
  // Implements a running cost at all timesteps using trapezoidal integration.
  const drake::multibody::MultibodyPlant<T>& plant_;
  std::vector<DirconKinematicDataSet<T>*> constraints_;
  const int num_modes_;
  const std::vector<int> mode_lengths_;
  std::vector<int> mode_start_;
  void DoAddRunningCost(const drake::symbolic::Expression& e) override;
  const drake::solvers::VectorXDecisionVariable v_post_impact_vars_;
  std::vector<drake::solvers::VectorXDecisionVariable> force_vars_;
  std::vector<drake::solvers::VectorXDecisionVariable> collocation_force_vars_;
  std::vector<drake::solvers::VectorXDecisionVariable> collocation_slack_vars_;
  std::vector<drake::solvers::VectorXDecisionVariable> offset_vars_;
  std::vector<drake::solvers::VectorXDecisionVariable> impulse_vars_;
  std::vector<drake::solvers::VectorXDecisionVariable> quaternion_slack_vars_;
  std::vector<int> num_kinematic_constraints_;
  std::vector<int> num_kinematic_constraints_wo_skipping_;

  std::unique_ptr<multibody::MultiposeVisualizer> callback_visualizer_;

  std::vector<std::pair<drake::VectorX<drake::symbolic::Variable>,
                        drake::VectorX<drake::symbolic::Expression>>>
      v_post_impact_vars_substitute_;
};

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace dairlib
