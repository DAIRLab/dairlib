#pragma once

#include <vector>

#include <memory.h>

#include "multibody/multipose_visualizer.h"
#include "systems/trajectory_optimization/dircon/dircon_mode.h"
#include "systems/trajectory_optimization/dircon/dynamics_cache.h"

#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic/expression.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/solvers/constraint.h"
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
class Dircon
    : public drake::planning::trajectory_optimization::MultipleShooting {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Dircon)

  /// The default, hybrid constructor. Takes a mode sequence.
  Dircon(const DirconModeSequence<T>& mode_sequence);

  /// For simplicity, a constructor that takes only a single mode as a pointer.
  Dircon(DirconMode<T>* mode);

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

  /// Get the state samples by mode, as a matrix. Each column corresponds to
  /// a knotpoint.
  Eigen::MatrixXd GetStateSamplesByMode(
      const drake::solvers::MathematicalProgramResult& result, int mode) const;

  /// Get the input samples by mode, as a matrix. Each column corresponds to
  /// a knotpoint.
  Eigen::MatrixXd GetInputSamplesByMode(
      const drake::solvers::MathematicalProgramResult& result, int mode) const;

  /// Get the state samples by mode, as a matrix. Each column corresponds to
  /// a knotpoint.
  Eigen::MatrixXd GetForceSamplesByMode(
      const drake::solvers::MathematicalProgramResult& result, int mode) const;

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
                                   double alpha,
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
                                   unsigned int num_poses, double alpha,
                                   std::string weld_frame_to_world = "");

  /// See CreateVisualizationCallback(std::string model_file,
  ///    unsigned int poses_per_mode,
  ///    std::string weld_frame_to_world)
  ///
  /// Creates a visualization callback that shows all knot points.
  void CreateVisualizationCallback(std::string model_file, double alpha,
                                   std::string weld_frame_to_world = "");

  /// Set the initial guess for the force variables for a specific mode
  /// @param mode the mode index
  /// @param traj_init_l contact forces lambda (interpreted at knot points)
  /// @param traj_init_lc contact forces (interpreted at collocation points)
  /// @param traj_init_vc velocity constraint slack variables (at collocation)
  void SetInitialForceTrajectory(
      int mode,
      const drake::trajectories::PiecewisePolynomial<double>& traj_init_l,
      const drake::trajectories::PiecewisePolynomial<double>& traj_init_lc,
      const drake::trajectories::PiecewisePolynomial<double>& traj_init_vc);

  /// Set the initial guess for the force variables for a specific mode.
  /// Sets both the contact forces lambda and the collocation forces lambda_c
  /// from the same trajectory. Does not set velocity constraint slack variables
  /// @param mode the mode index
  /// @param traj_init_l contact forces lambda (interpreted at knot points)
  void SetInitialForceTrajectory(
      int mode,
      const drake::trajectories::PiecewisePolynomial<double>& traj_init_l);

  /// Get all knotpoint force variables associated with a specific mode and
  /// knotpoint
  const drake::solvers::VectorXDecisionVariable force_vars(
      int mode_index, int knotpoint_index) const;

  /// Get all collocation force variables associated with a specific mode and
  /// collocation point
  const drake::solvers::VectorXDecisionVariable collocation_force_vars(
      int mode_index, int collocation_index) const;

  /// Get all kinematic relative offset variables associated with a specific
  /// mode
  const drake::solvers::VectorXDecisionVariable offset_vars(
      int mode_index) const;

  /// Get all velocity slack variables (gamma) associated with a specific mode
  /// and collocation point
  const drake::solvers::VectorXDecisionVariable collocation_slack_vars(
      int mode_index, int collocation_index) const;

  /// Get all quaternion slack variables associated with a specific mode
  /// and collocation point
  const drake::solvers::VectorXDecisionVariable quaternion_slack_vars(
      int mode_index, int collocation_index) const;

  /// Get all post-impact velocity variables associated with a specific
  /// mode transition (0 is the first transition between modes 0 and 1)
  const drake::solvers::VectorXDecisionVariable post_impact_velocity_vars(
      int mode_transition_index) const;

  /// Get all impulsive force variables associated with a specific
  /// mode transition (0 is the first transition between modes 0 and 1)
  const drake::solvers::VectorXDecisionVariable impulse_vars(
      int mode_transition_index) const;

  /// Get the state decision variables given a mode and a time_index
  /// (knotpoint_index is w.r.t that particular mode). This will use the
  ///  v_post_impact_vars_ if needed. Otherwise, it just returns the standard
  /// x_vars element
  const drake::solvers::VectorXDecisionVariable state_vars(
      int mode_index, int knotpoint_index) const;

  /// Get the input decision variables, given a mode and time index.
  /// (knotpoint_index is w.r.t that particular mode).
  const drake::solvers::VectorXDecisionVariable input_vars(
      int mode_index, int knotpoint_index) const;

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

  int num_modes() const;

  /// Get the number of knotpoints in a specified mode
  int mode_length(int mode_index) const;

  const multibody::KinematicEvaluatorSet<T>& get_evaluator_set(int mode) const {
    return mode_sequence_.mode(mode).evaluators();
  }

  const DirconMode<T>& get_mode(int mode) const {
    return mode_sequence_.mode(mode);
  }

  const drake::systems::Context<T>& get_context(int mode, int knotpoint_index) {
    return *contexts_.at(mode).at(knotpoint_index);
  }

  /// Setters for variable scaling
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
  // Private constructor to which public constructors funnel
  Dircon(std::unique_ptr<DirconModeSequence<T>> my_sequence,
         const DirconModeSequence<T>* ext_sequence,
         const drake::multibody::MultibodyPlant<T>& plant, int num_knotpoints);

  std::unique_ptr<DirconModeSequence<T>> my_sequence_;
  const drake::multibody::MultibodyPlant<T>& plant_;
  const DirconModeSequence<T>& mode_sequence_;
  std::vector<std::vector<std::unique_ptr<drake::systems::Context<T>>>>
      contexts_;
  std::vector<int> mode_start_;
  void DoAddRunningCost(const drake::symbolic::Expression& e) override;
  std::vector<drake::solvers::VectorXDecisionVariable> force_vars_;
  std::vector<drake::solvers::VectorXDecisionVariable> collocation_force_vars_;
  std::vector<drake::solvers::VectorXDecisionVariable> collocation_slack_vars_;
  std::vector<drake::solvers::VectorXDecisionVariable> v_post_impact_vars_;
  std::vector<drake::solvers::VectorXDecisionVariable> impulse_vars_;
  std::vector<drake::solvers::VectorXDecisionVariable> offset_vars_;
  std::vector<drake::solvers::VectorXDecisionVariable> quaternion_slack_vars_;
  std::unique_ptr<multibody::MultiposeVisualizer> callback_visualizer_;
  std::vector<std::unique_ptr<DynamicsCache<T>>> cache_;

  std::vector<std::pair<drake::VectorX<drake::symbolic::Variable>,
                        drake::VectorX<drake::symbolic::Expression>>>
      v_post_impact_vars_substitute_;
};

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace dairlib
