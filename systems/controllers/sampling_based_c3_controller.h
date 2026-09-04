#pragma once

#include <memory>
#include <queue>
#include <string>
#include <vector>

#include <drake/common/yaml/yaml_io.h>
#include <drake/geometry/geometry_set.h>
#include <drake/geometry/proximity/obj_to_surface_mesh.h>
#include <drake/geometry/proximity/triangle_surface_mesh.h>
#include <drake/geometry/query_object.h>

#include "c3/core/c3.h"
#include "c3/core/c3_options.h"
#include "c3/core/lcs.h"
#include "c3/core/solver_options_io.h"
#include "c3/multibody/lcs_factory.h"
#include "c3/multibody/lcs_factory_options.h"
#include "c3/systems/framework/c3_output.h"
#include "common/find_resource.h"
#include "common/update_context.h"
#include "dairlib/lcmt_sampling_c3_debug.hpp"
#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "examples/sampling_c3/jamming_ground_truth.h"
#include "examples/sampling_c3/jamming_metrics.h"
#include "examples/sampling_c3/parameter_headers/progress_params.h"
#include "examples/sampling_c3/parameter_headers/reposition_params.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_controller_params.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_options.h"
#include "examples/sampling_c3/parameter_headers/sampling_params.h"
#include "lcm/lcm_trajectory.h"
#include "systems/controllers/face.h"
#include "systems/framework/timestamped_vector.h"

#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::MatrixX;
using drake::SortedPair;
using drake::VectorX;
using drake::geometry::GeometryId;
using drake::math::ExtractGradient;
using drake::math::ExtractValue;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using systems::Face;
using systems::TimestampedVector;

namespace systems {

enum SampleIndex {
  kCurrentLocation,
  kCurrentReposTarget  // Only represents current reposition target when in
                       // reposition mode.
  // Could expand this enum if want to reference more samples.
};

enum ModeSwitchReason {
  kNoSwitch,
  kToC3Cost,
  kToC3ReachedReposTarget,
  kToReposCost,
  kToReposUnproductive,
  kToC3Xbox
};

enum PursuedTargetSource { kNoTarget, kPrevious, kNewSample, kFromBuffer };

/// One sample's worth of offline jamming analysis: the predicted peak EE effort
/// alongside the C3 cost the controller would have scored that sample with, so
/// the two can be compared.
struct SampleJammingResult {
  JammingMetrics metrics;
  double c3_cost = 0.0;
  /// EE positions of the retimed plan the metrics were measured on, one per
  /// knot.  A ground truth sim replays this, so its label describes the same
  /// trajectory the metrics scored rather than a push of its own invention.
  std::vector<Eigen::Vector3d> retimed_ee_plan;
  /// Seconds between those knots -- the planning dt this solve ran at, which
  /// the controller picks per goal.  Reported rather than left for a caller to
  /// re-derive, since replaying the plan at the wrong rate would rescale every
  /// speed in it.
  double planning_dt = 0.0;
  /// True if any of this sample's QP solves fell back to "hold state, zero
  /// inputs".  Such a sample's metrics understate the force and must not be
  /// read as a low-force location.
  bool solve_fell_back = false;
};

class SamplingC3Controller : public drake::systems::LeafSystem<double> {
 public:
  explicit SamplingC3Controller(
      drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context,
      drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
      drake::systems::Context<drake::AutoDiffXd>* context_ad,
      const std::vector<
          std::vector<drake::SortedPair<drake::geometry::GeometryId>>>&
          contact_geoms,
      SamplingC3ControllerParams controller_params, bool verbose = false);

  // Input ports
  const drake::systems::InputPort<double>& get_input_port_target() const {
    return this->get_input_port(target_input_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_final_target() const {
    return this->get_input_port(final_target_input_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_radio() const {
    return this->get_input_port(radio_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_lcs_state() const {
    return this->get_input_port(lcs_state_input_port_);
  }

  // Output ports
  const drake::systems::OutputPort<double>&
  get_output_port_c3_solution_curr_plan() const {
    return this->get_output_port(c3_solution_curr_plan_port_);
  }
  const drake::systems::OutputPort<double>&
  get_output_port_c3_solution_curr_plan_actor() const {
    return this->get_output_port(c3_solution_curr_plan_actor_port_);
  }
  const drake::systems::OutputPort<double>&
  get_output_port_c3_solution_curr_plan_object() const {
    return this->get_output_port(c3_solution_curr_plan_object_port_);
  }
  const drake::systems::OutputPort<double>&
  get_output_port_c3_intermediates_curr_plan() const {
    return this->get_output_port(c3_intermediates_curr_plan_port_);
  }
  const drake::systems::OutputPort<double>&
  get_output_port_lcs_contact_jacobian_curr_plan() const {
    return this->get_output_port(lcs_contact_jacobian_curr_plan_port_);
  }
  const drake::systems::OutputPort<double>&
  get_output_port_dynamically_feasible_curr_plan_actor() const {
    return this->get_output_port(dynamically_feasible_curr_plan_actor_port_);
  }
  const drake::systems::OutputPort<double>&
  get_output_port_dynamically_feasible_curr_plan_object() const {
    return this->get_output_port(dynamically_feasible_curr_plan_object_port_);
  }
  const drake::systems::OutputPort<double>&
  get_output_port_c3_solution_best_plan() const {
    return this->get_output_port(c3_solution_best_plan_port_);
  }
  const drake::systems::OutputPort<double>&
  get_output_port_c3_solution_best_plan_actor() const {
    return this->get_output_port(c3_solution_best_plan_actor_port_);
  }
  const drake::systems::OutputPort<double>&
  get_output_port_c3_solution_best_plan_object() const {
    return this->get_output_port(c3_solution_best_plan_object_port_);
  }
  const drake::systems::OutputPort<double>&
  get_output_port_c3_intermediates_best_plan() const {
    return this->get_output_port(c3_intermediates_best_plan_port_);
  }
  const drake::systems::OutputPort<double>&
  get_output_port_lcs_contact_jacobian_best_plan() const {
    return this->get_output_port(lcs_contact_jacobian_best_plan_port_);
  }
  const drake::systems::OutputPort<double>&
  get_output_port_dynamically_feasible_best_plan_actor() const {
    return this->get_output_port(dynamically_feasible_best_plan_actor_port_);
  }
  const drake::systems::OutputPort<double>&
  get_output_port_dynamically_feasible_best_plan_object() const {
    return this->get_output_port(dynamically_feasible_best_plan_object_port_);
  }
  const drake::systems::OutputPort<double>&
  get_output_port_c3_traj_execute_actor() const {
    return this->get_output_port(c3_traj_execute_actor_port_);
  }
  const drake::systems::OutputPort<double>&
  get_output_port_repos_traj_execute_actor() const {
    return this->get_output_port(repos_traj_execute_actor_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_traj_execute_actor()
      const {
    return this->get_output_port(traj_execute_actor_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_is_c3_mode() const {
    return this->get_output_port(is_c3_mode_port_);
  }
  const drake::systems::OutputPort<double>&
  get_output_port_all_sample_locations() const {
    return this->get_output_port(all_sample_locations_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_all_sample_costs()
      const {
    return this->get_output_port(all_sample_costs_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_debug() const {
    return this->get_output_port(debug_lcmt_port_);
  }
  const drake::systems::OutputPort<double>&
  get_output_port_sample_buffer_configurations() const {
    return this->get_output_port(sample_buffer_configurations_port_);
  }
  const drake::systems::OutputPort<double>&
  get_output_port_sample_buffer_costs() const {
    return this->get_output_port(sample_buffer_costs_port_);
  }
  const drake::systems::OutputPort<double>&
  get_output_port_unsuccessful_sample_buffer_configurations() const {
    return this->get_output_port(
        unsuccessful_sample_buffer_configurations_port_);
  }
  const drake::systems::OutputPort<double>&
  get_output_port_unsuccessful_sample_buffer_costs() const {
    return this->get_output_port(unsuccessful_sample_buffer_costs_port_);
  }

  /// Builds the Drake sims that C3CostComputationType::kSimDrakeObjectOnly
  /// scores samples with:  one per parallel thread, since the cost loop runs
  /// them concurrently.  The sim's contents come from the demo's
  /// sim_params.yaml, which the controller does not itself read, so the demo's
  /// controller binary has to hand them over -- and only the 3D printer demos
  /// can, since JammingGroundTruthSim builds a printer.  Call it after
  /// construction and before the first control loop; cost type 7 throws
  /// without it.
  void EnableGroundTruthCostSim(
      const std::vector<std::string>& object_models, double sim_dt);

  /// Solves C3 for each of @p ee_samples -- candidate end effector positions,
  /// all evaluated against the same scene state @p x_lcs_curr -- and returns
  /// the jamming metrics and C3 cost of each.  Intended for offline analysis
  /// (see three_d_printer/test/jamming_sweep.cc); it publishes nothing and does
  /// not touch the mode-switching state, but it does refresh the per-goal
  /// settings and cost matrices, so do not interleave it with ComputePlan.
  ///
  /// @p x_lcs_des is the state C3 tracks; @p x_lcs_final_des is the goal used
  /// to decide whether the position-tracking or pose-tracking cost applies.
  /// Callers with no lookahead sub-goal can pass the same vector for both.
  ///
  /// @p num_threads bounds the solve loop's parallelism; pass 1 for a serial
  /// loop, or 0 to use the controller's configured thread count.  Solver
  /// fallbacks are attributed per sample either way.
  ///
  /// The metrics describe the plan retimed to the configured EE velocity
  /// limits -- what the execution path publishes and what a
  /// kSimImpedanceRetimedObjectCostOnly cost scores.  With
  /// ee_velocity_{horizontal,vertical}_limits unconfigured there is nothing to
  /// retime and the plan is measured as solved.
  std::vector<SampleJammingResult> EvaluateJammingMetricsForSamples(
      const Eigen::VectorXd& x_lcs_curr, const Eigen::VectorXd& x_lcs_des,
      const Eigen::VectorXd& x_lcs_final_des,
      const std::vector<Eigen::Vector3d>& ee_samples, int goal_step,
      int num_threads = 0) const;

  /// Draws candidate end effector positions with the demo's configured sampling
  /// strategy, deliberately with keep-out regions DISABLED, so an offline sweep
  /// can measure the force field everywhere the sampler would otherwise be
  /// allowed to go -- including inside a hand-authored keep-out region, which
  /// is the thing under study.  Every other filter (workspace limits, fixed
  /// geometry avoidance) still applies.  Offline analysis only.
  std::vector<Eigen::Vector3d> GenerateSampleEEPositionsIgnoringKeepOut(
      const Eigen::VectorXd& x_lcs_curr, bool is_doing_c3,
      const std::vector<bool>& object_on_target) const;

 private:
  std::pair<double, std::vector<Eigen::VectorXd>> CalcCost(
      C3CostComputationType cost_type, const c3::LCS& lcs_for_cost,
      const c3::C3::CostMatrices& cost_mats,
      const std::shared_ptr<c3::C3>& c3_object,
      const bool& force_tracking_disabled, int num_objects,
      const bool& print_cost_breakdown) const;
  /// Function for computing one control loop
  drake::systems::EventStatus ComputePlan(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  /// Helper functions

  void ResolvePredictedEEState(const bool& is_teleop,
                               drake::VectorX<double>& x_lcs_curr) const;

  void ClampEndEffectorAcceleration(drake::VectorX<double>& x_lcs_curr) const;

  void CheckForWorkspaceLimitViolations(
      const TimestampedVector<double>* lcs_x_curr) const;

  void UpdateCostMatrices(const drake::VectorX<double>& x_lcs_curr,
                          const BasicVector<double>& x_lcs_des,
                          const C3Options& c3_options) const;

  std::vector<SortedPair<GeometryId>> GetResolvedContactPairs(
      const drake::multibody::MultibodyPlant<double>& plant,
      const drake::systems::Context<double>& context,
      const std::vector<std::vector<SortedPair<GeometryId>>>& contact_geoms,
      const std::vector<int>& resolve_contacts_to_list,
      std::vector<int> num_friction_directions, bool verbose) const;

  std::pair<std::vector<c3::LCS>, std::vector<c3::LCS>>
  CreateLCSObjectsForSamples(
      const std::vector<Eigen::VectorXd>& candidate_states,
      const drake::VectorX<double>& x_lcs_curr,
      const LCSFactoryOptions& lcs_factory_options) const;

  /// Builds (but does not solve) the C3 problem for a single sample: picks the
  /// projection type and adds the workspace, end effector velocity, and input
  /// constraints.
  std::shared_ptr<c3::C3> MakeC3ForSample(
      const c3::LCS& lcs, const c3::C3::CostMatrices& cost_matrices,
      const std::vector<Eigen::VectorXd>& x_desired,
      const C3Options& c3_options) const;

  void UpdateC3ExecutionTrajectory(const Eigen::VectorXd& x_lcs,
                                   const double& t_context) const;

  void UpdateRepositioningExecutionTrajectory(const Eigen::VectorXd& x_lcs,
                                              const double& t_context) const;

  void PruneOutdatedSamplesFromBuffer(
      const Eigen::VectorXd& x_lcs, int* num_in_buffer,
      Eigen::MatrixXd* sample_buffer, Eigen::VectorXd* sample_costs_buffer,
      const double& pos_error_sample_retention,
      const double& ang_error_sample_retention) const;

  void MaintainSampleBuffers(const Eigen::VectorXd& x_lcs) const;

  void AugmentSamplesWithBuffer(
      std::vector<std::shared_ptr<c3::C3>>& c3_objects) const;

  void AddToUnsuccessfulBuffer(const Eigen::VectorXd& x_lcs) const;

  void KeepTrackOfC3ModeProgress(
      const drake::VectorX<double>& x_lcs_curr,
      const BasicVector<double>& x_lcs_final_des,
      bool& reset_progress_cost_buffer,
      const bool& print_current_pos_and_rot_cost) const;

  void ResetProgressMetrics() const;

  void ResetSampleBuffers() const;

  /// Refresh the per-goal-step "active" settings (cost switching threshold and
  /// keep-out geometry) for the given 0-based goal-sequence step.  The step is
  /// clamped into range defensively; the *_sequence parameter lengths are
  /// already validated against the goal-step count at YAML-load time.  Falls
  /// back to the scalar cost_switching_threshold_distance / an empty keep-out
  /// set when the corresponding sequence is unset.
  void RefreshPerGoalSettings(int goal_step) const;

  /// For a kFixedGoalSequence, whether the goal generator's published final
  /// target `x_lcs_final_des` is the last step of the sequence.
  bool IsFinalTargetTerminalGoal(const Eigen::VectorXd& x_lcs_final_des) const;

  /// Build the keep-out scene from controller_params_.keep_out_model_sequence
  /// and populate keep_out_geometry_sets_ / keep_out_step_has_regions_.  A
  /// no-op when no goal step declares a keep-out model.  Called from the
  /// constructor before RefreshPerGoalSettings(0).
  void BuildKeepOutScene();

  /// QueryObject for the private keep-out scene, or nullptr when there is none.
  const drake::geometry::QueryObject<double>* keep_out_query_object() const;

  void IncludeEEOrientationTargetIfEnabled(
      LcmTrajectory* lcm_trajectory, const Eigen::Vector3d& ee_position,
      const Eigen::VectorXd& timestamps) const;

  void ClampPlanToWorkspaceLimits(Eigen::MatrixXd* ee_position_traj) const;

  void ProjectPlanAwayFromFixedGeometries(
      Eigen::MatrixXd* ee_position_traj) const;

  /// Collapse sampling_c3_options_.ee_velocity_{horizontal,vertical}_limits,
  /// each a [min, max] pair, into the scalar speeds the retiming works with.
  /// Returns false (with a one-time warning) if either limit is missing or
  /// non-positive, in which case callers should skip retiming entirely.
  bool GetEEVelocityLimits(double* v_xy_max, double* v_z_max) const;

  /// Stretch `time_vector` in place so that, under a FirstOrderHold over
  /// (*time_vector, ee_position_traj), no segment exceeds the printer's
  /// configured horizontal (xy) / vertical (z) EE speed limits
  /// (sampling_c3_options_.ee_velocity_{horizontal,vertical}_limits).  Only
  /// ever slows the plan down: knot 0 is left where it is and later knots slide
  /// out, so the geometric path is untouched and no discontinuity is
  /// introduced. Times are kept strictly increasing.  A no-op (with a one-time
  /// warning) if the configured limits are missing or non-positive.
  void RetimeEEPlanToVelocityLimits(
      const Eigen::Ref<const Eigen::MatrixXd>& ee_position_traj,
      Eigen::VectorXd* time_vector) const;

  /// Slow the C3 plan's EE path to the configured velocity limits and then
  /// resample it back onto the plan's original knot times, so that the returned
  /// reference covers the same amount of time as the unretimed plan but only as
  /// much of the path as the printer can actually cover in that time.
  ///
  /// Only the EE position and velocity rows of `x_plan` are rewritten; the
  /// object rows are left alone because SimulatePDControlWithLCS's Kp/Kd touch
  /// the EE rows only.  `u_plan` is carried onto the same map (held from the
  /// segment each resampled knot came from) so the feedforward force stays
  /// aligned with where along the path the reference is.  A no-op if the
  /// configured limits are missing or non-positive.
  void RetimeAndResampleC3PlanForCost(
      double dt, std::vector<Eigen::VectorXd>* x_plan,
      std::vector<Eigen::VectorXd>* u_plan) const;

  /// Interpolate the current plan (full LCS state `knots`, columns aligned with
  /// `timestamps`) at `filtered_solve_time_` past the plan start and store it
  /// in `x_pred_curr_plan_`.  Handles a non-uniform (retimed) `timestamps` grid
  /// and overwrites the predicted EE velocity with the retimed segment's actual
  /// slope, so the predicted x0 handed to the next solve stays within the
  /// printer's EE speed limits.
  void PredictPlanStateAtSolveTime(const Eigen::MatrixXd& knots,
                                   const Eigen::VectorXd& timestamps) const;

  /// Output port functions
  void OutputC3SolutionCurrPlan(
      const drake::systems::Context<double>& context,
      c3::systems::C3Output::C3Solution* c3_solution) const;
  void OutputC3SolutionCurrPlanActor(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_timestamped_saved_traj* output) const;
  void OutputC3SolutionCurrPlanObject(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_timestamped_saved_traj* output) const;
  void OutputC3IntermediatesCurrPlan(
      const drake::systems::Context<double>& context,
      c3::systems::C3Output::C3Intermediates* c3_intermediates) const;
  void OutputLCSContactJacobianCurrPlan(
      const drake::systems::Context<double>& context,
      std::vector<c3::multibody::LCSContactDescription>*
          lcs_contact_descriptions) const;
  void OutputC3SolutionBestPlan(
      const drake::systems::Context<double>& context,
      c3::systems::C3Output::C3Solution* c3_solution) const;
  void OutputC3SolutionBestPlanActor(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_timestamped_saved_traj* output) const;
  void OutputC3SolutionBestPlanObject(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_timestamped_saved_traj* output) const;
  void OutputC3IntermediatesBestPlan(
      const drake::systems::Context<double>& context,
      c3::systems::C3Output::C3Intermediates* c3_intermediates) const;
  void OutputLCSContactJacobianBestPlan(
      const drake::systems::Context<double>& context,
      std::vector<c3::multibody::LCSContactDescription>*
          lcs_contact_descriptions) const;
  void OutputDynamicallyFeasibleCurrPlanActor(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_timestamped_saved_traj*
          dynamically_feasible_curr_plan_actor) const;
  void OutputDynamicallyFeasibleCurrPlanObject(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_timestamped_saved_traj*
          dynamically_feasible_curr_plan_object) const;
  void OutputDynamicallyFeasibleBestPlanActor(
      const drake::systems::Context<double>& context,
      lcmt_timestamped_saved_traj* dynamically_feasible_best_plan_actor) const;
  void OutputDynamicallyFeasibleBestPlanObject(
      const drake::systems::Context<double>& context,
      lcmt_timestamped_saved_traj* dynamically_feasible_best_plan_object) const;
  void OutputAllSampleLocations(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_timestamped_saved_traj* all_sample_locations) const;
  void OutputAllSampleCosts(
      const drake::systems::Context<double>& context,
      lcmt_timestamped_saved_traj* output_all_sample_costs) const;
  void OutputC3TrajExecuteActor(
      const drake::systems::Context<double>& context,
      lcmt_timestamped_saved_traj* c3_execution_lcm_traj) const;
  void OutputReposTrajExecuteActor(
      const drake::systems::Context<double>& context,
      lcmt_timestamped_saved_traj* repos_execution_lcm_traj) const;
  void OutputTrajExecuteActor(
      const drake::systems::Context<double>& context,
      lcmt_timestamped_saved_traj* execution_lcm_traj) const;
  void OutputIsC3Mode(const drake::systems::Context<double>& context,
                      dairlib::lcmt_timestamped_saved_traj* is_c3_mode) const;
  void OutputDebug(const drake::systems::Context<double>& context,
                   dairlib::lcmt_sampling_c3_debug* debug_msg) const;
  void OutputSampleBufferConfigurations(
      const drake::systems::Context<double>& context,
      Eigen::MatrixXd* sample_buffer_configurations) const;
  void OutputSampleBufferCosts(const drake::systems::Context<double>& context,
                               Eigen::VectorXd* sample_buffer_costs) const;
  void OutputUnsuccessfulSampleBufferConfigurations(
      const drake::systems::Context<double>& context,
      Eigen::MatrixXd* unsuccessful_sample_buffer_configurations) const;
  void OutputUnsuccessfulSampleBufferCosts(
      const drake::systems::Context<double>& context,
      Eigen::VectorXd* unsuccessful_sample_buffer_costs) const;

  // Quantities computed once in the constructor for certain sampling
  // strategies.
  std::vector<double> face_bins_;
  std::vector<Face> faces_;
  std::vector<std::vector<Face>> faces_per_object_;
  std::vector<std::vector<double>> face_bins_per_object_;
  std::vector<double> total_area_per_object_;
  std::vector<drake::geometry::GeometryId> object_geometry_ids_;
  std::vector<double> object_enclosing_radius_;
  double ee_radius_ = 0.0;

  // Fixed (non-EE, non-manipulated-object) collision geometries in the
  // scene that exported EE plans must stay workspace_margins (plus ee_radius_)
  // away from.
  drake::geometry::GeometrySet fixed_obstacle_geometries_;

  // A private scene holding ONLY the per-goal keep-out geometry.  Null when no
  // goal step declares a keep-out model.
  std::unique_ptr<drake::systems::Diagram<double>> keep_out_diagram_;
  const drake::multibody::MultibodyPlant<double>* keep_out_plant_ = nullptr;
  std::unique_ptr<drake::systems::Context<double>> keep_out_diagram_context_;

  // One keep-out GeometrySet per goal-sequence step (empty when that step has
  // no keep-out model), indexing into the private scene above.
  std::vector<drake::geometry::GeometrySet> keep_out_geometry_sets_;
  // Parallel to keep_out_geometry_sets_: whether that step actually has any
  // keep-out geometry (a GeometrySet exposes no size, so track it explicitly).
  std::vector<bool> keep_out_step_has_regions_;

  drake::systems::InputPortIndex radio_port_;
  drake::systems::InputPortIndex final_target_input_port_;
  drake::systems::InputPortIndex target_input_port_;
  drake::systems::InputPortIndex lcs_state_input_port_;
  // Current sample output port indices
  drake::systems::OutputPortIndex c3_solution_curr_plan_port_;
  drake::systems::OutputPortIndex c3_solution_curr_plan_actor_port_;
  drake::systems::OutputPortIndex c3_solution_curr_plan_object_port_;
  drake::systems::OutputPortIndex c3_intermediates_curr_plan_port_;
  drake::systems::OutputPortIndex lcs_contact_jacobian_curr_plan_port_;
  // Best sample output port indices
  drake::systems::OutputPortIndex c3_solution_best_plan_port_;
  drake::systems::OutputPortIndex c3_solution_best_plan_actor_port_;
  drake::systems::OutputPortIndex c3_solution_best_plan_object_port_;
  drake::systems::OutputPortIndex c3_intermediates_best_plan_port_;
  drake::systems::OutputPortIndex lcs_contact_jacobian_best_plan_port_;
  // Execution trajectory output port indices
  drake::systems::OutputPortIndex c3_traj_execute_actor_port_;
  drake::systems::OutputPortIndex repos_traj_execute_actor_port_;
  drake::systems::OutputPortIndex traj_execute_actor_port_;
  drake::systems::OutputPortIndex is_c3_mode_port_;
  // Dynamically feasible plan output port indices
  drake::systems::OutputPortIndex dynamically_feasible_curr_plan_actor_port_;
  drake::systems::OutputPortIndex dynamically_feasible_curr_plan_object_port_;
  drake::systems::OutputPortIndex dynamically_feasible_best_plan_actor_port_;
  drake::systems::OutputPortIndex dynamically_feasible_best_plan_object_port_;
  // Sample related output port indices
  drake::systems::OutputPortIndex all_sample_locations_port_;
  drake::systems::OutputPortIndex all_sample_costs_port_;
  drake::systems::OutputPortIndex debug_lcmt_port_;
  drake::systems::OutputPortIndex sample_buffer_configurations_port_;
  drake::systems::OutputPortIndex sample_buffer_costs_port_;
  drake::systems::OutputPortIndex
      unsuccessful_sample_buffer_configurations_port_;
  drake::systems::OutputPortIndex unsuccessful_sample_buffer_costs_port_;

  // This plant_ has been made 'not const' so that the context can be updated.
  drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad_;
  drake::systems::Context<drake::AutoDiffXd>* context_ad_;
  const std::vector<
      std::vector<drake::SortedPair<drake::geometry::GeometryId>>>&
      contact_pairs_;
  c3::multibody::ContactModel contact_model_;

  const SamplingC3ControllerParams controller_params_;
  const SamplingC3Options sampling_c3_options_;
  const SamplingParams sampling_params_;
  const SamplingC3RepositionParams reposition_params_;
  const SamplingC3ProgressParams progress_params_;
  const SamplingC3GoalParams goal_params_;
  drake::solvers::SolverOptions solver_options_;

  const bool verbose_;
  int n_q_;
  int n_v_;
  int n_x_;
  int n_lambda_;
  int n_u_;
  int max_num_samples_;
  int N_;
  int n_z_;

  double solve_time_filter_constant_;
  drake::systems::DiscreteStateIndex plan_start_time_index_;

  const bool adaptive_ee_tilt_;
  double max_ee_dist_from_workspace_center_;
  Eigen::Vector3d workspace_center_;

  /// TODO:  There are many mutable class variables, which is not best practice
  /// in the Drake systems framework.  These could be converted to discrete
  /// state variables.
  mutable double dt_;

  Eigen::VectorXd Kp_for_cost_;
  Eigen::VectorXd Kd_for_cost_;

  mutable std::vector<Eigen::MatrixXd> Q_;
  mutable std::vector<Eigen::MatrixXd> R_;
  mutable std::vector<Eigen::MatrixXd> G_;
  mutable std::vector<Eigen::MatrixXd> U_;

  // Keep track of current C3 execution's best seen cost.
  mutable int best_progress_steps_ago_;
  mutable double lowest_cost_;
  mutable double lowest_pos_and_rot_current_cost_;
  mutable double lowest_position_error_;
  mutable double lowest_orientation_error_;
  mutable double current_position_error_;
  mutable double current_orientation_error_;
  mutable std::queue<double> object_config_cost_history_;

  mutable double filtered_solve_time_ = 0;

  // Predictions for the end effector location.
  mutable Eigen::VectorXd x_pred_curr_plan_;
  mutable Eigen::VectorXd x_from_last_control_loop_;
  mutable Eigen::VectorXd x_pred_from_last_control_loop_;
  mutable Eigen::Vector3d ee_position_curr_;

  // C3 solution for current location.
  mutable std::shared_ptr<c3::C3> c3_curr_plan_;
  // TODO: these are currently assigned values but go unused -- may be useful if
  // implementing warm start.
  mutable std::vector<Eigen::VectorXd> z_sol_curr_plan_;
  mutable std::vector<Eigen::VectorXd> delta_curr_plan_;
  mutable std::vector<Eigen::VectorXd> w_curr_plan_;

  // C3 solution for best sample location.
  mutable std::shared_ptr<c3::C3> c3_best_plan_;
  // TODO: these are currently assigned values but go unused -- may be useful if
  // implementing warm start.
  mutable std::vector<Eigen::VectorXd> z_sol_best_plan_;
  mutable std::vector<Eigen::VectorXd> delta_best_plan_;
  mutable std::vector<Eigen::VectorXd> w_best_plan_;

  // C3 solution for best sample in buffer.
  mutable std::shared_ptr<c3::C3> c3_buffer_plan_;
  mutable std::vector<Eigen::VectorXd> dynamically_feasible_buffer_plan_;

  // LCS trajectories for C3 or repositioning modes.
  mutable LcmTrajectory c3_execution_lcm_traj_;
  mutable LcmTrajectory repos_execution_lcm_traj_;

  // Samples and associated costs computed in current control loop.
  mutable std::vector<Eigen::Vector3d> all_sample_locations_;
  mutable std::vector<std::vector<Eigen::VectorXd>>
      all_sample_dynamically_feasible_plans_;
  mutable Eigen::Vector3d prev_repositioning_target_ = Eigen::Vector3d::Zero();
  mutable std::vector<double> all_sample_costs_;

  // For the published-trajectory sanity check in OutputTrajExecuteActor; see
  // the investigation notes in the repo history / plan doc for 2026-08-05.
  mutable Eigen::Vector3d last_published_ee_knot0_ = Eigen::Vector3d::Zero();
  mutable bool has_last_published_ee_knot0_ = false;

  // Per-goal-step settings, refreshed by RefreshPerGoalSettings() whenever the
  // detected goal step changes (and seeded for step 0 in the constructor).
  mutable double active_cost_switching_threshold_distance_ = 0.0;
  mutable const drake::geometry::GeometrySet* active_keep_out_geometries_ =
      nullptr;

  // To detect if the final goal has been updated.
  mutable Eigen::VectorXd x_final_target_;
  mutable int detected_goal_changes_ = -1;
  mutable bool achieved_fixed_goal_ = false;
  // Number of consecutive control loops during which every goal has been
  // reached without the final target changing.  Used to detect that a
  // kFixedGoalSequence has settled on its terminal goal (intermediate goals
  // advance the target as soon as they're reached), at which point the
  // controller parks the EE just like it does for kFixedGoal.
  mutable int consecutive_all_reached_loops_ = 0;
  static constexpr int kParkAfterAllReachedLoops = 3;

  // Sample buffer-related variables.
  mutable int num_in_buffer_ = 0;
  mutable Eigen::MatrixXd sample_buffer_;  // (N_sample_buffer x n_q)
  mutable Eigen::VectorXd sample_costs_buffer_;

  // Unsuccessful sample buffer-related variables.
  mutable int num_in_unsuccessful_buffer_ = 0;
  mutable Eigen::MatrixXd
      unsuccessful_sample_buffer_;  // (num_in_unsuccessful_buffer_ x n_q)
  mutable Eigen::VectorXd unsuccessful_sample_costs_buffer_;

  // Miscellaneous sample related variables.
  mutable bool is_doing_c3_ = true;
  mutable bool finished_reposition_flag_ = false;
  // Crossing a threshold as the object gets closer to the goal means the
  // controller goes from caring only about object position to caring about full
  // pose.
  mutable bool crossed_cost_switching_threshold_ = false;
  mutable int num_threads_to_use_;

  // One Drake sim per thread of the cost loop, empty unless
  // EnableGroundTruthCostSim has been called.  A single instance would do --
  // JammingGroundTruthSim::Rollout holds no state -- but a sim apiece keeps
  // the threads out of each other's allocators.
  std::vector<std::unique_ptr<JammingGroundTruthSim>> ground_truth_sims_;

  mutable SampleIndex best_sample_index_ = kCurrentLocation;
  mutable ModeSwitchReason mode_switch_reason_ = kNoSwitch;
  mutable PursuedTargetSource pursued_target_source_ = kNoTarget;
};

}  // namespace systems
}  // namespace dairlib
