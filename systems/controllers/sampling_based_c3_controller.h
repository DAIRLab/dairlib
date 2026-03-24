#pragma once

#include <queue>
#include <string>
#include <vector>

#include <drake/common/yaml/yaml_io.h>
#include <drake/geometry/proximity/obj_to_surface_mesh.h>
#include <drake/geometry/proximity/triangle_surface_mesh.h>

#include "common/find_resource.h"
#include "common/update_context.h"
#include "dairlib/lcmt_sampling_c3_debug.hpp"
#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "examples/sampling_c3/parameter_headers/progress_params.h"
#include "examples/sampling_c3/parameter_headers/reposition_params.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_controller_params.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_options.h"
#include "examples/sampling_c3/parameter_headers/sampling_params.h"
#include "lcm/lcm_trajectory.h"
#include "solvers/base_c3.h"
#include "solvers/c3_options.h"
#include "solvers/c3_output.h"
#include "solvers/lcs.h"
#include "solvers/lcs_factory.h"
#include "solvers/solver_options_io.h"
#include "systems/controllers/face.h"
#include "systems/framework/timestamped_vector.h"

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

class SamplingC3Controller : public drake::systems::LeafSystem<double> {
 public:
  // TODO make changes to constructor for EPSC3
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

 protected:
  /// Function for computing one control loop
  // TODO rework for EPSC3
  virtual drake::systems::EventStatus ComputePlan(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  virtual solvers::LCS CreatePlaceholderLCS() const;

  // TODO rework for EPSC3
  virtual std::pair<std::vector<solvers::LCS>, std::vector<solvers::LCS>>
  CreateLCSObjectsForSamples(
      const std::vector<Eigen::VectorXd>& candidate_states,
      const drake::VectorX<double>& x_lcs_curr, const C3Options& c3_options,
      const C3Options& c3_options_curr_location) const;

  virtual void PruneOutdatedSamplesFromBuffer(
      const Eigen::VectorXd& x_lcs, int* num_in_buffer,
      Eigen::MatrixXd* sample_buffer, Eigen::VectorXd* sample_costs_buffer,
      const double& pos_error_sample_retention,
      const double& ang_error_sample_retention) const;

  /// Variables that are accessible to child classes.
  SamplingC3ControllerParams controller_params_;
  int n_q_;
  int n_v_;
  int n_x_;
  int n_lambda_;
  int n_u_;
  int N_;
  const bool verbose_;
  mutable double dt_ = 0.1;
  mutable double dt_cost_ = 0.02;
  // This plant_ has been made 'not const' so that the context can be updated.
  drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad_;
  drake::systems::Context<drake::AutoDiffXd>* context_ad_;
  const std::vector<
      std::vector<drake::SortedPair<drake::geometry::GeometryId>>>&
      contact_pairs_;
  solvers::ContactModel contact_model_;

 private:
  /// Helper functions
  // TODO hardcodes robot state size
  void ResolvePredictedEEState(const bool& is_teleop,
                               drake::VectorX<double>& x_lcs_curr) const;

  // TODO hardcodes robot state size
  void ClampEndEffectorAcceleration(drake::VectorX<double>& x_lcs_curr) const;

  // TODO hardcodes robot state size
  void CheckForWorkspaceLimitViolations(
      const TimestampedVector<double>* lcs_x_curr) const;

  // TODO hardcodes object quaternions, etc. -- could avoid by having
  // use_quaternion_dependent_cost false
  void UpdateCostMatrices(const drake::VectorX<double>& x_lcs_curr,
                          const BasicVector<double>& x_lcs_des,
                          const C3Options& c3_options) const;

  // TODO needs c3_curr_plan_, is_doing_c3_, filtered_solve_time_
  // TODO hardcodes robot state size
  void UpdateC3ExecutionTrajectory(const Eigen::VectorXd& x_lcs,
                                   const double& t_context) const;

  // TODO needs all_sample_locations_, best_sample_index_, some other params and
  // flags
  void UpdateRepositioningExecutionTrajectory(const Eigen::VectorXd& x_lcs,
                                              const double& t_context) const;

  // TODO hardcodes robot state size
  void MaintainSampleBuffers(const Eigen::VectorXd& x_lcs) const;

  void AugmentSamplesWithBuffer(
      std::vector<std::shared_ptr<solvers::C3Base>>& c3_objects) const;

  void AddToUnsuccessfulBuffer(const Eigen::VectorXd& x_lcs) const;

  void KeepTrackOfC3ModeProgress(
      const drake::VectorX<double>& x_lcs_curr,
      const BasicVector<double>& x_lcs_final_des,
      bool& reset_progress_cost_buffer,
      const bool& print_current_pos_and_rot_cost) const;

  void ResetProgressMetrics() const;

  /// Output port functions
  /// TODO:  many of these can stay the same as long as these variables are
  /// properly set:  c3_curr_plan_, c3_best_plan_, all_sample_locations_,
  /// best_sample_index_, filtered_solve_time_, is_doing_c3_,
  /// c3_execution_lcm_traj_, repos_execution_lcm_traj_,
  /// all_sample_dynamically_feasible_plans_, all_sample_costs_, debugging
  /// stuff, sample_buffer_, sample_costs_buffer_, unsuccessful_sample_buffer_,
  /// unsuccessful_sample_costs_buffer_
  void OutputC3SolutionCurrPlan(const drake::systems::Context<double>& context,
                                C3Output::C3Solution* c3_solution) const;
  void OutputC3SolutionCurrPlanActor(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_timestamped_saved_traj* output) const;
  void OutputC3SolutionCurrPlanObject(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_timestamped_saved_traj* output) const;
  void OutputC3IntermediatesCurrPlan(
      const drake::systems::Context<double>& context,
      C3Output::C3Intermediates* c3_intermediates) const;
  void OutputLCSContactJacobianCurrPlan(
      const drake::systems::Context<double>& context,
      std::pair<Eigen::MatrixXd, std::vector<Eigen::VectorXd>>*
          lcs_contact_jacobian) const;
  void OutputC3SolutionBestPlan(const drake::systems::Context<double>& context,
                                C3Output::C3Solution* c3_solution) const;
  void OutputC3SolutionBestPlanActor(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_timestamped_saved_traj* output) const;
  void OutputC3SolutionBestPlanObject(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_timestamped_saved_traj* output) const;
  void OutputC3IntermediatesBestPlan(
      const drake::systems::Context<double>& context,
      C3Output::C3Intermediates* c3_intermediates) const;
  void OutputLCSContactJacobianBestPlan(
      const drake::systems::Context<double>& context,
      std::pair<Eigen::MatrixXd, std::vector<Eigen::VectorXd>>*
          lcs_contact_jacobian) const;
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

  std::vector<double> face_bins_;
  std::vector<Face> faces_;
  std::vector<std::vector<Face>> faces_per_object_;
  std::vector<std::vector<double>> face_bins_per_object_;
  std::vector<double> total_area_per_object_;

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

  SamplingC3Options sampling_c3_options_;
  SamplingParams sampling_params_;
  SamplingC3RepositionParams reposition_params_;
  SamplingC3ProgressParams progress_params_;
  SamplingC3GoalParams goal_params_;
  drake::solvers::SolverOptions solver_options_ =
      drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
          "solvers/osqp_options_default.yaml")
          .GetAsSolverOptions(drake::solvers::OsqpSolver::id());

  int max_num_samples_;
  int n_z_;

  double solve_time_filter_constant_;
  drake::systems::DiscreteStateIndex plan_start_time_index_;

  /// TODO:  There are many mutable class variables, which is not best practice
  /// in the Drake systems framework.  These could be converted to discrete
  /// state variables.
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
  mutable std::shared_ptr<solvers::C3Base> c3_curr_plan_;
  // TODO: these are currently assigned values but go unused -- may be useful if
  // implementing warm start.
  mutable std::vector<Eigen::VectorXd> z_sol_curr_plan_;
  mutable std::vector<Eigen::VectorXd> delta_curr_plan_;
  mutable std::vector<Eigen::VectorXd> w_curr_plan_;

  // C3 solution for best sample location.
  mutable std::shared_ptr<solvers::C3Base> c3_best_plan_;
  // TODO: these are currently assigned values but go unused -- may be useful if
  // implementing warm start.
  mutable std::vector<Eigen::VectorXd> z_sol_best_plan_;
  mutable std::vector<Eigen::VectorXd> delta_best_plan_;
  mutable std::vector<Eigen::VectorXd> w_best_plan_;

  // C3 solution for best sample in buffer.
  mutable std::shared_ptr<solvers::C3Base> c3_buffer_plan_;
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

  // To detect if the final goal has been updated.
  mutable Eigen::VectorXd x_final_target_;
  mutable int detected_goal_changes_ = -1;
  mutable bool achieved_fixed_goal_ = false;

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

  mutable SampleIndex best_sample_index_ = kCurrentLocation;
  mutable ModeSwitchReason mode_switch_reason_ = kNoSwitch;
  mutable PursuedTargetSource pursued_target_source_ = kNoTarget;
};

}  // namespace systems
}  // namespace dairlib
