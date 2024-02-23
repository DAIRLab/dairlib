#pragma once

#include <string>
#include <vector>

#include <drake/common/yaml/yaml_io.h>

#include "common/find_resource.h"
#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"
#include "solvers/c3.h"

#include "solvers/c3_options.h"
#include "solvers/c3_output.h"
#include "solvers/lcs.h"
#include "solvers/solver_options_io.h"
#include "systems/framework/timestamped_vector.h"

#include "drake/systems/framework/leaf_system.h"



namespace dairlib {
using systems::TimestampedVector;
using drake::systems::BasicVector;

namespace systems {

/// Outputs a lcmt_timestamped_saved_traj
class SamplingC3Controller : public drake::systems::LeafSystem<double> {
 public:
  explicit SamplingC3Controller(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context,
      const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
      drake::systems::Context<drake::AutoDiffXd>* context_ad,
      const std::vector<drake::SortedPair<drake::geometry::GeometryId>>& contact_geoms,
      C3Options c3_options);

  const drake::systems::InputPort<double>& get_input_port_target() const {
    return this->get_input_port(target_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_radio() const {
    return this->get_input_port(radio_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_lcs_state() const {
    return this->get_input_port(lcs_state_input_port_);
  }

  // Current location plan output ports
  const drake::systems::OutputPort<double>& get_output_port_c3_solution_curr_plan()
      const {
    return this->get_output_port(c3_solution_curr_plan_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_c3_intermediates_curr_plan()
      const {
    return this->get_output_port(c3_intermediates_curr_plan_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_lcs_contact_jacobian_curr_plan() 
      const {
    return this->get_output_port(lcs_contact_jacobian_curr_plan_port_);
  }

  // Best sample plan output ports
  const drake::systems::OutputPort<double>& get_output_port_c3_solution_best_plan()
      const {
    return this->get_output_port(c3_solution_best_plan_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_c3_intermediates_best_plan()
      const {
    return this->get_output_port(c3_intermediates_best_plan_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_lcs_contact_jacobian_best_plan() 
      const {
    return this->get_output_port(lcs_contact_jacobian_best_plan_port_);
  }

  // Execution trajectory output ports
  const drake::systems::OutputPort<double>& get_output_port_c3_traj_execute() 
      const {
    return this->get_output_port(c3_traj_execute_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_repos_traj_execute() 
      const {
    return this->get_output_port(repos_traj_execute_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_is_c3_mode() 
      const {
    return this->get_output_port(is_c3_mode_port_);
  }

  // Sample related output ports
  const drake::systems::OutputPort<double>& get_output_port_all_sample_locations() 
      const {
    return this->get_output_port(all_sample_locations_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_all_sample_costs() 
      const {
    return this->get_output_port(all_sample_costs_port_);
  }

  // The solver options need not be done twice i.e. one for each c3 solution 
  // object.
  void SetOsqpSolverOptions(const drake::solvers::SolverOptions& options) {
    solver_options_ = options;
    c3_curr_plan_->SetOsqpSolverOptions(solver_options_);
  }
  
 private:
  solvers::LCS CreatePlaceholderLCS() const;

  drake::systems::EventStatus ComputePlan(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void UpdateContext(Eigen::VectorXd lcs_state);

  void UpdateC3ExecutionTrajectory(const Eigen::VectorXd& x_lcs);

  void UpdateRepositioningExecutionTrajectory(const Eigen::VectorXd& x_lcs);

  void OutputC3SolutionCurrPlan(
    const drake::systems::Context<double>& context,
    C3Output::C3Solution* c3_solution) const;

  void OutputC3IntermediatesCurrPlan(
    const drake::systems::Context<double>& context,
    C3Output::C3Intermediates* c3_intermediates) const;

  void OutputLCSContactJacobianCurrPlan(
    const drake::systems::Context<double>& context,
    std::pair<Eigen::MatrixXd, std::vector<Eigen::VectorXd>>* lcs_contact_jacobian) const;

  void OutputC3SolutionBestPlan(
    const drake::systems::Context<double>& context,
    C3Output::C3Solution* c3_solution) const;

  void OutputC3IntermediatesBestPlan(
    const drake::systems::Context<double>& context,
    C3Output::C3Intermediates* c3_intermediates) const;

  void OutputLCSContactJacobianBestPlan(
    const drake::systems::Context<double>& context,
    std::pair<Eigen::MatrixXd, std::vector<Eigen::VectorXd>>* lcs_contact_jacobian) const;

  void OutputAllSampleLocations(
    const drake::systems::Context<double>& context,
    std::vector<Eigen::Vector3d>* all_sample_locations) const;

  void OutputAllSampleCosts(
    const drake::systems::Context<double>& context,
    std::vector<double>* all_sample_costs) const;

  void OutputC3TrajExecute(
    const drake::systems::Context<double>& context,
    std::vector<TimestampedVector<double>>* c3_traj_execute) const;

  void OutputReposTrajExecute(
    const drake::systems::Context<double>& context,
    std::vector<TimestampedVector<double>>* repos_traj_execute) const;

  void OutputIsC3Mode(
    const drake::systems::Context<double>& context,
    bool* is_c3_mode) const;

  drake::systems::InputPortIndex radio_port_;
  drake::systems::InputPortIndex target_input_port_;
  drake::systems::InputPortIndex lcs_state_input_port_;
  // Current sample output port indices
  drake::systems::OutputPortIndex c3_solution_curr_plan_port_;
  drake::systems::OutputPortIndex c3_intermediates_curr_plan_port_;
  drake::systems::OutputPortIndex lcs_contact_jacobian_curr_plan_port_;
  // Best sample output port indices
  drake::systems::OutputPortIndex c3_solution_best_plan_port_;
  drake::systems::OutputPortIndex c3_intermediates_best_plan_port_;
  drake::systems::OutputPortIndex lcs_contact_jacobian_best_plan_port_;
  // Execution trajectory output port indices
  drake::systems::OutputPortIndex c3_traj_execute_port_;
  drake::systems::OutputPortIndex repos_traj_execute_port_;
  drake::systems::OutputPortIndex is_c3_mode_port_;
  // Sample related output port indices
  drake::systems::OutputPortIndex all_sample_locations_port_;
  drake::systems::OutputPortIndex all_sample_costs_port_;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad_;
  drake::systems::Context<drake::AutoDiffXd>* context_ad_;
  const std::vector<drake::SortedPair<drake::geometry::GeometryId>>&
      contact_pairs_;

  C3Options c3_options_;
  drake::solvers::SolverOptions solver_options_ =
      drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
          "solvers/osqp_options_default.yaml")
          .GetAsSolverOptions(drake::solvers::OsqpSolver::id());

  // convenience for variable sizes
  int n_q_;
  int n_v_;
  int n_x_;
  int n_lambda_;
  int n_u_;
  double dt_;

  double solve_time_filter_constant_;
  drake::systems::DiscreteStateIndex plan_start_time_index_;
  std::vector<Eigen::MatrixXd> Q_;
  std::vector<Eigen::MatrixXd> R_;
  std::vector<Eigen::MatrixXd> G_;
  std::vector<Eigen::MatrixXd> U_;
  int N_;


  mutable double filtered_solve_time_ = 0;

  // C3 solutions for current location.
  // TODO: to be updated in ComputePlan
  mutable std::unique_ptr<solvers::C3> c3_curr_plan_;
  mutable std::vector<Eigen::VectorXd> delta_plan_;
  mutable std::vector<Eigen::VectorXd> w_plan_;
  mutable Eigen::VectorXd x_pred_curr_plan_;
  
  // C3 solutions for best sample location.
  // TODO: to be updated in ComputePlan
  mutable std::unique_ptr<solvers::C3> c3_best_plan_;
  mutable std::vector<Eigen::VectorXd> delta_best_plan_;
  mutable std::vector<Eigen::VectorXd> w_best_plan_;
  mutable Eigen::VectorXd x_pred_best_plan_;

  // LCS trajectories for C3 or repositioning modes.
  // mutable std::vector<TimestampedVector<double>> c3_traj_execute_;
  // mutable std::vector<TimestampedVector<double>> repos_traj_execute_;
  mutable LcmTrajectory c3_execution_lcm_traj_;
  mutable LcmTrajectory repos_execution_lcm_traj_;
 

  // Samples and associated costs.
  // TODO: to be updated in ComputePlan
  mutable std::vector<Eigen::Vector3d> all_sample_locations_;
  mutable std::vector<double> all_sample_costs_;

  // Miscellaneous sample related variables.
  mutable bool is_doing_c3_ = true;
  mutable bool finished_reposition_flag_ = false;
  const int num_threads_to_use_;

  enum SampleIndex { CURRENT_LOCATION_INDEX,
                     SAMPLE_INDEX_1, SAMPLE_INDEX_2, SAMPLE_INDEX_3,
                     SAMPLE_INDEX_4, SAMPLE_INDEX_5, SAMPLE_INDEX_6,
                     SAMPLE_INDEX_7, SAMPLE_INDEX_8, SAMPLE_INDEX_9,
                     SAMPLE_INDEX_10, SAMPLE_INDEX_11, SAMPLE_INDEX_12 };
  const int CURRENT_REPOSITION_INDEX = SAMPLE_INDEX_1;

  mutable SampleIndex best_sample_index_ = CURRENT_LOCATION_INDEX;

};

}  // namespace systems
}  // namespace dairlib
