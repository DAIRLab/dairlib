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

  // Current sample output ports
  const drake::systems::OutputPort<double>& get_output_port_c3_solution_curr()
      const {
    return this->get_output_port(c3_solution_curr_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_c3_intermediates_curr()
      const {
    return this->get_output_port(c3_intermediates_curr_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_lcs_contact_jacobian_curr() 
      const {
    return this->get_output_port(lcs_contact_jacobian_curr_port_);
  }

  // Best sample output ports
  const drake::systems::OutputPort<double>& get_output_port_c3_solution_best()
      const {
    return this->get_output_port(c3_solution_best_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_c3_intermediates_best()
      const {
    return this->get_output_port(c3_intermediates_best_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_lcs_contact_jacobian_best() 
      const {
    return this->get_output_port(lcs_contact_jacobian_best_port_);
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

  void SetOsqpSolverOptions(const drake::solvers::SolverOptions& options) {
    solver_options_ = options;
    c3_->SetOsqpSolverOptions(solver_options_);
  }

 private:
  solvers::LCS CreatePlaceholderLCS() const;

  drake::systems::EventStatus ComputePlan(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void UpdateContext(Eigen::VectorXd lcs_state);

  void OutputC3SolutionCurr(
    const drake::systems::Context<double>& context,
    C3Output::C3Solution* c3_solution) const;

  void OutputC3IntermediatesCurr(
    const drake::systems::Context<double>& context,
    C3Output::C3Intermediates* c3_intermediates) const;

  void OutputLCSContactJacobianCurr(
    const drake::systems::Context<double>& context,
    std::pair<Eigen::MatrixXd, std::vector<Eigen::VectorXd>>* lcs_contact_jacobian) const;

  void OutputC3SolutionBest(
    const drake::systems::Context<double>& context,
    C3Output::C3Solution* c3_solution) const;

  void OutputC3IntermediatesBest(
    const drake::systems::Context<double>& context,
    C3Output::C3Intermediates* c3_intermediates) const;

  void OutputLCSContactJacobianBest(
    const drake::systems::Context<double>& context,
    std::pair<Eigen::MatrixXd, std::vector<Eigen::VectorXd>>* lcs_contact_jacobian) const;

  void OutputAllSampleLocations(
    const drake::systems::Context<double>& context,
    std::vector<Eigen::Vector3d>* all_sample_locations) const;

  void OutputAllSampleCosts(
    const drake::systems::Context<double>& context,
    std::vector<double>* all_sample_costs) const;

  drake::systems::InputPortIndex radio_port_;
  drake::systems::InputPortIndex target_input_port_;
  drake::systems::InputPortIndex lcs_state_input_port_;
  // Current sample output port indices
  drake::systems::OutputPortIndex c3_solution_curr_port_;
  drake::systems::OutputPortIndex c3_intermediates_curr_port_;
  drake::systems::OutputPortIndex lcs_contact_jacobian_curr_port_;
  // Best sample output port indices
  drake::systems::OutputPortIndex c3_solution_best_port_;
  drake::systems::OutputPortIndex c3_intermediates_best_port_;
  drake::systems::OutputPortIndex lcs_contact_jacobian_best_port_;
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


  mutable double filtered_solve_time_;

  // C3 solutions for current location.
  // TODO: to be updated in ComputePlan
  mutable std::unique_ptr<solvers::C3> c3_curr_;
  mutable std::vector<Eigen::VectorXd> delta_;
  mutable std::vector<Eigen::VectorXd> w_;
  mutable Eigen::VectorXd x_pred_curr_;
  
  // C3 solutions for best sample location.
  // TODO: to be updated in ComputePlan
  mutable std::unique_ptr<solvers::C3> c3_best_;
  mutable std::vector<Eigen::VectorXd> delta_best_;
  mutable std::vector<Eigen::VectorXd> w_best_;
  mutable Eigen::VectorXd x_pred_best_;

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

};

}  // namespace systems
}  // namespace dairlib
