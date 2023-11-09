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
class C3Controller : public drake::systems::LeafSystem<double> {
 public:
  explicit C3Controller(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context,
      const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
      drake::systems::Context<drake::AutoDiffXd>* context_ad,
      const std::vector<drake::SortedPair<drake::geometry::GeometryId>>&
          contact_geoms,
      C3Options c3_options);

  const drake::systems::InputPort<double>& get_input_port_target() const {
    return this->get_input_port(target_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(lcs_state_input_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_c3_solution()
      const {
    return this->get_output_port(c3_solution_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_c3_intermediates()
      const {
    return this->get_output_port(c3_intermediates_port_);
  }

//  const drake::systems::InputPort<double>& get_input_port_radio() const {
//    return this->get_input_port(radio_port_);
//  }

  void SetOsqpSolverOptions(const drake::solvers::SolverOptions& options) {
    solver_options_ = options;
  }

  void SetPublishEndEffectorOrientation(bool publish_end_effector_orientation) {
    publish_end_effector_orientation_ = publish_end_effector_orientation;
  }

 private:
  drake::systems::EventStatus ComputePlan(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void OutputC3Solution(const drake::systems::Context<double>& context,
                        C3Output::C3Solution* c3_solution) const;

  void OutputC3Intermediates(const drake::systems::Context<double>& context,
                             C3Output::C3Intermediates* c3_intermediates) const;

  drake::systems::InputPortIndex target_input_port_;
  drake::systems::InputPortIndex lcs_state_input_port_;
//  drake::systems::InputPortIndex radio_port_;
  drake::systems::OutputPortIndex c3_solution_port_;
  drake::systems::OutputPortIndex c3_intermediates_port_;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad_;
  drake::systems::Context<drake::AutoDiffXd>* context_ad_;
  const std::vector<drake::SortedPair<drake::geometry::GeometryId>>&
      contact_pairs_;
  C3Options c3_options_;
  drake::solvers::SolverOptions solver_options_ =
      drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
          FindResourceOrThrow("solvers/osqp_options_default.yaml"))
          .GetAsSolverOptions(drake::solvers::OsqpSolver::id());

  bool publish_end_effector_orientation_ = false;

  // convenience for variable sizes
  int n_q_;
  int n_v_;
  int n_x_;
  int n_lambda_;
  int n_u_;

  mutable std::unique_ptr<solvers::C3> c3_;
  mutable std::vector<Eigen::VectorXd> delta_;
  mutable std::vector<Eigen::VectorXd> w_;
  //  std::unique_ptr<solvers::LCS> lcs_;
  drake::systems::DiscreteStateIndex plan_start_time_index_;
  std::vector<Eigen::MatrixXd> Q_;
  std::vector<Eigen::MatrixXd> R_;
  std::vector<Eigen::MatrixXd> G_;
  std::vector<Eigen::MatrixXd> U_;
  int N_;
};

}  // namespace systems
}  // namespace dairlib
