#pragma once

#include <memory>
#include <string>
#include <vector>

#include <drake/common/yaml/yaml_io.h>

#include "common/find_resource.h"
#include "dairlib/lcmt_c3_costs.hpp"
#include "dairlib/lcmt_c3_forces.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "examples/deform/parameter_headers/elastoplastic_c3_options.h"
#include "lcm/lcm_trajectory.h"
#include "solvers/base_c3.h"
#include "solvers/c3_output.h"
#include "solvers/c3_plus.h"
#include "solvers/lcs.h"
#include "solvers/lcs_factory.h"
#include "solvers/solver_options_io.h"
#include "systems/framework/timestamped_vector.h"

#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace examples {
namespace deform {

class ElastoPlasticController : public drake::systems::LeafSystem<double> {
 public:
  ElastoPlasticController(
      drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context,
      drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
      drake::systems::Context<drake::AutoDiffXd>* context_ad,
      const std::vector<
          std::vector<drake::SortedPair<drake::geometry::GeometryId>>>&
          contact_geoms,
      const ElastoPlasticC3Options& elastoplastic_c3_options);

  // Input ports
  const drake::systems::InputPort<double>& get_input_port_lcs_state() const {
    return this->get_input_port(x_lcs_input_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_target() const {
    return this->get_input_port(x_lcs_target_input_port_);
  }

  // Output ports
  const drake::systems::OutputPort<double>& get_output_port_c3_solution()
      const {
    return this->get_output_port(c3_solution_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_c3_intermediates()
      const {
    return this->get_output_port(c3_intermediates_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_c3_forces() const {
    return this->get_output_port(c3_forces_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_efforts() const {
    return this->get_output_port(efforts_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_c3_costs() const {
    return this->get_output_port(c3_costs_port_);
  }

 private:
  /// Function for computing one control loop
  drake::systems::EventStatus ComputePlan(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  /// Output port function
  void OutputC3Solution(const drake::systems::Context<double>& context,
                        C3Output::C3Solution* c3_solution) const;
  void OutputC3Intermediates(const drake::systems::Context<double>& context,
                             C3Output::C3Intermediates* c3_intermediates) const;
  void OutputC3Forces(const drake::systems::Context<double>& context,
                      dairlib::lcmt_c3_forces* output) const;
  void OutputRobotEfforts(const drake::systems::Context<double>& context,
                          dairlib::lcmt_robot_input* lcmt_efforts) const;
  void OutputC3Costs(const drake::systems::Context<double>& context,
                     dairlib::lcmt_c3_costs* lcmt_c3_costs) const;

  // Input/output port indices
  drake::systems::InputPortIndex x_lcs_input_port_;
  drake::systems::InputPortIndex x_lcs_target_input_port_;
  drake::systems::OutputPortIndex c3_solution_port_;
  drake::systems::OutputPortIndex c3_intermediates_port_;
  drake::systems::OutputPortIndex c3_forces_port_;
  drake::systems::OutputPortIndex efforts_port_;
  drake::systems::OutputPortIndex c3_costs_port_;

  // Plant, options, and controller
  drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad_;
  drake::systems::Context<drake::AutoDiffXd>* context_ad_;
  ElastoPlasticC3Options elastoplastic_c3_options_;
  mutable std::shared_ptr<solvers::C3Plus> c3_mpc_;

  // Contact information
  const std::vector<
      std::vector<drake::SortedPair<drake::geometry::GeometryId>>>&
      contact_pairs_;
  solvers::ContactModel contact_model_;

  // System dimensions
  int n_q_;
  int n_v_;
  int n_u_;
  int n_x_;
  int n_lambda_;
  int N_;
  double dt_;

  // Cost matrices
  mutable std::vector<Eigen::MatrixXd> Q_;
  mutable std::vector<Eigen::MatrixXd> R_;
  mutable std::vector<Eigen::MatrixXd> G_;
  mutable std::vector<Eigen::MatrixXd> U_;

  // Discrete state indices
  drake::systems::DiscreteStateIndex plan_start_time_index_;
  drake::systems::DiscreteStateIndex x_pred_index_;
  drake::systems::DiscreteStateIndex filtered_solve_time_index_;

  // Timing
  const double solve_time_filter_alpha_;
};

}  // namespace deform
}  // namespace examples
}  // namespace dairlib
