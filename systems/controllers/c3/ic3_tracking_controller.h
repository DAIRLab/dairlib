#pragma once

#include <string>
#include <vector>

#include <drake/common/yaml/yaml_io.h>

#include "common/find_resource.h"
#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "dairlib/lcmt_lqr_output.hpp"
#include "dairlib/lcmt_radio_out.hpp"
#include "lcm/lcm_trajectory.h"
#include <c3/core/c3.h>
#include <c3/core/c3_options.h>
#include <c3/core/lcs.h>
#include <c3/core/solver_options_io.h>
#include <c3/systems/framework/c3_output.h>
#include <c3/systems/c3_controller_options.h>

#include "examples/iC3/iC3_options.h"

#include "systems/framework/timestamped_vector.h"

#include "drake/systems/framework/leaf_system.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using dairlib::lcmt_radio_out;
using drake::SortedPair;
using drake::geometry::GeometryId;
using std::vector;

namespace dairlib {
namespace systems {

class iC3TrackingController : public drake::systems::LeafSystem<double> {
 public:
  explicit iC3TrackingController(const drake::multibody::MultibodyPlant<double>& plant,
                        c3::systems::C3ControllerOptions controller_options, iC3Options ic3_options,
                        int example_idx, MatrixXd A_x, VectorXd lb_x, VectorXd ub_x, MatrixXd A_u, VectorXd lb_u, VectorXd ub_u, 
                        drake::systems::Context<double>& plant_context, vector<SortedPair<GeometryId>> contact_geoms); // TODO: these two only used for debugging

  const drake::systems::InputPort<double>& get_input_port_target() const {
    return this->get_input_port(target_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_lcs_state() const {
    return this->get_input_port(lcs_state_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_lqr() const {
    return this->get_input_port(lqr_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_ic3_x() const {
    return this->get_input_port(ic3_x_port_);
  }
  
  const drake::systems::InputPort<double>& get_input_port_ic3_u() const {
    return this->get_input_port(ic3_u_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_ic3_lambda() const {
    return this->get_input_port(ic3_lambda_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_lcs() const {
    return this->get_input_port(lcs_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_timestep() const {
    return this->get_input_port(timestep_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_c3_solution()
      const {
    return this->get_output_port(c3_solution_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_tracking_target()
      const {
    return this->get_output_port(tracking_target_port_);
  }

  // void SetOsqpSolverOptions(const drake::solvers::SolverOptions& options) {
  //   solver_options_ = options;
  //   c3_->SetOsqpSolverOptions(solver_options_);
  // }

 private:
  c3::LCS CreatePlaceholderLCS() const;

  drake::systems::EventStatus ComputePlan(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void OutputC3Solution(const drake::systems::Context<double>& context,
                        c3::systems::C3Output::C3Solution* c3_solution) const;

  void OutputTrackingTarget(const drake::systems::Context<double>& context,
                        drake::systems::BasicVector<double>* tracking_target) const;

  void UpdateQuaternionCosts(
      const VectorXd& x_curr, const Eigen::VectorXd& x_des) const;
      
  int GetNearestXForValueFunction(
      Eigen::VectorXd x_curr, Eigen::MatrixXd x_hat_slice) const;

  drake::systems::InputPortIndex target_input_port_;
  drake::systems::InputPortIndex lcs_state_input_port_;
  drake::systems::InputPortIndex lcs_input_port_;
  drake::systems::InputPortIndex lqr_input_port_;
  drake::systems::InputPortIndex ic3_x_port_;
  drake::systems::InputPortIndex ic3_u_port_;
  drake::systems::InputPortIndex ic3_lambda_port_;
  drake::systems::InputPortIndex timestep_port_;
  drake::systems::OutputPortIndex c3_solution_port_;
  drake::systems::OutputPortIndex tracking_target_port_;

  const drake::multibody::MultibodyPlant<double>& plant_;

  std::vector<SortedPair<GeometryId>> contact_geoms_; // TODO: only used for debugging
  drake::systems::Context<double>& plant_context_;

  c3::systems::C3ControllerOptions controller_options_;
  c3::C3Options c3_options_;
  c3::LCSFactoryOptions lcs_factory_options_;

  iC3Options ic3_options_;
  drake::solvers::SolverOptions solver_options_;
  // drake::solvers::SolverOptions solver_options_ =
  //     drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
  //         "solvers/osqp_options_default.yaml")
  //         .GetAsSolverOptions(drake::solvers::OsqpSolver::id());

  // convenience for variable sizes
  int n_q_;
  int n_v_;
  int n_x_;
  int n_lambda_;
  int n_u_;

  int example_idx_;
  mutable MatrixXd A_x_;
  mutable VectorXd lb_x_;
  mutable VectorXd ub_x_;
  mutable MatrixXd A_u_;
  mutable VectorXd lb_u_;
  mutable VectorXd ub_u_;

  mutable std::unique_ptr<c3::C3> c3_;
  mutable VectorXd tracking_target_;
  
  std::vector<int> quaternion_indices_;  // indices for quaternion-valued joints

  double solve_time_filter_constant_;
  drake::systems::DiscreteStateIndex plan_start_time_index_;
  drake::systems::DiscreteStateIndex x_pred_index_;
  drake::systems::DiscreteStateIndex filtered_solve_time_index_;
  mutable std::vector<Eigen::MatrixXd> Q_;
  mutable std::vector<Eigen::MatrixXd> R_;
  mutable std::vector<Eigen::MatrixXd> G_;
  mutable std::vector<Eigen::MatrixXd> U_;

  mutable std::vector<Eigen::MatrixXd> H_;
  mutable std::vector<Eigen::VectorXd> g_;

  mutable Eigen::MatrixXd state_data_;
  mutable Eigen::MatrixXd input_data_;
  mutable Eigen::MatrixXd force_data_;

  int N_; // N for c3 (NOT the same as iC3 horizon)
  double dt_; // dt for c3 (NOT the same as iC3 dt)

  double dt_scaling_; // c3 tracking dt / ic3 dt

};

}  // namespace systems
}  // namespace dairlib
