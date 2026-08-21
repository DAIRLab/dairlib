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
#include <c3/core/lcs.h>
#include <c3/systems/framework/c3_output.h>
#include <c3/systems/c3_controller_options.h>
#include <c3/multibody/lcs_factory.h>
#include <drake/common/find_resource.h>

#include "examples/iC3/iC3_options.h"
#include "examples/iC3/hybrid_mpc_options.h"

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/osqp_solver.h"

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

class iC3HybridMpcTrackingController : public drake::systems::LeafSystem<double> {
 public:
  explicit iC3HybridMpcTrackingController(const drake::multibody::MultibodyPlant<double>& plant, 
          const c3::multibody::LCSFactory lcs_factory, HybridMpcOptions mpc_options, drake::solvers::SolverOptions solver_options,
          iC3Options ic3_options, int example_idx, MatrixXd A_x, VectorXd lb_x, VectorXd ub_x, MatrixXd A_u, VectorXd lb_u, VectorXd ub_u); 
  const drake::systems::InputPort<double>& get_input_port_lcs_state() const {
    return this->get_input_port(lcs_state_input_port_);
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

  const drake::systems::InputPort<double>& get_input_port_timestep() const {
    return this->get_input_port(timestep_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_nominal_position() const {
    return this->get_input_port(nominal_position_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_solution() const {
    return this->get_output_port(solution_port_);
  }



 private:
  c3::LCS CreatePlaceholderLCS() const;

  drake::systems::EventStatus ComputePlan(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void OutputSolution(const drake::systems::Context<double>& context,
                        c3::systems::C3Output::C3Solution* c3_solution) const;

  void UpdateQuaternionCosts(
      const VectorXd& x_curr, const Eigen::VectorXd& x_des) const;

  c3::LCS MakeTimeVaryingLCS(MatrixXd x_hat, MatrixXd u_hat) const;

  c3::LCS GetLCSSegment(int start_idx, int size) const;

  drake::systems::DiscreteStateIndex plan_start_time_index_;
  drake::systems::DiscreteStateIndex filtered_solve_time_index_;

  drake::systems::InputPortIndex lcs_state_input_port_;
  drake::systems::InputPortIndex ic3_x_port_;
  drake::systems::InputPortIndex ic3_u_port_;
  drake::systems::InputPortIndex ic3_lambda_port_;
  drake::systems::InputPortIndex timestep_port_;
  drake::systems::InputPortIndex nominal_position_port_;

  drake::systems::OutputPortIndex solution_port_;

  drake::solvers::MathematicalProgram prog_;
  drake::solvers::OsqpSolver osqp_;
  std::vector<drake::solvers::VectorXDecisionVariable> x_;
  std::vector<drake::solvers::VectorXDecisionVariable> u_;
  std::vector<drake::solvers::VectorXDecisionVariable> lambda_;
  std::vector<drake::solvers::VectorXDecisionVariable> epsilon_;

  std::vector<drake::solvers::QuadraticCost*> target_costs_;
  std::vector<drake::solvers::QuadraticCost*> input_costs_;
  std::vector<drake::solvers::QuadraticCost*> force_costs_;
  std::vector<drake::solvers::QuadraticCost*> slack_costs_;

  drake::solvers::LinearEqualityConstraint* initial_state_constraint_;
  std::vector<drake::solvers::LinearEqualityConstraint*> dynamics_constraints_;
  std::vector<drake::solvers::LinearConstraint*> lambda_constraints_;
  std::vector<drake::solvers::LinearConstraint*> eta_constraints_;

  mutable MatrixXd state_data_;
  mutable MatrixXd input_data_;
  mutable MatrixXd force_data_;

  const drake::multibody::MultibodyPlant<double>& plant_;
  mutable c3::multibody::LCSFactory lcs_factory_;

  mutable c3::LCS lcs_;

  HybridMpcOptions mpc_options_;
  iC3Options ic3_options_;
  drake::solvers::SolverOptions solver_options_;

  // convenience for variable sizes
  int n_q_;
  int n_v_;
  int n_x_;
  int n_lambda_;
  int n_u_;

  mutable MatrixXd Q_;
  MatrixXd R_;
  MatrixXd S_;
  MatrixXd G_;

  MatrixXd A_x_;
  VectorXd lb_x_;
  VectorXd ub_x_;
  MatrixXd A_u_;
  VectorXd lb_u_;
  VectorXd ub_u_;

  VectorXd lambda_threshold_;
  VectorXd eta_threshold_;

  int example_idx_;

  mutable drake::math::RigidTransform<double> X_delta_;

  mutable vector<VectorXd> x_sol_;
  mutable vector<VectorXd> u_sol_;
  mutable vector<VectorXd> lambda_sol_;
  mutable double solve_time_;

  int N_; // N for mpc (NOT the same as iC3 horizon)
  double dt_;
};

}  // namespace systems
}  // namespace dairlib
