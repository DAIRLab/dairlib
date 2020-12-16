#pragma once

#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/solver_interface.h"
#include "drake/solvers/solver_options.h"
#include "drake/systems/framework/leaf_system.h"

#include "dairlib/lcmt_trajectory_block.hpp"
#include "examples/goldilocks_models/goldilocks_utils.h"
#include "examples/goldilocks_models/reduced_order_models.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

namespace dairlib {
namespace goldilocks_models {

struct PlannerSetting {
  int rom_option;
  int iter;
  int sample;  // solution to use for initial guess and cost regularization

  int n_step;
  int knots_per_mode;
  double final_position_x;

  bool zero_touchdown_impact;

  bool equalize_timestep_size;
  bool fix_duration;

  double feas_tol;
  double opt_tol;
  int max_iter;

  bool use_ipopt;
  bool log_solver_info;
  double time_limit;

  // Cost weight
  double w_Q;
  double w_R;

  // Files parameters
  std::string dir_model;  // location of the model files
  std::string dir_data;   // location to store the opt result
  std::string init_file;
};

/// Assumption:
/// - we assume that there is no x, y and yaw dependency in the ROM (the mapping
/// function), because we rotate the robot state at touchdown to the origin in
/// MPC

class CassiePlannerWithMixedRomFom : public drake::systems::LeafSystem<double> {
 public:
  static const int ROBOT = 1;  // robot index for Cassie

  CassiePlannerWithMixedRomFom(
      const drake::multibody::MultibodyPlant<double>& plant_feedback,
      const drake::multibody::MultibodyPlant<double>& plant_controls,
      const std::vector<int>& left_right_support_fsm_states,
      double stride_period, const PlannerSetting& param, bool debug_mode);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm_and_lo_time()
      const {
    return this->get_input_port(fsm_and_lo_time_port_);
  }

 private:
  void SolveTrajOpt(const drake::systems::Context<double>& context,
                    dairlib::lcmt_trajectory_block* traj_msg) const;

  // Port indices
  int state_port_;
  int fsm_and_lo_time_port_;

  // Map position/velocity from model with spring to without spring
  Eigen::MatrixXd map_position_from_spring_to_no_spring_;
  Eigen::MatrixXd map_velocity_from_spring_to_no_spring_;

  std::map<std::string, int> positions_map_;

  const drake::multibody::MultibodyPlant<double>& plant_controls_;
  std::vector<int> left_right_support_fsm_states_;
  double stride_period_;

  std::unique_ptr<ReducedOrderModel> rom_;
  StateMirror state_mirror_;
  std::vector<BodyPoint> left_contacts_;
  std::vector<BodyPoint> right_contacts_;
  std::vector<std::tuple<std::string, double, double>> joint_name_lb_ub_;
  drake::solvers::SolverOptions solver_option_;
  std::unique_ptr<drake::solvers::SolverInterface> solver_;

  //
  mutable bool start_with_left_stance_ = true;

  // Parameters for traj opt
  PlannerSetting param_;

  // Cost weight
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;

  // Initial guesses
  Eigen::VectorXd h_guess_;
  Eigen::MatrixXd r_guess_;
  Eigen::MatrixXd dr_guess_;
  Eigen::MatrixXd tau_guess_;
  Eigen::VectorXd x_guess_left_in_front_;
  Eigen::VectorXd x_guess_right_in_front_;

  // For debugging
  bool debug_mode_;
};

}  // namespace goldilocks_models
}  // namespace dairlib
