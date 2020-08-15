#pragma once

#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/leaf_system.h"

#include "dairlib/lcmt_trajectory_block.hpp"
#include "examples/goldilocks_models/goldilocks_utils.h"
#include "examples/goldilocks_models/reduced_order_models.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

namespace dairlib {
namespace goldilocks_models {

/// This system is basically solving the same problem as
/// plan_with_rom_fom_cassie.cc

/// Assumption:
/// - we assume that there is no x, y and yaw dependency in the ROM (the mapping
/// function), because we rotate the robot state at touchdown to the origin in
/// MPC

class OptimalRomPlanner : public drake::systems::LeafSystem<double> {
 public:
  OptimalRomPlanner(
      const drake::multibody::MultibodyPlant<double>& plant_feedback,
      const drake::multibody::MultibodyPlant<double>& plant_controls,
      const std::vector<int>& unordered_fsm_states, double stride_period);

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
  std::vector<int> single_support_fsm_states_;
  double stride_period_;

  std::unique_ptr<ReducedOrderModel> rom_;
  StateMirror state_mirror_;
  std::vector<BodyPoint> left_contacts_;
  std::vector<BodyPoint> right_contacts_;
  std::vector<std::tuple<std::string, double, double>> joint_name_lb_ub_;

  // Parameters for traj opt
  int FLAGS_rom_option;
  int FLAGS_iter;

  int FLAGS_n_step;
  int FLAGS_knots_per_mode;
  double FLAGS_final_position_x;

  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;

  bool zero_touchdown_impact_;

  bool FLAGS_equalize_timestep_size;
  bool FLAGS_fix_duration;

  double FLAGS_feas_tol;
  double FLAGS_opt_tol;

  bool FLAGS_use_ipopt;
  bool FLAGS_log_solver_info;

  // Initial guesses
  Eigen::VectorXd h_guess_;
  Eigen::MatrixXd r_guess_;
  Eigen::MatrixXd dr_guess_;
  Eigen::MatrixXd tau_guess_;
  Eigen::VectorXd x_guess_left_in_front_;
  Eigen::VectorXd x_guess_right_in_front_;
};

}  // namespace goldilocks_models
}  // namespace dairlib
