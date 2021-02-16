#pragma once

#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/solver_interface.h"
#include "drake/solvers/solver_options.h"
#include "drake/systems/framework/leaf_system.h"

#include "dairlib/lcmt_trajectory_block.hpp"
#include "dairlib/lcmt_saved_traj.hpp"
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
  double w_rom_reg;

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
      const drake::multibody::MultibodyPlant<double>& plant_controls,
      double stride_period, const PlannerSetting& param, bool debug_mode);

  const drake::systems::InputPort<double>& get_input_port_stance_foot() const {
    return this->get_input_port(stance_foot_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_init_phase() const {
    return this->get_input_port(phase_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm_and_lo_time()
      const {
    return this->get_input_port(fsm_and_lo_time_port_);
  }

 private:
  void SolveTrajOpt(const drake::systems::Context<double>& context,
                    dairlib::lcmt_saved_traj* traj_msg) const;

  // Port indices
  int stance_foot_port_;
  int phase_port_;
  int state_port_;
  int fsm_and_lo_time_port_;

  // Map position/velocity from model with spring to without spring
  Eigen::MatrixXd map_position_from_spring_to_no_spring_;
  Eigen::MatrixXd map_velocity_from_spring_to_no_spring_;

  std::map<std::string, int> positions_map_;
  std::map<std::string, int> velocities_map_;

  int nq_;
  int nv_;
  int nx_;

  const drake::multibody::MultibodyPlant<double>& plant_controls_;
  double stride_period_;

  std::unique_ptr<ReducedOrderModel> rom_;
  StateMirror state_mirror_;
  std::vector<BodyPoint> left_contacts_;
  std::vector<BodyPoint> right_contacts_;
  std::vector<std::tuple<std::string, double, double>> joint_name_lb_ub_;
  drake::solvers::SolverOptions solver_option_;
  std::unique_ptr<drake::solvers::SolverInterface> solver_;

  // Parameters for traj opt
  const PlannerSetting& param_;

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

  // Since sometimes the planner replan every 1ms in the beginning of the
  // simulation (e.g. at 0, 1, 2 ms), we use min_time_difference_for_replanning_
  // to avoid replanning when the current time is too close to the previous
  // time. This method is just a workaround. The best solution should be
  // avoiding using old message in the lcm subscriber. However, somehow there is
  // still old message in the lcm subscriber after I clear the listening
  // channel. (it's gone and then come back)
  double min_time_difference_for_replanning_ = 0.01;
  mutable double timestamp_of_previous_plan_ = -1;
  mutable dairlib::lcmt_saved_traj previous_output_msg_;


  // Testing
  mutable int counter_ = 0;
};

}  // namespace goldilocks_models
}  // namespace dairlib
