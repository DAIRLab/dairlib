#pragma once

#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/solver_interface.h"
#include "drake/solvers/solver_options.h"
#include "drake/systems/framework/leaf_system.h"

#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_trajectory_block.hpp"
#include "examples/goldilocks_models/goldilocks_utils.h"
#include "examples/goldilocks_models/planning/rom_traj_opt.h"
#include "examples/goldilocks_models/reduced_order_models.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

using std::cout;
using std::endl;

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
  bool use_double_contact_points;

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
  // Regularization cost on FOM state
  double w_reg_quat_;
  double w_reg_xy_;
  double w_reg_z_joints_;

  // Files parameters
  std::string dir_model;  // location of the model files
  std::string dir_data;   // location to store the opt result
  std::string init_file;

  // Testing
  int solve_idx_for_read_from_file;

  void PrintAll() const {
    cout << "rom_option" << rom_option << endl;
    cout << "iter" << iter << endl;
    cout << "sample" << sample << endl;
    cout << "n_step" << n_step << endl;
    cout << "knots_per_mode" << knots_per_mode << endl;
    cout << "final_position_x" << final_position_x << endl;
    cout << "zero_touchdown_impact" << zero_touchdown_impact << endl;
    cout << "use_double_contact_points" << use_double_contact_points << endl;
    cout << "equalize_timestep_size" << equalize_timestep_size << endl;
    cout << "fix_duration" << fix_duration << endl;
    cout << "feas_tol" << feas_tol << endl;
    cout << "opt_tol" << opt_tol << endl;
    cout << "max_iter" << max_iter << endl;
    cout << "use_ipopt" << use_ipopt << endl;
    cout << "log_solver_info" << log_solver_info << endl;
    cout << "time_limit" << time_limit << endl;
    cout << "w_Q" << w_Q << endl;
    cout << "w_R" << w_R << endl;
    cout << "w_rom_reg" << w_rom_reg << endl;
    cout << "w_reg_quat_" << w_reg_quat_ << endl;
    cout << "w_reg_xy_" << w_reg_xy_ << endl;
    cout << "w_reg_z_joints_" << w_reg_z_joints_ << endl;
    cout << "dir_model" << dir_model << endl;
    cout << "dir_data" << dir_data << endl;
    cout << "init_file" << init_file << endl;
    cout << "solve_idx_for_read_from_file" << solve_idx_for_read_from_file
         << endl;
  }
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

  void PrintAllCostsAndConstraints(const RomTrajOptCassie& trajopt) const;
  void WarmStartGuess(
      const std::vector<Eigen::VectorXd>& des_xy_pos, int global_fsm_idx,
      int first_mode_knot_idx,
      dairlib::goldilocks_models::RomTrajOptCassie* trajopt) const;

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
  drake::solvers::SolverOptions solver_option_ipopt_;
  drake::solvers::SolverOptions solver_option_snopt_;
  std::unique_ptr<drake::solvers::SolverInterface> solver_ipopt_;
  std::unique_ptr<drake::solvers::SolverInterface> solver_snopt_;

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

  // For warm start with previous solution
  mutable int prev_global_fsm_idx_ = -1;
  mutable int prev_first_mode_knot_idx_ = -1;
  mutable std::vector<int> prev_mode_start_;
  // Previous solutions
  mutable std::vector<Eigen::VectorXd> time_breaks_;
  mutable std::vector<Eigen::MatrixXd> state_samples_;
  mutable Eigen::VectorXd h_solutions_;
  mutable Eigen::MatrixXd state_at_knots_;
  mutable Eigen::MatrixXd input_at_knots_;
  mutable Eigen::MatrixXd FOM_x0_;
  mutable Eigen::MatrixXd FOM_xf_;
  mutable Eigen::MatrixXd FOM_Lambda_;

  // For debugging
  bool debug_mode_;
  void PrintStatus(const std::string& msg) const {
    if (debug_mode_) std::cout << msg << std::endl;
  };

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

  // flags
  bool use_standing_pose_as_init_FOM_guess_ = true;
  // Although warm start helps most of the time, it could also make the solver
  // not able to find the optimal solution from time to time
  bool warm_start_with_previous_solution_ = true;

  // Init state relaxation (relax the mapping function)
  std::set<int> relax_index_ = {5};  //{3, 4, 5};
};

}  // namespace goldilocks_models
}  // namespace dairlib
