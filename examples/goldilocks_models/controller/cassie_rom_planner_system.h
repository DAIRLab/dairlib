#pragma once

#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "dairlib/lcmt_trajectory_block.hpp"
#include "examples/goldilocks_models/controller/osc_rom_walking_gains.h"
#include "examples/goldilocks_models/goldilocks_utils.h"
#include "examples/goldilocks_models/planning/rom_traj_opt.h"
#include "examples/goldilocks_models/reduced_order_models.h"
#include "lcm/rom_planner_saved_trajectory.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/solver_interface.h"
#include "drake/solvers/solver_options.h"
#include "drake/systems/framework/leaf_system.h"

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
  bool use_ipopt_in_first_loop;
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

  // Testing
  int solve_idx_for_read_from_file;

  OSCRomWalkingGains gains;

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
    cout << "w_reg_quat_" << gains.w_reg_quat << endl;
    cout << "w_reg_xy_" << gains.w_reg_xy << endl;
    cout << "w_reg_z_" << gains.w_reg_z << endl;
    cout << "w_reg_joints_" << gains.w_reg_joints << endl;
    cout << "w_reg_hip_yaw_" << gains.w_reg_hip_yaw << endl;
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
      double stride_period, const PlannerSetting& param, bool singel_eval_mode,
      bool log_data);

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
    return this->get_input_port(controller_signal_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_quat_xyz_shift()
      const {
    return this->get_input_port(quat_xyz_shift_port_);
  }

 private:
  void SolveTrajOpt(const drake::systems::Context<double>& context,
                    dairlib::lcmt_timestamped_saved_traj* traj_msg) const;

  void RotateBetweenGlobalAndLocalFrame(bool rotate_from_global_to_local,
                                        const Eigen::VectorXd& quat_xyz_shift,
                                        const Eigen::MatrixXd& original_x0_FOM,
                                        const Eigen::MatrixXd& original_xf_FOM,
                                        Eigen::MatrixXd* rotated_x0_FOM,
                                        Eigen::MatrixXd* rotated_xf_FOM) const;
  void BookKeeping(
      bool start_with_left_stance, const std::chrono::duration<double>& elapsed,
      const drake::solvers::MathematicalProgramResult& result) const;
  void WarmStartGuess(
      const Eigen::VectorXd& quat_xyz_shift,
      const std::vector<Eigen::VectorXd>& des_xy_pos, int global_fsm_idx,
      int first_mode_knot_idx,
      dairlib::goldilocks_models::RomTrajOptCassie* trajopt) const;

  // Port indices
  int stance_foot_port_;
  int phase_port_;
  int state_port_;
  int controller_signal_port_;
  int quat_xyz_shift_port_;

  // Map position/velocity from model with spring to without spring
  Eigen::MatrixXd map_position_from_spring_to_no_spring_;
  Eigen::MatrixXd map_velocity_from_spring_to_no_spring_;

  std::map<std::string, int> positions_map_;
  std::map<std::string, int> velocities_map_;

  int nq_;
  int nv_;
  int nx_;
  int n_tau_;

  const drake::multibody::MultibodyPlant<double>& plant_controls_;
  double stride_period_;
  double single_support_duration_;
  double double_support_duration_;

  std::unique_ptr<ReducedOrderModel> rom_;
  StateMirror state_mirror_;
  std::vector<BodyPoint> left_contacts_;
  std::vector<BodyPoint> right_contacts_;
  BodyPoint left_origin_;
  BodyPoint right_origin_;
  std::vector<std::tuple<std::string, double, double>> joint_name_lb_ub_;
  mutable drake::solvers::SolverOptions solver_option_ipopt_;
  mutable drake::solvers::SolverOptions solver_option_snopt_;
  std::unique_ptr<drake::solvers::SolverInterface> solver_ipopt_;
  std::unique_ptr<drake::solvers::SolverInterface> solver_snopt_;

  // Parameters for traj opt
  //  const PlannerSetting& param_;
  mutable PlannerSetting param_;

  // Cost weight
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;

  // Initial guesses
  Eigen::VectorXd h_guess_;
  Eigen::MatrixXd y_guess_;
  Eigen::MatrixXd dy_guess_;
  Eigen::MatrixXd tau_guess_;
  Eigen::VectorXd x_guess_left_in_front_pre_;
  Eigen::VectorXd x_guess_right_in_front_pre_;
  Eigen::VectorXd x_guess_left_in_front_post_;
  Eigen::VectorXd x_guess_right_in_front_post_;

  // For warm start with previous solution
  mutable int prev_global_fsm_idx_ = -1;
  mutable int prev_first_mode_knot_idx_ = -1;
  mutable std::vector<int> prev_mode_start_;
  // Previous solutions
  mutable RomPlannerTrajectory lightweight_saved_traj_;
  mutable Eigen::VectorXd h_solutions_;
  mutable Eigen::MatrixXd input_at_knots_;
  // TODO: we will need to store global not local for better warm-start. Same
  //  reason as x0_FOM_
  mutable Eigen::MatrixXd local_Lambda_FOM_;
  mutable Eigen::MatrixXd global_x0_FOM_;
  mutable Eigen::MatrixXd global_xf_FOM_;
  mutable Eigen::VectorXd touchdown_foot_pos_;

  // Init state relaxation (relax the mapping function)
  std::set<int> relax_index_ = {5};  //{3, 4, 5};

  // Time limit
  bool fixed_time_limit_;
  double time_limit_for_first_loop_ = 60;
  double buffer_ = 0.02;  // time for lcm packing/traveling, data saving, etc
  // We don't want the solver to use all the remaining time. Otherwise, there
  // won't be enough time in the next loop.
  // I think min_solve_time_preserved_for_next_loop_ could be set to half of
  // the potentailly-shortest time horizon of the planner (reserve time
  // equally for the current and the next loop)
  double min_solve_time_preserved_for_next_loop_;

  // Swing foot distance
  mutable std::vector<double> max_swing_distance_;

  // Flags
  bool use_standing_pose_as_init_FOM_guess_ = true;
  // Although warm start helps most of the time, it could also make the solver
  // not able to find the optimal solution from time to time
  bool warm_start_with_previous_solution_ = true;

  // For debugging and data logging
  mutable int counter_ = 0;

  mutable double total_solve_time_ = 0;
  mutable double max_solve_time_ = 0;

  mutable bool past_is_left_stance_ = true;
  mutable int total_number_of_first_solve_of_the_mode_ = 0;
  mutable double total_solve_time_of_first_solve_of_the_mode_ = 0;
  mutable double max_solve_time_of_first_solve_of_the_mode_ = 0;

  mutable int num_failed_solve_ = 0;
  mutable int latest_failed_solve_idx_ = -1;

  bool singel_eval_mode_;
  bool log_data_and_check_solution_;
  void PrintStatus(const std::string& msg) const {
    if (singel_eval_mode_) std::cout << msg << std::endl;
  };
  void SaveTrajIntoLcmBinary(
      const RomTrajOptCassie& trajopt,
      const drake::solvers::MathematicalProgramResult& result,
      const Eigen::MatrixXd& global_x0_FOM,
      const Eigen::MatrixXd& global_xf_FOM, const std::string& dir_data,
      const std::string& prefix) const;
  void SaveDataIntoFiles(
      double current_time, const Eigen::VectorXd& x_init, double init_phase,
      bool is_right_stance, const Eigen::VectorXd& quat_xyz_shift,
      const Eigen::MatrixXd& local_x0_FOM, const Eigen::MatrixXd& local_xf_FOM,
      const RomTrajOptCassie& trajopt,
      const drake::solvers::MathematicalProgramResult& result,
      const std::string& dir_data, const std::string& prefix,
      const std::string& prefix_next) const;
  void PrintCost(const RomTrajOptCassie& trajopt,
                 const drake::solvers::MathematicalProgramResult& result) const;
  void PrintAllCostsAndConstraints(const RomTrajOptCassie& trajopt) const;

  // Since sometimes the planner replan every 1ms in the beginning of the
  // simulation (e.g. at 0, 1, 2 ms), we use
  // min_time_difference_for_replanning_ to avoid replanning when the current
  // time is too close to the previous time. This method is just a workaround.
  // The best solution should be avoiding using old message in the lcm
  // subscriber. However, somehow there is still old message in the lcm
  // subscriber after I clear the listening channel. (it's gone and then come
  // back) Note that another workaround is to clear the lcm message twice in
  // the LcmDrivenLoop class.
  double min_time_difference_for_replanning_ = 0.01;
  mutable double timestamp_of_previous_plan_ = -1;
  mutable dairlib::lcmt_timestamped_saved_traj previous_output_msg_;
};

}  // namespace goldilocks_models
}  // namespace dairlib
