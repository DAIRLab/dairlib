#include "examples/goldilocks_models/controller/cassie_rom_planner_system.h"

#include <math.h>     /* fmod */
#include <algorithm>  // std::max

#include "common/eigen_utils.h"
#include "examples/goldilocks_models/planning/rom_traj_opt.h"
#include "lcm/rom_planner_saved_trajectory.h"
#include "systems/controllers/osc/osc_utils.h"

#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"

#include <string>

using std::cout;
using std::endl;
using std::string;
using std::to_string;
using std::vector;

using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;

using drake::multibody::Frame;
using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::SolutionResult;
using drake::trajectories::ExponentialPlusPiecewisePolynomial;
using drake::trajectories::PiecewisePolynomial;

using dairlib::systems::OutputVector;

namespace dairlib {
namespace goldilocks_models {

CassiePlannerWithMixedRomFom::CassiePlannerWithMixedRomFom(
    const MultibodyPlant<double>& plant_controls, double stride_period,
    const PlannerSetting& param, bool debug_mode)
    : nq_(plant_controls.num_positions()),
      nv_(plant_controls.num_velocities()),
      nx_(plant_controls.num_positions() + plant_controls.num_velocities()),
      plant_controls_(plant_controls),
      stride_period_(stride_period),
      param_(param),
      debug_mode_(debug_mode) {
  this->set_name("planner_traj");

  DRAKE_DEMAND(param_.knots_per_mode > 0);

  // Input/Output Setup
  stance_foot_port_ =
      this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();
  phase_port_ =
      this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();
  state_port_ = this->DeclareVectorInputPort(
                        OutputVector<double>(plant_controls.num_positions(),
                                             plant_controls.num_velocities(),
                                             plant_controls.num_actuators()))
                    .get_index();
  fsm_and_lo_time_port_ =
      this->DeclareVectorInputPort(BasicVector<double>(2)).get_index();
  this->DeclareAbstractOutputPort(&CassiePlannerWithMixedRomFom::SolveTrajOpt);

  // Create index maps
  positions_map_ = multibody::makeNameToPositionsMap(plant_controls);
  velocities_map_ = multibody::makeNameToVelocitiesMap(plant_controls);

  // Reduced order model
  rom_ = CreateRom(param_.rom_option, ROBOT, plant_controls, false);
  ReadModelParameters(rom_.get(), param_.dir_model, param_.iter);

  // Create mirror maps
  state_mirror_ = StateMirror(MirrorPosIndexMap(plant_controls, ROBOT),
                              MirrorPosSignChangeSet(plant_controls, ROBOT),
                              MirrorVelIndexMap(plant_controls, ROBOT),
                              MirrorVelSignChangeSet(plant_controls, ROBOT));

  // Provide initial guess
  bool with_init_guess = true;
  int n_y = rom_->n_y();
  int n_tau = rom_->n_tau();
  string model_dir_n_pref = param_.dir_model + to_string(param_.iter) +
                            string("_") + to_string(param_.sample) +
                            string("_");
  h_guess_ = VectorXd(param_.knots_per_mode);
  r_guess_ = MatrixXd(n_y, param_.knots_per_mode);
  dr_guess_ = MatrixXd(n_y, param_.knots_per_mode);
  tau_guess_ = MatrixXd(n_tau, param_.knots_per_mode);
  if (with_init_guess) {
    VectorXd h_guess_raw =
        readCSV(model_dir_n_pref + string("time_at_knots.csv")).col(0);
    MatrixXd r_guess_raw =
        readCSV(model_dir_n_pref + string("t_and_y.csv")).bottomRows(n_y);
    MatrixXd dr_guess_raw =
        readCSV(model_dir_n_pref + string("t_and_ydot.csv")).bottomRows(n_y);
    MatrixXd tau_guess_raw =
        readCSV(model_dir_n_pref + string("t_and_tau.csv")).bottomRows(n_tau);
    VectorXd x_guess_left_in_front_raw =
        readCSV(model_dir_n_pref + string("state_at_knots.csv")).col(0);
    VectorXd x_guess_right_in_front_raw =
        readCSV(model_dir_n_pref + string("state_at_knots.csv")).rightCols(1);
    cout << "\nWARNING: last column of state_at_knots.csv should be pre-impact "
            "state.\n";
    // TODO: store both pre and post impact in rom optimization

    // TODO: reconstruct cubic spline and resample
    double duration = h_guess_raw.tail(1)(0);
    for (int i = 0; i < param_.knots_per_mode; i++) {
      h_guess_(i) = duration / (param_.knots_per_mode - 1) * i;
    }
    for (int i = 0; i < param_.knots_per_mode; i++) {
      int n_mat_col_r = r_guess_raw.cols();
      int idx_r = (int)round(double(i * (n_mat_col_r - 1)) /
                             (param_.knots_per_mode - 1));
      r_guess_.col(i) = r_guess_raw.col(idx_r);
      int n_mat_col_dr = dr_guess_raw.cols();
      int idx_dr = (int)round(double(i * (n_mat_col_dr - 1)) /
                              (param_.knots_per_mode - 1));
      dr_guess_.col(i) = dr_guess_raw.col(idx_dr);
      int n_mat_col_tau = tau_guess_raw.cols();
      int idx_tau = (int)round(double(i * (n_mat_col_tau - 1)) /
                               (param_.knots_per_mode - 1));
      tau_guess_.col(i) = tau_guess_raw.col(idx_tau);
    }
    x_guess_left_in_front_ = x_guess_left_in_front_raw;
    x_guess_right_in_front_ = x_guess_right_in_front_raw;

    cout << "initial guess duration ~ " << duration << endl;
    // cout << "h_guess = " << h_guess << endl;
    // cout << "r_guess = " << r_guess << endl;
    // cout << "dr_guess = " << dr_guess << endl;
    // cout << "tau_guess = " << tau_guess << endl;
    // cout << "x_guess_left_in_front = " << x_guess_left_in_front << endl;
    // cout << "x_guess_right_in_front = " << x_guess_right_in_front << endl;
  }

  // Get foot contacts
  auto left_toe = LeftToeFront(plant_controls);
  auto left_heel = LeftToeRear(plant_controls);
  // auto right_toe = RightToeFront(plant_controls);
  // auto right_heel = RightToeRear(plant_controls);
  Vector3d mid_contact_point = (left_toe.first + left_heel.first) / 2;
  auto left_toe_mid =
      BodyPoint(mid_contact_point, plant_controls.GetFrameByName("toe_left"));
  auto right_toe_mid =
      BodyPoint(mid_contact_point, plant_controls.GetFrameByName("toe_right"));
  left_contacts_.push_back(left_toe_mid);
  right_contacts_.push_back(right_toe_mid);

  // Get joint limits of the robot
  std::vector<string> l_r_pair = {"_left", "_right"};
  std::vector<std::string> joint_names = {
      "hip_roll", "hip_yaw", "hip_pitch", "knee", "ankle_joint", "toe"};
  for (const auto& left_right : l_r_pair) {
    for (const auto& name : joint_names) {
      joint_name_lb_ub_.emplace_back(
          name + left_right,
          plant_controls.GetJointByName(name + left_right)
              .position_lower_limits()(0),
          plant_controls.GetJointByName(name + left_right)
              .position_upper_limits()(0));
    }
  }

  // Cost weight
  Q_ = param_.w_Q * MatrixXd::Identity(n_y, n_y);
  R_ = param_.w_R * MatrixXd::Identity(n_tau, n_tau);

  // Pick solver
  drake::solvers::SolverId solver_id("");
  if (param_.use_ipopt) {
    solver_id = drake::solvers::IpoptSolver().id();
  } else {
    solver_id = drake::solvers::SnoptSolver().id();
  }
  cout << "Solver: " << solver_id.name() << endl;
  solver_ = drake::solvers::MakeSolver(solver_id);

  // Set solver option
  if (param_.use_ipopt) {
    // Ipopt settings adapted from CaSaDi and FROST
    auto id = drake::solvers::IpoptSolver::id();
    solver_option_.SetOption(id, "tol", param_.feas_tol);
    solver_option_.SetOption(id, "dual_inf_tol", param_.feas_tol);
    solver_option_.SetOption(id, "constr_viol_tol", param_.feas_tol);
    solver_option_.SetOption(id, "compl_inf_tol", param_.feas_tol);
    solver_option_.SetOption(id, "max_iter", param_.max_iter);
    solver_option_.SetOption(id, "nlp_lower_bound_inf", -1e6);
    solver_option_.SetOption(id, "nlp_upper_bound_inf", 1e6);
    if (param_.log_solver_info) {
      solver_option_.SetOption(id, "print_timing_statistics", "yes");
      solver_option_.SetOption(id, "print_level", 5);
    } else {
      solver_option_.SetOption(id, "print_timing_statistics", "no");
      solver_option_.SetOption(id, "print_level", 0);
    }
    if (param_.time_limit > 0) {
      solver_option_.SetOption(id, "max_cpu_time", param_.time_limit);
    }

    // Set to ignore overall tolerance/dual infeasibility, but terminate when
    // primal feasible and objective fails to increase over 5 iterations.
    solver_option_.SetOption(id, "acceptable_compl_inf_tol", param_.feas_tol);
    solver_option_.SetOption(id, "acceptable_constr_viol_tol", param_.feas_tol);
    solver_option_.SetOption(id, "acceptable_obj_change_tol", 1e-3);
    solver_option_.SetOption(id, "acceptable_tol", 1e2);
    solver_option_.SetOption(id, "acceptable_iter", 5);
  } else {
    if (param_.log_solver_info) {
      solver_option_.SetOption(drake::solvers::SnoptSolver::id(), "Print file",
                               "../snopt_planning.out");
      cout << "Note that you are logging snopt result.\n";
    }
    if (param_.time_limit > 0) {
      solver_option_.SetOption(drake::solvers::SnoptSolver::id(), "Time limit",
                               param_.time_limit);
      solver_option_.SetOption(drake::solvers::SnoptSolver::id(),
                               "Timing level", 3);
    }
    solver_option_.SetOption(drake::solvers::SnoptSolver::id(),
                             "Major iterations limit", param_.max_iter);
    solver_option_.SetOption(drake::solvers::SnoptSolver::id(), "Verify level",
                             0);
    solver_option_.SetOption(drake::solvers::SnoptSolver::id(),
                             "Major optimality tolerance", param_.opt_tol);
    solver_option_.SetOption(drake::solvers::SnoptSolver::id(),
                             "Major feasibility tolerance", param_.feas_tol);
  }
}

void CassiePlannerWithMixedRomFom::SolveTrajOpt(
    const Context<double>& context, dairlib::lcmt_saved_traj* traj_msg) const {
  ///
  /// Read from input ports
  ///

  // Read in current robot state
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  VectorXd x_init = robot_output->GetState();

  // Get phase in the first mode
  const BasicVector<double>* phase_port =
      this->EvalVectorInput(context, phase_port_);
  double init_phase = phase_port->get_value()(0);

  // Get stance foot
  bool is_right_stance =
      (bool)this->EvalVectorInput(context, stance_foot_port_)->get_value()(0);
  bool start_with_left_stance = !is_right_stance;

  // Get current time
  double timestamp = robot_output->get_timestamp();
  auto current_time = static_cast<double>(timestamp);

  // Get lift-off time
  const BasicVector<double>* fsm_and_lo_time_port =
      this->EvalVectorInput(context, fsm_and_lo_time_port_);
  double lift_off_time = fsm_and_lo_time_port->get_value()(1);

  if (debug_mode_) {
    cout << "x_init used for the planner = " << x_init.transpose() << endl;
  }

  ///
  /// Decide if we need to re-plan (not ideal code. See header file)
  ///

  bool need_to_replan = ((current_time - timestamp_of_previous_plan_) >
                         min_time_difference_for_replanning_);
  if (!need_to_replan) {
    *traj_msg = previous_output_msg_;
    return;
  }

  ///
  /// Construct rom traj opt
  ///

  // Prespecify the number of knot points
  std::vector<int> num_time_samples;
  std::vector<double> min_dt;
  std::vector<double> max_dt;
  for (int i = 0; i < param_.n_step; i++) {
    num_time_samples.push_back(param_.knots_per_mode);
    min_dt.push_back(.01);
    max_dt.push_back(.3);
  }
  // We use int() to round down the index because we need to have at least one
  // timestep in the first mode, i.e. 2 knot points.
  int first_mode_phase_index = int((param_.knots_per_mode - 1) * init_phase);
  int knots_first_mode = param_.knots_per_mode - first_mode_phase_index;
  num_time_samples[0] = knots_first_mode;
  if (knots_first_mode == 2) {
    min_dt[0] = 1e-3;
  }
  cout << "init_phase = " << init_phase << endl;
  cout << "knots_first_mode = " << knots_first_mode << endl;
  cout << "first_mode_phase_index = " << first_mode_phase_index << endl;

  // Goal position
  VectorXd final_position(2);
  final_position << param_.final_position_x, 0;
  /*final_position << x_lift_off(positions_map_.at("base_x")) +
                        param_.final_position_x,
      0;*/

  // Construct
  if (debug_mode_) {
    cout << "\nConstructing optimization problem...\n";
  }
  auto start = std::chrono::high_resolution_clock::now();
  RomTrajOptCassie trajopt(num_time_samples, Q_, R_, *rom_, plant_controls_,
                           state_mirror_, left_contacts_, right_contacts_,
                           joint_name_lb_ub_, x_init, start_with_left_stance,
                           param_.zero_touchdown_impact,
                           debug_mode_ /*print_status*/);

  if (debug_mode_) {
    cout << "Other constraints/costs and initial guess===============\n";
  }
  // Time step constraints
  double first_mode_duration =
      stride_period_ - fmod(current_time, stride_period_);
  double remaining_mode_duration = stride_period_;
  trajopt.AddTimeStepConstraint(min_dt, max_dt, param_.fix_duration,
                                param_.equalize_timestep_size,
                                first_mode_duration, remaining_mode_duration);

  // Constraints for fourbar linkage
  // Note that if the initial pose in the constraint doesn't obey the fourbar
  // linkage relationship. You shouldn't add constraint to the initial pose here
  // TODO: update this part.
  /*double fourbar_angle = 13.0 / 180.0 * M_PI;
  auto q0_var =
  trajopt.x0_vars_by_mode(0).head(nq_);
  trajopt.AddLinearEqualityConstraint(
      q0_var(positions_map.at("knee_left")) +
      q0_var(positions_map.at("ankle_joint_left")), fourbar_angle);
  trajopt.AddLinearEqualityConstraint(
      q0_var(positions_map.at("knee_right")) +
      q0_var(positions_map.at("ankle_joint_right")), fourbar_angle);*/
  /*for (int i = 0; i < num_time_samples.size(); i++) {
    auto qf_var = trajopt.xf_vars_by_mode(i);
    trajopt.AddLinearEqualityConstraint(
        qf_var(positions_map.at("knee_left")) +
        qf_var(positions_map.at("ankle_joint_left")), fourbar_angle);
    trajopt.AddLinearEqualityConstraint(
        qf_var(positions_map.at("knee_right")) +
        qf_var(positions_map.at("ankle_joint_right")), fourbar_angle);
  }*/

  // Final goal position constraint
  if (debug_mode_) {
    cout << "Adding final position constraint for full-order model...\n";
  }
  trajopt.AddBoundingBoxConstraint(
      final_position, final_position,
      trajopt.xf_vars_by_mode(num_time_samples.size() - 1).segment(4, 2));

  // Add robot state in cost
  bool add_x_pose_in_cost = true;
  if (add_x_pose_in_cost) {
    trajopt.AddRegularizationCost(final_position, x_guess_left_in_front_,
                                  x_guess_right_in_front_,
                                  false /*straight_leg_cost*/);
  } else {
    // Since there are multiple q that could be mapped to the same r, I
    // penalize on q so it get close to a certain configuration
    MatrixXd Id = MatrixXd::Identity(3, 3);
    VectorXd zero_vec = VectorXd::Zero(3);
    for (int i = 0; i < num_time_samples.size(); i++) {
      trajopt.AddQuadraticErrorCost(Id, zero_vec,
                                    trajopt.xf_vars_by_mode(i).segment(1, 3));
    }
  }

  // Add rom state in cost
  bool add_rom_regularization = true;
  if (add_rom_regularization) {
    trajopt.AddRomRegularizationCost(h_guess_, r_guess_, dr_guess_, tau_guess_,
                                     first_mode_phase_index, param_.w_rom_reg);
  }

  // Default initial guess to avoid singularity (which messes with gradient)
  for (int i = 0; i < num_time_samples.size(); i++) {
    for (int j = 0; j < num_time_samples[i]; j++) {
      if ((param_.rom_option == 0) || (param_.rom_option == 1)) {
        trajopt.SetInitialGuess((trajopt.state_vars_by_mode(i, j))(1), 1);
      } else if (param_.rom_option == 4) {
        trajopt.SetInitialGuess((trajopt.state_vars_by_mode(i, j))(2), 1);
      } else {
        DRAKE_UNREACHABLE();
      }
    }
  }

  // Initial guess for all variables
  if (debug_mode_) {
    if (!param_.init_file.empty()) {
      VectorXd z0 = readCSV(param_.dir_data + param_.init_file).col(0);
      int n_dec = trajopt.decision_variables().size();
      if (n_dec > z0.rows()) {
        cout << "dim(initial guess) < dim(decision var). "
                "Fill the rest with zero's.\n";
        VectorXd old_z0 = z0;
        z0.resize(n_dec);
        z0 = VectorXd::Zero(n_dec);
        z0.head(old_z0.rows()) = old_z0;
      }
      trajopt.SetInitialGuessForAllVariables(z0);
    } else {
      trajopt.SetAllInitialGuess(
          h_guess_, r_guess_, dr_guess_, tau_guess_, x_guess_left_in_front_,
          x_guess_right_in_front_, final_position, first_mode_phase_index);
    }
  } else {
    // TODO: try warm start the problem with previous solution
    trajopt.SetAllInitialGuess(h_guess_, r_guess_, dr_guess_, tau_guess_,
                               x_guess_left_in_front_, x_guess_right_in_front_,
                               final_position, first_mode_phase_index);
  }

  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  cout << "\nConstruction time:" << elapsed.count() << "\n";

  // Testing
  if (debug_mode_) {
    // Print out the scaling factor
    /*for (int i = 0; i < trajopt.decision_variables().size(); i++) {
      cout << trajopt.decision_variable(i) << ", ";
      cout << trajopt.decision_variable(i).get_id() << ", ";
      cout << trajopt.FindDecisionVariableIndex(trajopt.decision_variable(i))
           << ", ";
      auto scale_map = trajopt.GetVariableScaling();
      auto it = scale_map.find(i);
      if (it != scale_map.end()) {
        cout << it->second;
      } else {
        cout << "none";
      }
      cout << ", ";
      cout << trajopt.GetInitialGuess(trajopt.decision_variable(i));
      cout << endl;
    }*/
  }

  // Solve
  cout << "\nSolving optimization problem...\n";
  start = std::chrono::high_resolution_clock::now();
  drake::solvers::MathematicalProgramResult result;
  solver_->Solve(trajopt, trajopt.initial_guess(), solver_option_, &result);
  finish = std::chrono::high_resolution_clock::now();
  elapsed = finish - start;
  cout << "    Time of arrival: " << current_time << " | ";
  cout << "Solve time:" << elapsed.count() << " | ";
  SolutionResult solution_result = result.get_solution_result();
  cout << solution_result << " | ";
  cout << "Cost:" << result.get_optimal_cost() << "\n";

  // Get solution
  // The time starts at 0. (by accumulating dt's)
  std::vector<Eigen::VectorXd> time_breaks;
  std::vector<Eigen::MatrixXd> state_samples;
  trajopt.GetStateSamples(result, &state_samples, &time_breaks);

  // Shift the timestamps by the current time
  for (auto& time_break_per_mode : time_breaks) {
    time_break_per_mode.array() += current_time;
  }

  // TODO(yminchen): Note that you will to rotate the coordinates back if the
  //  ROM is dependent on robot's x, y and yaw.

  ///
  /// Pack traj into lcm message (traj_msg)
  ///
  // Note that the trajectory is discontinuous between mode (even the position
  // jumps because left vs right stance leg).
  traj_msg->metadata.description = drake::solvers::to_string(solution_result);
  traj_msg->num_trajectories = param_.n_step + 2;

  traj_msg->trajectory_names.resize(param_.n_step + 2);
  traj_msg->trajectories.resize(param_.n_step + 2);

  // 1. ROM trajectory
  lcmt_trajectory_block traj_block;
  int traj_idx;
  traj_block.num_datatypes = state_samples[0].rows();
  traj_block.datatypes.resize(traj_block.num_datatypes);
  traj_block.datatypes = vector<string>(traj_block.num_datatypes, "");
  for (traj_idx = 0; traj_idx < param_.n_step; traj_idx++) {
    /// Create lcmt_trajectory_block
    traj_block.trajectory_name = to_string(traj_idx);
    traj_block.num_points = time_breaks[traj_idx].size();

    // Reserve space for vectors, then copy Eigentypes to std::vector
    traj_block.time_vec.resize(traj_block.num_points);
    traj_block.time_vec = CopyVectorXdToStdVector(time_breaks[traj_idx]);

    traj_block.datapoints.clear();
    for (int j = 0; j < traj_block.num_datatypes; ++j) {
      traj_block.datapoints.push_back(
          CopyVectorXdToStdVector(state_samples[traj_idx].row(j)));
    }

    /// Assign lcmt_trajectory_block
    traj_msg->trajectories[traj_idx] = traj_block;
    traj_msg->trajectory_names[traj_idx] = to_string(traj_idx);
  }
  // 2. Store start/end FOM states into one trajectory block
  // The order is mode_0_start, mode_0_end, mode_1_start, ...
  traj_block.num_datatypes = nx_;
  traj_block.datatypes.resize(nx_);
  traj_block.datatypes = vector<string>(nx_, "");
  traj_block.trajectory_name = "FOM";
  traj_block.num_points = 2 * param_.n_step;
  traj_block.time_vec.resize(2 * param_.n_step);
  // TODO: you can actually use the touchdown time here, but not sure if it's
  //  worth it
  traj_block.time_vec = vector<double>(2 * param_.n_step, 0);
  // traj_block.time_vec = CopyVectorXdToStdVector(time_breaks[i]);
  traj_block.datapoints.clear();
  Eigen::MatrixXd FOM_eigen_matrix(nx_, 2 * param_.n_step);
  for (int i = 0; i < param_.n_step; ++i) {
    FOM_eigen_matrix.col(2 * i) =
        result.GetSolution(trajopt.x0_vars_by_mode(i));
    FOM_eigen_matrix.col(2 * i + 1) =
        result.GetSolution(trajopt.xf_vars_by_mode(i));
  }
  for (int j = 0; j < nx_; ++j) {
    traj_block.datapoints.push_back(
        CopyVectorXdToStdVector(FOM_eigen_matrix.row(j)));
  }
  traj_msg->trajectories[traj_idx] = traj_block;
  traj_msg->trajectory_names[traj_idx] = "FOM";
  traj_idx++;
  // 3. stance foot (left is 0, right is 1)
  traj_block.num_datatypes = 1;
  traj_block.datatypes.resize(1);
  traj_block.datatypes = vector<string>(1, "");
  traj_block.trajectory_name = "stance_foot";
  traj_block.num_points = param_.n_step;
  traj_block.time_vec.resize(param_.n_step);
  traj_block.time_vec = vector<double>(param_.n_step, 0);
  traj_block.datapoints.clear();
  VectorXd stance_foot_vec = VectorXd::Zero(param_.n_step);
  for (int i = start_with_left_stance ? 1 : 0; i < param_.n_step; i += 2) {
    stance_foot_vec(i) = 1;
  }
  traj_block.datapoints.push_back(CopyVectorXdToStdVector(stance_foot_vec));
  traj_msg->trajectories[traj_idx] = traj_block;
  traj_msg->trajectory_names[traj_idx] = "stance_foot";
  traj_idx++;

  // Store the previous message
  previous_output_msg_ = *traj_msg;
  timestamp_of_previous_plan_ = current_time;

  ///
  /// For debugging
  ///

  // Extract and save solution into files (for debugging)
  //  if (debug_mode_) {
  //  if (debug_mode_ || (result.get_optimal_cost() > 50) || (elapsed.count() >
  //  0.5)) {
  //  if (!result.is_success()) {
  if (debug_mode_ || (elapsed.count() > 0.8)) {
    string dir_data = param_.dir_data;

    /// Save the solution vector
    VectorXd z_sol = result.GetSolution(trajopt.decision_variables());
    writeCSV(dir_data + string("z.csv"), z_sol);
    // cout << trajopt.decision_variables() << endl;

    /// Save traj to csv
    for (int i = 0; i < param_.n_step; i++) {
      writeCSV(dir_data + string("time_at_knots" + to_string(i) + ".csv"),
               time_breaks[i]);
      writeCSV(dir_data + string("state_at_knots" + to_string(i) + ".csv"),
               state_samples[i]);
    }
    MatrixXd input_at_knots = trajopt.GetInputSamples(result);
    writeCSV(dir_data + string("input_at_knots.csv"), input_at_knots);

    MatrixXd x0_each_mode(nx_, num_time_samples.size());
    MatrixXd xf_each_mode(nx_, num_time_samples.size());
    for (uint i = 0; i < num_time_samples.size(); i++) {
      x0_each_mode.col(i) = result.GetSolution(trajopt.x0_vars_by_mode(i));
      xf_each_mode.col(i) = result.GetSolution(trajopt.xf_vars_by_mode(i));
    }
    writeCSV(dir_data + string("x0_each_mode.csv"), x0_each_mode);
    writeCSV(dir_data + string("xf_each_mode.csv"), xf_each_mode);

    /// Save trajectory to lcm
    string file_name = "rom_trajectory";
    RomPlannerTrajectory saved_traj(
        trajopt, result, file_name,
        "Decision variables and state/input trajectories");
    saved_traj.WriteToFile(dir_data + file_name);
    std::cout << "Wrote to file: " << dir_data + file_name << std::endl;

    /// Save files for reproducing the same result
    // cout << "x_init = " << x_init << endl;
    writeCSV(param_.dir_data + string("x_init_test.csv"), x_init);
    writeCSV(param_.dir_data + string("init_phase_test.csv"),
             init_phase * VectorXd::Ones(1));
    writeCSV(param_.dir_data + string("is_right_stance_test.csv"),
             is_right_stance * VectorXd::Ones(1));
  }
}

}  // namespace goldilocks_models
}  // namespace dairlib
