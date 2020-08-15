#include "examples/goldilocks_models/controller/cassie_rom_planner_system.h"

#include "examples/goldilocks_models/planning/rom_traj_opt.h"
#include "systems/controllers/osc/osc_utils.h"

#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"

#include <math.h>
#include <string>

using std::cout;
using std::endl;
using std::string;
using std::to_string;
using std::vector;

using Eigen::MatrixXd;
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

OptimalRomPlanner::OptimalRomPlanner(
    const MultibodyPlant<double>& plant_feedback,
    const MultibodyPlant<double>& plant_controls,
    const std::vector<int>& single_support_fsm_states, double stride_period)
    : plant_controls_(plant_controls),
      single_support_fsm_states_(single_support_fsm_states),
      stride_period_(stride_period) {
  this->set_name("mpc_traj");

  // Parameters for the traj opt
  FLAGS_rom_option = 5;
  FLAGS_iter = 30;
  int sample = 19;

  FLAGS_n_step = 2;
  FLAGS_knots_per_mode = 10;
  FLAGS_final_position_x = 1;

  zero_touchdown_impact_ = true;

  FLAGS_equalize_timestep_size = true;
  FLAGS_fix_duration = false;

  FLAGS_feas_tol = 1e-4;
  FLAGS_opt_tol = 1e-4;

  FLAGS_use_ipopt = false;
  FLAGS_log_solver_info = false;

  // Files parameters
  int robot_option = 1;  // robot index for Cassie
  const string dir = "../dairlib_data/goldilocks_models/planning/robot_1/";
  const string dir_model = dir + "models/";  // location of the model files
  const string dir_data = dir + "data/";     // location to store the opt result
  /*string init_file = FLAGS_init_file;
  if (!CreateFolderIfNotExist(dir_model)) return 0;
  if (!CreateFolderIfNotExist(dir_data)) return 0;*/

  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(
                        OutputVector<double>(plant_feedback.num_positions(),
                                             plant_feedback.num_velocities(),
                                             plant_feedback.num_actuators()))
                    .get_index();
  fsm_and_lo_time_port_ =
      this->DeclareVectorInputPort(BasicVector<double>(2)).get_index();
  this->DeclareAbstractOutputPort(&OptimalRomPlanner::SolveTrajOpt);

  // Initialize the mapping from spring to no spring
  map_position_from_spring_to_no_spring_ =
      systems::controllers::PositionMapFromSpringToNoSpring(plant_feedback,
                                                            plant_controls);
  map_velocity_from_spring_to_no_spring_ =
      systems::controllers::VelocityMapFromSpringToNoSpring(plant_feedback,
                                                            plant_controls);

  // Create position maps
  positions_map_ = multibody::makeNameToPositionsMap(plant_controls);

  // Reduced order model
  rom_ = CreateRom(FLAGS_rom_option, robot_option, plant_controls, false);
  ReadModelParameters(rom_.get(), dir_model, FLAGS_iter);

  // Create mirror maps
  state_mirror_ =
      StateMirror(MirrorPosIndexMap(plant_controls, robot_option),
                  MirrorPosSignChangeSet(plant_controls, robot_option),
                  MirrorVelIndexMap(plant_controls, robot_option),
                  MirrorVelSignChangeSet(plant_controls, robot_option));

  // Optimization parameters
  int n_y = rom_->n_y();
  int n_tau = rom_->n_tau();
  Q_ = MatrixXd::Identity(n_y, n_y);
  R_ = MatrixXd::Identity(n_tau, n_tau);

  bool with_init_guess = true;
  // Provide initial guess
  string model_dir_n_pref = dir_model + to_string(FLAGS_iter) + string("_") +
                            to_string(sample) + string("_");
  h_guess_ = VectorXd(FLAGS_knots_per_mode);
  r_guess_ = MatrixXd(n_y, FLAGS_knots_per_mode);
  dr_guess_ = MatrixXd(n_y, FLAGS_knots_per_mode);
  tau_guess_ = MatrixXd(n_tau, FLAGS_knots_per_mode);
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
    for (int i = 0; i < FLAGS_knots_per_mode; i++) {
      h_guess_(i) = duration / (FLAGS_knots_per_mode - 1) * i;
    }
    for (int i = 0; i < FLAGS_knots_per_mode; i++) {
      int n_mat_col_r = r_guess_raw.cols();
      int idx_r = (int)round(double(i * (n_mat_col_r - 1)) /
                             (FLAGS_knots_per_mode - 1));
      r_guess_.col(i) = r_guess_raw.col(idx_r);
      int n_mat_col_dr = dr_guess_raw.cols();
      int idx_dr = (int)round(double(i * (n_mat_col_dr - 1)) /
                              (FLAGS_knots_per_mode - 1));
      dr_guess_.col(i) = dr_guess_raw.col(idx_dr);
      int n_mat_col_tau = tau_guess_raw.cols();
      int idx_tau = (int)round(double(i * (n_mat_col_tau - 1)) /
                               (FLAGS_knots_per_mode - 1));
      tau_guess_.col(i) = tau_guess_raw.col(idx_tau);
    }
    x_guess_left_in_front_ = x_guess_left_in_front_raw;
    x_guess_right_in_front_ = x_guess_right_in_front_raw;

    cout << "initial guess duration = " << duration << endl;
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
  auto right_toe = RightToeFront(plant_controls);
  auto right_heel = RightToeRear(plant_controls);
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
}

void OptimalRomPlanner::SolveTrajOpt(
    const Context<double>& context,
    dairlib::lcmt_trajectory_block* traj_msg) const {
  // Read in current robot state
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  VectorXd q_init =
      map_position_from_spring_to_no_spring_ * robot_output->GetPositions();
  VectorXd v_init =
      map_velocity_from_spring_to_no_spring_ * robot_output->GetVelocities();
  VectorXd x_init(q_init.size() + v_init.size());
  x_init << q_init, v_init;

  // Read in fsm state and lift-off time
  const BasicVector<double>* fsm_and_lo_time_port =
      this->EvalVectorInput(context, fsm_and_lo_time_port_);
  int fsm_state = (int)fsm_and_lo_time_port->get_value()(0);
  double lift_off_time = fsm_and_lo_time_port->get_value()(1);

  // Get time
  double timestamp = robot_output->get_timestamp();
  auto current_time = static_cast<double>(timestamp);

  // Calc phase
  double init_phase = (current_time - lift_off_time) / stride_period_;
  if (init_phase > 1) {
    cout << "WARNING: phase is larger than 1. There might be a bug somewhere\n";
    init_phase = 1;
  }

  // TODO: Move the touchdown state to the origin

  ///
  /// Construct rom traj opt
  ///

  bool start_with_left_stance = fsm_state == single_support_fsm_states_.at(0);

  // Prespecify the time steps
  std::vector<int> num_time_samples;
  std::vector<double> min_dt;
  std::vector<double> max_dt;
  for (int i = 0; i < FLAGS_n_step; i++) {
    num_time_samples.push_back(FLAGS_knots_per_mode);
    min_dt.push_back(.01);
    max_dt.push_back(.3);
  }
  int fisrt_mode_phase_index = int(FLAGS_knots_per_mode * init_phase);
  int knots_first_mode = FLAGS_knots_per_mode - fisrt_mode_phase_index;
  num_time_samples[0] = knots_first_mode;
  cout << "init_phase = " << init_phase << endl;
  cout << "knots_first_mode = " << knots_first_mode << endl;
  cout << "fisrt_mode_phase_index = " << fisrt_mode_phase_index << endl;

  // Goal position
  // TODO: update the final position
  VectorXd final_position(2);
  final_position << q_init(positions_map_.at("base_x")) +
                        FLAGS_final_position_x,
      0;

  // Construct
  cout << "\nConstructing optimization problem...\n";
  auto start = std::chrono::high_resolution_clock::now();
  RomTrajOptCassie trajopt(num_time_samples, Q_, R_, *rom_, plant_controls_,
                           state_mirror_, left_contacts_, right_contacts_,
                           joint_name_lb_ub_, x_init, start_with_left_stance,
                           zero_touchdown_impact_);

  cout << "Other constraints/costs and initial guess=============\n";
  // Time step cosntraints
  int n_time_samples =
      std::accumulate(num_time_samples.begin(), num_time_samples.end(), 0) -
      num_time_samples.size() + 1;
  trajopt.AddTimeStepConstraint(min_dt, max_dt, FLAGS_equalize_timestep_size,
                                FLAGS_fix_duration,
                                h_guess_(1) * (n_time_samples - 1));

  // Constraints for fourbar linkage
  // Note that if the initial pose in the constraint doesn't obey the fourbar
  // linkage relationship. You shouldn't add constraint to the initial pose here
  /*double fourbar_angle = 13.0 / 180.0 * M_PI;
  auto q0_var =
  trajopt.x0_vars_by_mode(0).head(plant_controls_.num_positions());
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
  cout << "Adding final position constraint for full-order model...\n";
  trajopt.AddBoundingBoxConstraint(
      final_position, final_position,
      trajopt.xf_vars_by_mode(num_time_samples.size() - 1).segment(4, 2));

  // Add_robot state in cost
  bool add_x_pose_in_cost = true;
  if (add_x_pose_in_cost) {
    trajopt.AddRegularizationCost(final_position, x_guess_left_in_front_,
                                  x_guess_right_in_front_,
                                  false /*straight_leg_cost*/);

    // Add more regularization cost
    /*auto qf_var = trajopt.xf_vars_by_mode(num_time_samples.size()-1);
    VectorXd quat_unity(4);
    quat_unity << 1, 0, 0, 0;
    trajopt.AddQuadraticErrorCost(100*MatrixXd::Identity(4, 4), quat_unity,
                                  qf_var.head(4));*/
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

  // Default initial guess to avoid singularity (which messes with gradient)
  for (int i = 0; i < num_time_samples.size(); i++) {
    for (int j = 0; j < num_time_samples[i]; j++) {
      if ((FLAGS_rom_option == 0) || (FLAGS_rom_option == 1)) {
        trajopt.SetInitialGuess((trajopt.state_vars_by_mode(i, j))(1), 1);
      } else if (FLAGS_rom_option == 4) {
        trajopt.SetInitialGuess((trajopt.state_vars_by_mode(i, j))(2), 1);
      } else {
        DRAKE_UNREACHABLE();
      }
    }
  }

  // Initial guess for all variables
  // TODO: try warm start the problem
  string init_file = "";
  if (!init_file.empty()) {
    /*VectorXd z0 = readCSV(dir_data + init_file).col(0);
    int n_dec = trajopt.decision_variables().size();
    if (n_dec > z0.rows()) {
      cout << "dim(initial guess) < dim(decision var). "
              "Fill the rest with zero's.\n";
      VectorXd old_z0 = z0;
      z0.resize(n_dec);
      z0 = VectorXd::Zero(n_dec);
      z0.head(old_z0.rows()) = old_z0;
    }
    trajopt.SetInitialGuessForAllVariables(z0);*/
  } else {
    trajopt.SetAllInitialGuess(h_guess_, r_guess_, dr_guess_, tau_guess_,
                               x_guess_left_in_front_, x_guess_right_in_front_,
                               final_position, fisrt_mode_phase_index);
  }

  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  cout << "Construction time:" << elapsed.count() << "\n";

  /*// Testing
  cout << "\nChoose the best solver: "
       << drake::solvers::ChooseBestSolver(trajopt).name() << endl;

  // Print out the scaling factor
  for (int i = 0; i < trajopt.decision_variables().size(); i++) {
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

  // Snopt setting
  int max_iter = 10000;
  if (FLAGS_use_ipopt) {
    // Ipopt settings adapted from CaSaDi and FROST
    auto id = drake::solvers::IpoptSolver::id();
    trajopt.SetSolverOption(id, "tol", FLAGS_feas_tol);
    trajopt.SetSolverOption(id, "dual_inf_tol", FLAGS_feas_tol);
    trajopt.SetSolverOption(id, "constr_viol_tol", FLAGS_feas_tol);
    trajopt.SetSolverOption(id, "compl_inf_tol", FLAGS_feas_tol);
    trajopt.SetSolverOption(id, "max_iter", max_iter);
    trajopt.SetSolverOption(id, "nlp_lower_bound_inf", -1e6);
    trajopt.SetSolverOption(id, "nlp_upper_bound_inf", 1e6);
    if (FLAGS_log_solver_info) {
      trajopt.SetSolverOption(id, "print_timing_statistics", "yes");
      trajopt.SetSolverOption(id, "print_level", 5);
    } else {
      trajopt.SetSolverOption(id, "print_timing_statistics", "no");
      trajopt.SetSolverOption(id, "print_level", 0);
    }

    // Set to ignore overall tolerance/dual infeasibility, but terminate when
    // primal feasible and objective fails to increase over 5 iterations.
    trajopt.SetSolverOption(id, "acceptable_compl_inf_tol", FLAGS_feas_tol);
    trajopt.SetSolverOption(id, "acceptable_constr_viol_tol", FLAGS_feas_tol);
    trajopt.SetSolverOption(id, "acceptable_obj_change_tol", 1e-3);
    trajopt.SetSolverOption(id, "acceptable_tol", 1e2);
    trajopt.SetSolverOption(id, "acceptable_iter", 5);
  } else {
    if (FLAGS_log_solver_info) {
      trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Print file",
                              "../snopt_planning.out");
    }
    trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                            "Major iterations limit", max_iter);
    trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Verify level",
                            0);
    trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                            "Major optimality tolerance", FLAGS_opt_tol);
    trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                            "Major feasibility tolerance", FLAGS_feas_tol);
  }

  // Pick solver
  drake::solvers::SolverId solver_id("");
  if (FLAGS_use_ipopt) {
    solver_id = drake::solvers::IpoptSolver().id();
  } else {
    solver_id = drake::solvers::ChooseBestSolver(trajopt);
  }
  auto solver = drake::solvers::MakeSolver(solver_id);

  // Solve
  cout << "\nSolving optimization problem...\n";
  start = std::chrono::high_resolution_clock::now();
  drake::solvers::MathematicalProgramResult result;
  solver->Solve(trajopt, trajopt.initial_guess(), trajopt.solver_options(),
                &result);
  finish = std::chrono::high_resolution_clock::now();
  elapsed = finish - start;
  cout << "    Solve time:" << elapsed.count() << " | ";
  SolutionResult solution_result = result.get_solution_result();
  cout << solution_result << " | ";
  cout << "Cost:" << result.get_optimal_cost() << "\n";

  // Check which solver we are using
  cout << "Solver: " << result.get_solver_id().name() << endl;

  // Extract solution
  VectorXd z_sol = result.GetSolution(trajopt.decision_variables());
  // writeCSV(dir_data + string("z.csv"), z_sol);
  // cout << trajopt.decision_variables() << endl;

  ///
  /// Pack the traj into lcm (traj_msg)
  ///
  // TODO: finish this
}

}  // namespace goldilocks_models
}  // namespace dairlib
