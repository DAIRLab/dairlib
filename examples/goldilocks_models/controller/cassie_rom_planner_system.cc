#include "examples/goldilocks_models/controller/cassie_rom_planner_system.h"

#include <algorithm>  // std::max

#include "common/eigen_utils.h"
#include "examples/goldilocks_models/planning/rom_traj_opt.h"
#include "lcm/rom_planner_saved_trajectory.h"
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
    const MultibodyPlant<double>& plant_feedback,
    const MultibodyPlant<double>& plant_controls,
    const std::vector<int>& left_right_support_fsm_states, double stride_period,
    const PlannerSetting& param, bool debug_mode)
    : nq_(plant_controls.num_positions()),
      nv_(plant_controls.num_velocities()),
      plant_controls_(plant_controls),
      left_right_support_fsm_states_(left_right_support_fsm_states),
      stride_period_(stride_period),
      param_(param),
      debug_mode_(debug_mode) {
  this->set_name("planner_traj");

  DRAKE_DEMAND(param_.knots_per_mode > 0);

  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(
                        OutputVector<double>(plant_feedback.num_positions(),
                                             plant_feedback.num_velocities(),
                                             plant_feedback.num_actuators()))
                    .get_index();
  fsm_and_lo_time_port_ =
      this->DeclareVectorInputPort(BasicVector<double>(2)).get_index();
  this->DeclareAbstractOutputPort(&CassiePlannerWithMixedRomFom::SolveTrajOpt);

  // Initialize the mapping from spring to no spring
  map_position_from_spring_to_no_spring_ =
      systems::controllers::PositionMapFromSpringToNoSpring(plant_feedback,
                                                            plant_controls);
  map_velocity_from_spring_to_no_spring_ =
      systems::controllers::VelocityMapFromSpringToNoSpring(plant_feedback,
                                                            plant_controls);

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
    const Context<double>& context,
    dairlib::lcmt_trajectory_block* traj_msg) const {
  // Read in current robot state
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  VectorXd x_init(map_position_from_spring_to_no_spring_.rows() +
                  map_velocity_from_spring_to_no_spring_.rows());
  x_init << map_position_from_spring_to_no_spring_ *
                robot_output->GetPositions(),
      map_velocity_from_spring_to_no_spring_ * robot_output->GetVelocities();

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
  if (init_phase >= 1) {
    cout << "WARNING: phase >= 1. There might be a bug somewhere, "
            "since we are using a time-based fsm\n";
    init_phase = 1 - 1e-8;
  }

  ///
  /// Rotate Cassie's floating base configuration
  ///
  // Rotate Cassie's floating base configuration to face toward world's x
  // direction and translate the x and y position to the origin

  // TODO: Our original plan was to move the touchdown state to the origin.
  //  But right now we move the current state (instead of touchdown state) to
  //  the corresponding x-y position based on init phase and desired traj

  // Rotate Cassie about the world’s z axis such that the x axis of the pelvis
  // frame is in the world’s x-z plane and toward world’s x axis.
  Quaterniond quat(x_init(0), x_init(1), x_init(2), x_init(3));
  Vector3d pelvis_x = quat.toRotationMatrix().col(0);
  pelvis_x(2) = 0;
  Vector3d world_x(1, 0, 0);
  Quaterniond relative_qaut = Quaterniond::FromTwoVectors(pelvis_x, world_x);
  Quaterniond rotated_quat = relative_qaut * quat;
  x_init.head(4) << rotated_quat.w(), rotated_quat.vec();
  // cout << "pelvis_Rxyz = \n" << quat.toRotationMatrix() << endl;
  // cout << "rotated_pelvis_Rxyz = \n" << rotated_quat.toRotationMatrix() <<
  // endl;

  // Shift pelvis in x, y direction
  x_init(positions_map_.at("base_x")) =
      init_phase * param_.final_position_x / param_.n_step;
  x_init(positions_map_.at("base_y")) = 0;
  // cout << "x(\"base_x\") = " << x_init(positions_map_.at("base_x")) << endl;
  // cout << "x(\"base_y\") = " << x_init(positions_map_.at("base_y")) << endl;

  // Also need to rotate floating base velocities (wrt global frame)
  x_init.segment<3>(nq_) =
      relative_qaut.toRotationMatrix() * x_init.segment<3>(nq_);
  x_init.segment<3>(nq_ + 3) =
      relative_qaut.toRotationMatrix() * x_init.segment<3>(nq_ + 3);

  ///
  /// Construct rom traj opt
  ///

  // Find fsm_state in left_right_support_fsm_states_
  bool is_single_support_phase =
      find(left_right_support_fsm_states_.begin(),
           left_right_support_fsm_states_.end(),
           fsm_state) != left_right_support_fsm_states_.end();
  if (!is_single_support_phase) {
    // do nothing here to use the previous start_with_left_stance_
  } else {
    start_with_left_stance_ = fsm_state == left_right_support_fsm_states_.at(0);
  }

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
                           joint_name_lb_ub_, x_init, start_with_left_stance_,
                           param_.zero_touchdown_impact,
                           debug_mode_ /*print_status*/);

  if (debug_mode_) {
    cout << "Other constraints/costs and initial guess===============\n";
  }
  // Time step constraints
  int n_time_samples =
      std::accumulate(num_time_samples.begin(), num_time_samples.end(), 0) -
      num_time_samples.size() + 1;
  // TODO: play with dt_value. Also don't hard-coded dt_value
  double dt_value =
      stride_period_ / (param_.knots_per_mode - 1);  // h_guess_(1);
  if (knots_first_mode == 2) {
    // Note that the timestep size of the first mode should be different from
    // the rest because the robot could be very close to the touchdown
    double remaining_time_of_first_mode =
        std::max(stride_period_ - current_time, min_dt[0]);
    trajopt.AddTimeStepConstraint(
        min_dt, max_dt, param_.fix_duration,
        remaining_time_of_first_mode + dt_value * (n_time_samples - 2),
        param_.equalize_timestep_size, remaining_time_of_first_mode);
  } else {
    trajopt.AddTimeStepConstraint(min_dt, max_dt, param_.fix_duration,
                                  dt_value * (n_time_samples - 1),
                                  param_.equalize_timestep_size);
  }

  // Constraints for fourbar linkage
  // Note that if the initial pose in the constraint doesn't obey the fourbar
  // linkage relationship. You shouldn't add constraint to the initial pose here
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
  cout << "    Current time: " << current_time << " | ";
  cout << "Solve time:" << elapsed.count() << " | ";
  SolutionResult solution_result = result.get_solution_result();
  cout << solution_result << " | ";
  cout << "Cost:" << result.get_optimal_cost() << "\n";

  // Get solution
  VectorXd time_at_knots = trajopt.GetSampleTimes(result);
  MatrixXd state_at_knots = trajopt.GetStateSamples(result);

  // Shift the timestamps by lift-off time
  time_at_knots.array() += lift_off_time;

  // Extract and save solution into files
  //  if (debug_mode_) {
  if (debug_mode_ || (result.get_optimal_cost() > 50)) {
    VectorXd z_sol = result.GetSolution(trajopt.decision_variables());
    writeCSV(param_.dir_data + string("z.csv"), z_sol);
    // cout << trajopt.decision_variables() << endl;

    MatrixXd input_at_knots = trajopt.GetInputSamples(result);
    writeCSV(param_.dir_data + string("time_at_knots.csv"), time_at_knots);
    writeCSV(param_.dir_data + string("state_at_knots.csv"), state_at_knots);
    writeCSV(param_.dir_data + string("input_at_knots.csv"), input_at_knots);

    MatrixXd x0_each_mode(nq_ + nv_, num_time_samples.size());
    MatrixXd xf_each_mode(nq_ + nv_, num_time_samples.size());
    for (uint i = 0; i < num_time_samples.size(); i++) {
      x0_each_mode.col(i) = result.GetSolution(trajopt.x0_vars_by_mode(i));
      xf_each_mode.col(i) = result.GetSolution(trajopt.xf_vars_by_mode(i));
    }
    writeCSV(param_.dir_data + string("x0_each_mode.csv"), x0_each_mode);
    writeCSV(param_.dir_data + string("xf_each_mode.csv"), xf_each_mode);

    // Save trajectory to file
    string file_name = "rom_trajectory";
    RomPlannerTrajectory saved_traj(
        trajopt, result, file_name,
        "Decision variables and state/input trajectories");
    saved_traj.WriteToFile(param_.dir_data + file_name);
    std::cout << "Wrote to file: " << param_.dir_data + file_name << std::endl;
  }

  ///
  /// Pack traj into lcm message (traj_msg)
  ///
  // TODO(yminchen): there is a bug: you are currenlty using
  //  trajopt.GetStateSamples(result), but the trajectory is discontinous
  //  between mode (even the position jumps because left vs right stance leg).
  //  You probably need to send bigger lcmtypes with multiple blocks, and
  //  reconstruct it into piecewise polynomial in rom_traj_receiver.cc

  //  if (true) {
  //  if (!result.is_success() || !start_with_left_stance_) {
  if (!result.is_success()) {
    // If the solution failed, send an empty-size data. This tells the
    // controller thread to fall back to LIPM traj
    // Note that we cannot really send an empty traj because we use CubicHermite
    // in construction in rom_traj_receiver. We need at least one segment
    // TODO(yminchen): you can create lcmt_planner_traj which contains a flag
    //  solutino_found and a lcmt_trajectory_block
    time_at_knots.resize(2);
    time_at_knots << -std::numeric_limits<double>::infinity(), 0;
    state_at_knots.resize(state_at_knots.rows(), 2);
  }

  traj_msg->trajectory_name = "";
  traj_msg->num_points = time_at_knots.size();
  traj_msg->num_datatypes = state_at_knots.rows();

  // Reserve space for vectors
  traj_msg->time_vec.resize(traj_msg->num_points);
  traj_msg->datatypes.resize(traj_msg->num_datatypes);
  traj_msg->datapoints.clear();

  // Copy Eigentypes to std::vector
  traj_msg->time_vec = CopyVectorXdToStdVector(time_at_knots);
  traj_msg->datatypes = vector<string>(traj_msg->num_datatypes, "");
  for (int i = 0; i < traj_msg->num_datatypes; ++i) {
    traj_msg->datapoints.push_back(
        CopyVectorXdToStdVector(state_at_knots.row(i)));
  }

  // Testing
  if (elapsed.count() > 0.5) {
    cout << "x_init = " << x_init << endl;
    writeCSV(param_.dir_data + string("x_init_test.csv"), x_init);
  }
}

}  // namespace goldilocks_models
}  // namespace dairlib
