#include "examples/goldilocks_models/controller/cassie_rom_planner_system.h"

#include <math.h>     /* fmod */
#include <algorithm>  // std::max
#include <fstream>
#include <iostream>
#include <limits>

#include "common/eigen_utils.h"
#include "examples/goldilocks_models/planning/rom_traj_opt.h"
#include "solvers/optimization_utils.h"
#include "systems/controllers/osc/osc_utils.h"

#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"

#include <string>

typedef std::numeric_limits<double> dbl;

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
using dairlib::systems::TimestampedVector;

namespace dairlib {
namespace goldilocks_models {

CassiePlannerWithMixedRomFom::CassiePlannerWithMixedRomFom(
    const MultibodyPlant<double>& plant_controls, double stride_period,
    double double_support_duration, const PlannerSetting& param,
    bool debug_mode)
    : nq_(plant_controls.num_positions()),
      nv_(plant_controls.num_velocities()),
      nx_(plant_controls.num_positions() + plant_controls.num_velocities()),
      plant_controls_(plant_controls),
      stride_period_(stride_period),
      double_support_duration_(double_support_duration),
      left_origin_(BodyPoint(Vector3d::Zero(),
                             plant_controls.GetFrameByName("toe_left"))),
      right_origin_(BodyPoint(Vector3d::Zero(),
                              plant_controls.GetFrameByName("toe_right"))),
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
      this->DeclareVectorInputPort(TimestampedVector<double>(2)).get_index();
  quat_xyz_shift_port_ =
      this->DeclareVectorInputPort(TimestampedVector<double>(7)).get_index();
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
  n_tau_ = rom_->n_tau();
  string model_dir_n_pref = param_.dir_model + to_string(param_.iter) +
                            string("_") + to_string(param_.sample) +
                            string("_");
  h_guess_ = VectorXd(param_.knots_per_mode);
  y_guess_ = MatrixXd(n_y, param_.knots_per_mode);
  dy_guess_ = MatrixXd(n_y, param_.knots_per_mode);
  tau_guess_ = MatrixXd(n_tau_, param_.knots_per_mode);
  if (with_init_guess) {
    // Construct cubic spline from y and ydot and resample, and construct
    // first-order hold from tau and resample.
    // Note that this is an approximation. In the model optimization stage, we
    // do not construct cubic spline (for the version where we impose
    // constraint at the knot points)
    PiecewisePolynomial<double> y_traj =
        PiecewisePolynomial<double>::CubicHermite(
            readCSV(model_dir_n_pref + string("t_breaks0.csv")).col(0),
            readCSV(model_dir_n_pref + string("y_samples0.csv")),
            readCSV(model_dir_n_pref + string("ydot_samples0.csv")));
    PiecewisePolynomial<double> tau_traj;
    if (n_tau_ != 0) {
      tau_traj = PiecewisePolynomial<double>::FirstOrderHold(
          readCSV(model_dir_n_pref + string("t_breaks0.csv")).col(0),
          readCSV(model_dir_n_pref + string("tau_samples0.csv")));
    }

    double duration = y_traj.end_time();
    for (int i = 0; i < param_.knots_per_mode; i++) {
      h_guess_(i) = duration / (param_.knots_per_mode - 1) * i;
      y_guess_.col(i) = y_traj.value(h_guess_(i));
      dy_guess_.col(i) = y_traj.EvalDerivative(h_guess_(i), 1);
      if (n_tau_ != 0) {
        tau_guess_.col(i) = tau_traj.value(h_guess_(i));
      }
    }

    if (use_standing_pose_as_init_FOM_guess_) {
      // Use standing pose for FOM guess
      // Note that it's dangerous to hard-code the state here because the MBP
      // joint order might change depending on upstream (Drake)
      /*VectorXd x_standing_with_springs(45);
      x_standing_with_springs << 1, 0, -2.21802e-13, 0, 0, 0, 1, 0.0194984,
          -0.0194984, 0, 0, 0.479605, 0.479605, -1.1579, -1.1579, -0.0369181,
          -0.0368807, 1.45305, 1.45306, -0.0253012, -1.61133, -0.0253716,
          -1.61137, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0;*/
      VectorXd x_standing_fixed_spring(37);
      x_standing_fixed_spring << 1, -2.06879e-13, -2.9985e-13, 0, 0, 0, 1,
          0.0194983, -0.0194983, 0, 0, 0.510891, 0.510891, -1.22176, -1.22176,
          1.44587, 1.44587, -1.60849, -1.60849, 0, 0, 0,
          param.gains.const_walking_speed_x, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0;
      x_guess_left_in_front_pre_ = x_standing_fixed_spring;
      x_guess_right_in_front_pre_ = x_standing_fixed_spring;
      x_guess_left_in_front_post_ = x_standing_fixed_spring;
      x_guess_right_in_front_post_ = x_standing_fixed_spring;
    } else {
      VectorXd x_guess_right_in_front_pre =
          readCSV(model_dir_n_pref + string("x_samples0.csv")).rightCols(1);
      VectorXd x_guess_right_in_front_post =
          readCSV(model_dir_n_pref + string("x_samples1.csv")).col(0);
      VectorXd x_guess_left_in_front_pre(nx_);
      x_guess_left_in_front_pre
          << state_mirror_.MirrorPos(x_guess_right_in_front_pre.head(nq_)),
          state_mirror_.MirrorVel(x_guess_right_in_front_pre.tail(nv_));
      VectorXd x_guess_left_in_front_post(nx_);
      x_guess_left_in_front_post
          << state_mirror_.MirrorPos(x_guess_right_in_front_post.head(nq_)),
          state_mirror_.MirrorVel(x_guess_right_in_front_post.tail(nv_));

      x_guess_right_in_front_pre_ = x_guess_right_in_front_pre;
      x_guess_right_in_front_post_ = x_guess_right_in_front_post;
      x_guess_left_in_front_pre_ = x_guess_left_in_front_pre;
      x_guess_left_in_front_post_ = x_guess_left_in_front_post;
    }

    // cout << "initial guess duration ~ " << duration << endl;
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
  Vector3d front_contact_point = left_toe.first;
  Vector3d rear_contact_point = left_heel.first;
  if (param_.use_double_contact_points) {
    auto left_toe_front = BodyPoint(front_contact_point,
                                    plant_controls.GetFrameByName("toe_left"));
    auto left_toe_rear = BodyPoint(rear_contact_point,
                                   plant_controls.GetFrameByName("toe_left"));
    auto right_toe_front = BodyPoint(
        front_contact_point, plant_controls.GetFrameByName("toe_right"));
    auto right_toe_rear = BodyPoint(rear_contact_point,
                                    plant_controls.GetFrameByName("toe_right"));
    left_contacts_.push_back(left_toe_front);
    left_contacts_.push_back(left_toe_rear);
    right_contacts_.push_back(right_toe_front);
    right_contacts_.push_back(right_toe_rear);
  } else {
    Vector3d mid_contact_point = (front_contact_point + rear_contact_point) / 2;
    auto left_toe_mid =
        BodyPoint(mid_contact_point, plant_controls.GetFrameByName("toe_left"));
    auto right_toe_mid = BodyPoint(mid_contact_point,
                                   plant_controls.GetFrameByName("toe_right"));
    left_contacts_.push_back(left_toe_mid);
    right_contacts_.push_back(right_toe_mid);
  }

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
  R_ = param_.w_R * MatrixXd::Identity(n_tau_, n_tau_);

  // Pick solver
  drake::solvers::SolverId solver_id("");
  //  if (param_.use_ipopt) {
  //  } else {
  //  }
  solver_id = drake::solvers::IpoptSolver().id();
  cout << "Solver: " << solver_id.name() << endl;
  solver_ipopt_ = drake::solvers::MakeSolver(solver_id);
  solver_id = drake::solvers::SnoptSolver().id();
  cout << "Solver: " << solver_id.name() << endl;
  solver_snopt_ = drake::solvers::MakeSolver(solver_id);

  // Set solver option
  fixed_time_limit_ = param_.time_limit > 0;
  //  if (param_.use_ipopt) {
  // Ipopt settings adapted from CaSaDi and FROST
  auto id = drake::solvers::IpoptSolver::id();
  solver_option_ipopt_.SetOption(id, "tol", param_.feas_tol);
  solver_option_ipopt_.SetOption(id, "dual_inf_tol", param_.feas_tol);
  solver_option_ipopt_.SetOption(id, "constr_viol_tol", param_.feas_tol);
  solver_option_ipopt_.SetOption(id, "compl_inf_tol", param_.feas_tol);
  solver_option_ipopt_.SetOption(id, "max_iter", param_.max_iter);
  solver_option_ipopt_.SetOption(id, "nlp_lower_bound_inf", -1e6);
  solver_option_ipopt_.SetOption(id, "nlp_upper_bound_inf", 1e6);
  if (param_.log_solver_info) {
    solver_option_ipopt_.SetOption(id, "print_timing_statistics", "yes");
    solver_option_ipopt_.SetOption(id, "print_level", 0);
    solver_option_ipopt_.SetOption(id, "output_file",
                                   "../ipopt_planning_latest.out");
    solver_option_ipopt_.SetOption(id, "file_print_level", 5);
  } else {
    solver_option_ipopt_.SetOption(id, "print_timing_statistics", "no");
    solver_option_ipopt_.SetOption(id, "print_level", 0);
  }
  if (param_.time_limit > 0) {
    solver_option_ipopt_.SetOption(id, "max_cpu_time", param_.time_limit);
  } else {
    solver_option_ipopt_.SetOption(id, "max_cpu_time",
                                   time_limit_for_first_loop_);
  }

  // Set to ignore overall tolerance/dual infeasibility, but terminate when
  // primal feasible and objective fails to increase over 5 iterations.
  solver_option_ipopt_.SetOption(id, "acceptable_compl_inf_tol",
                                 param_.feas_tol);
  solver_option_ipopt_.SetOption(id, "acceptable_constr_viol_tol",
                                 param_.feas_tol);
  solver_option_ipopt_.SetOption(id, "acceptable_obj_change_tol", 1e-3);
  solver_option_ipopt_.SetOption(id, "acceptable_tol", 1e2);
  solver_option_ipopt_.SetOption(id, "acceptable_iter", 5);
  //  } else {
  if (param_.log_solver_info) {
    solver_option_snopt_.SetOption(drake::solvers::SnoptSolver::id(),
                                   "Print file", "../snopt_planning.out");
    cout << "Note that you are logging snopt result.\n";
  }
  if (param_.time_limit > 0) {
    solver_option_snopt_.SetOption(drake::solvers::SnoptSolver::id(),
                                   "Time limit", param_.time_limit);
    solver_option_snopt_.SetOption(drake::solvers::SnoptSolver::id(),
                                   "Timing level", 3);
  } else {
    solver_option_snopt_.SetOption(drake::solvers::SnoptSolver::id(),
                                   "Time limit", time_limit_for_first_loop_);
    solver_option_snopt_.SetOption(drake::solvers::SnoptSolver::id(),
                                   "Timing level", 3);
  }
  solver_option_snopt_.SetOption(drake::solvers::SnoptSolver::id(),
                                 "Major iterations limit", param_.max_iter);
  solver_option_snopt_.SetOption(drake::solvers::SnoptSolver::id(),
                                 "Verify level", 0);
  solver_option_snopt_.SetOption(drake::solvers::SnoptSolver::id(),
                                 "Major optimality tolerance",
                                 param_.opt_tol /* * 0.01*/);
  solver_option_snopt_.SetOption(drake::solvers::SnoptSolver::id(),
                                 "Major feasibility tolerance",
                                 param_.feas_tol /* * 0.01*/);
  //  }

  // Initialization
  if (param_.zero_touchdown_impact) {
    FOM_Lambda_ = Eigen::MatrixXd::Zero(0, (param_.n_step));
  } else {
    FOM_Lambda_ =
        Eigen::MatrixXd::Zero(3 * left_contacts_.size(), param_.n_step);
  }

  // Initialization
  prev_mode_start_ = std::vector<int>(param_.n_step, -1);

  // Initialization for warm starting in debug mode
  if (param_.init_file.empty()) {
    if (warm_start_with_previous_solution_ &&
        // Load the saved traj (not light weight) for the first iteration
        param_.solve_idx_for_read_from_file > 0) {
      lightweight_saved_traj_ = RomPlannerTrajectory(
          param_.dir_data + to_string(param_.solve_idx_for_read_from_file - 1) +
          "_rom_trajectory");
      h_solutions_ = readCSV(param_.dir_data +
                             to_string(param_.solve_idx_for_read_from_file) +
                             "_prev_h_solutions.csv");
      input_at_knots_ =
          (n_tau_ == 0)
              ? MatrixXd::Zero(0, h_solutions_.size() + 1)
              : readCSV(param_.dir_data +
                        to_string(param_.solve_idx_for_read_from_file) +
                        "_prev_input_at_knots.csv");
      FOM_Lambda_ =
          param_.zero_touchdown_impact
              ? Eigen::MatrixXd::Zero(0, (param_.n_step))
              : readCSV(param_.dir_data +
                        to_string(param_.solve_idx_for_read_from_file) +
                        "_prev_FOM_Lambda.csv");

      prev_global_fsm_idx_ = readCSV(
          param_.dir_data + to_string(param_.solve_idx_for_read_from_file) +
          "_prev_global_fsm_idx.csv")(0, 0);
      prev_first_mode_knot_idx_ = readCSV(
          param_.dir_data + to_string(param_.solve_idx_for_read_from_file) +
          "_prev_first_mode_knot_idx.csv")(0, 0);
      VectorXd in_eigen =
          readCSV(param_.dir_data +
                  to_string(param_.solve_idx_for_read_from_file) +
                  "_prev_mode_start.csv")
              .col(0);
      for (int i = 0; i < prev_mode_start_.size(); i++) {
        prev_mode_start_[i] = int(in_eigen(i));
      }
    }
  }
}

void CassiePlannerWithMixedRomFom::SolveTrajOpt(
    const Context<double>& context, dairlib::lcmt_saved_traj* traj_msg) const {
  ///
  /// Decide if we need to re-plan (not ideal code. See header file)
  ///

  // Get current time
  // Note that we can use context time here becasue this is an output function
  // instead of discrete update function
  auto current_time = context.get_time();

  // Commented out this code because we are clearing the lcm message twice in
  // the LcmDrivenLoop class (this is another workaround).
  //  bool need_to_replan = ((current_time - timestamp_of_previous_plan_) >
  //                         min_time_difference_for_replanning_);
  //  if (!need_to_replan) {
  //    *traj_msg = previous_output_msg_;
  //    return;
  //  }

  ///
  /// Read from input ports
  ///

  // Read in current robot state
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  VectorXd x_init = robot_output->GetState();

  // Testing
  //  x_init.segment(nq_, 3) << 0, 0, 0;

  // Get phase in the first mode
  const BasicVector<double>* phase_port =
      this->EvalVectorInput(context, phase_port_);
  double init_phase = phase_port->get_value()(0);

  // Get stance foot
  bool is_right_stance =
      (bool)this->EvalVectorInput(context, stance_foot_port_)->get_value()(0);
  bool start_with_left_stance = !is_right_stance;

  // Get lift-off time
  /*const BasicVector<double>* fsm_and_lo_time_port =
      this->EvalVectorInput(context, fsm_and_lo_time_port_);
  double lift_off_time = fsm_and_lo_time_port->get_value()(1);*/

  if (debug_mode_) {
    cout.precision(dbl::max_digits10);
    cout << "Used for the planner: \n";
    cout << "  x_init  = " << x_init.transpose() << endl;
    cout << "  current_time  = " << current_time << endl;
    cout << "  start_with_left_stance  = " << start_with_left_stance << endl;
    cout << "  init_phase  = " << init_phase << endl;
  }

  // For data logging
  string prefix = debug_mode_ ? "debug_" : to_string(counter_) + "_";
  string prefix_next =
      debug_mode_ ? "debug_next_" : to_string(counter_ + 1) + "_";

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
  int first_mode_knot_idx = int((param_.knots_per_mode - 1) * init_phase);
  int n_knots_first_mode = param_.knots_per_mode - first_mode_knot_idx;
  num_time_samples[0] = n_knots_first_mode;
  if (n_knots_first_mode == 2) {
    min_dt[0] = 1e-3;
  }
  cout << "start_with_left_stance  = " << start_with_left_stance << endl;
  cout << "init_phase = " << init_phase << endl;
  cout << "n_knots_first_mode = " << n_knots_first_mode << endl;
  cout << "first_mode_knot_idx = " << first_mode_knot_idx << endl;

  // Goal position
  VectorXd final_position(2);
  final_position << param_.final_position_x, 0;
  /*final_position << x_lift_off(positions_map_.at("base_x")) +
                        param_.final_position_x,
      0;*/

  // Get the desired xy positions for the FOM states
  vector<VectorXd> des_xy_pos =
      vector<VectorXd>(param_.n_step + 1, VectorXd::Zero(2));
  int n_segment_total =
      std::accumulate(num_time_samples.begin(), num_time_samples.end(), 0) -
      num_time_samples.size();
  VectorXd adjusted_final_pos = final_position * n_segment_total /
                                (param_.n_step * (param_.knots_per_mode - 1));
  for (int i = 1; i < des_xy_pos.size(); i++) {
    des_xy_pos[i] = des_xy_pos[i - 1] + adjusted_final_pos *
                                            (num_time_samples.at(i - 1) - 1) /
                                            n_segment_total;
  }

  // Maximum step length
  double MAX_FOOT_SPEED = 1.5;  // m/s
  double first_mode_duration = stride_period_ * (1 - init_phase);
  double remaining_time_til_touchdown = first_mode_duration;
  // Take into account the double stance duration
  //  double remaining_time_til_touchdown =
  //      std::max(0.0, first_mode_duration - double_support_duration_);
  vector<double> max_swing_distance =
      vector<double>(param_.n_step, MAX_FOOT_SPEED * stride_period_);
  max_swing_distance[0] = MAX_FOOT_SPEED * remaining_time_til_touchdown;
  cout << "remaining_time_til_touchdown = " << remaining_time_til_touchdown
       << endl;

  // Construct
  PrintStatus("\nConstructing optimization problem...");
  auto start = std::chrono::high_resolution_clock::now();
  RomTrajOptCassie trajopt(
      num_time_samples, Q_, R_, *rom_, plant_controls_, state_mirror_,
      left_contacts_, right_contacts_, left_origin_, right_origin_,
      joint_name_lb_ub_, x_init, max_swing_distance, start_with_left_stance,
      param_.zero_touchdown_impact, relax_index_, debug_mode_ /*print_status*/);

  PrintStatus("Other constraints and costs ===============");
  // Time step constraints
  trajopt.AddTimeStepConstraint(min_dt, max_dt, param_.fix_duration,
                                param_.equalize_timestep_size,
                                first_mode_duration, stride_period_);

  // Constraints for fourbar linkage
  // Note that if the initial pose in the constraint doesn't obey the fourbar
  // linkage relationship.
  // I believe we shouldn't impose this constraint on the init pose because we
  // use a model without spring in the planner (while in the sim and real life,
  // we use model with springs). The constraint here will conflict the initial
  // FOM pose constraint
  double fourbar_angle = 13.0 / 180.0 * M_PI;
  MatrixXd Aeq = MatrixXd::Ones(1, 2);
  VectorXd angle = fourbar_angle * VectorXd::Ones(1);
  for (int i = 0; i < num_time_samples.size(); i++) {
    auto xf = trajopt.xf_vars_by_mode(i);
    trajopt.AddLinearEqualityConstraint(
        Aeq, angle,
        {xf.segment<1>(positions_map_.at("knee_left")),
         xf.segment<1>(positions_map_.at("ankle_joint_left"))});
    trajopt.AddLinearEqualityConstraint(
        Aeq, angle,
        {xf.segment<1>(positions_map_.at("knee_right")),
         xf.segment<1>(positions_map_.at("ankle_joint_right"))});
  }

  // Final goal position constraint
  /*PrintStatus("Adding constraint -- FoM final position");
  trajopt.AddBoundingBoxConstraint(
      adjusted_final_position, adjusted_final_position,
      trajopt.xf_vars_by_mode(num_time_samples.size() - 1).segment(4, 2));*/

  // Add robot state in cost
  bool add_x_pose_in_cost = true;
  if (add_x_pose_in_cost) {
    trajopt.AddRegularizationCost(
        des_xy_pos, x_guess_left_in_front_pre_, x_guess_right_in_front_pre_,
        x_guess_left_in_front_post_, x_guess_right_in_front_post_,
        param_.gains.w_reg_quat, param_.gains.w_reg_xy, param_.gains.w_reg_z,
        param_.gains.w_reg_joints, param_.gains.w_reg_hip_yaw,
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
    trajopt.AddRomRegularizationCost(h_guess_, y_guess_, dy_guess_, tau_guess_,
                                     first_mode_knot_idx, param_.w_rom_reg);
  }

  // Default initial guess to avoid singularity (which messes with gradient)
  for (int i = 0; i < num_time_samples.size(); i++) {
    for (int j = 0; j < num_time_samples.at(i); j++) {
      if ((param_.rom_option == 0) || (param_.rom_option == 1)) {
        trajopt.SetInitialGuess((trajopt.state_vars_by_mode(i, j))(1), 1);
      } else if ((param_.rom_option == 4) || (param_.rom_option == 8)) {
        trajopt.SetInitialGuess((trajopt.state_vars_by_mode(i, j))(2), 1);
      } else {
        DRAKE_UNREACHABLE();
      }
    }
  }

  PrintStatus("Initial guesses ===============");

  // Initial guess for all variables
  int global_fsm_idx = int((current_time + 1e-8) / stride_period_);
  //  if (!param_.init_file.empty()) {
  if (counter_ == 0 && !param_.init_file.empty()) {
    PrintStatus("Set initial guess from the file " + param_.init_file);
    VectorXd z0 = readCSV(param_.dir_data + param_.init_file).col(0);
    // writeCSV(param_.dir_data + "testing_" + string("init_file.csv"), z0,
    // true);
    int n_dec = trajopt.decision_variables().size();
    if (n_dec > z0.rows()) {
      cout << "dim(initial guess) < dim(decision var). "
              "Fill the rest with zero's.\n";
      VectorXd old_z0 = z0;
      z0.resize(n_dec);
      z0 = VectorXd::Zero(n_dec);
      z0.head(old_z0.rows()) = old_z0;
    } else if (n_dec < z0.rows()) {
      cout << "The init file is longer than the length of decision variable\n";
    }
    trajopt.SetInitialGuessForAllVariables(z0);
  } else {
    //    PrintStatus("global_fsm_idx = " + to_string(global_fsm_idx));
    cout << "global_fsm_idx = " + to_string(global_fsm_idx) << endl;
    if (warm_start_with_previous_solution_ && (prev_global_fsm_idx_ >= 0)) {
      PrintStatus("Warm start initial guess with previous solution...");
      WarmStartGuess(des_xy_pos, global_fsm_idx, first_mode_knot_idx, &trajopt);
    } else {
      // Set heuristic initial guess for all variables
      PrintStatus("Set heuristic initial guess...");
      trajopt.SetHeuristicInitialGuess(
          h_guess_, y_guess_, dy_guess_, tau_guess_, x_guess_left_in_front_pre_,
          x_guess_right_in_front_pre_, x_guess_left_in_front_post_,
          x_guess_right_in_front_post_, des_xy_pos, first_mode_knot_idx, 0);
    }
    trajopt.SetInitialGuess(trajopt.x0_vars_by_mode(0), x_init);

    // Avoid zero-value initial guess!
    // This sped up the solve and sometimes unstuck the solver!
    const auto& all_vars = trajopt.decision_variables();
    int n_var = all_vars.size();
    VectorXd rand = 0.01 * VectorXd::Random(n_var);
    for (int i = 0; i < n_var; i++) {
      double init_guess = trajopt.GetInitialGuess(all_vars(i));
      if (init_guess == 0 || isnan(init_guess)) {
        cout << all_vars(i) << " init guess was " << init_guess << endl;
        trajopt.SetInitialGuess(all_vars(i), rand(i));
      }
    }
  }

  // Scaling
  if (param_.rom_option == 4) {
    //    cout << "Scaling constraints... \n";
    //    trajopt.SetScalingForLIPM();
  }

  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  cout << "\nConstruction time:" << elapsed.count() << "\n";

  // Testing
  if (true /*debug_mode_*/) {
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

  // Set time limit in the solver dynamically if no time_limit specified
  if (!fixed_time_limit_ && counter_ > 0) {
    // allowed time =
    //   last traj's end time - current time - time for lcm packing/traveling
    double time_limit =
        lightweight_saved_traj_.GetStateBreaks(param_.n_step - 1).tail(1)(0) -
        current_time - buffer_;
    cout << "Set the time limit to " << time_limit << endl;
    solver_option_ipopt_.SetOption(drake::solvers::IpoptSolver::id(),
                                   "max_cpu_time", time_limit);
    solver_option_snopt_.SetOption(drake::solvers::SnoptSolver::id(),
                                   "Time limit", time_limit);
  }

  // Solve
  cout << "\nSolving optimization problem...\n";
  start = std::chrono::high_resolution_clock::now();
  drake::solvers::MathematicalProgramResult result;
  if (param_.use_ipopt) {
    solver_ipopt_->Solve(trajopt, trajopt.initial_guess(), solver_option_ipopt_,
                         &result);
  } else {
    solver_snopt_->Solve(trajopt, trajopt.initial_guess(), solver_option_snopt_,
                         &result);
  }
  finish = std::chrono::high_resolution_clock::now();
  elapsed = finish - start;
  SolutionResult solution_result = result.get_solution_result();
  cout << "    Time of arrival: " << current_time << " | ";
  cout << "Solve time:" << elapsed.count() << " | ";
  cout << solution_result << " | ";
  cout << "Cost:" << result.get_optimal_cost() << "\n";

  // Testing -- solve with another solver
  if (false) {
    start = std::chrono::high_resolution_clock::now();
    drake::solvers::MathematicalProgramResult result2;
    if (param_.use_ipopt) {
      solver_snopt_->Solve(trajopt, trajopt.initial_guess(),
                           solver_option_snopt_, &result2);
    } else {
      solver_ipopt_->Solve(trajopt, trajopt.initial_guess(),
                           solver_option_ipopt_, &result2);
    }
    finish = std::chrono::high_resolution_clock::now();
    elapsed = finish - start;
    cout << "    Time of arrival: " << current_time << " | ";
    cout << "Solve time:" << elapsed.count() << " | ";
    cout << result2.get_solution_result() << " | ";
    cout << "Cost:" << result2.get_optimal_cost() << "\n";

    /// For visualization of the second solver
    MatrixXd x0_each_mode(nx_, trajopt.num_modes());
    MatrixXd xf_each_mode(nx_, trajopt.num_modes());
    for (uint i = 0; i < trajopt.num_modes(); i++) {
      x0_each_mode.col(i) = result2.GetSolution(trajopt.x0_vars_by_mode(i));
      xf_each_mode.col(i) = result2.GetSolution(trajopt.xf_vars_by_mode(i));
    }
    writeCSV(param_.dir_data + string(prefix + "x0_each_mode_snopt.csv"),
             x0_each_mode);
    writeCSV(param_.dir_data + string(prefix + "xf_each_mode_snopt.csv"),
             xf_each_mode);
  }

  // Testing -- solve with another solver and feed it with solution as init
  // guess
  if (false) {
    cout << "Use previous solution as a initial condition...\n";
    start = std::chrono::high_resolution_clock::now();
    drake::solvers::MathematicalProgramResult result2;
    if (param_.use_ipopt) {
      solver_snopt_->Solve(trajopt, result.GetSolution(), solver_option_snopt_,
                           &result2);
    } else {
      solver_ipopt_->Solve(trajopt, result.GetSolution(), solver_option_ipopt_,
                           &result2);
    }
    finish = std::chrono::high_resolution_clock::now();
    elapsed = finish - start;
    cout << "    Time of arrival: " << current_time << " | ";
    cout << "Solve time:" << elapsed.count() << " | ";
    cout << result2.get_solution_result() << " | ";
    cout << "Cost:" << result2.get_optimal_cost() << "\n";
  }

  // Testing -- print all param, costs and constriants for debugging
  // PrintAllCostsAndConstraints(trajopt);

  // Testing -- store the initial guess to the result (to visualize init guess)
  // result.set_x_val(trajopt.initial_guess());

  // TODO(yminchen): Note that you will to rotate the coordinates back if the
  //  ROM is dependent on robot's x, y and yaw.

  ///
  /// Pack traj into lcm message (traj_msg)
  ///
  // Note that the trajectory is discontinuous between mode (even the position
  // jumps because left vs right stance leg).

  VectorXd quat_xyz_shift = VectorXd::Zero(7);
  if (!debug_mode_) {
    quat_xyz_shift = static_cast<const TimestampedVector<double>*>(
                         this->EvalVectorInput(context, quat_xyz_shift_port_))
                         ->get_data();
  }

  // Benchmark: for n_step = 3, the packing time is about 60us and the message
  // size is about 4.5KB (use WriteToFile() to check).
  lightweight_saved_traj_ = RomPlannerTrajectory(
      trajopt, result, quat_xyz_shift, prefix, "", true, current_time);
  *traj_msg = lightweight_saved_traj_.GenerateLcmObject();

  // Store the previous message
  previous_output_msg_ = *traj_msg;
  timestamp_of_previous_plan_ = current_time;

  ///
  /// Save solutions for either logging for warm-starting
  ///

  // TODO: maybe don't save the trajectory for warmstart if the solver didn't
  //  find an optimal solution

  h_solutions_ = trajopt.GetTimeStepSolution(result);
  input_at_knots_ = trajopt.GetInputSamples(result);

  for (int i = 0; i < param_.n_step; i++) {
    FOM_Lambda_.col(i) = result.GetSolution(trajopt.impulse_vars(i));
  }

  prev_global_fsm_idx_ = global_fsm_idx;
  prev_first_mode_knot_idx_ = first_mode_knot_idx;
  prev_mode_start_ = trajopt.mode_start();

  ///
  /// For debugging
  ///
  start = std::chrono::high_resolution_clock::now();

  if (param_.log_solver_info && param_.use_ipopt) {
    // Ipopt doesn't seem to have the append feature, so we do it manually
    std::system(
        "cat ../ipopt_planning_latest.out >> ../ipopt_planning_combined.out");
  }

  bool log_data_and_check_solution = true;
  if (log_data_and_check_solution) {
    // Extract and save solution into files (for debugging)
    SaveDataIntoFiles(current_time, x_init, init_phase, is_right_stance,
                      trajopt, result, param_.dir_data, prefix, prefix_next);
    // Save trajectory to lcm
    SaveTrajIntoLcmBinary(trajopt, result, quat_xyz_shift, param_.dir_data,
                          prefix);

    // Check the cost
    PrintCost(trajopt, result);

    // Check constraint violation
    //    //    double tol = 1e-3;
    double tol = param_.feas_tol;
    solvers::CheckGenericConstraints(trajopt, result, tol);
  }

  // Keep track of solve time and stuffs
  BookKeeping(start_with_left_stance, elapsed, result);

  // Switch to snopt after one iteration (use ipopt to get a good solution for
  // the first loop)
  if (counter_ == 0) {
    //    cout << "***\n*** WARNING: switch to snopt solver\n***\n";
    //    param_.use_ipopt = false;
  }

  finish = std::chrono::high_resolution_clock::now();
  elapsed = finish - start;
  cout << "Runtime for data saving (for debugging):" << elapsed.count() << endl;

  ///
  counter_++;
}

void CassiePlannerWithMixedRomFom::SaveTrajIntoLcmBinary(
    const RomTrajOptCassie& trajopt, const MathematicalProgramResult& result,
    const VectorXd& quat_xyz_shift, const string& dir_data,
    const string& prefix) const {
  string file_name = prefix + "rom_trajectory";
  RomPlannerTrajectory saved_traj(
      trajopt, result, quat_xyz_shift, file_name,
      "Decision variables and state/input trajectories");
  saved_traj.WriteToFile(dir_data + file_name);
  std::cout << "Wrote to file: " << dir_data + file_name << std::endl;
}

void CassiePlannerWithMixedRomFom::SaveDataIntoFiles(
    double current_time, const VectorXd& x_init, double init_phase,
    bool is_right_stance, const RomTrajOptCassie& trajopt,
    const MathematicalProgramResult& result, const string& dir_data,
    const string& prefix, const string& prefix_next) const {
  /// Save the solution vector
  VectorXd z_sol = result.GetSolution(trajopt.decision_variables());
  writeCSV(dir_data + string(prefix + "z.csv"), z_sol);
  // cout << trajopt.decision_variables() << endl;

  /// Save traj to csv
  for (int i = 0; i < param_.n_step; i++) {
    writeCSV(
        dir_data + string(prefix + "time_at_knots" + to_string(i) + ".csv"),
        lightweight_saved_traj_.GetStateBreaks(i));
    writeCSV(
        dir_data + string(prefix + "state_at_knots" + to_string(i) + ".csv"),
        lightweight_saved_traj_.GetStateSamples(i));
  }
  MatrixXd input_at_knots = trajopt.GetInputSamples(result);
  writeCSV(dir_data + string(prefix + "input_at_knots.csv"), input_at_knots);

  MatrixXd x0_each_mode(nx_, trajopt.num_modes() + 1);
  MatrixXd xf_each_mode(nx_, trajopt.num_modes());
  for (uint i = 0; i < trajopt.num_modes(); i++) {
    x0_each_mode.col(i) = result.GetSolution(trajopt.x0_vars_by_mode(i));
    xf_each_mode.col(i) = result.GetSolution(trajopt.xf_vars_by_mode(i));
  }
  x0_each_mode.col(trajopt.num_modes()) =
      result.GetSolution(trajopt.x0_vars_by_mode(trajopt.num_modes()));
  writeCSV(dir_data + string(prefix + "x0_each_mode.csv"), x0_each_mode);
  writeCSV(dir_data + string(prefix + "xf_each_mode.csv"), xf_each_mode);

  /// Save files for reproducing the same result
  // cout << "x_init = " << x_init << endl;
  writeCSV(param_.dir_data + prefix + string("x_init.csv"), x_init, true);
  writeCSV(param_.dir_data + prefix + string("init_phase.csv"),
           init_phase * VectorXd::Ones(1), true);
  writeCSV(param_.dir_data + prefix + string("is_right_stance.csv"),
           is_right_stance * VectorXd::Ones(1), true);
  writeCSV(param_.dir_data + prefix + string("init_file.csv"),
           trajopt.initial_guess(), true);
  writeCSV(param_.dir_data + prefix + string("current_time.csv"),
           current_time * VectorXd::Ones(1), true);
  // for warm-start
  writeCSV(param_.dir_data + prefix_next + string("prev_h_solutions.csv"),
           h_solutions_, true);
  writeCSV(param_.dir_data + prefix_next + string("prev_input_at_knots.csv"),
           input_at_knots_, true);
  writeCSV(param_.dir_data + prefix_next + string("prev_FOM_Lambda.csv"),
           FOM_Lambda_, true);
  writeCSV(param_.dir_data + prefix_next + string("prev_global_fsm_idx.csv"),
           prev_global_fsm_idx_ * VectorXd::Ones(1), true);
  writeCSV(
      param_.dir_data + prefix_next + string("prev_first_mode_knot_idx.csv"),
      prev_first_mode_knot_idx_ * VectorXd::Ones(1), true);
  VectorXd in_eigen(param_.n_step);
  for (int i = 0; i < param_.n_step; i++) {
    in_eigen(i) = prev_mode_start_[i];
  }
  writeCSV(param_.dir_data + prefix_next + string("prev_mode_start.csv"),
           in_eigen, true);
}

void CassiePlannerWithMixedRomFom::PrintCost(
    const RomTrajOptCassie& trajopt,
    const MathematicalProgramResult& result) const {
  double cost_ydot =
      solvers::EvalCostGivenSolution(result, trajopt.rom_state_cost_bindings_);
  if (cost_ydot > 0) {
    cout << "cost_ydot = " << cost_ydot << endl;
  }
  double cost_u =
      solvers::EvalCostGivenSolution(result, trajopt.rom_input_cost_bindings_);
  if (cost_u > 0) {
    cout << "cost_u = " << cost_u << endl;
  }
  double rom_regularization_cost = solvers::EvalCostGivenSolution(
      result, trajopt.rom_regularization_cost_bindings_);
  if (rom_regularization_cost > 0) {
    cout << "rom_regularization_cost = " << rom_regularization_cost << endl;
  }
  double fom_reg_quat_cost = solvers::EvalCostGivenSolution(
      result, trajopt.fom_reg_quat_cost_bindings_);
  if (fom_reg_quat_cost > 0) {
    cout << "fom_reg_quat_cost = " << fom_reg_quat_cost << endl;
  }
  double fom_xy_cost =
      solvers::EvalCostGivenSolution(result, trajopt.fom_reg_xy_cost_bindings_);
  if (fom_xy_cost > 0) {
    cout << "fom_xy_cost = " << fom_xy_cost << endl;
  }
  double fom_reg_z_cost =
      solvers::EvalCostGivenSolution(result, trajopt.fom_reg_z_cost_bindings_);
  if (fom_reg_z_cost > 0) {
    cout << "fom_reg_z_cost = " << fom_reg_z_cost << endl;
  }
  double fom_reg_joint_cost = solvers::EvalCostGivenSolution(
      result, trajopt.fom_reg_joint_cost_bindings_);
  if (fom_reg_joint_cost > 0) {
    cout << "fom_reg_joint_cost = " << fom_reg_joint_cost << endl;
  }
  double fom_reg_vel_cost = solvers::EvalCostGivenSolution(
      result, trajopt.fom_reg_vel_cost_bindings_);
  if (fom_reg_vel_cost > 0) {
    cout << "fom_reg_vel_cost = " << fom_reg_vel_cost << endl;
  }
  double lambda_cost =
      solvers::EvalCostGivenSolution(result, trajopt.lambda_cost_bindings_);
  if (lambda_cost > 0) {
    cout << "lambda_cost = " << lambda_cost << endl;
  }
  double x0_relax_cost =
      solvers::EvalCostGivenSolution(result, trajopt.x0_relax_cost_bindings_);
  if (x0_relax_cost > 0) {
    cout << "x0_relax_cost = " << x0_relax_cost << endl;
  }
  double v0_relax_cost =
      solvers::EvalCostGivenSolution(result, trajopt.v0_relax_cost_bindings_);
  if (v0_relax_cost > 0) {
    cout << "v0_relax_cost = " << v0_relax_cost << endl;
  }
  double init_rom_relax_cost = solvers::EvalCostGivenSolution(
      result, trajopt.init_rom_relax_cost_bindings_);
  if (init_rom_relax_cost > 0) {
    cout << "init_rom_relax_cost = " << init_rom_relax_cost << endl;
  }
}

void CassiePlannerWithMixedRomFom::BookKeeping(
    bool start_with_left_stance, const std::chrono::duration<double>& elapsed,
    const MathematicalProgramResult& result) const {
  // Keep track of solve time and stuffs

  total_solve_time_ += elapsed.count();
  if (elapsed.count() > max_solve_time_) {
    max_solve_time_ = elapsed.count();
  }
  if (!result.is_success()) {
    num_failed_solve_++;
    latest_failed_solve_idx_ = counter_;
  }
  if (counter_ == 0 || past_is_left_stance_ != start_with_left_stance) {
    total_solve_time_of_first_solve_of_the_mode_ += elapsed.count();
    if (elapsed.count() > max_solve_time_of_first_solve_of_the_mode_) {
      max_solve_time_of_first_solve_of_the_mode_ = elapsed.count();
    }
    total_number_of_first_solve_of_the_mode_++;
    past_is_left_stance_ = start_with_left_stance;
  }
  cout << "\nsolve time (average, max) = " << total_solve_time_ / counter_
       << ", " << max_solve_time_ << endl;
  cout << "solve time of the first solve of the mode (average, max) = "
       << total_solve_time_of_first_solve_of_the_mode_ /
              total_number_of_first_solve_of_the_mode_
       << ", " << max_solve_time_of_first_solve_of_the_mode_ << endl;
  cout << "num_failed_solve_ = " << num_failed_solve_
       << " (latest failed index: " << latest_failed_solve_idx_
       << ", total solves = " << counter_ << ")"
       << "\n\n";
}

void CassiePlannerWithMixedRomFom::PrintAllCostsAndConstraints(
    const RomTrajOptCassie& trajopt) const {
  cout.precision(dbl::max_digits10);
  //    cout << "dbl::max_digits10 = " << dbl::max_digits10 << endl;
  // cout << "trajopt.initial_guess() = " << trajopt.initial_guess() << endl;
  // param_.PrintAll();

  auto constraints = trajopt.GetAllConstraints();
  int i = 0;
  for (auto const& binding : constraints) {
    auto const& c = binding.evaluator();
    if (c->get_description() != "rom_dyn_1_0") {
      continue;
    }
    cout << "================== i = " << i << ": ";
    std::cout << c->get_description() << std::endl;
    int n = c->num_constraints();
    VectorXd lb = c->lower_bound();
    VectorXd ub = c->upper_bound();
    VectorXd input = trajopt.GetInitialGuess(binding.variables());
    // cout << "eval point = " << input << endl;
    drake::VectorX<double> output(n);
    c.get()->Eval(input, &output);
    for (int j = 0; j < n; j++) {
      cout << lb(j) << ", " << output(j) << ", " << ub(j) << endl;
    }
    i++;
  }

  /*auto costs = trajopt.GetAllCosts();
  int i = 0;
  for (auto const& binding : costs) {
    auto const& c = binding.evaluator();
    cout << "================== i = " << i << ": ";
    std::cout << c->get_description() << std::endl;
    VectorXd input = trajopt.GetInitialGuess(binding.variables());
    //    cout << "eval point = " << input << endl;
    drake::VectorX<double> output(1);
    c.get()->Eval(input, &output);
    cout << output(0) << endl;
    i++;
  }*/
}

void CassiePlannerWithMixedRomFom::WarmStartGuess(
    const vector<VectorXd>& des_xy_pos, int global_fsm_idx,
    int first_mode_knot_idx, RomTrajOptCassie* trajopt) const {
  int starting_mode_idx_for_heuristic =
      (param_.n_step - 1) - (global_fsm_idx - prev_global_fsm_idx_) + 1;

  if (starting_mode_idx_for_heuristic <= 0) {
    PrintStatus("Set heuristic initial guess for all variables");
    // Set heuristic initial guess for all variables
    trajopt->SetHeuristicInitialGuess(
        h_guess_, y_guess_, dy_guess_, tau_guess_, x_guess_left_in_front_pre_,
        x_guess_right_in_front_pre_, x_guess_left_in_front_post_,
        x_guess_right_in_front_post_, des_xy_pos, first_mode_knot_idx, 0);
  } else {
    trajopt->SetHeuristicInitialGuess(
        h_guess_, y_guess_, dy_guess_, tau_guess_, x_guess_left_in_front_pre_,
        x_guess_right_in_front_pre_, x_guess_left_in_front_post_,
        x_guess_right_in_front_post_, des_xy_pos, first_mode_knot_idx,
        starting_mode_idx_for_heuristic);

    // Reuse the solution
    int knot_idx = first_mode_knot_idx;
    for (int i = global_fsm_idx; i < prev_global_fsm_idx_ + param_.n_step;
         i++) {
      // Global fsm and knot index pair are (i, knot_idx)
      // Local fsm index
      int local_fsm_idx = i - global_fsm_idx;
      int prev_local_fsm_idx = i - prev_global_fsm_idx_;
      while (knot_idx < param_.knots_per_mode) {
        // Local knot index
        int local_knot_idx =
            (i == global_fsm_idx) ? knot_idx - first_mode_knot_idx : knot_idx;
        int prev_local_knot_idx = (i == prev_global_fsm_idx_)
                                      ? knot_idx - prev_first_mode_knot_idx_
                                      : knot_idx;
        // Trajopt index
        int trajopt_idx = trajopt->mode_start()[local_fsm_idx] + local_knot_idx;
        int prev_trajopt_idx =
            prev_mode_start_[prev_local_fsm_idx] + prev_local_knot_idx;

        // 1. time
        if (knot_idx < param_.knots_per_mode - 1) {
          trajopt->SetInitialGuess(trajopt->timestep(trajopt_idx),
                                   h_solutions_.segment<1>(prev_trajopt_idx));
        }
        // 2. rom state (including both pre and post impact)
        trajopt->SetInitialGuess(
            trajopt->state_vars_by_mode(local_fsm_idx, local_knot_idx),
            lightweight_saved_traj_.GetStateSamples(prev_local_fsm_idx)
                .col(prev_local_knot_idx));
        // 3. rom input
        trajopt->SetInitialGuess(trajopt->input(trajopt_idx),
                                 input_at_knots_.col(prev_trajopt_idx));

        knot_idx++;
      }
      knot_idx = 0;

      // 5. FOM init
      if (local_fsm_idx == 0) {
        // Use x_init as a guess (I set it outside this warm-start function)
      }
      // 6. FOM pre-impact
      trajopt->SetInitialGuess(
          trajopt->xf_vars_by_mode(local_fsm_idx),
          lightweight_saved_traj_.get_xf().col(prev_local_fsm_idx));
      // 7. FOM post-impact
      trajopt->SetInitialGuess(
          trajopt->x0_vars_by_mode(local_fsm_idx + 1),
          lightweight_saved_traj_.get_x0().col(prev_local_fsm_idx + 1));
      // 8. FOM impulse
      trajopt->SetInitialGuess(trajopt->impulse_vars(local_fsm_idx),
                               FOM_Lambda_.col(prev_local_fsm_idx));
    }
  }
}

}  // namespace goldilocks_models
}  // namespace dairlib
