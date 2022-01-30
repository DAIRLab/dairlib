#include "examples/goldilocks_models/controller/cassie_rom_planner_system.h"

#include <math.h> /* fmod */

#include <algorithm>  // std::max
#include <fstream>
#include <iostream>
#include <limits>
#include <string>

#include "common/eigen_utils.h"
#include "examples/goldilocks_models/planning/lipm_mpc.h"
#include "examples/goldilocks_models/planning/rom_traj_opt.h"
#include "multibody/kinematic/kinematic_constraints.h"
#include "multibody/multibody_solvers.h"
#include "multibody/multibody_utils.h"
#include "solvers/optimization_utils.h"

#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/solve.h"

typedef std::numeric_limits<double> dbl;

using std::cout;
using std::endl;
using std::string;
using std::to_string;
using std::vector;

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;

using drake::multibody::Frame;
using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::solvers::Binding;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::QuadraticCost;
using drake::solvers::SolutionResult;
using drake::trajectories::ExponentialPlusPiecewisePolynomial;
using drake::trajectories::PiecewisePolynomial;

using dairlib::systems::OutputVector;
using dairlib::systems::TimestampedVector;

namespace dairlib {
namespace goldilocks_models {

CassiePlannerWithMixedRomFom::CassiePlannerWithMixedRomFom(
    const MultibodyPlant<double>& plant_control, double stride_period,
    const PlannerSetting& param, bool singel_eval_mode, bool log_data,
    int print_level)
    : nq_(plant_control.num_positions()),
      nv_(plant_control.num_velocities()),
      nx_(plant_control.num_positions() + plant_control.num_velocities()),
      plant_control_(plant_control),
      context_plant_control_(plant_control.CreateDefaultContext()),
      stride_period_(stride_period),
      single_support_duration_(param.gains.left_support_duration),
      double_support_duration_(param.gains.double_support_duration),
      left_origin_(BodyPoint(Vector3d::Zero(),
                             plant_control.GetFrameByName("toe_left"))),
      right_origin_(BodyPoint(Vector3d::Zero(),
                              plant_control.GetFrameByName("toe_right"))),
      left_mid_(BodyPoint((LeftToeFront(plant_control).first +
                           LeftToeRear(plant_control).first) /
                              2,
                          plant_control.GetFrameByName("toe_left"))),
      right_mid_(BodyPoint((LeftToeFront(plant_control).first +
                            LeftToeRear(plant_control).first) /
                               2,
                           plant_control.GetFrameByName("toe_right"))),
      param_(param),
      single_eval_mode_(singel_eval_mode),
      log_data_and_check_solution_(log_data) {
  this->set_name("planner_traj");

  DRAKE_DEMAND(param_.knots_per_mode > 0);

  // print level
  if (single_eval_mode_) print_level_ = 2;
  print_level_ = print_level;

  // Input/Output Setup
  stance_foot_port_ =
      this->DeclareVectorInputPort("stance_foot", BasicVector<double>(1))
          .get_index();
  phase_port_ =
      this->DeclareVectorInputPort("phase", BasicVector<double>(1)).get_index();
  state_port_ =
      this->DeclareVectorInputPort(
              "x, u, t", OutputVector<double>(plant_control.num_positions(),
                                              plant_control.num_velocities(),
                                              plant_control.num_actuators()))
          .get_index();
  controller_signal_port_ =
      this->DeclareVectorInputPort("ctrl_thread", TimestampedVector<double>(5))
          .get_index();
  quat_xyz_shift_port_ =
      this->DeclareVectorInputPort("quat_xyz_shift", BasicVector<double>(7))
          .get_index();
  planner_final_pos_port_ =
      this->DeclareVectorInputPort("final_pos", BasicVector<double>(2))
          .get_index();
  this->DeclareAbstractOutputPort("planner_output",
                                  &CassiePlannerWithMixedRomFom::SolveTrajOpt);

  // Create index maps
  pos_map_ = multibody::makeNameToPositionsMap(plant_control);
  vel_map_ = multibody::makeNameToVelocitiesMap(plant_control);

  // Reduced order model
  PrintEssentialStatus("model directory = " + param_.dir_model);
  rom_ = CreateRom(param_.rom_option, ROBOT, plant_control, false);
  ReadModelParameters(rom_.get(), param_.dir_model, param_.iter);

  // Create mirror maps
  state_mirror_ = StateMirror(MirrorPosIndexMap(plant_control, ROBOT),
                              MirrorPosSignChangeSet(plant_control, ROBOT),
                              MirrorVelIndexMap(plant_control, ROBOT),
                              MirrorVelSignChangeSet(plant_control, ROBOT));

  /// Provide initial guess
  n_y_ = rom_->n_y();
  n_tau_ = rom_->n_tau();
  string model_dir_n_pref = param_.dir_model + to_string(param_.iter) +
                            string("_") + to_string(param_.sample) +
                            string("_");
  h_guess_ = 1e-4 * VectorXd::Random(param_.knots_per_mode);
  y_guess_ = 1e-4 * MatrixXd::Random(n_y_, param_.knots_per_mode);
  dy_guess_ = 1e-4 * MatrixXd::Random(n_y_, param_.knots_per_mode);
  tau_guess_ = 1e-4 * MatrixXd::Random(n_tau_, param_.knots_per_mode);
  if (param_.dir_and_prefix_FOM.empty() &&
      file_exist(model_dir_n_pref + "y_samples0.csv")) {
    cout << "Construct rom regularization from ROM traj: " + model_dir_n_pref +
                "y_samples0.csv\n";
    // y_samples0 might not always exist, becasue sometimes I didn't enforce
    // that all samples have to be successful before I proceed to the next
    // iteration in model optimization.

    // Construct cubic spline from y and ydot and resample, and construct
    // first-order hold from tau and resample.
    // Note that this is an approximation. In the model optimization stage, we
    // do not construct cubic spline (for the version where we impose
    // constraint at the knot points)
    DRAKE_DEMAND(file_exist(model_dir_n_pref + "t_breaks0.csv"));
    DRAKE_DEMAND(file_exist(model_dir_n_pref + "y_samples0.csv"));
    DRAKE_DEMAND(file_exist(model_dir_n_pref + "ydot_samples0.csv"));

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
    if (std::abs(duration - stride_period) > 1e-15) {
      cout << "duration = " << duration << endl;
      cout << "stride_period = " << stride_period << endl;
      DRAKE_DEMAND(std::abs(duration - stride_period) < 1e-15);
    }
    for (int i = 0; i < param_.knots_per_mode; i++) {
      h_guess_(i) = duration / (param_.knots_per_mode - 1) * i;
      y_guess_.col(i) = y_traj.value(h_guess_(i));
      dy_guess_.col(i) = y_traj.EvalDerivative(h_guess_(i), 1);
      if (n_tau_ != 0) {
        tau_guess_.col(i) = tau_traj.value(h_guess_(i));
      }
    }
  } else {
    cout << "Construct rom regularization from ROM traj: " + model_dir_n_pref +
                "x_samples0.csv\n";
    DRAKE_DEMAND(rom_->n_tau() == 0);
    DRAKE_DEMAND(file_exist(model_dir_n_pref + "t_breaks0.csv"));
    DRAKE_DEMAND(file_exist(model_dir_n_pref + "x_samples0.csv"));
    DRAKE_DEMAND(file_exist(model_dir_n_pref + "xdot_samples0.csv"));

    PiecewisePolynomial<double> x_traj =
        PiecewisePolynomial<double>::CubicHermite(
            readCSV(model_dir_n_pref + string("t_breaks0.csv")).col(0),
            readCSV(model_dir_n_pref + string("x_samples0.csv")),
            readCSV(model_dir_n_pref + string("xdot_samples0.csv")));
    double duration = x_traj.end_time();
    DRAKE_DEMAND(std::abs(duration - stride_period) < 1e-15);
    DRAKE_DEMAND(x_traj.cols() == 1);
    DRAKE_DEMAND(x_traj.rows() == nx_);

    auto context = plant_control.CreateDefaultContext();
    for (int i = 0; i < param_.knots_per_mode; i++) {
      VectorXd x_sample =
          x_traj.value(duration / (param_.knots_per_mode - 1) * i);
      plant_control.SetPositionsAndVelocities(context.get(), x_sample);

      // I don't initialize h_guess_ here because we are not using it anyway
      y_guess_.col(i) = rom_->EvalMappingFunc(x_sample.head(nq_), *context);
      dy_guess_.col(i) = rom_->EvalMappingFuncJV(x_sample.head(nq_),
                                                 x_sample.tail(nv_), *context);
    }
  }

  // Regularization terms
  h_reg_ = h_guess_;
  y_reg_ = y_guess_;
  dy_reg_ = dy_guess_;
  tau_reg_ = tau_guess_;

  cout << "use_standing_pose_as_init_FOM_guess_ = "
       << use_standing_pose_as_init_FOM_guess_ << endl;
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
    x_standing_fixed_spring_ = VectorXd(37);
    x_standing_fixed_spring_ << 1, -2.06879e-13, -2.9985e-13, 0, 0, 0, 1,
        0.0194983, -0.0194983, 0, 0, 0.510891, 0.510891, -1.22176, -1.22176,
        1.44587, 1.44587, -1.60849, -1.60849, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0;
    // A different height
    /*x_standing_fixed_spring_ << 1, 0, 0, 0, 0, 0, 0.9, 0.0216645,
       -0.0216645, 0, 0, 0.665022, 0.665022, -1.53461, -1.53461,
       1.75905, 1.75905, -1.76295, -1.76295, 0, 0, 0, 0, 0, 0,
       0.000343939, 7.9638e-05, -0.00224901, -0.00052079, -0.000889046,
       0.00115963, -7.96077e-05, 0.000149216, 6.91934e-05, -9.6974e-05,
       0.000948192, -0.00122315;*/
    x_guess_left_in_front_pre_ = x_standing_fixed_spring_;
    x_guess_right_in_front_pre_ = x_standing_fixed_spring_;
    x_guess_left_in_front_post_ = x_standing_fixed_spring_;
    x_guess_right_in_front_post_ = x_standing_fixed_spring_;
  } else {
    string dir_and_prefix_FOM = param_.dir_and_prefix_FOM.empty()
                                    ? model_dir_n_pref
                                    : param_.dir_and_prefix_FOM;
    cout << "dir_and_prefix_FOM = " << dir_and_prefix_FOM << endl;
    VectorXd x_guess_right_in_front_pre =
        readCSV(dir_and_prefix_FOM + string("x_samples0.csv")).rightCols(1);
    VectorXd x_guess_right_in_front_post =
        readCSV(dir_and_prefix_FOM + string("x_samples1.csv")).col(0);
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
  cout << endl;

  //   cout << "initial guess duration ~ " << duration << endl;
  //   cout << "h_guess = " << h_guess_ << endl;
  //   cout << "y_guess = " << y_guess_ << endl;
  //   cout << "dy_guess = " << dy_guess_ << endl;
  //   cout << "tau_guess = " << tau_guess_ << endl;
  //    cout << "x_guess_right_in_front_pre_ = "
  //         << x_guess_right_in_front_pre_.transpose() << endl;
  //    cout << "x_guess_right_in_front_post_ = "
  //         << x_guess_right_in_front_post_.transpose() << endl;
  //    cout << "x_guess_left_in_front_pre_ = "
  //         << x_guess_left_in_front_pre_.transpose() << endl;
  //    cout << "x_guess_left_in_front_post_ = "
  //         << x_guess_left_in_front_post_.transpose() << endl;

  /// Inverse kinematics
  auto left_toe = LeftToeFront(plant_control_);
  auto left_heel = LeftToeRear(plant_control_);
  auto right_toe = RightToeFront(plant_control_);
  auto right_heel = RightToeRear(plant_control_);

  // left foot origin xy evaluators
  std::vector<int> active_dir_xy = {0, 1};
  left_foot_evaluator_ = std::make_unique<WorldPointEvaluator<double>>(
      plant_control_, Vector3d::Zero(), left_toe.second, Matrix3d::Identity(),
      Vector3d::Zero(), active_dir_xy);
  left_foot_evaluators_ =
      std::make_unique<KinematicEvaluatorSet<double>>(plant_control_);
  left_foot_evaluators_->add_evaluator(left_foot_evaluator_.get());
  // right foot origin xy evaluators
  right_foot_evaluator_ = std::make_unique<WorldPointEvaluator<double>>(
      plant_control_, Vector3d::Zero(), right_toe.second, Matrix3d::Identity(),
      Vector3d::Zero(), active_dir_xy);
  right_foot_evaluators_ =
      std::make_unique<KinematicEvaluatorSet<double>>(plant_control_);
  right_foot_evaluators_->add_evaluator(right_foot_evaluator_.get());

  // contact z evaluators
  left_toe_z_evaluator_ = std::make_unique<WorldPointEvaluator<double>>(
      plant_control_, left_toe.first, left_toe.second, Vector3d(0, 0, 1),
      Vector3d::Zero(), false);
  left_heel_z_evaluator_ = std::make_unique<WorldPointEvaluator<double>>(
      plant_control_, left_heel.first, left_heel.second, Vector3d(0, 0, 1),
      Vector3d::Zero(), false);
  right_toe_z_evaluator_ = std::make_unique<WorldPointEvaluator<double>>(
      plant_control_, right_toe.first, right_toe.second, Vector3d(0, 0, 1),
      Vector3d::Zero(), false);
  right_heel_z_evaluator_ = std::make_unique<WorldPointEvaluator<double>>(
      plant_control_, right_heel.first, right_heel.second, Vector3d(0, 0, 1),
      Vector3d::Zero(), false);
  contact_z_evaluators_ =
      std::make_unique<KinematicEvaluatorSet<double>>(plant_control_);
  contact_z_evaluators_->add_evaluator(left_toe_z_evaluator_.get());
  contact_z_evaluators_->add_evaluator(left_heel_z_evaluator_.get());
  contact_z_evaluators_->add_evaluator(right_toe_z_evaluator_.get());
  contact_z_evaluators_->add_evaluator(right_heel_z_evaluator_.get());

  // contact evaluators
  // In one case, active_dir_yz = {0, 1, 2} solves 10 times slower than {1, 2};
  std::vector<int> active_dir_yz = {1, 2};
  left_toe_evaluator_ = std::make_unique<WorldPointEvaluator<double>>(
      plant_control_, left_toe.first, left_toe.second);
  left_heel_evaluator_ = std::make_unique<WorldPointEvaluator<double>>(
      plant_control_, left_heel.first, left_heel.second, Matrix3d::Identity(),
      Vector3d::Zero(), active_dir_yz);
  right_toe_evaluator_ = std::make_unique<WorldPointEvaluator<double>>(
      plant_control_, right_toe.first, right_toe.second);
  right_heel_evaluator_ = std::make_unique<WorldPointEvaluator<double>>(
      plant_control_, right_heel.first, right_heel.second, Matrix3d::Identity(),
      Vector3d::Zero(), active_dir_yz);
  contact_evaluators_ =
      std::make_unique<KinematicEvaluatorSet<double>>(plant_control_);
  contact_evaluators_->add_evaluator(left_toe_evaluator_.get());
  contact_evaluators_->add_evaluator(left_heel_evaluator_.get());
  contact_evaluators_->add_evaluator(right_toe_evaluator_.get());
  contact_evaluators_->add_evaluator(right_heel_evaluator_.get());

  /// Get foot contacts
  Vector3d front_contact_point = left_toe.first;
  Vector3d rear_contact_point = left_heel.first;
  if (param_.use_double_contact_points) {
    auto left_toe_front = BodyPoint(front_contact_point,
                                    plant_control.GetFrameByName("toe_left"));
    auto left_toe_rear =
        BodyPoint(rear_contact_point, plant_control.GetFrameByName("toe_left"));
    auto right_toe_front = BodyPoint(front_contact_point,
                                     plant_control.GetFrameByName("toe_right"));
    auto right_toe_rear = BodyPoint(rear_contact_point,
                                    plant_control.GetFrameByName("toe_right"));
    left_contacts_.push_back(left_toe_rear);
    left_contacts_.push_back(left_toe_front);
    right_contacts_.push_back(right_toe_rear);
    right_contacts_.push_back(right_toe_front);
  } else {
    left_contacts_.push_back(left_mid_);
    right_contacts_.push_back(right_mid_);
  }

  // Get joint limits of the robot
  std::vector<string> l_r_pair = {"_left", "_right"};
  std::vector<std::string> joint_names = {
      "hip_roll", "hip_yaw", "hip_pitch", "knee", "ankle_joint", "toe"};
  for (const auto& left_right : l_r_pair) {
    for (const auto& name : joint_names) {
      joint_name_lb_ub_.emplace_back(
          name + left_right,
          plant_control.GetJointByName(name + left_right)
              .position_lower_limits()(0),
          plant_control.GetJointByName(name + left_right)
              .position_upper_limits()(0));
    }
  }

  /// MPC variables setup
  use_lipm_mpc_and_ik_ = param_.gains.use_lipm_mpc_and_ik;

  // Swing foot distance
  max_swing_distance_ = vector<double>(
      param_.n_step, param_.gains.max_foot_speed * stride_period_);

  // Time limit
  fixed_time_limit_ = param_.time_limit > 0;
  min_solve_time_preserved_for_next_loop_ =
      ((param_.n_step - 1) * stride_period) / 2;

  // Cost weight
  Q_ = param_.gains.w_Q * MatrixXd::Identity(n_y_, n_y_);
  R_ = param_.gains.w_R * MatrixXd::Identity(n_tau_, n_tau_);

  // Pick solver
  drake::solvers::SolverId solver_id("");
  solver_id = drake::solvers::IpoptSolver().id();
  PrintEssentialStatus("Solver: " + solver_id.name());
  solver_ipopt_ = drake::solvers::MakeSolver(solver_id);
  solver_id = drake::solvers::SnoptSolver().id();
  PrintEssentialStatus("Solver: " + solver_id.name());
  solver_snopt_ = drake::solvers::MakeSolver(solver_id);

  // Set solver option
  /// Ipopt
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
  /// Snopt
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

  // Allocate memory
  if (param_.zero_touchdown_impact) {
    local_Lambda_FOM_ = Eigen::MatrixXd::Zero(0, (param_.n_step));
  } else {
    local_Lambda_FOM_ =
        Eigen::MatrixXd::Zero(3 * left_contacts_.size(), param_.n_step);
  }
  global_x0_FOM_ = MatrixXd(nx_, param_.n_step + 1);
  global_xf_FOM_ = MatrixXd(nx_, param_.n_step);

  // Initialization
  prev_mode_start_ = std::vector<int>(param_.n_step, -1);

  // For warm start
  if (warm_start_with_previous_solution_ && single_eval_mode_) {
    // Only allow warm-start with init file, in order to keep the code clean.
    // If we don't use init_file for initial guess, there will be a segfault
    // from `prev_global_fsm_idx_` in WarmStartGuess()
    DRAKE_DEMAND(!param_.init_file.empty());
  }

  // Some checks
  // We hard-coded the joint index in RomTrajOptCassie::AddFomRegularizationCost
  DRAKE_DEMAND(pos_map_.at("ankle_joint_left") == 7 + 8);
  DRAKE_DEMAND(pos_map_.at("ankle_joint_right") == 7 + 9);

  /// Save data for (offline) debug mode
  writeCSV(param.dir_data + "rom_option.csv",
           param.rom_option * VectorXd::Ones(1));
  writeCSV(param.dir_data + "model_iter.csv", param.iter * VectorXd::Ones(1));
  writeCSV(param.dir_data + "sample_idx.csv", param.sample * VectorXd::Ones(1));
}

void CassiePlannerWithMixedRomFom::SolveTrajOpt(
    const Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* traj_msg) const {
  if (!single_eval_mode_) {
    PrintEssentialStatus(
        "\n================= Time = " +
        std::to_string(
            static_cast<const TimestampedVector<double>*>(
                this->EvalVectorInput(context, controller_signal_port_))
                ->get_timestamp()) +
        " =======================\n\n");
  }

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

  // Testing -- no replanning
  //  if (counter_ > 0) {
  //    *traj_msg = previous_output_msg_;
  //    return;
  //  }

  ///
  /// Read from input ports
  ///
  auto start = std::chrono::high_resolution_clock::now();

  // Read in current robot state
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  VectorXd x_init = robot_output->GetState();

  double msg_time_difference = robot_output->get_timestamp() - current_time;
  if (msg_time_difference > 0.01) {
    PrintEssentialStatus("message time difference is big: " +
                         to_string(msg_time_difference) + " ms");
  }

  // Testing
  //  x_init.segment(nq_, 3) << 0, 0, 0;

  // iter 1
  //  x_init << 1, 0, 0, 0, 0, -9.56197E-05, 0.95, -0.0773051, 0.0793097,
  //      -0.00607204, 0.00507128, 0.695605, 0.434697, -1.34401, -1.36278,
  //      1.56831,
  //      1.5871, -1.79386,
  //      -1.53291,  // q
  //      0.380501, -0.407692, -0.195629, 0.781246, 0.124929, 0.00254015,
  //      -0.525971,
  //      -0.505969, 0.226525, 0.162958, -1.27832, -1.38541, -0.292468,
  //      0.025524,
  //      0.292816, -0.0255519, 0.885733, 0.961515;

  // Testing -- start the init ROM state form the previous plan
  bool initialize_with_rom_state = false;
  VectorXd init_rom_state(2 * n_y_);
  if (initialize_with_rom_state) {
    if (counter_ == 0) {
      initialize_with_rom_state = false;
    } else {
      auto rom_traj = lightweight_saved_traj_.ConstructPositionTrajectory();
      initialize_with_rom_state = rom_traj.end_time() > current_time;
      if (initialize_with_rom_state) {
        init_rom_state << rom_traj.value(current_time),
            rom_traj.EvalDerivative(current_time, 1);
      }
    }
  }

  // Get phase in the first mode
  const BasicVector<double>* phase_port =
      this->EvalVectorInput(context, phase_port_);
  double init_phase = phase_port->get_value()(0);

  // Get stance foot
  bool is_right_stance =
      (bool)this->EvalVectorInput(context, stance_foot_port_)->get_value()(0);
  bool start_with_left_stance = !is_right_stance;

  // Get quat_xyz_shift
  VectorXd quat_xyz_shift =
      this->EvalVectorInput(context, quat_xyz_shift_port_)->get_value();

  // Get global_fsm_idx
  const BasicVector<double>* controller_signal_port =
      this->EvalVectorInput(context, controller_signal_port_);
  int global_fsm_idx = int(controller_signal_port->get_value()(2) + 1e-8);

  int fsm = (int)controller_signal_port->get_value()(0);

  // Get final position of
  VectorXd final_position =
      this->EvalVectorInput(context, planner_final_pos_port_)->get_value();
  /*cout << "in planner system: final_position = " << final_position.transpose()
       << endl;*/

  if (single_eval_mode_) {
    cout.precision(dbl::max_digits10);
    cout << "Used for the planner: \n";
    cout << "  x_init  = " << x_init.transpose() << endl;
    cout << "  current_time  = " << current_time << endl;
    cout << "  start_with_left_stance  = " << start_with_left_stance << endl;
    cout << "  init_phase  = " << init_phase << endl;
  }

  // For data logging
  string prefix = single_eval_mode_ ? "debug_" : to_string(counter_) + "_";

  ///
  /// Get desired xy position and velocity
  ///
  auto break1 = std::chrono::high_resolution_clock::now();

  // Get adjusted_final_pos
  // 1. if final_position is a constant
  //  VectorXd final_position(2);
  //  final_position << param_.final_position_x, 0;
  //  final_position << x_lift_off(pos_map_.at("base_x")) +
  //                        param_.final_position_x,
  //      0;
  //  VectorXd adjusted_final_pos = final_position * n_segment_total /
  //      (param_.n_step * (param_.knots_per_mode - 1));
  // 2 final_position is transformed from global coordinates
  vector<VectorXd> des_xy_pos;
  vector<VectorXd> des_xy_vel;
  CreateDesiredPelvisPosAndVel(param_.n_step + param_.n_step_lipm,
                               start_with_left_stance, init_phase,
                               final_position, &des_xy_pos, &des_xy_vel);
  vector<VectorXd> des_xy_pos_com;
  vector<VectorXd> des_xy_vel_com;
  CreateDesiredComPosAndVel(param_.n_step + param_.n_step_lipm,
                            start_with_left_stance, init_phase, final_position,
                            &des_xy_pos_com, &des_xy_vel_com);

  ///
  /// Use LIPM MPC and IK to get desired configuration to guide ROM MPC
  ///
  // First mode duration
  double first_mode_duration = stride_period_ * (1 - init_phase);

  bool lipm_ik_success = false;
  MatrixXd local_regularization_state(nx_, param_.n_step);
  MatrixXd local_preprocess_x_lipm;
  MatrixXd local_preprocess_u_lipm;
  if (use_lipm_mpc_and_ik_) {
    // Run LIPM MPC
    lipm_ik_success = RunLipmMPC(
        start_with_left_stance, init_phase, first_mode_duration, final_position,
        x_init, &local_preprocess_x_lipm, &local_preprocess_u_lipm);
    // Saving data for debugging
    global_preprocess_x_lipm_ = local_preprocess_x_lipm;
    global_preprocess_u_lipm_ = local_preprocess_u_lipm;
    RotatePosBetweenGlobalAndLocalFrame(false, false, quat_xyz_shift,
                                        local_preprocess_x_lipm,
                                        &global_preprocess_x_lipm_);
    RotatePosBetweenGlobalAndLocalFrame(false, true, quat_xyz_shift,
                                        local_preprocess_u_lipm,
                                        &global_preprocess_u_lipm_);
    // Run IK to get desired full state
    if (lipm_ik_success) {
      lipm_ik_success = GetDesiredFullStateFromLipmMPCSol(
          x_init, start_with_left_stance, local_preprocess_x_lipm,
          local_preprocess_u_lipm, &local_regularization_state);
    }
  }

  ///
  /// Construct regularization full-states
  ///
  // reg_x_FOM contains
  //    first_touchdown_preimpact_state, first_touchdown_postimpact_state,
  //    second_touchdown_preimpact_state, second_touchdown_postimpact_state, ...
  vector<VectorXd> reg_x_FOM(2 * param_.n_step, VectorXd(nx_));
  if (use_lipm_mpc_and_ik_ && lipm_ik_success) {
    for (int i = 0; i < param_.n_step; i++) {
      reg_x_FOM.at(2 * i) = local_regularization_state.col(i);
      reg_x_FOM.at(2 * i + 1) = local_regularization_state.col(i);
    }
  } else {
    bool left_stance = start_with_left_stance;
    for (int i = 0; i < param_.n_step; i++) {
      if (left_stance) {
        reg_x_FOM.at(2 * i) = x_guess_right_in_front_pre_;
        reg_x_FOM.at(2 * i + 1) = x_guess_right_in_front_post_;
      } else {
        reg_x_FOM.at(2 * i) = x_guess_left_in_front_pre_;
        reg_x_FOM.at(2 * i + 1) = x_guess_left_in_front_post_;
      }
      reg_x_FOM.at(2 * i).segment<2>(4) = des_xy_pos.at(i + 1);
      reg_x_FOM.at(2 * i + 1).segment<2>(4) = des_xy_pos.at(i + 1);
      reg_x_FOM.at(2 * i).segment<2>(nq_ + 3) = des_xy_vel.at(i);
      reg_x_FOM.at(2 * i + 1).segment<2>(nq_ + 3) = des_xy_vel.at(i);

      left_stance = !left_stance;
    }
  }

  ///
  /// Construct rom traj opt
  ///
  auto break2 = std::chrono::high_resolution_clock::now();

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
  PrintEssentialStatus("start_with_left_stance  = " +
                       to_string(start_with_left_stance));
  PrintEssentialStatus("init_phase  = " + to_string(init_phase));
  PrintEssentialStatus("n_knots_first_mode  = " +
                       to_string(n_knots_first_mode));
  PrintEssentialStatus("first_mode_knot_idx  = " +
                       to_string(first_mode_knot_idx));

  // Maximum swing foot travel distance
  double remaining_time_til_touchdown = first_mode_duration;
  // Update date the step length of the first mode
  // Take into account the double stance duration
  remaining_time_til_touchdown =
      std::max(0.0, remaining_time_til_touchdown - double_support_duration_);
  // Linearly decrease the max speed to 0 after mid-swing
  double max_foot_speed_first_mode =
      param_.gains.max_foot_speed *
      std::min(1.0,
               2 * remaining_time_til_touchdown / single_support_duration_);
  // We need a bit of slack because the swing foot travel distance constraint is
  // imposed on toe origin, while stance foot position stitching constraint is
  // imposed on the two contact points.
  // Ideally we should impose the travel distance constraint through the two
  // contact points, so that we don't need this artificial slack
  double slack_to_avoid_overconstraint = 0.01;  // 0.01;
  max_swing_distance_[0] =
      std::max(slack_to_avoid_overconstraint,
               max_foot_speed_first_mode * remaining_time_til_touchdown);
  PrintEssentialStatus("remaining_time_til_touchdown  = " +
                       to_string(remaining_time_til_touchdown));

  // Construct
  PrintStatus("\nConstructing optimization problem...");
  RomTrajOptCassie trajopt(
      num_time_samples, Q_, R_, *rom_, plant_control_, state_mirror_,
      left_contacts_, right_contacts_, left_origin_, right_origin_,
      joint_name_lb_ub_, x_init, init_rom_state, max_swing_distance_,
      start_with_left_stance, param_.zero_touchdown_impact, relax_index_,
      param_, initialize_with_rom_state, print_level_ > 1 /*print_status*/);

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
  for (int i = 0; i < num_time_samples.size(); i++) {
    auto xf = trajopt.xf_vars_by_mode(i);
    trajopt.AddLinearEqualityConstraint(
        Aeq_fourbar_, angle_fourbar_,
        {xf.segment<1>(pos_map_.at("knee_left")),
         xf.segment<1>(pos_map_.at("ankle_joint_left"))});
    trajopt.AddLinearEqualityConstraint(
        Aeq_fourbar_, angle_fourbar_,
        {xf.segment<1>(pos_map_.at("knee_right")),
         xf.segment<1>(pos_map_.at("ankle_joint_right"))});
  }

  // Future steps
  if (param_.n_step_lipm > 1) {
    trajopt.AddCascadedLipmMPC(
        param_.gains.w_predict_lipm_p, param_.gains.w_predict_lipm_v,
        des_xy_pos, des_xy_vel_com, param_.n_step_lipm, stride_period_,
        param_.gains.max_lipm_step_length / 2,
        param_.gains.right_limit_wrt_pelvis);
  } else {
    // TODO: check if adding predicted step is necessary after using
    //  lipm_mpc_and_ik
    PrintEssentialStatus(
        "REMINDER!! check if adding predicted step is necessary after using "
        "lipm_mpc_and_ik");

    Vector2d des_predicted_xy_vel;
    if (use_lipm_mpc_and_ik_ && lipm_ik_success) {
      des_predicted_xy_vel =
          local_preprocess_x_lipm.col(param_.n_step).tail<2>();
    } else {
      DRAKE_DEMAND(param_.n_step > 1);
      // des_xy_vel is of size n_step.
      // The predicted step (step after n_step) corresponds to
      // des_xy_vel.at(n_step), but des_xy_vel's size is not big enough to cover
      // the predicted step, so we -2. Note that, we go back 2 steps instead of
      // 1 step because velocity is asymmetric in y direction
      des_predicted_xy_vel = des_xy_vel_com.at(param_.n_step - 2);
    }

    // Constraint and cost for the last foot step location
    trajopt.AddConstraintAndCostForLastFootStep(
        param_.gains.w_predict_lipm_v, des_predicted_xy_vel, stride_period_);
  }

  // Final goal position constraint
  /*PrintStatus("Adding constraint -- FoM final position");
  trajopt.AddBoundingBoxConstraint(
      adjusted_final_position, adjusted_final_position,
      trajopt.xf_vars_by_mode(num_time_samples.size() - 1).segment(4, 2));*/

  // Add robot state in cost
  bool add_x_pose_in_cost = true;
  if (add_x_pose_in_cost) {
    trajopt.AddFomRegularizationCost(
        reg_x_FOM, param_.gains.w_reg_quat, param_.gains.w_reg_xy,
        param_.gains.w_reg_z, param_.gains.w_reg_joints,
        param_.gains.w_reg_hip_yaw, param_.gains.w_reg_xy_vel,
        param_.gains.w_reg_vel);
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
    trajopt.AddRomRegularizationCost(h_reg_, y_reg_, dy_reg_, tau_reg_,
                                     first_mode_knot_idx,
                                     param_.gains.w_rom_reg);
  }

  // Default initial guess to avoid singularity (which messes with gradient)
  for (int i = 0; i < num_time_samples.size(); i++) {
    for (int j = 0; j < num_time_samples.at(i); j++) {
      if ((param_.rom_option == 0) || (param_.rom_option == 1)) {
        trajopt.SetInitialGuess((trajopt.state_vars_by_mode(i, j))(1), 1);
      } else if ((param_.rom_option == 4) ||
                 ((param_.rom_option >= 8) && (param_.rom_option <= 17))) {
        trajopt.SetInitialGuess((trajopt.state_vars_by_mode(i, j))(2), 1);
      } else {
        DRAKE_UNREACHABLE();
      }
    }
  }

  PrintStatus("Initial guesses ===============");

  // Initial guess for all variables
  if (counter_ == 0 && !param_.init_file.empty()) {
    PrintStatus("Set initial guess from the file " + param_.init_file);
    VectorXd z0 = readCSV(param_.dir_data + param_.init_file).col(0);
    // writeCSV(param_.dir_data + "testing_" + string("init_file.csv"), z0,
    // true);
    int n_dec = trajopt.decision_variables().size();
    if (n_dec > z0.rows()) {
      PrintEssentialStatus(
          "dim(initial guess) < dim(decision var). Fill the rest with zero's.");
      VectorXd old_z0 = z0;
      z0.resize(n_dec);
      z0 = VectorXd::Zero(n_dec);
      z0.head(old_z0.rows()) = old_z0;
    } else if (n_dec < z0.rows()) {
      PrintEssentialStatus(
          "The init file is longer than the length of decision variable");
    }
    trajopt.SetInitialGuessForAllVariables(z0);
  } else {
    PrintEssentialStatus("global_fsm_idx = " + to_string(global_fsm_idx));
    if (fsm < 0 && warm_start_with_previous_solution_ && counter_ > 0) {
      trajopt.SetInitialGuessForAllVariables(z_);
    } else if (warm_start_with_previous_solution_ &&
               (prev_global_fsm_idx_ >= 0)) {
      PrintStatus("Warm start initial guess with previous solution...");
      WarmStartGuess(quat_xyz_shift, reg_x_FOM, global_fsm_idx,
                     first_mode_knot_idx, current_time, &trajopt);
    } else {
      // Set heuristic initial guess for all variables
      PrintStatus("Set heuristic initial guess...");
      trajopt.SetHeuristicInitialGuess(param_, h_guess_, y_guess_, dy_guess_,
                                       tau_guess_, reg_x_FOM,
                                       first_mode_knot_idx, 0);
    }
    trajopt.SetHeuristicInitialGuessForCascadedLipm(param_, des_xy_pos,
                                                    des_xy_vel_com);
    trajopt.SetInitialGuess(trajopt.x0_vars_by_mode(0), x_init);

    // Avoid zero-value initial guess!
    // This sped up the solve and sometimes unstuck the solver!
    const auto& all_vars = trajopt.decision_variables();
    int n_var = all_vars.size();
    VectorXd rand = 0.001 * VectorXd::Random(n_var);
    for (int i = 0; i < n_var; i++) {
      double init_guess = trajopt.GetInitialGuess(all_vars(i));
      if (init_guess == 0 || isnan(init_guess)) {
        if (print_level_ > 0)
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

  // Testing
  if (true /*single_eval_mode_*/) {
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
  if (!fixed_time_limit_ && counter_ > 0 && !single_eval_mode_) {
    // allowed time =
    //   last traj's end time - current time - time for lcm packing/traveling
    double time_limit =
        lightweight_saved_traj_.GetStateBreaks(param_.n_step - 1).tail(1)(0) -
        current_time - buffer_;
    if (global_fsm_idx == prev_global_fsm_idx_) {
      time_limit -= min_solve_time_preserved_for_next_loop_;
    }
    time_limit /= param_.realtime_rate_for_time_limit;
    if (time_limit > 0) {
      PrintEssentialStatus("Set the time limit to " + to_string(time_limit));
      solver_option_ipopt_.SetOption(drake::solvers::IpoptSolver::id(),
                                     "max_cpu_time", time_limit);
      solver_option_snopt_.SetOption(drake::solvers::SnoptSolver::id(),
                                     "Time limit", time_limit);
    } else {
      PrintEssentialStatus(
          "WARNING: Not setting time limit because it's negative.");
    }
  }

  auto break3 = std::chrono::high_resolution_clock::now();

  // Solve
  PrintEssentialStatus("\nSolving optimization problem... ");
  drake::solvers::MathematicalProgramResult result;
  //  dairlib::solvers::ResetEvalTime();
  if (param_.use_ipopt) {
    PrintEssentialStatus("(ipopt)");
    solver_ipopt_->Solve(trajopt, trajopt.initial_guess(), solver_option_ipopt_,
                         &result);
  } else {
    PrintEssentialStatus("(snopt)");
    solver_snopt_->Solve(trajopt, trajopt.initial_guess(), solver_option_snopt_,
                         &result);
  }
  auto finish = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double> elapsed_input_reading = break1 - start;
  std::chrono::duration<double> elapsed_lipm_mpc_and_ik = break2 - break1;
  std::chrono::duration<double> elapsed_trajopt_construct = break3 - break2;
  std::chrono::duration<double> elapsed_solve = finish - break3;
  if (print_level_ > 0) {
    cout << "Time for reading input ports:" << elapsed_input_reading.count()
         << "\n";
    cout << "Time for lipm mpc & IK:" << elapsed_lipm_mpc_and_ik.count()
         << "\n";
    cout << "Construction time:" << elapsed_trajopt_construct.count() << "\n";
    cout << "    Time of arrival: " << current_time << " | ";
    cout << "Solve time:" << elapsed_solve.count() << " | ";
    cout << result.get_solution_result() << " | ";
    cout << "Cost:" << result.get_optimal_cost() << "\n";
  }

  // Testing -- use different solver to test the solution quality.
  //  ResolveWithAnotherSolver(trajopt, result, prefix, current_time,
  //                           quat_xyz_shift);

  // Testing -- print all param, costs and constriants for debugging
  // PrintAllCostsAndConstraints(trajopt);

  // Testing -- store the initial guess to the result (to visualize init guess)
  if (single_eval_mode_) {
    /*cout << "***\n*** WARNING: set the solution to be initial guess\n***\n";
    result.set_x_val(trajopt.initial_guess());*/
  }

  // TODO(yminchen): Note that you will to rotate the coordinates back if the
  //  ROM is dependent on robot's x, y and yaw.

  ///
  /// Pack traj into lcm message (traj_msg)
  ///
  // Note that the trajectory is discontinuous between mode (even the position
  // jumps because left vs right stance leg).

  // Rotate the local state to global state
  MatrixXd local_x0_FOM(nx_, trajopt.num_modes() + 1);
  MatrixXd local_xf_FOM(nx_, trajopt.num_modes());
  for (int i = 0; i < param_.n_step; ++i) {
    local_x0_FOM.col(i) = result.GetSolution(trajopt.x0_vars_by_mode(i));
    local_xf_FOM.col(i) = result.GetSolution(trajopt.xf_vars_by_mode(i));
  }
  local_x0_FOM.col(param_.n_step) =
      result.GetSolution(trajopt.x0_vars_by_mode(param_.n_step));
  global_x0_FOM_ = local_x0_FOM;
  global_xf_FOM_ = local_xf_FOM;
  RotateBetweenGlobalAndLocalFrame(false, quat_xyz_shift, local_x0_FOM,
                                   &global_x0_FOM_);
  RotateBetweenGlobalAndLocalFrame(false, quat_xyz_shift, local_xf_FOM,
                                   &global_xf_FOM_);

  // Unit Testing RotateBetweenGlobalAndLocalFrame
  /*MatrixXd local_x0_FOM2 = global_x0_FOM_;
  MatrixXd local_xf_FOM2 = global_xf_FOM_;
  RotateBetweenGlobalAndLocalFrame(true, quat_xyz_shift, global_x0_FOM_,
                                   &local_x0_FOM2);
  RotateBetweenGlobalAndLocalFrame(true, quat_xyz_shift, global_xf_FOM_,
                                   &local_xf_FOM2);
  DRAKE_DEMAND((local_x0_FOM2 - local_x0_FOM).norm() < 1e-14);
  DRAKE_DEMAND((local_xf_FOM2 - local_xf_FOM).norm() < 1e-14);*/

  // TODO: maybe I should not assign the new desired traj to controller thread
  //  when the solver didn't find optimal solution (unless it's going to run out
  //  of traj to use)? We should just use the old previous_output_msg_

  // Benchmark: for n_step = 3, the packing time is about 60us and the message
  // size is about 4.5KB (use WriteToFile() to check).
  lightweight_saved_traj_ =
      RomPlannerTrajectory(trajopt, result, global_x0_FOM_, global_xf_FOM_,
                           prefix, "", true, current_time);
  *traj_msg = lightweight_saved_traj_.GenerateLcmObject();
  //  PrintTrajMsg(traj_msg);

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
    local_Lambda_FOM_.col(i) = result.GetSolution(trajopt.impulse_vars(i));
  }

  eps_rom_ = result.GetSolution(trajopt.eps_rom_var_);
  if (param_.n_step_lipm > 1) {
    local_x_lipm_ =
        Eigen::Map<MatrixXd>(result.GetSolution(trajopt.x_lipm_vars_).data(), 4,
                             param_.n_step_lipm + 1);
    local_u_lipm_ = Eigen::Map<MatrixXd>(
        result.GetSolution(trajopt.u_lipm_vars_).data(), 2, param_.n_step_lipm);
  } else {
    local_predicted_com_vel_ =
        result.GetSolution(trajopt.predicted_com_vel_var_);
  }

  // For hardware -- before we switch from standing to MPC walking
  if (fsm < 0) {
    z_ = result.GetSolution();
  }

  prev_global_fsm_idx_ = global_fsm_idx;
  prev_first_mode_knot_idx_ = first_mode_knot_idx;
  prev_mode_start_ = trajopt.mode_start();

  // Transform some solutions into global frame
  if (param_.n_step_lipm > 1) {
    global_x_lipm_ = local_x_lipm_;
    global_u_lipm_ = local_u_lipm_;
    RotatePosBetweenGlobalAndLocalFrame(false, false, quat_xyz_shift,
                                        local_x_lipm_, &global_x_lipm_);
    RotatePosBetweenGlobalAndLocalFrame(false, true, quat_xyz_shift,
                                        local_u_lipm_, &global_u_lipm_);
  }

  ///
  /// For debugging
  ///
  start = std::chrono::high_resolution_clock::now();

  if (param_.log_solver_info && param_.use_ipopt) {
    // Ipopt doesn't seem to have the append feature, so we do it manually
    (void)std::system(
        "cat ../ipopt_planning_latest.out >> ../ipopt_planning_combined.out");
  }

  if (log_data_and_check_solution_) {
    // Rotate from local to global
    MatrixXd global_regularization_state(nx_, param_.n_step + 1);
    MatrixXd local_regu_state_augmented(nx_, param_.n_step + 1);
    local_regu_state_augmented.leftCols<1>() = x_init;
    for (int i = 0; i < param_.n_step; i++) {
      // We only save preimpact (since we assume 0 impact anyway)
      local_regu_state_augmented.middleCols<1>(1 + i) = reg_x_FOM.at(2 * i);
    }
    global_regularization_state = local_regu_state_augmented;
    RotateBetweenGlobalAndLocalFrame(false, quat_xyz_shift,
                                     local_regu_state_augmented,
                                     &global_regularization_state);

    // Extract and save solution into files (for debugging)
    SaveDataIntoFiles(current_time, global_fsm_idx, x_init, init_phase,
                      is_right_stance, quat_xyz_shift, final_position,
                      global_regularization_state, local_x0_FOM, local_xf_FOM,
                      trajopt, result, param_.dir_data, prefix);
    // Save trajectory to lcm
    SaveTrajIntoLcmBinary(trajopt, result, global_x0_FOM_, global_xf_FOM_,
                          param_.dir_data, prefix, current_time);

    // Check the cost
    PrintCost(trajopt, result);

    // Check constraint violation
    if (!result.is_success()) {
      //    double tol = 1e-3;
      double tol = param_.feas_tol;
      solvers::CheckGenericConstraints(trajopt, result, tol);
    }
  }

  // Keep track of solve time and stuffs
  BookKeeping(start_with_left_stance, elapsed_lipm_mpc_and_ik, elapsed_solve,
              result);

  // Switch to snopt after one iteration (use ipopt to get a good solution for
  // the first loop)
  if (counter_ == 0) {
    if (param_.switch_to_snopt_after_first_loop) {
      PrintEssentialStatus("***\n*** WARNING: switch to snopt solver\n***");
      param_.use_ipopt = false;
    }
  }

  finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  PrintEssentialStatus("Runtime for data saving (for debugging):" +
                       to_string(elapsed.count()));

  /// Some checks
  if (counter_ == 0 && completely_use_trajs_from_model_opt_as_target_) {
    // The cost shouldn't be too big in the first solve
    if (result.get_optimal_cost() > 5e-3) {
      cout << "\n\nWARNING: Cost shouldn't be too big in the first solve\n\n";
    }
    //    DRAKE_DEMAND(result.get_optimal_cost() < 1e-2);
  }

  ///
  counter_++;
}

bool CassiePlannerWithMixedRomFom::RunLipmMPC(
    bool start_with_left_stance, double init_phase, double first_mode_duration,
    const VectorXd& final_position, const VectorXd& x_init,
    MatrixXd* local_preprocess_x_lipm,
    MatrixXd* local_preprocess_u_lipm) const {
  cout << "=== start of LIPM MPC ===\n";
  auto start_build = std::chrono::high_resolution_clock::now();

  // Parameters
  double target_height = 0.85;
  // Long horizon
  int minimum_n_step = 10;
  double max_length_foot_to_body = 0.3;  // speed = 2 * foot_to_body / time
  // The body cannot be too forward. Otherwise it hits the toe joint limit.
  double max_length_foot_to_body_front = 0.3;  // 0.3

  double first_mode_duration_lipm = first_mode_duration;
  double stride_period_lipm = stride_period_;
  // Heuristic
  // Removing double support phase produces better foot step when the target
  // position is far. However, in this case, ROM MPC's first foot step
  // prediction would be different from LIPM MPC's.
  bool remove_double_support_druation_in_first_mode = true;
  if (remove_double_support_druation_in_first_mode) {
    // Use remaining time until touchdown
    first_mode_duration_lipm =
        std::max(0.0, first_mode_duration_lipm - double_support_duration_);
  }
  bool remove_double_support_druation_after_first_mode = true;
  if (remove_double_support_druation_after_first_mode) {
    // Ignore double support duration
    stride_period_lipm = single_support_duration_;
  }
  /*cout << "first_mode_duration_lipm = " << first_mode_duration_lipm << endl;
  cout << "stride_period_lipm = " << stride_period_lipm << endl;*/
  // TODO: With the above heuristic, it seems to underestimate the prediction.
  //  However, after removing it, the stride druation is larger which cause
  //  bigger stride length, and we need to relax the stride length constraint in
  //  LIPM MPC to make the problem feasible.
  //  We need to add soft constraint for the stride length (in the cost), so we
  //  can at least get a solution.
  //  Additionally, the big stride length caused instability.

  // +1 because IK needs swing ft
  int n_step = std::max(minimum_n_step, param_.n_step + 1);

  // Get des_xy_pos, des_xy_vel
  vector<VectorXd> des_xy_pos_com;
  vector<VectorXd> des_xy_vel_com;
  CreateDesiredComPosAndVel(n_step, start_with_left_stance, init_phase,
                            final_position, &des_xy_pos_com, &des_xy_vel_com);

  // Get initial COM position, velocity and stance foot position
  plant_control_.SetPositions(context_plant_control_.get(), x_init.head(nq_));
  // CoM position
  Vector3d com_pos =
      plant_control_.CalcCenterOfMassPositionInWorld(*context_plant_control_);
  // CoM velocity
  Eigen::MatrixXd J_com(3, plant_control_.num_velocities());
  plant_control_.CalcJacobianCenterOfMassTranslationalVelocity(
      *context_plant_control_, JacobianWrtVariable::kV,
      plant_control_.world_frame(), plant_control_.world_frame(), &J_com);
  Vector2d com_vel = J_com.topRows<2>() * x_init.tail(nv_);
  // Stance foot position
  auto stance_toe_mid = start_with_left_stance ? left_mid_ : right_mid_;
  VectorXd stance_foot_pos(3);
  plant_control_.CalcPointsPositions(
      *context_plant_control_, stance_toe_mid.second, stance_toe_mid.first,
      plant_control_.world_frame(), &stance_foot_pos);

  // Height vector
  vector<double> height_vec(n_step);
  height_vec.at(0) = com_pos(2);
  double delta = (target_height - com_pos(2)) / (n_step - 1);
  for (int i = 1; i < n_step; i++) {
    height_vec.at(i) = height_vec.at(i - 1) + delta;
  }
  // DRAKE_DEMAND(abs(height_vec.at(n_step - 1) - target_height) < 1e-14);

  // Construct LIPM MPC (QP)
  // TODO: need to add constraint for the swing foot travel distance in the
  //  first mode
  cout << "Reminder here.\n";
  LipmMpc mpc_qp(
      des_xy_pos_com, des_xy_vel_com, param_.gains.w_p_lipm_mpc,
      param_.gains.w_v_lipm_mpc, com_pos.head<2>(), com_vel,
      stance_foot_pos.head<2>(), n_step, first_mode_duration_lipm,
      stride_period_lipm, height_vec,
      std::min(max_length_foot_to_body, param_.gains.max_lipm_step_length / 2),
      max_length_foot_to_body_front, param_.gains.right_limit_wrt_pelvis,
      start_with_left_stance);

  // Set initial guess (warm start after the first loop)
  // TODO: use global_preprocess_x_lipm_ and global_preprocess_u_lipm_
  //  Update -- Currently the speed is fast enough, so it's probably uncessary.

  // Solve
  auto start_solve = std::chrono::high_resolution_clock::now();
  const auto result = qp_solver_.Solve(mpc_qp, mpc_qp.initial_guess());
  auto finish = std::chrono::high_resolution_clock::now();

  // Get solution
  *local_preprocess_x_lipm = mpc_qp.GetStateSamples(result);
  *local_preprocess_u_lipm = mpc_qp.GetInputSamples(result);

  // Printing
  // Normally the whole thing takes 1ms without a initial guess
  std::chrono::duration<double> elapsed_build = start_solve - start_build;
  std::chrono::duration<double> elapsed_solve = finish - start_solve;
  SolutionResult solution_result = result.get_solution_result();
  cout << "Solver:" << result.get_solver_id().name() << " | ";
  cout << "Build time:" << elapsed_build.count() << " | ";
  cout << "Solve time:" << elapsed_solve.count() << " | ";
  cout << solution_result << " | ";
  cout << "Cost:" << result.get_optimal_cost() << "\n";

  double lipm_p_cost =
      solvers::EvalCostGivenSolution(result, mpc_qp.lipm_p_bindings_);
  if (lipm_p_cost > 0) {
    cout << "lipm_p_cost = " << lipm_p_cost << endl;
  }
  double lipm_v_cost =
      solvers::EvalCostGivenSolution(result, mpc_qp.lipm_v_bindings_);
  if (lipm_v_cost > 0) {
    cout << "lipm_v_cost = " << lipm_v_cost << endl;
  }

  cout << "==== end of LIPM MPC ====\n";
  if (!result.is_success()) {
    //    DRAKE_DEMAND(result.is_success());
    cout << "\n!!!\n !!! WARNING: LIPM MPC failed!\n !!!\n\n";
  }
  return result.is_success();
}

bool CassiePlannerWithMixedRomFom::GetDesiredFullStateFromLipmMPCSol(
    const Eigen::VectorXd& x_init, bool start_with_left_stance,
    const Eigen::MatrixXd& local_preprocess_x_lipm,
    const Eigen::MatrixXd& local_preprocess_u_lipm,
    MatrixXd* regularization_state) const {
  cout << "=== start of IK ===\n";
  bool success = true;

  // Parameter
  // TODO: desired_quat should point towards goal position. Same for ROM MPC.
  Vector4d desired_quat(1, 0, 0, 0);
  //  double desired_height = 1;
  double desired_height = x_init(6);  // for testing
  double time_limit = 0.2;            // 0.02;  // in seconds
  bool include_velocity = false;

  double scale = 1;
  double w_q_reg = 0.0001 * scale;
  double w_v_reg = 0.0001 * scale;
  double w_pelvis_z = 1 * scale;
  double w_hip_yaw = 1 * scale;

  bool left_stance = start_with_left_stance;
  for (int i = 0; i < regularization_state->cols(); i++) {
    auto start_build = std::chrono::high_resolution_clock::now();

    auto prog = multibody::MultibodyProgram(plant_control_);
    auto q = prog.AddPositionVariables();
    drake::solvers::VectorXDecisionVariable v;
    if (include_velocity) {
      v = prog.NewContinuousVariables(nv_, "v");
    }

    // Bounding box for quaternion
    prog.AddBoundingBoxConstraint(0, 1, q(0));              // qw
    prog.AddBoundingBoxConstraint(-1, 1, q.segment<3>(1));  // qx, qy, qz

    // Quaternion unit norm constraint (it solves faster with this constraint)
    //    auto quat_norm_constraint =
    //        std::make_shared<drake::solvers::QuadraticConstraint>(
    //            2 * MatrixXd::Identity(4, 4), VectorXd::Zero(4), 1, 1);
    //    prog.AddConstraint(quat_norm_constraint, q.head(4));

    // Joint limit constraint
    prog.AddJointLimitConstraints(q);

    // Four bar linkage constraint (without spring)
    prog.AddLinearEqualityConstraint(
        Aeq_fourbar_, angle_fourbar_,
        {q.segment<1>(pos_map_.at("knee_left")),
         q.segment<1>(pos_map_.at("ankle_joint_left"))});
    prog.AddLinearEqualityConstraint(
        Aeq_fourbar_, angle_fourbar_,
        {q.segment<1>(pos_map_.at("knee_right")),
         q.segment<1>(pos_map_.at("ankle_joint_right"))});

    // Feet xy position
    VectorXd left_ft_pos = left_stance ? local_preprocess_u_lipm.col(i)
                                       : local_preprocess_u_lipm.col(i + 1);
    VectorXd right_ft_pos = left_stance ? local_preprocess_u_lipm.col(i + 1)
                                        : local_preprocess_u_lipm.col(i);
    auto left_ft_pos_constraint =
        std::make_shared<multibody::KinematicPositionConstraint<double>>(
            plant_control_, *left_foot_evaluators_, left_ft_pos, left_ft_pos,
            std::set<int>(), prog.get_context());
    auto left_ft_pos_binding = prog.AddConstraint(left_ft_pos_constraint, q);
    auto right_ft_pos_constraint =
        std::make_shared<multibody::KinematicPositionConstraint<double>>(
            plant_control_, *right_foot_evaluators_, right_ft_pos, right_ft_pos,
            std::set<int>(), prog.get_context());
    auto right_ft_pos_binding = prog.AddConstraint(right_ft_pos_constraint, q);

    // Zero foot height
    auto foot_height_binding =
        prog.AddKinematicConstraint(*contact_z_evaluators_, q);

    if (include_velocity) {
      // Zero velocity on feet
      // Interestingly the speed is much slower when getting rid of redundancy
      auto foot_vel_constraint =
          std::make_shared<multibody::KinematicVelocityConstraint<double>>(
              plant_control_, *contact_evaluators_, prog.get_context());
      auto velocity_binding = prog.AddConstraint(foot_vel_constraint, {q, v});

      // Bounding box for vel
      prog.AddBoundingBoxConstraint(-10, 10, v);

      //    prog.AddBoundingBoxConstraint(0, 0, v(vel_map_.at("base_wx")));
      //    prog.AddBoundingBoxConstraint(0, 0, v(vel_map_.at("base_wy")));
      //    prog.AddBoundingBoxConstraint(0, 0, v(vel_map_.at("base_wz")));

      prog.AddBoundingBoxConstraint(
          local_preprocess_x_lipm.col(i + 1).segment<2>(2),
          local_preprocess_x_lipm.col(i + 1).segment<2>(2),
          v.segment<2>(vel_map_.at("base_vx")));
      prog.AddBoundingBoxConstraint(0, 0, v(vel_map_.at("base_vz")));
    }

    // Add costs
    // Regularization term
    auto q_cost_binding = prog.AddQuadraticErrorCost(
        w_q_reg * MatrixXd::Identity(nq_ - 7, nq_ - 7),
        x_standing_fixed_spring_.tail(nq_ - 7), q.tail(nq_ - 7));
    std::unique_ptr<Binding<QuadraticCost>> v_cost_binding;
    if (include_velocity) {
      v_cost_binding =
          std::make_unique<Binding<QuadraticCost>>(prog.AddQuadraticErrorCost(
              w_v_reg * MatrixXd::Identity(nv_, nv_), VectorXd::Zero(nv_), v));
    }

    // Quaternion
    prog.AddBoundingBoxConstraint(desired_quat, desired_quat, q.head<4>());
    //    auto quat_cost_binding = prog.AddQuadraticErrorCost(
    //        0.01 * MatrixXd::Identity(4, 4), desired_quat, q.tail<4>());

    // pelvis xy
    prog.AddBoundingBoxConstraint(local_preprocess_x_lipm.col(i + 1).head<2>(),
                                  local_preprocess_x_lipm.col(i + 1).head<2>(),
                                  q.segment<2>(4));
    //    auto pelvis_xy_cost_binding = prog.AddQuadraticErrorCost(
    //        1 * MatrixXd::Identity(2, 2),
    //        local_preprocess_x_lipm.col(i + 1).head<2>(), q.segment<2>(4));

    // pelvis z
    //    prog.AddBoundingBoxConstraint(desired_height, desired_height,
    //    q(pos_map_.at("base_z")));
    auto height_cost_binding = prog.AddQuadraticErrorCost(
        w_pelvis_z * MatrixXd::Identity(1, 1),
        desired_height * VectorXd::Ones(1), q.segment<1>(6));

    // Hip yaw
    //    prog.AddBoundingBoxConstraint(Vector2d::Zero(), Vector2d::Zero(),
    //                                  q.segment<2>(9));
    auto left_hip_cost_binding =
        prog.AddQuadraticErrorCost(w_hip_yaw * MatrixXd::Identity(1, 1),
                                   VectorXd::Zero(1), q.segment<1>(9));
    auto right_hip_cost_binding =
        prog.AddQuadraticErrorCost(w_hip_yaw * MatrixXd::Identity(1, 1),
                                   VectorXd::Zero(1), q.segment<1>(10));

    // Initial guesses
    cout << "REMINDER!! We should warm start IK with previous solution if mode "
            "stay the same. Also, warm start only if previous solve is "
            "successful \n";
    prog.SetInitialGuessForAllVariables(
        0.01 * Eigen::VectorXd::Random(prog.num_vars()));
    prog.SetInitialGuess(q, x_standing_fixed_spring_.head(nq_));
    //    prog.SetInitialGuess(v, v_desired);

    // Snopt settings
    /*prog.SetSolverOption(drake::solvers::SnoptSolver::id(), "Print file",
                         "../snopt_test.out");*/
    prog.SetSolverOption(drake::solvers::SnoptSolver::id(), "Verify level", 0);
    prog.SetSolverOption(drake::solvers::SnoptSolver::id(),
                         "Major optimality tolerance", 1e-2);
    prog.SetSolverOption(drake::solvers::SnoptSolver::id(),
                         "Major feasibility tolerance", 1e-2);
    prog.SetSolverOption(drake::solvers::SnoptSolver::id(), "Time limit",
                         time_limit);

    auto start_solve = std::chrono::high_resolution_clock::now();
    const auto result = snopt_solver_.Solve(prog, prog.initial_guess());
    auto finish = std::chrono::high_resolution_clock::now();

    // Assign
    if (include_velocity) {
      regularization_state->col(i) << result.GetSolution(q),
          result.GetSolution(v);
    } else {
      regularization_state->col(i) << result.GetSolution(q), 0, 0, 0,
          local_preprocess_x_lipm.col(i + 1).segment<2>(2),
          VectorXd::Zero(nv_ - 5);
    }
    left_stance = !left_stance;

    // Print
    std::chrono::duration<double> elapsed_build = start_solve - start_build;
    std::chrono::duration<double> elapsed_solve = finish - start_solve;
    SolutionResult solution_result = result.get_solution_result();
    cout << "Solver:" << result.get_solver_id().name() << " | ";
    cout << "Build time:" << elapsed_build.count() << " | ";
    cout << "Solve time:" << elapsed_solve.count() << " | ";
    cout << solution_result << " | ";
    cout << "Cost:" << result.get_optimal_cost() << "\n";

    //    cout << "q = " << regularization_state->col(i).head(nq_) <<
    //    endl; cout << "v = " <<
    //    regularization_state->col(i).tail(nv_) << endl;
    cout << "height  = " << regularization_state->col(i)(6) << endl;
    cout << "height_cost_binding = "
         << solvers::EvalCostGivenSolution(result, height_cost_binding) << endl;
    cout << "left_hip_cost_binding = "
         << solvers::EvalCostGivenSolution(result, left_hip_cost_binding)
         << endl;
    cout << "right_hip_cost_binding = "
         << solvers::EvalCostGivenSolution(result, right_hip_cost_binding)
         << endl;
    /*cout << "quat_cost_binding = "
         << solvers::EvalCostGivenSolution(result, quat_cost_binding) << endl;*/
    cout << "q_cost_binding = "
         << solvers::EvalCostGivenSolution(result, q_cost_binding) << endl;
    if (include_velocity) {
      cout << "v_cost_binding = "
           << solvers::EvalCostGivenSolution(result, *v_cost_binding) << endl;
    }

    // Exiting
    if (!result.is_success()) {
      // DRAKE_DEMAND(result.is_success());
      cout << "\n!!!\n !!! WARNING: IK failed!\n !!!\n\n";
    }
    success = success && result.is_success();
  }
  cout << "==== end of IK ====\n";

  return success;
}

void CassiePlannerWithMixedRomFom::CreateDesiredPelvisPosAndVel(
    int n_total_step, bool start_with_left_stance, double init_phase,
    const VectorXd& final_position, vector<VectorXd>* des_xy_pos,
    vector<VectorXd>* des_xy_vel) const {
  CreateDesiredBodyPosAndVel(true, n_total_step, start_with_left_stance,
                             init_phase, final_position, des_xy_pos,
                             des_xy_vel);
}
void CassiePlannerWithMixedRomFom::CreateDesiredComPosAndVel(
    int n_total_step, bool start_with_left_stance, double init_phase,
    const VectorXd& final_position, vector<VectorXd>* des_xy_pos,
    vector<VectorXd>* des_xy_vel) const {
  CreateDesiredBodyPosAndVel(false, n_total_step, start_with_left_stance,
                             init_phase, final_position, des_xy_pos,
                             des_xy_vel);
}
void CassiePlannerWithMixedRomFom::CreateDesiredBodyPosAndVel(
    bool pelvis_or_com, int n_total_step, bool start_with_left_stance,
    double init_phase, const VectorXd& final_position,
    vector<VectorXd>* des_xy_pos, vector<VectorXd>* des_xy_vel) const {
  // Parameters
  double y_vel_offset = 0.1;  // This affects the foot spread

  double total_phase_length = n_total_step - init_phase;

  Vector2d adjusted_final_pos = final_position;
  double pos_diff_norm = adjusted_final_pos.norm();
  double max_pos_diff_norm =
      std::abs(param_.gains.max_desired_step_length * total_phase_length);
  if (pos_diff_norm > max_pos_diff_norm) {
    adjusted_final_pos *= max_pos_diff_norm / pos_diff_norm;
  }

  // Get the desired xy positions for the FOM states
  *des_xy_pos = vector<VectorXd>(n_total_step + 1, VectorXd::Zero(2));
  des_xy_pos->at(1) = des_xy_pos->at(0) + adjusted_final_pos *
                                              (1 - init_phase) /
                                              total_phase_length;
  for (int i = 2; i < des_xy_pos->size(); i++) {
    des_xy_pos->at(i) =
        des_xy_pos->at(i - 1) + adjusted_final_pos / total_phase_length;
  }

  // Get the desired xy velocities for the FOM states
  if (completely_use_trajs_from_model_opt_as_target_) {
    if (pelvis_or_com) {
      *des_xy_vel = vector<VectorXd>(
          n_total_step, Vector2d(x_guess_left_in_front_post_(nq_ + 3), 0));
      y_vel_offset = x_guess_left_in_front_post_(nq_ + 4);
    } else {
      plant_control_.SetPositionsAndVelocities(context_plant_control_.get(),
                                               x_guess_left_in_front_post_);
      Vector3d com_vel =
          plant_control_.CalcCenterOfMassTranslationalVelocityInWorld(
              *context_plant_control_);
      *des_xy_vel = vector<VectorXd>(n_total_step, Vector2d(com_vel(0), 0));
      y_vel_offset = com_vel(1);
    }
  } else {
    // Use the average vel for the vel at hybrid event (which underestimates)
    *des_xy_vel = vector<VectorXd>(
        n_total_step,
        adjusted_final_pos / (stride_period_ * total_phase_length));
  }
  // Heuristically shift the desired velocity in y direction
  bool dummy_bool = start_with_left_stance;
  for (int i = 0; i < n_total_step; i++) {
    if (dummy_bool) {
      des_xy_vel->at(i)(1) -= y_vel_offset;
    } else {
      des_xy_vel->at(i)(1) += y_vel_offset;
    }
    dummy_bool = !dummy_bool;
  }

  // Check and print
  DRAKE_DEMAND((des_xy_pos->back() - adjusted_final_pos).norm() < 1e-14);
  /*cout << "des_xy_pos = \n";
  for (int i = 0; i < des_xy_pos->size(); i++) {
    cout << des_xy_pos->at(i).transpose() << endl;
  }
  cout << "des_xy_vel = \n";
  for (int i = 0; i < des_xy_vel->size(); i++) {
    cout << des_xy_vel->at(i).transpose() << endl;
  }*/
}

void CassiePlannerWithMixedRomFom::RotateBetweenGlobalAndLocalFrame(
    bool rotate_from_global_to_local, const VectorXd& quat_xyz_shift,
    const MatrixXd& original_x_FOM, MatrixXd* rotated_x_FOM) const {
  Quaterniond relative_quat =
      rotate_from_global_to_local
          ? Quaterniond(quat_xyz_shift(0), quat_xyz_shift(1), quat_xyz_shift(2),
                        quat_xyz_shift(3))
          : Quaterniond(quat_xyz_shift(0), quat_xyz_shift(1), quat_xyz_shift(2),
                        quat_xyz_shift(3))
                .conjugate();
  Matrix3d relative_rot_mat = relative_quat.toRotationMatrix();
  double sign = rotate_from_global_to_local ? 1 : -1;
  for (int j = 0; j < original_x_FOM.cols(); j++) {
    Quaterniond rotated_x_quat =
        relative_quat *
        Quaterniond(original_x_FOM.col(j)(0), original_x_FOM.col(j)(1),
                    original_x_FOM.col(j)(2), original_x_FOM.col(j)(3));
    rotated_x_FOM->col(j).segment<4>(0) << rotated_x_quat.w(),
        rotated_x_quat.vec();
    if (rotate_from_global_to_local) {
      rotated_x_FOM->col(j).segment<3>(4)
          << relative_rot_mat * (original_x_FOM.col(j).segment<3>(4) +
                                 sign * quat_xyz_shift.segment<3>(4));
    } else {
      rotated_x_FOM->col(j).segment<3>(4)
          << relative_rot_mat * original_x_FOM.col(j).segment<3>(4) +
                 sign * quat_xyz_shift.segment<3>(4);
    }
    rotated_x_FOM->col(j).segment<3>(nq_)
        << relative_rot_mat * original_x_FOM.col(j).segment<3>(nq_);
    rotated_x_FOM->col(j).segment<3>(nq_ + 3)
        << relative_rot_mat * original_x_FOM.col(j).segment<3>(nq_ + 3);
  }
}

void CassiePlannerWithMixedRomFom::RotatePosBetweenGlobalAndLocalFrame(
    bool rotate_from_global_to_local, bool position_only,
    const Eigen::VectorXd& quat_xyz_shift, const Eigen::MatrixXd& original_x,
    Eigen::MatrixXd* rotated_x) const {
  DRAKE_DEMAND(quat_xyz_shift(1) < 1e-14);  // rotate in z axis
  DRAKE_DEMAND(quat_xyz_shift(2) < 1e-14);  // rotate in z axis
  Quaterniond relative_quat =
      rotate_from_global_to_local
          ? Quaterniond(quat_xyz_shift(0), quat_xyz_shift(1), quat_xyz_shift(2),
                        quat_xyz_shift(3))
          : Quaterniond(quat_xyz_shift(0), quat_xyz_shift(1), quat_xyz_shift(2),
                        quat_xyz_shift(3))
                .conjugate();
  Matrix2d relative_rot_mat =
      relative_quat.toRotationMatrix().topLeftCorner<2, 2>();
  double sign = rotate_from_global_to_local ? 1 : -1;
  for (int j = 0; j < original_x.cols(); j++) {
    // position
    if (rotate_from_global_to_local) {
      rotated_x->col(j).head<2>()
          << relative_rot_mat * (original_x.col(j).head<2>() +
                                 sign * quat_xyz_shift.segment<2>(4));
    } else {
      rotated_x->col(j).head<2>()
          << relative_rot_mat * original_x.col(j).head<2>() +
                 sign * quat_xyz_shift.segment<2>(4);
    }
    // velocity
    if (!position_only) {
      rotated_x->col(j).tail<2>()
          << relative_rot_mat * original_x.col(j).tail<2>();
    }
  }
}

void CassiePlannerWithMixedRomFom::SaveTrajIntoLcmBinary(
    const RomTrajOptCassie& trajopt, const MathematicalProgramResult& result,
    const MatrixXd& global_x0_FOM, const MatrixXd& global_xf_FOM,
    const string& dir_data, const string& prefix, double current_time) const {
  string file_name = prefix + "rom_trajectory";
  RomPlannerTrajectory saved_traj(
      trajopt, result, global_x0_FOM, global_xf_FOM, file_name,
      drake::solvers::to_string(result.get_solution_result()), false,
      current_time);
  saved_traj.WriteToFile(dir_data + file_name);
  cout << "Wrote to file: " << dir_data + file_name << endl;
}

void CassiePlannerWithMixedRomFom::SaveDataIntoFiles(
    double current_time, int global_fsm_idx, const VectorXd& x_init,
    double init_phase, bool is_right_stance, const VectorXd& quat_xyz_shift,
    const VectorXd& final_position, const MatrixXd& global_regularization_x_FOM,
    const MatrixXd& local_x0_FOM, const MatrixXd& local_xf_FOM,
    const RomTrajOptCassie& trajopt, const MathematicalProgramResult& result,
    const string& dir_data, const string& prefix) const {
  string dir_pref = dir_data + prefix;

  /// Save the solution vector
  writeCSV(dir_pref + "z.csv",
           result.GetSolution(trajopt.decision_variables()));
  // cout << trajopt.decision_variables() << endl;

  /// Save traj to csv
  for (int i = 0; i < param_.n_step; i++) {
    writeCSV(dir_pref + "time_at_knots" + to_string(i) + ".csv",
             lightweight_saved_traj_.GetStateBreaks(i));
    writeCSV(dir_pref + "state_at_knots" + to_string(i) + ".csv",
             lightweight_saved_traj_.GetStateSamples(i));
  }
  writeCSV(dir_pref + "input_at_knots.csv", trajopt.GetInputSamples(result));

  writeCSV(dir_pref + "local_x0_FOM.csv", local_x0_FOM);
  writeCSV(dir_pref + "local_xf_FOM.csv", local_xf_FOM);
  writeCSV(dir_pref + "global_x0_FOM.csv", lightweight_saved_traj_.get_x0());
  writeCSV(dir_pref + "global_xf_FOM.csv", lightweight_saved_traj_.get_xf());
  writeCSV(dir_pref + "global_regularization_x_FOM.csv",
           global_regularization_x_FOM);
  writeCSV(dir_pref + "global_x_lipm.csv", global_x_lipm_);
  writeCSV(dir_pref + "global_u_lipm.csv", global_u_lipm_);
  if (use_lipm_mpc_and_ik_) {
    writeCSV(dir_pref + "global_preprocess_x_lipm.csv",
             global_preprocess_x_lipm_);
    writeCSV(dir_pref + "global_preprocess_u_lipm.csv",
             global_preprocess_u_lipm_);
  }

  /// Save files for reproducing the same result
  // cout << "x_init = " << x_init << endl;
  writeCSV(dir_pref + string("x_init.csv"), x_init, true);
  writeCSV(dir_pref + string("init_phase.csv"), init_phase * VectorXd::Ones(1),
           true);
  writeCSV(dir_pref + string("is_right_stance.csv"),
           is_right_stance * VectorXd::Ones(1), true);
  writeCSV(dir_pref + string("quat_xyz_shift.csv"),
           quat_xyz_shift * VectorXd::Ones(1), true);
  writeCSV(dir_pref + string("final_position.csv"), final_position, true);
  writeCSV(dir_pref + string("init_file.csv"), trajopt.initial_guess(), true);
  writeCSV(dir_pref + string("current_time.csv"),
           current_time * VectorXd::Ones(1), true);
  writeCSV(dir_pref + string("global_fsm_idx.csv"),
           global_fsm_idx * VectorXd::Ones(1), true);
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
  double fom_xy_pos_cost = solvers::EvalCostGivenSolution(
      result, trajopt.fom_reg_xy_pos_cost_bindings_);
  if (fom_xy_pos_cost > 0) {
    cout << "fom_xy_pos_cost = " << fom_xy_pos_cost << endl;
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
  double fom_reg_xy_vel_cost = solvers::EvalCostGivenSolution(
      result, trajopt.fom_reg_xy_vel_cost_bindings_);
  if (fom_reg_xy_vel_cost > 0) {
    cout << "fom_reg_xy_vel_cost = " << fom_reg_xy_vel_cost << endl;
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
  double predict_lipm_p_cost =
      solvers::EvalCostGivenSolution(result, trajopt.predict_lipm_p_bindings_);
  if (predict_lipm_p_cost > 0) {
    cout << "predict_lipm_p_cost = " << predict_lipm_p_cost << endl;
  }
  double predict_lipm_v_cost =
      solvers::EvalCostGivenSolution(result, trajopt.predict_lipm_v_bindings_);
  if (predict_lipm_v_cost > 0) {
    cout << "predict_lipm_v_cost = " << predict_lipm_v_cost << endl;
  }
}

// Keep track of solve time and stuffs
void CassiePlannerWithMixedRomFom::BookKeeping(
    bool start_with_left_stance,
    const std::chrono::duration<double>& elapsed_lipm_mpc_and_ik,
    const std::chrono::duration<double>& elapsed_solve,
    const MathematicalProgramResult& result) const {
  /// lipm solve and IK
  if (use_lipm_mpc_and_ik_) {
    total_mpc_and_ik_solve_time_ += elapsed_lipm_mpc_and_ik.count();
    if (elapsed_lipm_mpc_and_ik.count() > max_mpc_and_ik_solve_time_) {
      max_mpc_and_ik_solve_time_ = elapsed_lipm_mpc_and_ik.count();
    }
    cout << "\nlipm mpc & ik time (average, max) = "
         << total_mpc_and_ik_solve_time_ / (counter_ + 1) << ", "
         << max_mpc_and_ik_solve_time_ << endl;
  }

  /// rom mpc solve
  total_solve_time_ += elapsed_solve.count();
  if (elapsed_solve.count() > max_solve_time_) {
    max_solve_time_ = elapsed_solve.count();
  }
  if (!result.is_success()) {
    num_failed_solve_++;
    latest_failed_solve_idx_ = counter_;
  }
  if (counter_ == 0 || past_is_left_stance_ != start_with_left_stance) {
    total_solve_time_of_first_solve_of_the_mode_ += elapsed_solve.count();
    if (elapsed_solve.count() > max_solve_time_of_first_solve_of_the_mode_) {
      max_solve_time_of_first_solve_of_the_mode_ = elapsed_solve.count();
    }
    total_number_of_first_solve_of_the_mode_++;
    past_is_left_stance_ = start_with_left_stance;
  }
  if (print_level_ > 0) {
    cout << "\nsolve time (average, max) = "
         << total_solve_time_ / (counter_ + 1) << ", " << max_solve_time_
         << endl;
    cout << "solve time of the first solve of the mode (average, max) = "
         << total_solve_time_of_first_solve_of_the_mode_ /
                total_number_of_first_solve_of_the_mode_
         << ", " << max_solve_time_of_first_solve_of_the_mode_ << endl;
    cout << "num_failed_solve_ = " << num_failed_solve_
         << " (latest failed index: " << latest_failed_solve_idx_
         << ", total solves = " << counter_ << ")"
         << "\n\n";
  }
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
    cout << c->get_description() << endl;
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
    cout << c->get_description() << endl;
    VectorXd input = trajopt.GetInitialGuess(binding.variables());
    //    cout << "eval point = " << input << endl;
    drake::VectorX<double> output(1);
    c.get()->Eval(input, &output);
    cout << output(0) << endl;
    i++;
  }*/
}

void CassiePlannerWithMixedRomFom::WarmStartGuess(
    const VectorXd& quat_xyz_shift, const vector<VectorXd>& reg_x_FOM,
    const int global_fsm_idx, int first_mode_knot_idx, double current_time,
    RomTrajOptCassie* trajopt) const {
  int starting_mode_idx_for_heuristic =
      (param_.n_step - 1) - (global_fsm_idx - prev_global_fsm_idx_) + 1;

  if (starting_mode_idx_for_heuristic <= 0) {
    PrintStatus("Set heuristic initial guess for all variables");
    // Set heuristic initial guess for all variables
    trajopt->SetHeuristicInitialGuess(param_, h_guess_, y_guess_, dy_guess_,
                                      tau_guess_, reg_x_FOM,
                                      first_mode_knot_idx, 0);
  } else {
    trajopt->SetHeuristicInitialGuess(
        param_, h_guess_, y_guess_, dy_guess_, tau_guess_, reg_x_FOM,
        first_mode_knot_idx, starting_mode_idx_for_heuristic);

    /// Reuse the solution
    // Rotate the previous global x floating base state according to the
    // current global-to-local-shift
    // TODO: also need to do the same thing to local_Lambda_FOM_
    // TODO: also need to do the same thing to predicted_com_vel_
    MatrixXd local_x0_FOM = global_x0_FOM_;
    MatrixXd local_xf_FOM = global_xf_FOM_;
    RotateBetweenGlobalAndLocalFrame(true, quat_xyz_shift, global_x0_FOM_,
                                     &local_x0_FOM);
    RotateBetweenGlobalAndLocalFrame(true, quat_xyz_shift, global_xf_FOM_,
                                     &local_xf_FOM);

    // Get time breaks of current problem (not solved yet so read from guesses)
    VectorXd times =
        trajopt->GetSampleTimes(trajopt->GetInitialGuess(trajopt->h_vars()));
    std::vector<VectorXd> breaks;
    const std::vector<int>& mode_lengths = trajopt->mode_lengths();
    const std::vector<int>& mode_start = trajopt->mode_start();
    for (int i = 0; i < trajopt->num_modes(); i++) {
      VectorXd times_i(mode_lengths[i]);
      for (int j = 0; j < mode_lengths[i]; j++) {
        int k_data = mode_start[i] + j;
        times_i(j) = times(k_data);
      }
      // Shift the timestamps by the current time
      times_i.array() += current_time;

      // Shift time by eps to ensure we evaluate the correct piece of polynomial
      times_i(0) += 1e-8;
      times_i(mode_lengths[i] - 1) -= 1e-8;

      breaks.push_back(times_i);
    }
    // Construct state traj from previous solution
    auto prev_state_traj = lightweight_saved_traj_.ReconstructStateTrajectory();

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
        // The difference between the first and second version is only in the
        // first mode
        // Version 1: use the closest knot point to initialize
        /*trajopt->SetInitialGuess(
            trajopt->state_vars_by_mode(local_fsm_idx, local_knot_idx),
            lightweight_saved_traj_.GetStateSamples(prev_local_fsm_idx)
                .col(prev_local_knot_idx));*/
        // Version 2: reconstruct traj and evalute the traj at the new time
        trajopt->SetInitialGuess(
            trajopt->state_vars_by_mode(local_fsm_idx, local_knot_idx),
            prev_state_traj.value(breaks.at(local_fsm_idx)(local_knot_idx)));

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
      trajopt->SetInitialGuess(trajopt->xf_vars_by_mode(local_fsm_idx),
                               local_xf_FOM.col(prev_local_fsm_idx));
      // 7. FOM post-impact
      trajopt->SetInitialGuess(trajopt->x0_vars_by_mode(local_fsm_idx + 1),
                               local_x0_FOM.col(prev_local_fsm_idx + 1));
      // 8. FOM impulse
      trajopt->SetInitialGuess(trajopt->impulse_vars(local_fsm_idx),
                               local_Lambda_FOM_.col(prev_local_fsm_idx));
    }

    // The robot fell when initializing eps_rom_ and local_predicted_com_vel_.
    // This makes sense, because eps_rom_ should be close to 0, and smaller
    // local_predicted_com_vel_ is more stable (walking slower).
    // 9. slack variable for initial fom-rom mapping
    //    trajopt->SetInitialGuess(trajopt->eps_rom_var_, eps_rom_);
    // 10. predicted com vel at the end of the immediate future mode
    //    trajopt->SetInitialGuess(trajopt->predicted_com_vel_var_,
    //                             local_predicted_com_vel_);

    // TODO: we can use LIPM state to warmstart the robot's poses (will nee IK)

    // For cascaded LIPM MPC
    if (param_.n_step_lipm > 1) {
      MatrixXd local_x_lipm = global_x_lipm_;
      MatrixXd local_u_lipm = global_u_lipm_;
      RotatePosBetweenGlobalAndLocalFrame(true, false, quat_xyz_shift,
                                          global_x_lipm_, &local_x_lipm);
      RotatePosBetweenGlobalAndLocalFrame(true, true, quat_xyz_shift,
                                          global_u_lipm_, &local_u_lipm);

      // The global_fsm_idx actually start from
      //    `global_fsm_idx + n_step`
      // to
      //    `global_fsm_idx + n_step + n_step_lipm`.
      // We removed n_step in the code below because we only need indices wrt
      // previous ones (n_step would be canceled out)
      for (int i = global_fsm_idx;
           i < prev_global_fsm_idx_ + param_.n_step_lipm; i++) {
        // Global fsm and knot index pair are (i, knot_idx)
        // Local fsm index
        int local_fsm_idx = i - global_fsm_idx;
        int prev_local_fsm_idx = i - prev_global_fsm_idx_;

        // 11. LIPM x
        trajopt->SetInitialGuess(trajopt->x_lipm_vars_by_idx(local_fsm_idx),
                                 local_x_lipm.col(prev_local_fsm_idx));
        if (prev_local_fsm_idx == param_.n_step_lipm - 1) {
          trajopt->SetInitialGuess(
              trajopt->x_lipm_vars_by_idx(local_fsm_idx + 1),
              local_x_lipm.col(prev_local_fsm_idx + 1));
        }
        // 12. LIPM u
        trajopt->SetInitialGuess(trajopt->u_lipm_vars_by_idx(local_fsm_idx),
                                 local_u_lipm.col(prev_local_fsm_idx));
      }
    }
  }
}

void CassiePlannerWithMixedRomFom::ResolveWithAnotherSolver(
    const RomTrajOptCassie& trajopt, const MathematicalProgramResult& result,
    const string& prefix, double current_time,
    const VectorXd& quat_xyz_shift) const {
  // Testing -- solve with another solver
  if (false) {
    auto start = std::chrono::high_resolution_clock::now();
    drake::solvers::MathematicalProgramResult result2;
    if (param_.use_ipopt) {
      solver_snopt_->Solve(trajopt, trajopt.initial_guess(),
                           solver_option_snopt_, &result2);
    } else {
      solver_ipopt_->Solve(trajopt, trajopt.initial_guess(),
                           solver_option_ipopt_, &result2);
    }
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    cout << "    Time of arrival: " << current_time << " | ";
    cout << "Solve time:" << elapsed.count() << " | ";
    cout << result2.get_solution_result() << " | ";
    cout << "Cost:" << result2.get_optimal_cost() << "\n";

    /// For visualization of the second solver
    MatrixXd x0_each_mode(nx_, trajopt.num_modes() + 1);
    MatrixXd xf_each_mode(nx_, trajopt.num_modes());
    for (uint i = 0; i < trajopt.num_modes(); i++) {
      x0_each_mode.col(i) = result2.GetSolution(trajopt.x0_vars_by_mode(i));
      xf_each_mode.col(i) = result2.GetSolution(trajopt.xf_vars_by_mode(i));
    }
    x0_each_mode.col(trajopt.num_modes()) =
        result.GetSolution(trajopt.x0_vars_by_mode(trajopt.num_modes()));
    MatrixXd global_x0_FOM = x0_each_mode;
    MatrixXd global_xf_FOM = xf_each_mode;
    RotateBetweenGlobalAndLocalFrame(false, quat_xyz_shift, x0_each_mode,
                                     &global_x0_FOM);
    RotateBetweenGlobalAndLocalFrame(false, quat_xyz_shift, xf_each_mode,
                                     &global_xf_FOM);
    writeCSV(param_.dir_data + prefix + "local_x0_FOM_snopt.csv", x0_each_mode);
    writeCSV(param_.dir_data + prefix + "local_xf_FOM_snopt.csv", xf_each_mode);
    writeCSV(param_.dir_data + prefix + "global_x0_FOM_snopt.csv",
             global_x0_FOM);
    writeCSV(param_.dir_data + prefix + "global_xf_FOM_snopt.csv",
             global_xf_FOM);
  }

  // Testing -- solve with another solver and feed it with solution as init
  // guess
  if (false) {
    cout << "Use previous solution as a initial condition...\n";
    auto start = std::chrono::high_resolution_clock::now();
    drake::solvers::MathematicalProgramResult result2;
    if (param_.use_ipopt) {
      solver_snopt_->Solve(trajopt, result.GetSolution(), solver_option_snopt_,
                           &result2);
    } else {
      solver_ipopt_->Solve(trajopt, result.GetSolution(), solver_option_ipopt_,
                           &result2);
    }
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    cout << "    Time of arrival: " << current_time << " | ";
    cout << "Solve time:" << elapsed.count() << " | ";
    cout << result2.get_solution_result() << " | ";
    cout << "Cost:" << result2.get_optimal_cost() << "\n";
  }
}

void CassiePlannerWithMixedRomFom::PrintTrajMsg(
    dairlib::lcmt_timestamped_saved_traj* traj_msg) const {
  // I created this function to debug the memory leak. (I didn't copy utime in
  // RomPlannerTrajectory's copy assignment)
  cout << "Printing lcm objects\n";
  cout << "traj_msg->utime = " << traj_msg->utime << endl;
  auto saved_traj = traj_msg->saved_traj;
  //  dairlib::lcmt_metadata metadata;
  //  int32_t    num_trajectories;
  //  std::vector< dairlib::lcmt_trajectory_block > trajectories;
  //  std::vector< std::string > trajectory_names;
  cout << "num_trajectories = " << saved_traj.num_trajectories << endl;
  cout << "name = ";
  for (auto name : saved_traj.trajectory_names) {
    cout << name << ", ";
  }
  cout << endl;
  cout << "---\n";
  for (auto traj_block : saved_traj.trajectories) {
    //    std::string trajectory_name;
    //    int32_t    num_points;
    //    int32_t    num_datatypes;
    //    std::vector< double > time_vec;
    //    std::vector< std::vector< double > > datapoints;
    //    std::vector< std::string > datatypes;
    cout << "trajectory_name = " << traj_block.trajectory_name << endl;
    cout << "num_points = " << traj_block.num_points << endl;
    cout << "num_datatypes = " << traj_block.num_datatypes << endl;
    cout << "time_vec = ";
    for (auto time : traj_block.time_vec) {
      cout << time << ", ";
    }
    cout << endl;
    cout << "datatypes = ";
    for (auto data_element_name : traj_block.datatypes) {
      cout << data_element_name << ", ";
    }
    cout << endl;
    cout << "datapoints = \n";
    for (auto row : traj_block.datapoints) {
      for (auto element : row) {
        cout << element << ", ";
      }
      cout << endl;
    }
    cout << endl;
  }
  cout << "END;\n";
}

}  // namespace goldilocks_models
}  // namespace dairlib
