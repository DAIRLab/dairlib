#include "examples/goldilocks_models/controller/cassie_hybrid_rom_planner_system.h"

#include <math.h> /* fmod */

#include <algorithm>  // std::max
#include <fstream>
#include <iostream>
#include <limits>
#include <string>

#include "common/eigen_utils.h"
#include "examples/goldilocks_models/planning/hybrid_rom_traj_opt.h"
#include "examples/goldilocks_models/planning/lipm_mpc.h"
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
using std::set;
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

using drake::math::RotationMatrix;
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

CassiePlannerWithOnlyRom::CassiePlannerWithOnlyRom(
    const MultibodyPlant<double>& plant_control, double stride_period,
    const PlannerSetting& param, const set<int>& relax_index,
    const set<int>& set_init_state_from_prev_solution,
    const set<int>& idx_const_rom_vel_during_double_support,
    bool singel_eval_mode, bool log_data, int print_level)
    : is_RL_training_(param.is_RL_training),
      collect_rewards_via_lcmlogs_(param.collect_rewards_via_lcmlogs),
      nq_(plant_control.num_positions()),
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
      relax_index_(relax_index),
      set_init_state_from_prev_solution_(set_init_state_from_prev_solution),
      idx_const_rom_vel_during_double_support_(
          idx_const_rom_vel_during_double_support),
      single_eval_mode_(singel_eval_mode),
      log_data_and_check_solution_(log_data) {
  this->set_name("hybrid_rom_planner_traj");

  DRAKE_DEMAND(param_.knots_per_mode > 0);

  // print level
  print_level_ = print_level;
  if (single_eval_mode_ && print_level_ > 0) {
    print_level_ = 2;
    cout.precision(dbl::max_digits10);
  }
  // TODO: comment the following out, once you finished implementing everything
  // if (param_.is_RL_training) print_level_ = 0;

  // Input/Output Setup
  stance_foot_port_ =
      this->DeclareVectorInputPort("stance_foot", BasicVector<double>(1))
          .get_index();
  phase_port_ =
      this->DeclareVectorInputPort("phase", BasicVector<double>(1)).get_index();
  state_port_ =
      this->DeclareVectorInputPort(
              "x_init", OutputVector<double>(plant_control.num_positions(),
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
                                  &CassiePlannerWithOnlyRom::SolveTrajOpt);

  // Create index maps
  pos_map_ = multibody::makeNameToPositionsMap(plant_control);
  vel_map_ = multibody::makeNameToVelocitiesMap(plant_control);

  // Reduced order model
  PrintEssentialStatus("model directory = " + param_.dir_model);
  rom_ = CreateRom(param_.rom_option, ROBOT, plant_control, false);
  if (!param_.is_RL_training) {
    ReadModelParameters(rom_.get(), param_.dir_model, param_.iter);

  } else {
    ReadModelParameters(rom_.get(), param_.dir_model, 1);

    cout << "param_.path_model_params = " << param_.path_model_params << endl;
    MatrixXd theta_yddot = readCSV(param_.path_model_params);
    DRAKE_DEMAND(theta_yddot.size() > 0);
    rom_->SetThetaYddot(theta_yddot.col(0));
  }
  // This leafsystem hasn't taken care of active ROM case (e.g. no good warm
  // starting)
  DRAKE_DEMAND(rom_->n_tau() == 0);

  // Create mirror maps
  state_mirror_ = StateMirror(MirrorPosIndexMap(plant_control, ROBOT),
                              MirrorPosSignChangeSet(plant_control, ROBOT),
                              MirrorVelIndexMap(plant_control, ROBOT),
                              MirrorVelSignChangeSet(plant_control, ROBOT));
  // Create mirrored ROM
  rom_mirrored_ = std::make_unique<MirroredReducedOrderModel>(
      plant_control, *rom_, state_mirror_);

  /// Provide initial guess
  n_y_ = rom_->n_y();
  n_z_ = 2 * n_y_;
  n_tau_ = rom_->n_tau();
  string model_dir_n_pref = param_.dir_model + to_string(param_.iter) +
                            string("_") + to_string(param_.sample) +
                            string("_");
  h_guess_ = 1e-4 * VectorXd::Random(param_.knots_per_mode);
  y_guess_ = 1e-4 * MatrixXd::Random(n_y_, param_.knots_per_mode);
  dy_guess_ = 1e-4 * MatrixXd::Random(n_y_, param_.knots_per_mode);
  tau_guess_ = 1e-4 * MatrixXd::Random(n_tau_, param_.knots_per_mode);
  cout << "dir_and_prefix_FOM.empty()?" << param_.dir_and_prefix_FOM.empty()
       << endl;
  cout << model_dir_n_pref + "y_samples0.csv exist?"
       << file_exist(model_dir_n_pref + "y_samples0.csv") << endl;
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

    double traj_duration = y_traj.end_time();
    if (std::abs(traj_duration - stride_period) > 1e-15) {
      cout << "WARNING: trajectory duration is different from stride length!\n";
      cout << "  traj_duration = " << traj_duration << endl;
      cout << "  stride_period = " << stride_period << endl;
      bool close_sim_gap = (double_support_duration_ == 0);
      if (close_sim_gap) {
        DRAKE_DEMAND(std::abs(traj_duration - stride_period) < 1e-15);
      }
    }
    for (int i = 0; i < param_.knots_per_mode; i++) {
      // Note that if we evaluate the traj at a time outside the segment we
      // constructed the traj with, it's going to return the value at the
      // closest time. So it's not too bad if stride_period > traj_duration.
      h_guess_(i) = stride_period / (param_.knots_per_mode - 1) * i;
      y_guess_.col(i) = y_traj.value(h_guess_(i));
      dy_guess_.col(i) = y_traj.EvalDerivative(h_guess_(i), 1);
      if (n_tau_ != 0) {
        tau_guess_.col(i) = tau_traj.value(h_guess_(i));
      }

      // Heuristic -- constant ydot outside outside the single support duration
      // (extend to outside the nominal traj time duration)
      if (h_guess_(i) > traj_duration) {
        dy_guess_.col(i) = y_traj.EvalDerivative(traj_duration, 1);
      } else if (h_guess_(i) > single_support_duration_) {
        for (auto idx : idx_const_rom_vel_during_double_support) {
          dy_guess_.col(i)(idx - n_y_) =
              y_traj.EvalDerivative(single_support_duration_, 1)(idx - n_y_);
        }
      }
      //      cout << "t = " << h_guess_(i) << endl;
      //      cout << "y = " << y_guess_.col(i)(0) << endl;
      //      cout << "dy = " << dy_guess_.col(i)(0) << endl;
    }
  } else {
    cout << "Construct rom regularization from FOM traj: " + model_dir_n_pref +
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
    DRAKE_DEMAND(x_traj.cols() == 1);
    DRAKE_DEMAND(x_traj.rows() == nx_);
    //    DRAKE_DEMAND(std::abs(duration - stride_period) < 1e-15);
    if (std::abs(duration - stride_period) > 1e-15) {
      cout << "WARNING: trajectory duration is different from stride length!\n";
      cout << "  duration = " << duration << endl;
      cout << "  stride_period = " << stride_period << endl;
      bool close_sim_gap = (double_support_duration_ == 0);
      if (close_sim_gap) {
        DRAKE_DEMAND(std::abs(duration - stride_period) < 1e-15);
      }
    }

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
    x_guess_steppingleft_pre_ = x_standing_fixed_spring_;
    x_guess_steppingright_pre_ = x_standing_fixed_spring_;
    x_guess_steppingleft_post_ = x_standing_fixed_spring_;
    x_guess_steppingright_post_ = x_standing_fixed_spring_;
  } else {
    string dir_and_prefix_FOM = param_.dir_and_prefix_FOM.empty()
                                    ? model_dir_n_pref
                                    : param_.dir_and_prefix_FOM;
    cout << "dir_and_prefix_FOM = " << dir_and_prefix_FOM << endl;
    VectorXd x_guess_steppingright_pre =
        readCSV(dir_and_prefix_FOM + string("x_samples0.csv")).rightCols(1);
    VectorXd x_guess_steppingright_post =
        readCSV(dir_and_prefix_FOM + string("x_samples1.csv")).col(0);

    //    x_guess_right_in_front_pre(pos_map_.at("hip_roll_left")) = 0;
    //    x_guess_right_in_front_pre(pos_map_.at("hip_roll_right")) = 0;
    //    x_guess_right_in_front_post(pos_map_.at("hip_roll_left")) = 0;
    //    x_guess_right_in_front_post(pos_map_.at("hip_roll_right")) = 0;

    VectorXd x_guess_steppingleft_pre(nx_);
    x_guess_steppingleft_pre
        << state_mirror_.MirrorPos(x_guess_steppingright_pre.head(nq_)),
        state_mirror_.MirrorVel(x_guess_steppingright_pre.tail(nv_));
    VectorXd x_guess_steppingleft_post(nx_);
    x_guess_steppingleft_post
        << state_mirror_.MirrorPos(x_guess_steppingright_post.head(nq_)),
        state_mirror_.MirrorVel(x_guess_steppingright_post.tail(nv_));

    x_guess_steppingright_pre_ = x_guess_steppingright_pre;
    x_guess_steppingright_post_ = x_guess_steppingright_post;
    x_guess_steppingleft_pre_ = x_guess_steppingleft_pre;
    x_guess_steppingleft_post_ = x_guess_steppingleft_post;
  }
  cout << endl;

  // Footstep regularization (for initial guess and mild cost regularization)
  drake::VectorX<double> left_foot_pos(3);
  drake::VectorX<double> right_foot_pos(3);
  plant_control_.SetPositionsAndVelocities(context_plant_control_.get(),
                                           x_guess_steppingleft_pre_);
  this->plant_control_.CalcPointsPositions(
      *context_plant_control_, left_mid_.second, left_mid_.first,
      plant_control_.world_frame(), &left_foot_pos);
  this->plant_control_.CalcPointsPositions(
      *context_plant_control_, right_mid_.second, right_mid_.first,
      plant_control_.world_frame(), &right_foot_pos);
  footstep_during_right_support_ = (left_foot_pos - right_foot_pos).head<2>();
  plant_control_.SetPositionsAndVelocities(context_plant_control_.get(),
                                           x_guess_steppingright_pre_);
  this->plant_control_.CalcPointsPositions(
      *context_plant_control_, left_mid_.second, left_mid_.first,
      plant_control_.world_frame(), &left_foot_pos);
  this->plant_control_.CalcPointsPositions(
      *context_plant_control_, right_mid_.second, right_mid_.first,
      plant_control_.world_frame(), &right_foot_pos);
  footstep_during_left_support_ = (right_foot_pos - left_foot_pos).head<2>();

  cout << "footstep_during_right_support_ = " << footstep_during_right_support_
       << endl;
  cout << "footstep_during_left_support_ = " << footstep_during_left_support_
       << endl;

  // Desired CoM height.
  // (if "pelvis_height" is not set, we default com height to `y_reg_`)
  desired_com_height_ = y_reg_.col(0)(2);
  if (param_.gains.pelvis_height > 0) {
    // Method #1 -- use heuristics (cannot really use this one if we make the
    // toe very heavy)
    desired_com_height_ = param_.gains.pelvis_height -
                          param_.gains.pelvis_com_height_diff_heuristic;
    // Method #2 -- use IK to get desired standing pose given pelivs height,
    // then compute the CoM height
    VectorXd q_ik =
        RunIkForCoMHeight(plant_control, param_.gains.pelvis_height);
    plant_control_.SetPositions(context_plant_control_.get(), q_ik);
    desired_com_height_ =
        plant_control_.CalcCenterOfMassPositionInWorld(*context_plant_control_)
            .tail<1>()(0);
    cout << "desired_com_height_ from IK = " << desired_com_height_ << endl;
  }

  //   cout << "initial guess duration ~ " << duration << endl;
  //   cout << "h_guess = " << h_guess_ << endl;
  //   cout << "y_guess = " << y_guess_ << endl;
  //   cout << "dy_guess = " << dy_guess_ << endl;
  //   cout << "tau_guess = " << tau_guess_ << endl;
  //    cout << "x_guess_right_in_front_pre_ = "
  //         << x_guess_right_in_front_pre_.transpose() << endl;
  //    cout << "x_guess_steppingright_post_ = "
  //         << x_guess_steppingright_post_.transpose() << endl;
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

  // double support phase's number of kont points
  phase_from_ss_to_ds_ = single_support_duration_ /
                         (single_support_duration_ + double_support_duration_);
  knots_per_double_support_ =
      (double_support_duration_ == 0)
          ? 0
          : int((param_.knots_per_mode - 2) * double_support_duration_ /
                (single_support_duration_ + double_support_duration_)) +
                2;  // +2 because we need at least 2 knot points
  knots_per_single_support_ =
      (double_support_duration_ == 0)
          ? param_.knots_per_mode
          : param_.knots_per_mode - knots_per_double_support_ + 1;

  // Default timestep and number of knot points
  for (int i = 0; i < param_.n_step; i++) {
    num_time_samples_.push_back(param_.knots_per_mode);
    num_time_samples_ds_.push_back(knots_per_double_support_);
    min_dt_.push_back(.01);
    max_dt_.push_back(.3);
  }

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
  global_delta_footstep_ = MatrixXd(2, param_.n_step);
  global_y_end_of_last_mode_ = MatrixXd(2, 1);

  // Initialization
  prev_mode_start_ = std::vector<int>(param_.n_step, -1);

  // For warm start
  if (warm_start_with_previous_solution_ && single_eval_mode_) {
    // Only allow warm-start with init file, in order to keep the code clean.
    // If we don't use init_file for initial guess, there will be a segfault
    // from `prev_global_fsm_idx_` in WarmStartGuess()
    DRAKE_DEMAND(!param_.init_file.empty());
  }

  // Index of state which we initialize to 1 in order to avoid singularity
  // (which messes with gradient)
  if ((param_.rom_option == 0) || (param_.rom_option == 1) ||
      (param_.rom_option == 28) || (param_.rom_option == 29)) {
    idx_state_init_to_1_ = 1;
  } else if ((param_.rom_option == 4) ||
             ((param_.rom_option >= 8) && (param_.rom_option <= 27)) ||
             (param_.rom_option == 30) || (param_.rom_option == 31) ||
             (param_.rom_option == 32)) {
    idx_state_init_to_1_ = 2;
  } else {
    DRAKE_UNREACHABLE();
  }

  // Some checks -- we don't have double support phase in the LIPM MPC yet.
  if (use_lipm_mpc_and_ik_) {
    DRAKE_DEMAND(!param_.gains.constant_rom_vel_during_double_support);
  }
  // Checks for idx_const_rom_vel_during_double_support_
  if (idx_const_rom_vel_during_double_support_.size() > 0) {
    DRAKE_DEMAND(param_.gains.constant_rom_vel_during_double_support);
    for (auto& element : idx_const_rom_vel_during_double_support_) {
      DRAKE_DEMAND(element >= rom_->n_y());  // idx is of state
      DRAKE_DEMAND(element < 2 * rom_->n_y());
    }
    // Enforce that the constant vel is on the physically interpretable element
    const auto& rom_invariant_elements = rom_->invariant_elements();
    for (auto& element : idx_const_rom_vel_during_double_support_) {
      bool is_part_of_invariant_elements =
          (rom_invariant_elements.find(element - rom_->n_y()) !=
           rom_invariant_elements.end());
      DRAKE_DEMAND(is_part_of_invariant_elements);
    }
  }

  /// For single evaluation mode (debug mode)
  if (param.solve_idx_for_read_from_file >= 0) {
    lightweight_saved_traj_ = HybridRomPlannerTrajectory(
        param.dir_data + to_string(param.solve_idx_for_read_from_file) +
            "_rom_trajectory",
        true);

    counter_ = param.solve_idx_for_read_from_file;
  }

  /// Save data for (offline) debug mode
  if (!param_.get_RL_gradient_offline) {
    writeCSV(param.dir_data + "rom_option.csv",
             param.rom_option * VectorXd::Ones(1));
    writeCSV(param.dir_data + "model_iter.csv", param.iter * VectorXd::Ones(1));
    writeCSV(param.dir_data + "sample_idx.csv",
             param.sample * VectorXd::Ones(1));
  }
}

void CassiePlannerWithOnlyRom::InitializeForRL(
    const MultibodyPlant<double>& plant_feedback, int task_dim,
    bool include_previous_vel_in_rl_state) {
  include_previous_vel_in_rl_state_ = include_previous_vel_in_rl_state;

  // Create an input port for logging robot output
  robot_output_port_ =
      this->DeclareVectorInputPort(
              "(x,u,t) for RL data",
              OutputVector<double>(plant_feedback.num_positions(),
                                   plant_feedback.num_velocities(),
                                   plant_feedback.num_actuators()))
          .get_index();

  /// Initialization for RL training
  int s_dim = plant_feedback.num_positions() + plant_feedback.num_velocities() +
              plant_feedback.num_actuators() + task_dim + 2;
  if (include_previous_vel_in_rl_state) {
    s_dim += plant_feedback.num_velocities();
  }
  if (only_use_rom_state_in_near_future_for_RL_) {
    // We cut off the rom state in the far future that we don't use
    n_knots_used_for_RL_action_ = param_.knots_per_mode * 1;
  } else {
    n_knots_used_for_RL_action_ = param_.knots_per_mode * param_.n_step;
  }
  a_dim_rom_state_part_ = n_z_ * n_knots_used_for_RL_action_;
  a_dim_rom_input_part_ = 2 * param_.n_step;
  int a_dim = a_dim_rom_state_part_ + a_dim_rom_input_part_ + 1;
  task_dim_ = task_dim;
  RL_state_prev_ = VectorXd::Zero(s_dim);
  RL_state_ = VectorXd::Zero(s_dim);
  RL_action_prev_ = VectorXd::Zero(a_dim);
  RL_action_ = VectorXd::Zero(a_dim);
  RL_action_noise_prev_ = VectorXd::Zero(a_dim);
  RL_task_prev_ = VectorXd::Zero(task_dim);
  RL_task_ = VectorXd::Zero(task_dim);
  RL_vel_prev_ = VectorXd::Zero(plant_feedback.num_velocities());

  // Save the action and state name
  auto mbp_state_name = multibody::createStateNameVectorFromMap(plant_feedback);
  auto mbp_act_name =
      multibody::createActuatorNameVectorFromMap(plant_feedback);
  vector<string> RL_state_names;
  RL_state_names.insert(RL_state_names.end(), mbp_state_name.begin(),
                        mbp_state_name.end());
  if (include_previous_vel_in_rl_state) {
    for (int i = 0; i < plant_feedback.num_velocities(); i++) {
      RL_state_names.push_back(
          mbp_state_name.at(plant_feedback.num_positions() + i) + "_prev");
    }
  }
  RL_state_names.insert(RL_state_names.end(), mbp_act_name.begin(),
                        mbp_act_name.end());
  RL_state_names.push_back("des_com_vel_x");
  RL_state_names.push_back("des_com_height");
  RL_state_names.push_back("left_stance");
  RL_state_names.push_back("init_phase");
  vector<string> RL_action_names;
  for (int i = 0; i < a_dim_rom_state_part_; i++) {
    RL_action_names.push_back("rom_state");
  }
  for (int i = a_dim_rom_state_part_;
       i < a_dim_rom_state_part_ + 2 * param_.n_step; i++) {
    RL_action_names.push_back("rom_delta_footstep");
  }
  RL_action_names.push_back("dt");
  vector<string> RL_addtl_info_names;
  RL_addtl_info_names.push_back("com_vel_x");
  RL_addtl_info_names.push_back("com_height");
  RL_addtl_info_names.push_back("s_prime_time");

  if (!param_.get_RL_gradient_offline) {
    SaveStringVecToCsv(RL_state_names,
                       param_.dir_data + string("RL_state_names.csv"));
    SaveStringVecToCsv(RL_action_names,
                       param_.dir_data + string("RL_action_names.csv"));
    SaveStringVecToCsv(RL_addtl_info_names,
                       param_.dir_data + string("RL_addtl_info_names.csv"));
  }

  // Initialize gaussian distribution for RL policy noise
  if (!param_.path_var.empty()) {
    MatrixXd var = readCSV(param_.path_var);
    DRAKE_DEMAND(var.size() > 1);
    RL_policy_output_variances_ = var.col(0);
    DRAKE_DEMAND(RL_policy_output_variances_.size() == a_dim);
  } else {
    RL_policy_output_variances_ = VectorXd::Zero(a_dim);
  }

  generator_ = std::make_unique<std::default_random_engine>();
  generator_->seed(std::chrono::system_clock::now().time_since_epoch().count());
  distributions_ =
      std::make_unique<std::vector<std::normal_distribution<double>>>();
  for (int i = 0; i < RL_policy_output_variances_.size(); i++) {
    distributions_->push_back(std::normal_distribution<double>(
        0, std::sqrt(RL_policy_output_variances_(i))));
  }

  // Sanity check
  cout << "RL state dim = " << s_dim << endl;
  cout << "RL action dim = " << a_dim << endl;
  DRAKE_DEMAND(RL_state_names.size() == s_dim);
  DRAKE_DEMAND(RL_action_names.size() == a_dim);
}

void CassiePlannerWithOnlyRom::SolveTrajOpt(
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

  if (single_eval_mode_) {
    if (print_level_ > 1) {
      cout << "Used for the planner: \n";
      cout << "  x_init  = " << x_init.transpose() << endl;
      cout << "  quat_xyz_shift  = " << quat_xyz_shift.transpose() << endl;
      cout << "  current_time  = " << current_time << endl;
      cout << "  start_with_left_stance  = " << start_with_left_stance << endl;
      cout << "  init_phase  = " << init_phase << endl;
      cout << "  final_position = " << final_position.transpose() << endl;
    }
  }

  // For data logging
  string prefix;
  if (single_eval_mode_ && !param_.is_RL_training) {
    prefix = "debug_";
  } else {
    prefix = to_string(counter_) + "_";
  }

  ///
  /// Some setups
  ///

  // Get current ROM position
  VectorXd init_rom_state_from_current_feedback(6);
  plant_control_.SetPositionsAndVelocities(context_plant_control_.get(),
                                           x_init);
  if (start_with_left_stance) {
    init_rom_state_from_current_feedback.head<3>() =
        rom_->EvalMappingFunc(x_init.head(nq_), *context_plant_control_);
    init_rom_state_from_current_feedback.tail<3>() = rom_->EvalMappingFuncJV(
        x_init.head(nq_), x_init.tail(nv_), *context_plant_control_);
  } else {
    init_rom_state_from_current_feedback.head<3>() =
        rom_mirrored_->EvalMappingFunc(x_init.head(nq_),
                                       *context_plant_control_);
    init_rom_state_from_current_feedback.tail<3>() =
        rom_mirrored_->EvalMappingFuncJV(x_init.head(nq_), x_init.tail(nv_),
                                         *context_plant_control_);
  }

  // Get current stance foot position
  drake::VectorX<double> current_local_stance_foot_pos(3);
  const auto& stance_toe_mid = start_with_left_stance ? left_mid_ : right_mid_;
  this->plant_control_.CalcPointsPositions(
      *context_plant_control_, stance_toe_mid.second, stance_toe_mid.first,
      plant_control_.world_frame(), &current_local_stance_foot_pos);
  drake::VectorX<double> current_local_swing_foot_pos(3);
  const auto& swing_toe_mid = start_with_left_stance ? right_mid_ : left_mid_;
  this->plant_control_.CalcPointsPositions(
      *context_plant_control_, swing_toe_mid.second, swing_toe_mid.first,
      plant_control_.world_frame(), &current_local_swing_foot_pos);

  ///
  /// Get initial ROM state
  ///

  // Get the init ROM state form the previous plan
  std::set<int> set_init_state_from_prev_solution =
      set_init_state_from_prev_solution_;
  VectorXd init_rom_state_from_prev_sol(n_z_);
  if (!set_init_state_from_prev_solution.empty()) {
    if (counter_ == 0) {
      set_init_state_from_prev_solution.clear();
    } else {
      auto rom_traj = lightweight_saved_traj_.ConstructPositionTrajectory();
      if (rom_traj.end_time() > current_time) {
        init_rom_state_from_prev_sol << rom_traj.value(current_time),
            rom_traj.EvalDerivative(current_time, 1);
      }
    }
  }

  // Set initial ROM state for the trajopt constraint
  VectorXd init_rom_state_mirrored(6);
  for (int i = 0; i < 6; i++) {
    // Get the init state value for the constraint
    init_rom_state_mirrored(i) = (set_init_state_from_prev_solution.find(i) ==
                                  set_init_state_from_prev_solution.end())
                                     ? init_rom_state_from_current_feedback(i)
                                     : init_rom_state_from_prev_sol(i);
  }
  // TODO: might want to fix the init height pos to a desired constant and vel
  //  to 0?
  init_rom_state_mirrored(2) = desired_com_height_;
  init_rom_state_mirrored(5) = 0;
  if (single_eval_mode_) {
    if (print_level_ > 1) {
      cout << "init_rom_state_from_current_feedback = "
           << init_rom_state_from_current_feedback.transpose() << endl;
      if (counter_ > 0) {
        cout << "init_rom_state_from_prev_sol = "
             << init_rom_state_from_prev_sol.transpose() << endl;
      }
      cout << "init_rom_state = " << init_rom_state_mirrored.transpose()
           << endl;
    }
  }

  // Init footstep
  Vector2d init_footstep =
      (current_local_swing_foot_pos - current_local_stance_foot_pos).head<2>();
  if (single_eval_mode_) {
    if (print_level_ > 1) {
      cout << "current_local_swing_foot_pos = "
           << current_local_swing_foot_pos.transpose() << endl;
      cout << "current_local_stance_foot_pos = "
           << current_local_stance_foot_pos.transpose() << endl;
      cout << "init_footstep = " << init_footstep.transpose() << endl;
    }
  }

  ///
  ///
  ///

  // First mode duration
  double first_mode_duration = stride_period_ * (1 - init_phase);

  // Remaining single support duration
  double remaining_single_support_duration =
      std::max(0.0, first_mode_duration - double_support_duration_);

  ///
  /// Get desired xy position and velocity relative to INITIAL stance foot
  ///
  auto break1 = std::chrono::high_resolution_clock::now();

  // Get adjusted_final_pos
  vector<Vector2d> des_xy_pos_com_rt_init_stance_foot;
  vector<Vector2d> des_xy_vel_com_rt_init_stance_foot;
  CreateDesiredComPosAndVel(param_.n_step + param_.n_step_lipm,
                            start_with_left_stance, init_phase, final_position,
                            &des_xy_pos_com_rt_init_stance_foot,
                            &des_xy_vel_com_rt_init_stance_foot);
  for (auto& pos : des_xy_pos_com_rt_init_stance_foot) {
    pos -= current_local_stance_foot_pos.head<2>();
  }
  Vector2d des_com_pos_rt_stance_foot_at_end_of_mode =
      des_xy_pos_com_rt_init_stance_foot.at(0);
  if (single_eval_mode_) {
    if (print_level_ > 1) {
      for (auto& pos : des_xy_pos_com_rt_init_stance_foot) {
        cout << "pos = " << pos.transpose() << endl;
      }
      for (auto& vel : des_xy_vel_com_rt_init_stance_foot) {
        cout << "vel = " << vel.transpose() << endl;
      }
    }
  }
  // My ntoes: Only the goal position needs the above shift.
  //    When translating the planned traj between each solve, I believe we don't
  //    need to care about the above shift. Also when "translating" the planned
  //    traj, we need to
  //       1. rotate both foot steps and com
  //       2. translate foot steps (no need to translate com, because com is wrt
  //       foot steps in the current formulation)
  //    After thinking a bit more, I think
  //       1. we can just rotate and no need to translate, because all decision
  //       variable are relative to the stance foot. And the goal position is
  //       adjusted (above) already.
  //       2. if the MPC runs fast enough, maybe we don't even need to rotate,
  //       because the delta between each solve is small.
  //       3. if you were to transform between frames, you only need to do 2D.

  // Get desired pos and vel values relative to its stance foot
  // TODO: the horizontal vel (and maybe pos) needs different value for right
  //  and left support.
  //  Try velocity first, and also implement position if it's necessary.
  Vector2d des_com_pos_rt_stance_foot_at_start_of_mode__during_left_stance;
  Vector2d des_com_vel_rt_stance_foot_at_start_of_mode__during_left_stance;
  if (completely_use_trajs_from_model_opt_as_target_) {
    des_com_pos_rt_stance_foot_at_start_of_mode__during_left_stance =
        y_guess_.col(0).head<2>();
    des_com_vel_rt_stance_foot_at_start_of_mode__during_left_stance =
        y_guess_.col(0).segment<2>(3);
  } else {
    des_com_pos_rt_stance_foot_at_start_of_mode__during_left_stance
        << -des_com_pos_rt_stance_foot_at_end_of_mode(0),
        -abs(des_com_pos_rt_stance_foot_at_end_of_mode(1));
    des_com_vel_rt_stance_foot_at_start_of_mode__during_left_stance
        << des_xy_vel_com_rt_init_stance_foot.at(0)(0),
        abs(des_xy_vel_com_rt_init_stance_foot.at(0)(1));
  }
  /*cout << "des_com_pos = "
       << des_com_pos_rt_stance_foot_at_start_of_mode__during_left_stance
              .transpose()
       << endl;
  cout << "des_com_vel = "
       << des_com_vel_rt_stance_foot_at_start_of_mode__during_left_stance
              .transpose()
       << endl;*/

  ///
  /// Use LIPM MPC and IK to get desired configuration to guide ROM MPC
  ///
  // I deleted the method RunLipmMPC and GetDesiredFullStateFromLipmMPCSol,
  // becuase I'm not using it in `CassiePlannerWithOnlyRom`.
  // The implementation is in `CassiePlannerWithMixedRomFom`.

  ///
  /// Construct regularization for footsteps
  ///
  Vector2d footstep_during_right_support;
  Vector2d footstep_during_left_support;
  if (completely_use_trajs_from_model_opt_as_target_) {
    footstep_during_right_support = footstep_during_right_support_;
    footstep_during_left_support = footstep_during_left_support_;
  } else {
    footstep_during_right_support
        << 2 * des_com_pos_rt_stance_foot_at_end_of_mode(0),
        2 * abs(des_com_pos_rt_stance_foot_at_end_of_mode(1));
    footstep_during_left_support
        << 2 * des_com_pos_rt_stance_foot_at_end_of_mode(0),
        -2 * abs(des_com_pos_rt_stance_foot_at_end_of_mode(1));
  }
  vector<Vector2d> reg_local_delta_footstep(param_.n_step, Vector2d());
  bool left_stance = start_with_left_stance;
  for (int i = 0; i < param_.n_step; i++) {
    reg_local_delta_footstep.at(i) = left_stance
                                         ? footstep_during_left_support
                                         : footstep_during_right_support;
    left_stance = !left_stance;
  }

  ///
  /// Construct rom traj opt
  ///
  auto break2 = std::chrono::high_resolution_clock::now();

  // Prespecify the number of knot points
  int first_mode_knot_idx = DetermineNumberOfKnotPoints(
      init_phase, first_mode_duration, remaining_single_support_duration);
  PrintEssentialStatus("start_with_left_stance  = " +
                       to_string(start_with_left_stance));
  PrintEssentialStatus("init_phase  = " + to_string(init_phase));
  PrintEssentialStatus("first_mode_knot_idx  = " +
                       to_string(first_mode_knot_idx));

  /*// We don't use swing foot travel distance constriant now.

  // Maximum swing foot travel distance
  // Update date the step length of the first mode
  // Take into account the double stance duration
  // Linearly decrease the max speed to 0 after mid-swing
  double max_foot_speed_first_mode =
      param_.gains.max_foot_speed *
      std::min(1.0, 2 * remaining_single_support_duration /
                        single_support_duration_);

  // * We don't need slack now because we don't use full model.
  // * Currently I only impose distance cosntraint during swing phase
  //  (so that it doesn't affect the swing foot tracking towards the end of
  //  swing phase)
  max_swing_distance_[0] =
      max_foot_speed_first_mode * remaining_single_support_duration;
  PrintEssentialStatus("remaining_time_til_touchdown  = " +
                       to_string(remaining_single_support_duration));*/

  // Construct
  PrintStatus("\nConstructing optimization problem...");
  HybridRomTrajOptCassie trajopt(
      num_time_samples_, Q_, R_, *rom_, init_rom_state_mirrored, init_footstep,
      max_swing_distance_, start_with_left_stance, param_.zero_touchdown_impact,
      relax_index_, param_, num_time_samples_ds_,
      first_mode_duration <= double_support_duration_,
      idx_const_rom_vel_during_double_support_,
      print_level_ > 1 /*print_status*/);

  PrintStatus("Other constraints and costs ===============");
  // Time step constraints
  trajopt.AddTimeStepConstraint(min_dt_, max_dt_, param_.fix_duration,
                                param_.equalize_timestep_size,
                                first_mode_duration, stride_period_);

  // Add small regularization cost on rom state
  trajopt.AddRomRegularizationCost(h_reg_, y_reg_, dy_reg_, tau_reg_,
                                   first_mode_knot_idx, param_.gains.w_rom_reg);

  // Desired CoM position and center of mass (via cost)
  trajopt.AddDesiredCoMPosVelCost(
      param_.gains.w_reg_xy, param_.gains.w_reg_xy_vel,
      des_com_pos_rt_stance_foot_at_start_of_mode__during_left_stance,
      des_com_vel_rt_stance_foot_at_start_of_mode__during_left_stance);

  // Final goal position constraint
  /*PrintStatus("Adding constraint -- FoM final position");
  trajopt.AddBoundingBoxConstraint(
      adjusted_final_position, adjusted_final_position,
      trajopt.xf_vars_by_mode(num_time_samples.size() - 1).segment(4, 2));*/

  // Add target position cost for final rom position
  //  trajopt.AddFinalGoalPositionCost(param_.gains.w_reg_xy,
  //                                   des_xy_pos_com_rt_init_stance_foot.at(param_.n_step
  //                                   - 1));

  // Future steps
  if (param_.n_step_lipm > 1) {
    trajopt.AddCascadedLipmMPC(
        param_.gains.w_predict_lipm_p, param_.gains.w_predict_lipm_v,
        des_xy_pos_com_rt_init_stance_foot, des_xy_vel_com_rt_init_stance_foot,
        param_.n_step_lipm, stride_period_,
        param_.gains.max_lipm_step_length / 2,
        param_.gains.right_limit_wrt_pelvis, desired_com_height_);
  } else {
    if (use_lipm_mpc_and_ik_) {
      // TODO: check if adding predicted step is necessary after using
      //  lipm_mpc_and_ik
      PrintEssentialStatus(
          "REMINDER!! check if adding predicted step is necessary after using "
          "lipm_mpc_and_ik");
    }

    Vector2d des_predicted_xy_vel;
    DRAKE_DEMAND(param_.n_step > 1);
    // des_xy_vel is of size n_step.
    // The predicted step (step after n_step) corresponds to
    // des_xy_vel.at(n_step), but des_xy_vel's size is not big enough to cover
    // the predicted step, so we -2. Note that, we go back 2 steps instead of
    // 1 step because velocity is asymmetric in y direction
    des_predicted_xy_vel =
        des_xy_vel_com_rt_init_stance_foot.at(param_.n_step - 2);
    if (single_eval_mode_) {
      if (print_level_ > 1) {
        cout << "des_predicted_xy_vel = " << des_predicted_xy_vel << endl;
      }
    }

    // Constraint and cost for the last foot step location
    trajopt.AddConstraintAndCostForLastFootStep(
        param_.gains.w_predict_lipm_v, des_predicted_xy_vel, stride_period_,
        desired_com_height_);
  }

  PrintStatus("Initial guesses ===============");

  // Default initial guess to avoid singularity (which messes with gradient)
  // These guesses are just default values, and will be overwritten afterward.
  for (int i = 0; i < num_time_samples_.size(); i++) {
    for (int j = 0; j < num_time_samples_.at(i); j++) {
      trajopt.SetInitialGuess(
          (trajopt.state_vars_by_mode(i, j))(idx_state_init_to_1_), 1);
    }
  }

  // Initial guess for all variables
  if ((counter_ == 0 && !param_.init_file.empty()) || single_eval_mode_) {
    PrintStatus("Set initial guess from the file " + param_.init_file);
    VectorXd z0 = readCSV(param_.dir_data + param_.init_file).col(0);
    /*if (param_.get_RL_gradient_offline) {
      z0 = readCSV(param_.dir_data + prefix + "z.csv").col(0);
      DRAKE_DEMAND(z0.rows() == trajopt.decision_variables().size());
    }*/
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
      WarmStartGuess(quat_xyz_shift, current_local_stance_foot_pos,
                     reg_local_delta_footstep, global_fsm_idx,
                     first_mode_knot_idx, current_time, &trajopt);
    } else {
      // Set heuristic initial guess for all variables
      PrintStatus("Set heuristic initial guess...");
      trajopt.SetHeuristicInitialGuess(param_, h_guess_, y_guess_, dy_guess_,
                                       tau_guess_, reg_local_delta_footstep,
                                       first_mode_knot_idx, 0);
    }
    trajopt.SetHeuristicInitialGuessForCascadedLipm(
        param_, des_xy_pos_com_rt_init_stance_foot,
        des_xy_vel_com_rt_init_stance_foot);
    trajopt.SetInitialGuess(trajopt.state_vars_by_mode(0, 0),
                            init_rom_state_mirrored);

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
  /*if (is_RL_training_ && param_.get_RL_gradient_offline) {
    result.set_decision_variable_index(trajopt.decision_variable_index());
    result.set_x_val(trajopt.initial_guess());
    result.set_solution_result(SolutionResult::kSolutionFound);
    result.set_optimal_cost(-1);
  } else {
  }*/
  if (param_.use_ipopt) {
    PrintEssentialStatus("(ipopt)");
    solver_ipopt_->Solve(trajopt, trajopt.initial_guess(), solver_option_ipopt_,
                         &result);
  } else {
    PrintEssentialStatus("(snopt)");
    solver_snopt_->Solve(trajopt, trajopt.initial_guess(), solver_option_snopt_,
                         &result);
  }
  auto break4 = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double> elapsed_input_reading = break1 - start;
  std::chrono::duration<double> elapsed_lipm_mpc_and_ik = break2 - break1;
  std::chrono::duration<double> elapsed_trajopt_construct = break3 - break2;
  std::chrono::duration<double> elapsed_solve = break4 - break3;
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

  if (!result.is_success()) {
    *traj_msg = previous_output_msg_;
  }

  VectorXd RL_action_noise(RL_policy_output_variances_.size());
  drake::solvers::MathematicalProgramResult result_with_noise;
  if (is_RL_training_ && !collect_rewards_via_lcmlogs_) {
    for (int i = 0; i < RL_policy_output_variances_.size(); i++) {
      double rand = (distributions_->at(i))(*generator_);
      // Heurisitics -- clamp it to avoid tail distribution
      RL_action_noise(i) = std::clamp(rand, -param_.policy_output_noise_bound,
                                      param_.policy_output_noise_bound);
    }
    // Last row (zeros) is the gradient of delta_t, which we don't randomize
    RL_action_noise.bottomRows<1>().setZero();

    // Map RL_action_noise to mpc_sol_noise
    VectorXd mpc_sol_noise = VectorXd::Zero(trajopt.num_vars());

    int idx_start_x;
    int idx_start_h;
    int idx_start_xp;
    int idx_start_footsteps;
    trajopt.GetStartOfVariableIndices(idx_start_x, idx_start_h, idx_start_xp,
                                      idx_start_footsteps);

    // ROM state part
    const std::vector<int>& mode_start = trajopt.mode_start();
    const std::vector<int>& mode_lengths = trajopt.mode_lengths();
    int i = 0;
    for (int mode = 0; mode < mode_lengths.size(); mode++) {
      for (int knot_idx = 0; knot_idx < mode_lengths.at(mode); knot_idx++) {
        if (knot_idx == 0 && mode > 0) {
          mpc_sol_noise.middleRows(idx_start_xp + (mode - 1) * n_z_, n_z_) =
              RL_action_noise.middleRows(i * n_z_, n_z_);
        } else {
          mpc_sol_noise.middleRows((mode_start[mode] + knot_idx) * n_z_, n_z_) =
              RL_action_noise.middleRows(i * n_z_, n_z_);
        }
        i++;
        if (i == n_knots_used_for_RL_action_) {
          mode = mode_lengths.size();  // To break the outer for loop
          break;                       // To break the inner for loop
        }
      }
    }
    // ROM footstep part
    // WARNING: we assume two footsteps here
    DRAKE_DEMAND(param_.n_step == 2);
    mpc_sol_noise.middleRows<2>(idx_start_footsteps) =
        RL_action_noise.middleRows<2>(a_dim_rom_state_part_);
    mpc_sol_noise.middleRows<2>(idx_start_footsteps + 2) =
        RL_action_noise.middleRows<2>(a_dim_rom_state_part_ + 2);

    // Apply noise to solution
    result_with_noise = result;
    result_with_noise.set_x_val(result.GetSolution() + mpc_sol_noise);
  }

  ///
  /// Pack traj into lcm message (traj_msg)
  ///
  // Notes
  //  - reminder: the trajectory is discontinuous between mode (even the
  //    position jumps because left vs right stance leg).
  //  - lightweight_saved_traj_ is also used in warm starting (we can implement
  //    in the way that doesn't use lightweight_saved_traj_)

  // Get global com and feet position
  MatrixXd global_feet_pos(2, param_.n_step + 1);
  MatrixXd global_com_pos(2, param_.n_step + 1);
  ExtractGlobalComAndFootstepPosition(
      quat_xyz_shift, current_local_stance_foot_pos, start_with_left_stance,
      trajopt, result, &global_feet_pos, &global_com_pos);
  MatrixXd global_feet_pos_with_noise;
  MatrixXd global_com_pos_with_noise;
  if (is_RL_training_ && !collect_rewards_via_lcmlogs_) {
    global_feet_pos_with_noise = MatrixXd(2, param_.n_step + 1);
    global_com_pos_with_noise = MatrixXd(2, param_.n_step + 1);
    ExtractGlobalComAndFootstepPosition(
        quat_xyz_shift, current_local_stance_foot_pos, start_with_left_stance,
        trajopt, result_with_noise, &global_feet_pos_with_noise,
        &global_com_pos_with_noise);
  }

  // Benchmark: for n_step = 3, the packing time is about 60us and the message
  // size is about 4.5KB (use WriteToFile() to check).
  lightweight_saved_traj_ = HybridRomPlannerTrajectory(
      trajopt, result, global_feet_pos, global_com_pos, quat_xyz_shift,
      current_local_stance_foot_pos, prefix, "", true, current_time);
  if (is_RL_training_ && !collect_rewards_via_lcmlogs_) {
    lightweight_saved_traj_with_noise_ = HybridRomPlannerTrajectory(
        trajopt, result_with_noise, global_feet_pos_with_noise,
        global_com_pos_with_noise, quat_xyz_shift,
        current_local_stance_foot_pos, prefix, "", true, current_time);
  }

  if (is_RL_training_ && !collect_rewards_via_lcmlogs_) {
    *traj_msg = lightweight_saved_traj_with_noise_.GenerateLcmObject();
  } else {
    *traj_msg = lightweight_saved_traj_.GenerateLcmObject();
  }
  //  PrintTrajMsg(traj_msg);

  // Store the previous message
  previous_output_msg_ = *traj_msg;
  timestamp_of_previous_plan_ = current_time;

  ///
  /// Save solutions for either logging for warm-starting
  ///
  // TODO: maybe don't save the trajectory for warmstart if the solver didn't
  //  find an optimal solution

  TransformToGlobalFrameForWarmStartLater(
      quat_xyz_shift, current_local_stance_foot_pos, trajopt, result);

  h_solutions_ = trajopt.GetTimeStepSolution(result);
  input_at_knots_ = trajopt.GetInputSamples(result);

  eps_rom_ = result.GetSolution(trajopt.eps_rom_var_);
  if (param_.n_step_lipm > 1) {
    local_x_lipm_ =
        Eigen::Map<MatrixXd>(result.GetSolution(trajopt.x_lipm_vars_).data(), 4,
                             param_.n_step_lipm + 1);
    local_u_lipm_ = Eigen::Map<MatrixXd>(
        result.GetSolution(trajopt.u_lipm_vars_).data(), 2, param_.n_step_lipm);

    // Transform solutions into global frame for warm start later
    global_x_lipm_ = local_x_lipm_;
    global_u_lipm_ = local_u_lipm_;
    TransformBetweenGlobalAndLocalFrame2D(false, false, false, quat_xyz_shift,
                                          local_x_lipm_, &global_x_lipm_);
    TransformBetweenGlobalAndLocalFrame2D(false, false, true, quat_xyz_shift,
                                          local_u_lipm_, &global_u_lipm_);
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

  auto break5 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed_pack_lcm = break5 - break4;
  PrintEssentialStatus(
      "Runtime for packing lcm and setting variable for warmstarts:" +
      to_string(elapsed_pack_lcm.count()));

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
    // Extract and save solution into files (for debugging)
    SaveDataIntoFiles(current_time, global_fsm_idx, x_init, init_phase,
                      is_right_stance, quat_xyz_shift,
                      current_local_stance_foot_pos, final_position,
                      reg_local_delta_footstep, global_delta_footstep_, trajopt,
                      result, param_.dir_data, prefix);
    if (is_RL_training_ && !collect_rewards_via_lcmlogs_) {
      SaveStateAndActionIntoFilesForRLTraining(
          context, des_xy_vel_com_rt_init_stance_foot.at(0)(0),
          desired_com_height_, start_with_left_stance, init_phase,
          lightweight_saved_traj_with_noise_, trajopt, result_with_noise,
          current_time, RL_action_noise, param_.dir_data);
      SaveGradientIntoFilesForRLTraining(trajopt, result, param_.dir_data);
      // Sanity check
      /*if (param_.get_RL_gradient_offline) {
        cout << "(trajopt.initial_guess() - result.GetSolution()).norm() = "
             << (trajopt.initial_guess() - result.GetSolution()).norm() << endl;
        DRAKE_DEMAND((trajopt.initial_guess() - result.GetSolution()).norm() <
                     1e-8);
      }*/
    }
    break5 = std::chrono::high_resolution_clock::now();
    // Save trajectory into lcm binary
    {
      string file_name = prefix + "rom_trajectory";
      HybridRomPlannerTrajectory full_saved_traj(
          trajopt, result, global_feet_pos, global_com_pos, quat_xyz_shift,
          current_local_stance_foot_pos, file_name,
          drake::solvers::to_string(result.get_solution_result()),
          param_.gains.lightweight_lcm_log, current_time);
      full_saved_traj.WriteToFile(param_.dir_data + file_name);
      cout << "Wrote to file: " << param_.dir_data + file_name << endl;

      if (is_RL_training_ && !collect_rewards_via_lcmlogs_) {
        string file_name = prefix + "rom_trajectory_w_noise";
        HybridRomPlannerTrajectory full_saved_traj(
            trajopt, result_with_noise, global_feet_pos_with_noise,
            global_com_pos_with_noise, quat_xyz_shift,
            current_local_stance_foot_pos, file_name,
            drake::solvers::to_string(result_with_noise.get_solution_result()),
            param_.gains.lightweight_lcm_log, current_time);
        full_saved_traj.WriteToFile(param_.dir_data + file_name);
        cout << "Wrote to file: " << param_.dir_data + file_name << endl;
      }
    }

    // Check the cost
    if (print_level_ > 0) {
      PrintCost(trajopt, result);
    }

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

  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = break5 - start;
  PrintEssentialStatus("Runtime for data saving (for debugging) -- csv: " +
                       to_string(elapsed.count()));
  elapsed = finish - break5;
  PrintEssentialStatus("Runtime for data saving (for debugging) -- lcm: " +
                       to_string(elapsed.count()));

  /// Some checks
  if (counter_ == 0 && completely_use_trajs_from_model_opt_as_target_) {
    // The cost shouldn't be too big in the first solve
    if (result.get_optimal_cost() > 5e-3) {
      cout << "\n\nWARNING: Cost shouldn't be too big in the first solve\n\n";
    }
    //    DRAKE_DEMAND(result.get_optimal_cost() < 1e-2);
  }

  // Unit Testing TransformBetweenGlobalAndLocalFrame3D
  if (single_eval_mode_ || param_.unit_testing) {
    MatrixXd global_x_init = x_init;
    MatrixXd x_init_2 = x_init;
    TransformBetweenGlobalAndLocalFrame3D(false, quat_xyz_shift, x_init,
                                          &global_x_init);
    TransformBetweenGlobalAndLocalFrame3D(true, quat_xyz_shift, global_x_init,
                                          &x_init_2);
    DRAKE_DEMAND((x_init_2 - x_init).norm() < 1e-14);
  }

  ///
  counter_++;
}

MatrixXd CassiePlannerWithOnlyRom::ExtractMpcFootsteps(
    const HybridRomTrajOptCassie& trajopt,
    const MathematicalProgramResult& result) const {
  MatrixXd local_delta_footstep(2, param_.n_step);
  local_delta_footstep.row(0) = result.GetSolution(
      trajopt.discrete_swing_foot_pos_rt_stance_foot_x_vars());
  local_delta_footstep.row(1) = result.GetSolution(
      trajopt.discrete_swing_foot_pos_rt_stance_foot_y_vars());
  return local_delta_footstep;
}

void CassiePlannerWithOnlyRom::TransformToGlobalFrameForWarmStartLater(
    const VectorXd& quat_xyz_shift,
    const drake::VectorX<double>& current_local_stance_foot_pos,
    const HybridRomTrajOptCassie& trajopt,
    const MathematicalProgramResult& result) const {
  MatrixXd local_delta_footstep = ExtractMpcFootsteps(trajopt, result);

  // 1. Transform relative foot step from local to global
  // Note that since it's relative, we only need to rotate the vector
  TransformBetweenGlobalAndLocalFrame2D(false, true, true, quat_xyz_shift,
                                        local_delta_footstep,
                                        &global_delta_footstep_);

  // 2. Transform from the local com to global com
  TransformBetweenGlobalAndLocalFrame2D(
      false, false, true, quat_xyz_shift,
      result.GetSolution(trajopt.y_end_of_last_mode_rt_init_stance_foot_var()) +
          current_local_stance_foot_pos.head<2>(),
      &global_y_end_of_last_mode_);
}

void CassiePlannerWithOnlyRom::ExtractGlobalComAndFootstepPosition(
    const VectorXd& quat_xyz_shift,
    const drake::VectorX<double>& current_local_stance_foot_pos,
    bool start_with_left_stance, const HybridRomTrajOptCassie& trajopt,
    const MathematicalProgramResult& result, MatrixXd* global_feet_pos,
    MatrixXd* global_com_pos) const {
  MatrixXd local_delta_footstep = ExtractMpcFootsteps(trajopt, result);

  // Get global feet position (for both start and end of each mode)
  // This is used to generate swing foot pos in the controller thread
  MatrixXd local_feet_pos(2, param_.n_step + 1);
  local_feet_pos.col(0) = current_local_stance_foot_pos.head<2>();
  for (int i = 1; i < param_.n_step + 1; i++) {
    local_feet_pos.col(i) =
        local_feet_pos.col(i - 1) + local_delta_footstep.col(i - 1);
  }
  TransformBetweenGlobalAndLocalFrame2D(false, false, true, quat_xyz_shift,
                                        local_feet_pos, global_feet_pos);
  // Get global com position (for both start and end of each mode)
  // This is used to generate *relative* swing foot pos in the controller thread
  // (i.e. swing foot pos relative to the COM)
  MatrixXd local_com_pos(2, param_.n_step + 1);
  bool left_stance = start_with_left_stance;
  for (int i = 0; i < param_.n_step + 1; i++) {
    local_com_pos.col(i) =
        result.GetSolution(trajopt.state_vars_by_mode(i, 0).head<2>());
    if (!left_stance) local_com_pos.col(i)(1) *= -1;
    local_com_pos.col(i) += local_feet_pos.col(i);
    left_stance = !left_stance;
  }
  TransformBetweenGlobalAndLocalFrame2D(false, false, true, quat_xyz_shift,
                                        local_com_pos, global_com_pos);
  if (single_eval_mode_) {
    if (print_level_ > 1) {
      cout << "local_com_pos.col(0) = " << local_com_pos.col(0).transpose()
           << endl;
      cout << "global_com_pos.col(0) = " << global_com_pos->col(0).transpose()
           << endl;
      cout << "global_feet_pos.col(0) = " << global_feet_pos->col(0).transpose()
           << endl;
    }
  }
}

int CassiePlannerWithOnlyRom::DetermineNumberOfKnotPoints(
    double init_phase, double first_mode_duration,
    double remaining_single_support_duration) const {
  // Prespecify the number of knot points
  // We use int() to round down the index because we need to have at least one
  // timestep in the first mode, i.e. 2 knot points.
  int first_mode_knot_idx;
  if (param_.gains.constant_rom_vel_during_double_support) {
    // I had to add 1e-8 because of a floating point error from somehwere.
    // Without this epsilon, there would be an invalid constraint bound when the
    // current time is exactly at the boundary between single support and double
    // support phase.
    if (init_phase + 1e-8 < phase_from_ss_to_ds_) {
      double phase_in_ss =
          1 - remaining_single_support_duration / single_support_duration_;
      int ss_knot_idx = int((knots_per_single_support_ - 1) * phase_in_ss);
      first_mode_knot_idx = ss_knot_idx;
      num_time_samples_ds_[0] = knots_per_double_support_;
    } else {
      double phase_in_ds = 1 - first_mode_duration / double_support_duration_;
      int ds_knot_idx = int((knots_per_double_support_ - 1) * phase_in_ds);
      first_mode_knot_idx = (knots_per_single_support_ - 1) + ds_knot_idx;
      num_time_samples_ds_[0] = knots_per_double_support_ - ds_knot_idx;
    }
  } else {
    first_mode_knot_idx = int((param_.knots_per_mode - 1) * init_phase);
  }

  int n_knots_first_mode = param_.knots_per_mode - first_mode_knot_idx;
  num_time_samples_[0] = n_knots_first_mode;
  // TODO: we should have two min_dt_, one for single support and the other one
  //  for double support. This is not urgent, because it's not used when we fix
  //  the timestep
  min_dt_[0] = ((n_knots_first_mode == 2) ||
                (first_mode_knot_idx == knots_per_single_support_ - 2))
                   ? 1e-3
                   : 1e-2;
  PrintEssentialStatus("n_knots_first_mode  = " +
                       to_string(n_knots_first_mode));

  return first_mode_knot_idx;
}

void CassiePlannerWithOnlyRom::CreateDesiredPelvisPosAndVel(
    int n_total_step, bool start_with_left_stance, double init_phase,
    const Vector2d& final_position, vector<Vector2d>* des_xy_pos,
    vector<Vector2d>* des_xy_vel) const {
  CreateDesiredBodyPosAndVel(true, n_total_step, start_with_left_stance,
                             init_phase, final_position, des_xy_pos,
                             des_xy_vel);
}
void CassiePlannerWithOnlyRom::CreateDesiredComPosAndVel(
    int n_total_step, bool start_with_left_stance, double init_phase,
    const Vector2d& final_position, vector<Vector2d>* des_xy_pos,
    vector<Vector2d>* des_xy_vel) const {
  CreateDesiredBodyPosAndVel(false, n_total_step, start_with_left_stance,
                             init_phase, final_position, des_xy_pos,
                             des_xy_vel);
}
void CassiePlannerWithOnlyRom::CreateDesiredBodyPosAndVel(
    bool pelvis_or_com, int n_total_step, bool start_with_left_stance,
    double init_phase, const Vector2d& final_position,
    vector<Vector2d>* des_xy_pos, vector<Vector2d>* des_xy_vel) const {
  // Parameters
  double y_vel_offset;
  y_vel_offset = param_.gains.y_vel_offset;
  DRAKE_DEMAND(y_vel_offset >= 0);
  // TODO: we can derive y_pos_offset from y_vel_offset and maybe add it to the
  //  COM rt stance foot.

  // Limit the desired position by the max desired step length
  Vector2d adjusted_final_pos = final_position;
  double total_phase_length = n_total_step - init_phase;
  double pos_diff_norm = adjusted_final_pos.norm();
  double max_pos_diff_norm =
      std::abs(param_.gains.max_desired_step_length * total_phase_length);
  if (pos_diff_norm > max_pos_diff_norm) {
    adjusted_final_pos *= max_pos_diff_norm / pos_diff_norm;
  }

  // 1. Get the desired xy positions
  *des_xy_pos = vector<Vector2d>(n_total_step + 1, Vector2d::Zero());
  des_xy_pos->at(1) = des_xy_pos->at(0) + adjusted_final_pos *
                                              (1 - init_phase) /
                                              total_phase_length;
  for (int i = 2; i < des_xy_pos->size(); i++) {
    des_xy_pos->at(i) =
        des_xy_pos->at(i - 1) + adjusted_final_pos / total_phase_length;
  }
  //  if (!completely_use_trajs_from_model_opt_as_target_) {
  // Heuristically shift the desired velocity in y direction
  // Actually, I think we don't need to shift. Think about the case of walking
  // forward.
  //  }

  // 2. Get the desired xy velocities
  if (completely_use_trajs_from_model_opt_as_target_) {
    *des_xy_vel = vector<Vector2d>(n_total_step, Vector2d());
    Vector2d body_vel_at_the_end_of_left_support;
    Vector2d body_vel_at_the_end_of_right_support;
    if (pelvis_or_com) {
      body_vel_at_the_end_of_left_support =
          x_guess_steppingright_post_.segment<2>(nq_ + 3);
      body_vel_at_the_end_of_right_support =
          x_guess_steppingleft_post_.segment<2>(nq_ + 3);
    } else {
      plant_control_.SetPositionsAndVelocities(context_plant_control_.get(),
                                               x_guess_steppingleft_post_);
      body_vel_at_the_end_of_right_support =
          plant_control_
              .CalcCenterOfMassTranslationalVelocityInWorld(
                  *context_plant_control_)
              .head<2>();
      plant_control_.SetPositionsAndVelocities(context_plant_control_.get(),
                                               x_guess_steppingright_post_);
      body_vel_at_the_end_of_left_support =
          plant_control_
              .CalcCenterOfMassTranslationalVelocityInWorld(
                  *context_plant_control_)
              .head<2>();
    }

    bool left_stance = start_with_left_stance;
    for (int i = 0; i < n_total_step; i++) {
      if (left_stance) {
        des_xy_vel->at(i) = body_vel_at_the_end_of_left_support;
      } else {
        des_xy_vel->at(i) = body_vel_at_the_end_of_right_support;
      }
      left_stance = !left_stance;
    }

  } else {
    // Use the average vel for the vel at hybrid event (which underestimates)
    *des_xy_vel = vector<Vector2d>(
        n_total_step,
        adjusted_final_pos / (stride_period_ * total_phase_length));

    // Heuristically shift the desired velocity in y direction
    bool left_stance = start_with_left_stance;
    for (int i = 0; i < n_total_step; i++) {
      if (left_stance) {
        des_xy_vel->at(i)(1) -= y_vel_offset;
      } else {
        des_xy_vel->at(i)(1) += y_vel_offset;
      }
      left_stance = !left_stance;
    }
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

void CassiePlannerWithOnlyRom::TransformBetweenGlobalAndLocalFrame3D(
    bool transform_from_global_to_local, const VectorXd& quat_xyz_shift,
    const MatrixXd& original_x_FOM, MatrixXd* rotated_x_FOM) const {
  Quaterniond relative_quat =
      transform_from_global_to_local
          ? Quaterniond(quat_xyz_shift(0), quat_xyz_shift(1), quat_xyz_shift(2),
                        quat_xyz_shift(3))
          : Quaterniond(quat_xyz_shift(0), quat_xyz_shift(1), quat_xyz_shift(2),
                        quat_xyz_shift(3))
                .conjugate();
  Matrix3d relative_rot_mat = relative_quat.toRotationMatrix();
  double sign = transform_from_global_to_local ? 1 : -1;
  for (int j = 0; j < original_x_FOM.cols(); j++) {
    Quaterniond rotated_x_quat =
        relative_quat *
        Quaterniond(original_x_FOM.col(j)(0), original_x_FOM.col(j)(1),
                    original_x_FOM.col(j)(2), original_x_FOM.col(j)(3));
    rotated_x_FOM->col(j).segment<4>(0) << rotated_x_quat.w(),
        rotated_x_quat.vec();
    if (transform_from_global_to_local) {
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

  // TODO (20220401): minor bug, the planner problem seems to be wrong when
  //  Cassie faces backward (-z world axis).
  //  I did the following change, but it didn't fix the bug -- Make sure that
  //  the quaternion doesn't pick the other solution of the same rotation.
  // TODO: I will need to turn off constraints gradually to find where the bug
  //    is. I think the quaternion constraint in rom_traj_opt could be the bug.
  //  20230317 Update: I did mujoco and hardware experiments in December where I
  //  turned Cassie around a lot and it was fine.
  // Make sure that the quaternion doesn't pick the other solution of the same
  // rotation.
  // Hacks: Not sure what's the best the way to deal with this.
  // Currently, I'm just checking the sign of the largest element.
  /*auto abs_vec = rotated_x_FOM->col(0).head<4>().cwiseAbs();
  double max_value = abs_vec.maxCoeff();
  int index_of_max_value = 0;
  for (int i = 0; i < 4; i++) {
    if (abs_vec(i) == max_value) {
      index_of_max_value = i;
      break;
    }
  }
  bool positive_of_big_value = (rotated_x_FOM->col(0)(index_of_max_value) > 0);
  if (positive_of_big_value) {
    for (int j = 0; j < original_x_FOM.cols(); j++) {
      if (rotated_x_FOM->col(j)(index_of_max_value) < 0) {
        rotated_x_FOM->col(j).head<4>() *= -1;
      }
    }
  } else {
    for (int j = 0; j < original_x_FOM.cols(); j++) {
      if (rotated_x_FOM->col(j)(index_of_max_value) > 0) {
        rotated_x_FOM->col(j).head<4>() *= -1;
      }
    }
  }*/
}

void CassiePlannerWithOnlyRom::TransformBetweenGlobalAndLocalFrame2D(
    bool transform_from_global_to_local, bool rotation_transformation_only,
    bool position_only, const Eigen::VectorXd& quat_xyz_shift,
    const Eigen::MatrixXd& original_x, Eigen::MatrixXd* rotated_x) const {
  DRAKE_DEMAND(quat_xyz_shift(1) < 1e-14);  // rotate about z axis
  DRAKE_DEMAND(quat_xyz_shift(2) < 1e-14);  // rotate about z axis
  Quaterniond relative_quat =
      transform_from_global_to_local
          ? Quaterniond(quat_xyz_shift(0), quat_xyz_shift(1), quat_xyz_shift(2),
                        quat_xyz_shift(3))
          : Quaterniond(quat_xyz_shift(0), quat_xyz_shift(1), quat_xyz_shift(2),
                        quat_xyz_shift(3))
                .conjugate();
  Matrix2d relative_rot_mat =
      relative_quat.toRotationMatrix().topLeftCorner<2, 2>();
  double sign = transform_from_global_to_local ? 1 : -1;
  if (rotation_transformation_only) sign = 0;
  for (int j = 0; j < original_x.cols(); j++) {
    // position
    if (transform_from_global_to_local) {
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

void CassiePlannerWithOnlyRom::SaveDataIntoFiles(
    double current_time, int global_fsm_idx, const VectorXd& x_init,
    double init_phase, bool is_right_stance, const VectorXd& quat_xyz_shift,
    const VectorXd& current_local_stance_foot_pos,
    const VectorXd& final_position,
    const vector<Vector2d>& reg_local_delta_footstep,
    const MatrixXd& global_delta_footstep,
    const HybridRomTrajOptCassie& trajopt,
    const MathematicalProgramResult& result, const string& dir_data,
    const string& prefix) const {
  string dir_pref = dir_data + prefix;

  /// Rotate some local vectors to global
  // Rotate from local to global
  /*MatrixXd global_regularization_footstep(2, param_.n_step);
  MatrixXd local_regularization_footstep(2, param_.n_step);
  for (int i = 0; i < param_.n_step; i++) {
    local_regularization_footstep.middleCols<1>(i) =
        reg_local_delta_footstep.at(i);
  }
  global_regularization_footstep = local_regularization_footstep;
  TransformBetweenGlobalAndLocalFrame2D(false, false, true, quat_xyz_shift,
                                        local_regularization_footstep,
                                        &global_regularization_footstep);*/
  //    cout << "global_regularization_footstep = \n"
  //         << global_regularization_footstep << endl;
  //    cout << "local_regularization_footstep = \n"
  //         << local_regularization_footstep << endl;

  MatrixXd global_x_init(nx_, 1);
  global_x_init = x_init;
  TransformBetweenGlobalAndLocalFrame3D(false, quat_xyz_shift, x_init,
                                        &global_x_init);

  /// Save the solution vector (used in initializing the first iter)
  writeCSV(dir_pref + "z.csv", result.GetSolution());
  //  writeCSV(dir_pref + "z.csv", result.GetSolution(), is_RL_training_);
  // cout << trajopt.decision_variables() << endl;

  /// Save traj to csv
  /*for (int i = 0; i < param_.n_step; i++) {
    writeCSV(dir_pref + "time_at_knots" + to_string(i) + ".csv",
             lightweight_saved_traj_.GetStateBreaks(i));
    writeCSV(dir_pref + "state_at_knots" + to_string(i) + ".csv",
             lightweight_saved_traj_.GetStateSamples(i));
  }
  writeCSV(dir_pref + "input_at_knots.csv", trajopt.GetInputSamples(result));*/

  /*writeCSV(dir_pref + "local_delta_footstep.csv", local_delta_footstep);
  writeCSV(dir_pref + "global_delta_footstep.csv", global_delta_footstep);
  writeCSV(dir_pref + "global_regularization_footstep.csv",
           global_regularization_footstep);*/

  /*writeCSV(dir_pref + "global_x_lipm.csv", global_x_lipm_);
  writeCSV(dir_pref + "global_u_lipm.csv", global_u_lipm_);
  if (use_lipm_mpc_and_ik_) {
    writeCSV(dir_pref + "global_preprocess_x_lipm.csv",
             global_preprocess_x_lipm_);
    writeCSV(dir_pref + "global_preprocess_u_lipm.csv",
             global_preprocess_u_lipm_);
  }*/

  writeCSV(dir_pref + string("global_x_init.csv"), global_x_init);

  /// Save files for reproducing the same result
  // cout << "x_init = " << x_init << endl;
  writeCSV(dir_pref + string("x_init.csv"), x_init, true);
  writeCSV(dir_pref + string("init_phase.csv"), init_phase * VectorXd::Ones(1),
           true);
  writeCSV(dir_pref + string("is_right_stance.csv"),
           is_right_stance * VectorXd::Ones(1), true);
  writeCSV(dir_pref + string("quat_xyz_shift.csv"), quat_xyz_shift, true);
  writeCSV(dir_pref + string("current_local_stance_foot_pos.csv"),
           current_local_stance_foot_pos, true);
  writeCSV(dir_pref + string("final_position.csv"), final_position, true);
  writeCSV(dir_pref + string("init_file.csv"), trajopt.initial_guess(), true);
  writeCSV(dir_pref + string("current_time.csv"),
           current_time * VectorXd::Ones(1), true);
  writeCSV(dir_pref + string("global_fsm_idx.csv"),
           global_fsm_idx * VectorXd::Ones(1), true);
}

void CassiePlannerWithOnlyRom::SaveStateAndActionIntoFilesForRLTraining(
    const Context<double>& context, double dex_x_vel, double des_com_height,
    bool start_with_left_stance, double init_phase,
    const HybridRomPlannerTrajectory& saved_traj,
    const HybridRomTrajOptCassie& trajopt,
    const MathematicalProgramResult& result, double current_time,
    const VectorXd& RL_action_noise, const string& dir_data) const {
  PrintStatus("Save RL state and action data\n");

  auto start = std::chrono::high_resolution_clock::now();

  // Get robot_output for RL
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, robot_output_port_);

  // Assign dt
  RL_action_prev_.tail<1>() << current_time - prev_time_;

  // Assign new state and action
  if (include_previous_vel_in_rl_state_) {
    RL_state_ << robot_output->GetState(),
        (RL_vel_prev_.norm() == 0) ? robot_output->GetVelocities()
                                   : RL_vel_prev_,
        robot_output->GetEfforts(), dex_x_vel, des_com_height,
        start_with_left_stance, init_phase;
  } else {
    RL_state_ << robot_output->GetState(), robot_output->GetEfforts(),
        dex_x_vel, des_com_height, start_with_left_stance, init_phase;
  }
  if (only_use_rom_state_in_near_future_for_RL_) {
    RL_action_.segment(0, saved_traj.GetStateSamples(0).size()) =
        Eigen::Map<const VectorXd>(saved_traj.GetStateSamples(0).data(),
                                   saved_traj.GetStateSamples(0).size());
    int idx = saved_traj.GetStateSamples(0).size();
    RL_action_.segment(idx, a_dim_rom_state_part_ - idx) =
        (Eigen::Map<const VectorXd>(saved_traj.GetStateSamples(1).data(),
                                    saved_traj.GetStateSamples(1).size()))
            .head(a_dim_rom_state_part_ - idx);
  } else {
    RL_action_.segment(0, a_dim_rom_state_part_).setZero();
    int idx = 0;
    for (int mode = 0; mode < param_.n_step; mode++) {
      RL_action_.segment(idx, saved_traj.GetStateSamples(mode).size()) =
          Eigen::Map<const VectorXd>(saved_traj.GetStateSamples(mode).data(),
                                     saved_traj.GetStateSamples(mode).size());
      idx += saved_traj.GetStateSamples(mode).size();
    }
  }
  RL_action_.segment(a_dim_rom_state_part_, a_dim_rom_input_part_)
      << result.GetSolution(
             trajopt.discrete_swing_foot_pos_rt_stance_foot_x_vars_),
      result.GetSolution(
          trajopt.discrete_swing_foot_pos_rt_stance_foot_y_vars_);
  /*// Comment out the following code, since we don't use global_com_pos and
  // global_feet_pos for gradient simplicity
  RL_action_.segment(a_dim_rom_state_part_, a_dim_rom_input_part_)
      << Eigen::Map<const VectorXd>(saved_traj.get_global_com_pos().data(),
                                    saved_traj.get_global_com_pos().size()),
      Eigen::Map<const VectorXd>(saved_traj.get_global_feet_pos().data(),
                                 saved_traj.get_global_feet_pos().size());
  // RL_action_.tail<1>() is reserved for the next iteration*/

  /*// Extract the action noise
  MatrixXd RL_action_noise(RL_action_.size(), 1);
  ExtractAndReorderFromMpcSolToRlAction(trajopt, mpc_sol_noise,
                                        RL_action_noise);*/

  // Get tasks (For RL, we use CoM instead of pelvis. Because this is what the
  // planner is using)
  const OutputVector<double>* robot_output_control =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  plant_control_.SetPositionsAndVelocities(context_plant_control_.get(),
                                           robot_output_control->GetState());
  RL_task_ << plant_control_
                  .CalcCenterOfMassTranslationalVelocityInWorld(
                      *context_plant_control_)
                  .head<1>(),
      plant_control_.CalcCenterOfMassPositionInWorld(*context_plant_control_)
          .tail<1>();

  // Save data
  if (single_eval_mode_) {
    string dir_pref = dir_data + to_string(counter_) + "_";
    RL_action_.tail<1>() << param_.min_mpc_thread_loop_duration;  // nominally
    writeCSV(dir_pref + string("a.csv"), RL_action_, true);
    writeCSV(dir_pref + string("a_noise.csv"), RL_action_noise, true);
  } else {
    if (counter_ > 0) {
      string dir_pref = dir_data + to_string(counter_ - 1) + "_";
      writeCSV(dir_pref + string("s.csv"), RL_state_prev_, true);
      writeCSV(dir_pref + string("s_prime.csv"), RL_state_, true);
      writeCSV(dir_pref + string("a.csv"), RL_action_prev_, true);
      writeCSV(dir_pref + string("a_noise.csv"), RL_action_noise_prev_, true);
      VectorXd RL_addtl_info(task_dim_ + 1);
      RL_addtl_info << RL_task_prev_, current_time;
      writeCSV(dir_pref + string("RL_addtl_info.csv"), RL_addtl_info, true);
      // Initial guess of the MPC is in: <idx>_init_file.csv
    }
  }

  // Hold data for one time step
  RL_state_prev_ = RL_state_;
  RL_action_prev_ = RL_action_;
  RL_action_noise_prev_ = RL_action_noise;
  RL_task_prev_ = RL_task_;
  RL_vel_prev_ = robot_output->GetVelocities();
  prev_time_ = current_time;

  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  cout << "Time to save RL state and action data: " << elapsed.count() << endl;
}

void CassiePlannerWithOnlyRom::SaveGradientIntoFilesForRLTraining(
    const HybridRomTrajOptCassie& trajopt,
    const MathematicalProgramResult& result, const string& dir_data) const {
  // The argument `result` here should be the one WITHOUT noise

  if (!param_.get_RL_gradient_offline) {
    // Avoid computing gradient online
    return;
  }

  auto start = std::chrono::high_resolution_clock::now();
  PrintStatus("Approximate the problem with a QP\n");
  // Get the linear approximation of the cosntraints and second order
  // approximation of the cost.
  // Notes:
  //  - I just checked. If we added two bounding box constraints on the same
  //    variable, what's gonna happen is that there will be two rows of the
  //    linearized cosntraint corresponding to this. We dont' need to worry
  //    that we overconstrain the QP, since at most only one of the constraints
  //    will be active (Drake also goes through all the bounding box constraints
  //    and take the min/max of all ub/lb's)
  //  - Additionally, the solver tolerance for nonlinear constraints shouldn't
  //    matter, because my nonlinear constraints are all equality constraints,
  //    so the code here will detect that it's always active.
  MatrixXd A, H;
  VectorXd y, lb, ub, b;
  solvers::LinearizeConstraints(trajopt, result.GetSolution(), &y, &A, &lb,
                                &ub);
  solvers::SecondOrderCost(trajopt, result.GetSolution(), &H, &b);
  auto checkpoint0 = std::chrono::high_resolution_clock::now();

  // Get the linear approximation of the constraints wrt parameters
  int n_theta_yddot = rom_->n_theta_yddot();
  MatrixXd B = MatrixXd::Zero(A.rows(), n_theta_yddot);
  for (int i = 0; i < trajopt.dynamics_constraints.size(); i++) {
    // Get the row index of B matrix where dynamics constraint starts
    VectorXd ind_start = solvers::GetConstraintRows(
        trajopt, trajopt.dynamics_constraints_bindings[i]);
    // cout << "ind_start = " << ind_start(0) << endl;

    // Get gradient and fill in B matrix
    B.middleRows(ind_start(0), n_z_) =
        trajopt.dynamics_constraints[i]->GetGradientWrtTheta(result.GetSolution(
            trajopt.dynamics_constraints_bindings[i].variables()));

    // TODO: Could we use dual_sol to speed up the computation?
    //    // Testing -- getting cost gradient by envelope thm
    //    // Minus sign for gradient descent instead of ascent
    //    VectorXd dual_sol =
    //        result.GetDualSolution(trajopt.dynamics_constraints_bindings[i]);
  }

  auto checkpoint1 = std::chrono::high_resolution_clock::now();

  // Do gradient here (inside the MPC loop)?
  PrintStatus("ExtractActiveConstraintAndDoLinearSolve\n");
  MatrixXd grad_w_wrt_theta =
      ExtractActiveConstraintAndDoLinearSolve(A, B, H, y, lb, ub, b);
  auto checkpoint2 = std::chrono::high_resolution_clock::now();

  // Select and pack gradient in the order of policy action
  PrintStatus("Select and pack gradient\n");
  MatrixXd grad_w_wrt_theta_orderred(RL_action_.size(), n_theta_yddot);
  ExtractAndReorderFromMpcSolToRlAction(trajopt, grad_w_wrt_theta,
                                        grad_w_wrt_theta_orderred);

  auto checkpoint3 = std::chrono::high_resolution_clock::now();

  //  writeCSV("test_A.csv", A);
  //  writeCSV("test_B.csv", B);
  //  writeCSV("test_H.csv", H);
  //  writeCSV("test_y.csv", y);
  //  writeCSV("test_lb.csv", lb);
  //  writeCSV("test_ub.csv", ub);
  //  writeCSV("test_b.csv", b);
  //  writeCSV("test_grad_w_wrt_theta.csv", grad_w_wrt_theta);
  //  writeCSV("test_grad_w_wrt_theta_orderred.csv", grad_w_wrt_theta_orderred);

  // Save gradient
  string dir_pref = dir_data + to_string(counter_) + "_";
  writeCSV(dir_pref + "grad_a_wrt_theta.csv", grad_w_wrt_theta_orderred);

  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  cout << "Time to get MPC gradient: " << elapsed.count() << endl;
  elapsed = checkpoint1 - start;
  cout << "  Approximate with QP: " << elapsed.count() << endl;
  elapsed = checkpoint1 - checkpoint0;
  cout << "  Approximate with QP (only B part): " << elapsed.count() << endl;
  elapsed = checkpoint2 - checkpoint1;
  cout << "  Linear solve: " << elapsed.count() << endl;
  elapsed = checkpoint3 - checkpoint2;
  cout << "  Ordering the rows of the gradient: " << elapsed.count() << endl;
}

void CassiePlannerWithOnlyRom::ExtractAndReorderFromMpcSolToRlAction(
    const HybridRomTrajOptCassie& trajopt, const MatrixXd& matrix_in_w_order,
    MatrixXd& matrix_in_a_order) const {
  DRAKE_DEMAND(matrix_in_w_order.rows() == trajopt.num_vars());
  DRAKE_DEMAND(matrix_in_a_order.rows() == RL_action_.size());

  int idx_start_x;
  int idx_start_h;
  int idx_start_xp;
  int idx_start_footsteps;
  trajopt.GetStartOfVariableIndices(idx_start_x, idx_start_h, idx_start_xp,
                                    idx_start_footsteps);

  // ROM state part
  const std::vector<int>& mode_start = trajopt.mode_start();
  const std::vector<int>& mode_lengths = trajopt.mode_lengths();
  int i = 0;
  for (int mode = 0; mode < mode_lengths.size(); mode++) {
    for (int knot_idx = 0; knot_idx < mode_lengths.at(mode); knot_idx++) {
      if (knot_idx == 0 && mode > 0) {
        matrix_in_a_order.middleRows(i * n_z_, n_z_) =
            matrix_in_w_order.middleRows(idx_start_xp + (mode - 1) * n_z_,
                                         n_z_);
      } else {
        matrix_in_a_order.middleRows(i * n_z_, n_z_) =
            matrix_in_w_order.middleRows((mode_start[mode] + knot_idx) * n_z_,
                                         n_z_);
      }
      i++;
      if (i == n_knots_used_for_RL_action_) {
        mode = mode_lengths.size();  // To break the outer for loop
        break;                       // To break the inner for loop
      }
    }
  }
  // ROM footstep part
  // WARNING: we assume two footsteps here
  DRAKE_DEMAND(param_.n_step == 2);
  matrix_in_a_order.middleRows<2>(a_dim_rom_state_part_) =
      matrix_in_w_order.middleRows<2>(idx_start_footsteps);
  matrix_in_a_order.middleRows<2>(a_dim_rom_state_part_ + 2) =
      matrix_in_w_order.middleRows<2>(idx_start_footsteps + 2);
  // Last row (zeros) is the gradient of delta_t
  matrix_in_a_order.bottomRows<1>().setZero();

  // Some unit testing
  if (single_eval_mode_ || param_.unit_testing) {
    // Testing
    const auto& var_idx_map = trajopt.decision_variable_index();
    DRAKE_DEMAND(var_idx_map.at(trajopt.timestep(0)(0).get_id()) ==
                 idx_start_h);
    DRAKE_DEMAND(var_idx_map.at(trajopt.z_post_impact_vars()(0).get_id()) ==
                 idx_start_xp);
    DRAKE_DEMAND(var_idx_map.at(
                     trajopt.discrete_swing_foot_pos_rt_stance_foot_x_vars()(0)
                         .get_id()) == idx_start_footsteps);
  }
}

MatrixXd CassiePlannerWithOnlyRom::ExtractActiveConstraintAndDoLinearSolve(
    const MatrixXd& A, const MatrixXd& B, const MatrixXd& H, const VectorXd& y,
    const VectorXd& lb, const VectorXd& ub, const VectorXd& b,
    double active_tol, int method_to_solve_system_of_equations) const {
  /// Step 1 - Extract active cosntraints
  int nt_i = B.cols();
  int nw_i = A.cols();

  int nc_active_i = 0;
  vector<int> active_row_indices(y.rows());
  for (int i = 0; i < y.rows(); i++) {
    if (y(i) >= ub(i) - active_tol || y(i) <= lb(i) + active_tol) {
      active_row_indices.at(nc_active_i) = i;
      nc_active_i++;
    }
  }

  MatrixXd A_active(nc_active_i, nw_i);
  MatrixXd B_active(nc_active_i, nt_i);
  for (int i = 0; i < nc_active_i; i++) {
    A_active.row(i) = A.row(active_row_indices.at(i));
    B_active.row(i) = B.row(active_row_indices.at(i));
  }

  //  cout << "nt_i = " << nt_i << endl;
  //  cout << "nw_i = " << nw_i << endl;
  //  cout << "nc_active_i = " << nc_active_i << endl;

  // Note that no need to extract independent rows here (the method we use
  // doesn't requires it)

  bool is_testing = false;
  if (is_testing) {
    // Run a quadprog to check if the solution to the following problem is 0
    // Theoratically, it should be 0. Otherwise, something is wrong
    // min 0.5*w^T Q w + c^T w
    // st  A w = 0
    MathematicalProgram quadprog;
    auto w2 = quadprog.NewContinuousVariables(nw_i, "w2");
    quadprog.AddLinearConstraint(A_active, VectorXd::Zero(nc_active_i),
                                 VectorXd::Zero(nc_active_i), w2);
    quadprog.AddQuadraticCost(H, b, w2);

    // (Testing) use snopt to solve the QP
    bool use_snopt = true;
    drake::solvers::SnoptSolver snopt_solver;

    auto start = std::chrono::high_resolution_clock::now();
    const auto result =
        use_snopt ? snopt_solver.Solve(quadprog) : Solve(quadprog);
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;

    auto solution_result = result.get_solution_result();
    if (result.is_success()) {
      VectorXd w_sol_check = result.GetSolution(quadprog.decision_variables());
      cout << solution_result << " | " << elapsed.count() << "| "
           << result.get_optimal_cost() << " | " << w_sol_check.norm() << " | "
           << w_sol_check.transpose() * (b) << endl;
    } else {
      cout << solution_result << " | " << elapsed.count() << " | "
           << result.get_optimal_cost() << endl;
    }
  }

  /// Step 2 linear solve
  MatrixXd Pi(nw_i, B_active.cols());

  double residual_tol = 1e-4;
  bool successful = false;
  while (!successful) {
    // H_ext = [H A'; A 0]
    MatrixXd H_ext(nw_i + nc_active_i, nw_i + nc_active_i);
    H_ext.block(0, 0, nw_i, nw_i) = (H);
    H_ext.block(0, nw_i, nw_i, nc_active_i) = A_active.transpose();
    H_ext.block(nw_i, 0, nc_active_i, nw_i) = (A_active);
    H_ext.block(nw_i, nw_i, nc_active_i, nc_active_i).setZero();

    if (method_to_solve_system_of_equations == 4) {
      // Method 4: linear solve with householderQr ///////////////////////////
      MatrixXd B_aug = MatrixXd::Zero(H_ext.rows(), B_active.cols());
      B_aug.bottomRows(nc_active_i) = -(B_active);

      MatrixXd sol = H_ext.householderQr().solve(B_aug);
      Pi = sol.topRows(nw_i);

      double max_error = (H_ext * sol - B_aug).cwiseAbs().maxCoeff();
      cout << "max_error = " << max_error << "\n";
      if (max_error > residual_tol || std::isnan(max_error) ||
          std::isinf(max_error)) {
        cout << "max_error = " << max_error << "; switch to method 5\n";
        method_to_solve_system_of_equations = 5;
      } else {
        successful = true;
      }

    } else if (method_to_solve_system_of_equations == 5) {
      // Method 5: linear solve with ColPivHouseholderQR /////////////////////
      MatrixXd B_aug = MatrixXd::Zero(H_ext.rows(), B_active.cols());
      B_aug.bottomRows(nc_active_i) = -(B_active);

      MatrixXd sol = Eigen::ColPivHouseholderQR<MatrixXd>(H_ext).solve(B_aug);
      Pi = sol.topRows(nw_i);

      double max_error = (H_ext * sol - B_aug).cwiseAbs().maxCoeff();
      cout << "max_error = " << max_error << "\n";
      if (max_error > residual_tol || std::isnan(max_error) ||
          std::isinf(max_error)) {
        cout << "max_error = " << max_error << "; move on.\n";
      }
      break;
    }
  }

  //
  return Pi;
}

void CassiePlannerWithOnlyRom::PrintCost(
    const HybridRomTrajOptCassie& trajopt,
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
  double rom_des_pos_cost = solvers::EvalCostGivenSolution(
      result, trajopt.rom_des_pos_cost_bindings_);
  if (rom_des_pos_cost > 0) {
    cout << "rom_des_pos_cost = " << rom_des_pos_cost << endl;
  }
  double rom_des_vel_cost = solvers::EvalCostGivenSolution(
      result, trajopt.rom_des_vel_cost_bindings_);
  if (rom_des_vel_cost > 0) {
    cout << "rom_des_vel_cost = " << rom_des_vel_cost << endl;
  }
  double rom_end_goal_pos_cost = solvers::EvalCostGivenSolution(
      result, trajopt.rom_end_goal_pos_cost_bindings_);
  if (rom_end_goal_pos_cost > 0) {
    cout << "rom_end_goal_pos_cost = " << rom_end_goal_pos_cost << endl;
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
void CassiePlannerWithOnlyRom::BookKeeping(
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

void CassiePlannerWithOnlyRom::PrintAllCostsAndConstraints(
    const HybridRomTrajOptCassie& trajopt) const {
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

void CassiePlannerWithOnlyRom::WarmStartGuess(
    const VectorXd& quat_xyz_shift,
    const VectorXd& current_local_stance_foot_pos,
    const vector<Vector2d>& reg_local_delta_footstep, const int global_fsm_idx,
    int first_mode_knot_idx, double current_time,
    HybridRomTrajOptCassie* trajopt) const {
  int starting_mode_idx_for_heuristic =
      (param_.n_step - 1) - (global_fsm_idx - prev_global_fsm_idx_) + 1;

  if (starting_mode_idx_for_heuristic <= 0) {
    PrintStatus("Set heuristic initial guess for all variables");
    // Set heuristic initial guess for all variables
    trajopt->SetHeuristicInitialGuess(param_, h_guess_, y_guess_, dy_guess_,
                                      tau_guess_, reg_local_delta_footstep,
                                      first_mode_knot_idx, 0);
  } else {
    trajopt->SetHeuristicInitialGuess(param_, h_guess_, y_guess_, dy_guess_,
                                      tau_guess_, reg_local_delta_footstep,
                                      first_mode_knot_idx,
                                      starting_mode_idx_for_heuristic);

    /// Reuse the solution
    // Rotate the previous solution in global frame according to the
    // current global-to-local-shift
    // TODO: also need to do the same thing to predicted_com_vel_
    // TODO: currently I am NOT transforming the ROM state between frames.
    //  I believe transforming it is better theoretically.
    // We only rotate frames for footstep, because it's always relative to the
    // previous foot.
    MatrixXd local_delta_footstep = global_delta_footstep_;
    TransformBetweenGlobalAndLocalFrame2D(true, true, true, quat_xyz_shift,
                                          global_delta_footstep_,
                                          &local_delta_footstep);

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

      // 4. FOM pre-impact
      trajopt->SetInitialGuess(
          trajopt->discrete_swing_foot_pos_rt_stance_foot_x_vars(local_fsm_idx),
          local_delta_footstep.col(prev_local_fsm_idx).head<1>());
      trajopt->SetInitialGuess(
          trajopt->discrete_swing_foot_pos_rt_stance_foot_y_vars(local_fsm_idx),
          local_delta_footstep.col(prev_local_fsm_idx).tail<1>());
    }

    // The robot fell when initializing eps_rom_ and local_predicted_com_vel_.
    // This makes sense, because eps_rom_ should be close to 0, and smaller
    // local_predicted_com_vel_ is more stable (walking slower).
    if (global_fsm_idx == prev_global_fsm_idx_) {
      // 5. slack variable for initial fom-rom mapping
      //    trajopt->SetInitialGuess(trajopt->eps_rom_var_, eps_rom_);
      // 6. predicted com vel at the end of the immediate future mode
      //    trajopt->SetInitialGuess(trajopt->predicted_com_vel_var_,
      //                             local_predicted_com_vel_);
    }

    // 7. y_end_of_last_mode_rt_init_stance_foot_var
    if (global_fsm_idx == prev_global_fsm_idx_) {
      MatrixXd local_y_end_of_last_mode = global_y_end_of_last_mode_;
      TransformBetweenGlobalAndLocalFrame2D(true, false, true, quat_xyz_shift,
                                            global_y_end_of_last_mode_,
                                            &local_y_end_of_last_mode);
      local_y_end_of_last_mode -= current_local_stance_foot_pos.head<2>();

      trajopt->SetInitialGuess(
          trajopt->y_end_of_last_mode_rt_init_stance_foot_var(),
          local_y_end_of_last_mode);
    }

    // For cascaded LIPM MPC
    if (param_.n_step_lipm > 1) {
      MatrixXd local_x_lipm = global_x_lipm_;
      MatrixXd local_u_lipm = global_u_lipm_;
      TransformBetweenGlobalAndLocalFrame2D(true, false, false, quat_xyz_shift,
                                            global_x_lipm_, &local_x_lipm);
      TransformBetweenGlobalAndLocalFrame2D(true, false, true, quat_xyz_shift,
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

void CassiePlannerWithOnlyRom::ResolveWithAnotherSolver(
    const HybridRomTrajOptCassie& trajopt,
    const MathematicalProgramResult& result, const string& prefix,
    double current_time, const VectorXd& quat_xyz_shift,
    const VectorXd& current_local_stance_foot_pos) const {
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
    MatrixXd local_delta_footstep(2, trajopt.num_modes());
    local_delta_footstep.row(0) = result2.GetSolution(
        trajopt.discrete_swing_foot_pos_rt_stance_foot_x_vars());
    local_delta_footstep.row(1) = result2.GetSolution(
        trajopt.discrete_swing_foot_pos_rt_stance_foot_y_vars());
    // Transform back to the world frame
    MatrixXd global_delta_footstep = local_delta_footstep;
    TransformBetweenGlobalAndLocalFrame2D(false, false, true, quat_xyz_shift,
                                          local_delta_footstep,
                                          &global_delta_footstep);

    writeCSV(param_.dir_data + prefix + "global_delta_footstep_snopt.csv",
             global_delta_footstep);
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

void CassiePlannerWithOnlyRom::PrintTrajMsg(
    dairlib::lcmt_timestamped_saved_traj* traj_msg) const {
  // I created this function to debug the memory leak. (I didn't copy utime in
  // HybridRomPlannerTrajectory's copy assignment)
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

VectorXd CassiePlannerWithOnlyRom::RunIkForCoMHeight(
    const MultibodyPlant<double>& plant, double pelvis_height) {
  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  int n_x = n_q + n_v;
  std::map<string, int> positions_map =
      multibody::makeNameToPositionsMap(plant);

  VectorXd q_init_guess;
  VectorXd q_ik_guess = VectorXd::Zero(n_q);
  Eigen::Vector4d quat(2000.06, -0.339462, -0.609533, -0.760854);
  q_ik_guess << quat.normalized(), 0.000889849, 0.000626865, 1.0009, -0.0112109,
      0.00927845, -0.000600725, -0.000895805, 1.15086, 0.610808, -1.38608,
      -1.35926, 0.806192, 1.00716, -M_PI / 2, -M_PI / 2;

  double eps = 1e-3;
  Vector3d eps_vec = eps * VectorXd::Ones(3);
  Vector3d pelvis_pos(0.0, 0.0, pelvis_height);
  double stance_toe_pos_x = 0;
  Vector3d stance_toe_pos(stance_toe_pos_x, 0.12, 0.05);
  double swing_toe_pos_x = 0;
  Vector3d swing_toe_pos(swing_toe_pos_x, -0.12, 0.05);
  // cout << "swing foot height = " <<
  //      0.05 + 0.1 * (-abs((i - N / 2.0) / (N / 2.0)) + 1);

  const auto& world_frame = plant.world_frame();
  const auto& pelvis_frame = plant.GetFrameByName("pelvis");
  const auto& toe_left_frame = plant.GetFrameByName("toe_left");
  const auto& toe_right_frame = plant.GetFrameByName("toe_right");

  drake::multibody::InverseKinematics ik(plant);
  ik.AddPositionConstraint(pelvis_frame, Vector3d(0, 0, 0), world_frame,
                           pelvis_pos - eps * VectorXd::Ones(3),
                           pelvis_pos + eps * VectorXd::Ones(3));
  ik.AddOrientationConstraint(pelvis_frame, RotationMatrix<double>(),
                              world_frame, RotationMatrix<double>(), eps);
  ik.AddPositionConstraint(toe_left_frame, Vector3d(0, 0, 0), world_frame,
                           stance_toe_pos - eps_vec, stance_toe_pos + eps_vec);
  ik.AddPositionConstraint(toe_right_frame, Vector3d(0, 0, 0), world_frame,
                           swing_toe_pos - eps_vec, swing_toe_pos + eps_vec);
  ik.get_mutable_prog()->AddLinearConstraint(
      (ik.q())(positions_map.at("hip_yaw_left")) == 0);
  ik.get_mutable_prog()->AddLinearConstraint(
      (ik.q())(positions_map.at("hip_yaw_right")) == 0);
  // Four bar linkage constraint (without spring)
  ik.get_mutable_prog()->AddLinearConstraint(
      (ik.q())(positions_map.at("knee_left")) +
          (ik.q())(positions_map.at("ankle_joint_left")) ==
      M_PI * 13 / 180.0);
  ik.get_mutable_prog()->AddLinearConstraint(
      (ik.q())(positions_map.at("knee_right")) +
          (ik.q())(positions_map.at("ankle_joint_right")) ==
      M_PI * 13 / 180.0);

  ik.get_mutable_prog()->SetInitialGuess(ik.q(), q_ik_guess);
  const auto result = Solve(ik.prog());
  // SolutionResult solution_result = result.get_solution_result();
  // cout << "\n" << to_string(solution_result) << endl;
  // cout << "  Cost:" << result.get_optimal_cost() << std::endl;
  const auto q_sol = result.GetSolution(ik.q());
  // cout << "  q_sol = " << q_sol.transpose() << endl;
  VectorXd q_sol_normd(n_q);
  q_sol_normd << q_sol.head(4).normalized(), q_sol.tail(n_q - 4);
  // cout << "  q_sol_normd = " << q_sol_normd << endl;
  q_ik_guess = q_sol_normd;
  q_init_guess = q_sol_normd;
  return q_init_guess;
}

}  // namespace goldilocks_models
}  // namespace dairlib
