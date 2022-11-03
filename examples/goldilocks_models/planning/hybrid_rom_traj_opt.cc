#include "examples/goldilocks_models/planning/hybrid_rom_traj_opt.h"

#include <math.h>

#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "examples/goldilocks_models/planning/FoM_guard_and_feet_constraint.h"
#include "examples/goldilocks_models/planning/FoM_reset_map_constraint.h"
#include "examples/goldilocks_models/planning/FoM_stance_foot_constraint.h"
#include "examples/goldilocks_models/planning/dynamics_constraint.h"
#include "examples/goldilocks_models/planning/kinematics_constraint.h"

#include "drake/math/autodiff.h"
#include "drake/solvers/decision_variable.h"

typedef std::numeric_limits<double> dbl;

namespace dairlib {
namespace goldilocks_models {

using std::cout;
using std::endl;
using std::pair;
using std::string;
using std::to_string;
using std::vector;

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::AutoDiffXd;
using drake::VectorX;
using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::solvers::Binding;
using drake::solvers::Constraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::VectorXDecisionVariable;
using drake::symbolic::Expression;
using drake::systems::trajectory_optimization::MultipleShooting;
using drake::trajectories::PiecewisePolynomial;

HybridRomTrajOpt::HybridRomTrajOpt(
    const vector<int>& num_time_samples, const MatrixXd& Q, const MatrixXd& R,
    const ReducedOrderModel& rom, const VectorXd& init_rom_state,
    const std::vector<double>& max_swing_distance, bool start_with_left_stance,
    bool zero_touchdown_impact, const std::set<int>& relax_index,
    const PlannerSetting& param, const vector<int>& num_time_samples_ds,
    bool start_in_double_support_phase,
    const std::set<int>& idx_constant_rom_vel_during_double_support,
    bool print_status)
    : MultipleShooting(
          rom.n_tau(), 2 * rom.n_y(),
          std::accumulate(num_time_samples.begin(), num_time_samples.end(), 0) -
              num_time_samples.size() + 1,
          1e-8, 1e8),
      num_modes_(num_time_samples.size()),
      mode_lengths_(num_time_samples),
      z_post_impact_vars_(NewContinuousVariables(
          (2 * rom.n_y()) * (num_time_samples.size()), "xp")),
      n_y_(rom.n_y()),
      n_z_(2 * rom.n_y()),
      rom_(rom),
      start_with_left_stance_(start_with_left_stance),
      num_time_samples_ds_(num_time_samples_ds),
      use_double_support_mode_in_planner_(num_time_samples_ds.at(0) > 0),
      start_in_double_support_phase_(start_in_double_support_phase),
      constant_rom_vel_during_double_support_(
          !idx_constant_rom_vel_during_double_support.empty()),
      single_support_duration_(param.gains.left_support_duration),
      double_support_duration_(param.gains.double_support_duration),
      print_status_(print_status) {
  DRAKE_DEMAND(max_swing_distance.size() == num_time_samples.size());
  DRAKE_DEMAND(n_y_ == 3);  // hard-coded in many places

  /// Some paramters
  const double back_limit_wrt_pelvis = param.gains.back_limit_wrt_pelvis;
  const double front_limit_wrt_pelvis = param.gains.front_limit_wrt_pelvis;
  const double right_limit_wrt_pelvis = param.gains.right_limit_wrt_pelvis;
  const double left_limit_wrt_pelvis = param.gains.left_limit_wrt_pelvis;
  const double right_limit_wrt_stance_ft =
      param.gains.right_limit_wrt_stance_ft;
  const double left_limit_wrt_stance_ft = param.gains.max_lipm_step_length;
  const double max_step_length = param.gains.max_lipm_step_length;
  // Bounds on the rom state
  Eigen::VectorXd z_lb(6);
  z_lb << -2, -2, 0.5, -10, -10, -10;
  Eigen::VectorXd z_ub(6);
  z_ub << 2, 2, 0.5, 10, 10, 10;

  /// Some constant matrices
  Eigen::RowVectorXd A_discrete_map_3d(3);
  A_discrete_map_3d << 1, -1, -1;
  Eigen::RowVectorXd A_discrete_map_2d(2);
  A_discrete_map_2d << 1, -1;

  /// Add more decision variables
  // TODO: haven't finished foot variable; need to warm start it
  discrete_swing_foot_pos_rt_stance_foot_x_vars_ =
      NewContinuousVariables(num_time_samples.size(), "touchdown_foot_pos_x");
  discrete_swing_foot_pos_rt_stance_foot_y_vars_ =
      NewContinuousVariables(num_time_samples.size(), "touchdown_foot_pos_y");
  y_end_of_last_mode_rt_init_stance_foot_var_ =
      NewContinuousVariables(2, "y_end_of_last_mode_rt_init_stance_foot");

  /// Construct mode_start
  int counter = 0;
  for (int i = 0; i < num_modes_; i++) {
    mode_start_.push_back(counter);
    counter += mode_lengths_[i] - 1;
  }

  /// Adding costs and constraints
  // Add cost
  PrintStatus("Adding cost...");
  auto y = this->state();
  auto tau = this->input();
  this->DoAddRunningCost((y.tail(n_y_).transpose() * Q * y.tail(n_y_))(0, 0),
                         &rom_state_cost_bindings_);
  this->DoAddRunningCost((tau.transpose() * R * tau)(0, 0),
                         &rom_input_cost_bindings_);

  // Initial state relaxation (both ROM and FOM init use this)
  int n_eps = relax_index.size();
  eps_rom_var_ = NewContinuousVariables(n_eps, "eps_rom");
  if (!relax_index.empty()) {
    init_rom_relax_cost_bindings_.push_back(AddQuadraticCost(
        MatrixXd::Identity(n_eps, n_eps), VectorXd::Zero(n_eps), eps_rom_var_));
  }
  /* // "linear cost + lower bound" version
  if (relax_index.size() == 1 && *(relax_index.begin()) == 5) {
    init_rom_relax_cost_bindings_.push_back(AddLinearCost(eps_rom_var_(0)));
    AddBoundingBoxConstraint(0, std::numeric_limits<double>::infinity(),
                             eps_rom_var_);
  } else {
    init_rom_relax_cost_bindings_.push_back(AddQuadraticCost(
        MatrixXd::Identity(n_eps, n_eps), VectorXd::Zero(n_eps),
        eps_rom_var_));
  }*/

  // Add initial state constraint
  // TODO: we can probably even set all state's height to the desired constant
  //  for model
  //  `20220417_rom27_big_range_bigger_step_size_6e-3_torque_weight_dominate`
  //  because it's almost a constant for the optimal model.
  VectorXDecisionVariable z_00 = state_vars_by_mode(0, 0);
  PrintStatus("Adding constraint -- initial state of ROM...");
  int idx_eps = 0;
  Eigen::RowVectorXd A = Eigen::RowVectorXd::Ones(2);
  for (int i = 0; i < 6; i++) {
    // Impose constraint (either relaxed ot not)
    if (relax_index.find(i) == relax_index.end()) {
      AddBoundingBoxConstraint(init_rom_state(i), init_rom_state(i), z_00(i));
    } else {
      // Constraint: init_rom_state(i) == z_00(i) + eps_rom_var_(idx_eps)
      AddLinearConstraint(
          A, init_rom_state(i), init_rom_state(i),
          {z_00.segment<1>(i), eps_rom_var_.segment<1>(idx_eps)});
      idx_eps++;
    }
  }

  // Add constraint for the last ROM position rt init stance foot
  // Constraint:
  //    COM at the end of the last mode rt the init stance foot =
  //      step_1 + ... + step_n + COM at the end of the last mode rt stance foot
  // => y_end_of_last_mode_rt_init_stance_foot_var_ =
  //      sum(discrete_swing_foot_pos_rt_stance_foot_vars_) +
  //      y_of_last_mode_post_event
  // => 0 =
  //      -y_end_of_last_mode_rt_init_stance_foot_var_ +
  //      y_of_last_mode_post_event +
  //      sum(discrete_swing_foot_pos_rt_stance_foot_vars_)

  VectorXDecisionVariable z_ff_post = state_vars_by_mode(num_modes_, 0);
  Eigen::RowVectorXd A_last_pos = Eigen::RowVectorXd::Ones(num_modes_ + 2);
  A_last_pos(0) = -1;
  AddLinearEqualityConstraint(
      A_last_pos, 0,
      {y_end_of_last_mode_rt_init_stance_foot_var_.segment<1>(0),
       z_ff_post.segment<1>(0),
       discrete_swing_foot_pos_rt_stance_foot_x_vars_});
  AddLinearEqualityConstraint(
      A_last_pos, 0,
      {y_end_of_last_mode_rt_init_stance_foot_var_.segment<1>(1),
       z_ff_post.segment<1>(1),
       discrete_swing_foot_pos_rt_stance_foot_y_vars_});

  // Loop over modes to add more constraints
  bool left_stance = start_with_left_stance;
  for (int mode = 0; mode < num_modes_; mode++) {
    PrintStatus("Mode " + std::to_string(mode) + "===========================");

    // Common decision variables
    VectorXDecisionVariable z_f_pre =
        state_vars_by_mode(mode, mode_lengths_[mode] - 1);
    VectorXDecisionVariable z_f_post = state_vars_by_mode(mode + 1, 0);
    VectorXDecisionVariable touchdown_foot_var_x =
        discrete_swing_foot_pos_rt_stance_foot_x_vars(mode);
    VectorXDecisionVariable touchdown_foot_var_y =
        discrete_swing_foot_pos_rt_stance_foot_y_vars(mode);

    // Add bounding box constraition on the state
    PrintStatus("Adding constraint -- bounding box on ROM state");
    for (int j = 0; j < mode_lengths_[mode]; j++) {
      AddBoundingBoxConstraint(z_lb, z_ub, state_vars_by_mode(mode, j));
    }
    if (mode == num_modes_ - 1)
      AddBoundingBoxConstraint(z_lb, z_ub, state_vars_by_mode(mode + 1, 0));

    // Add dynamics constraints at collocation points
    PrintStatus("Adding constraint -- dynamics");
    for (int j = 0; j < mode_lengths_[mode] - 1; j++) {
      bool set_zero_accel =
          (constant_rom_vel_during_double_support_ &&
           (j >= mode_lengths_[mode] - num_time_samples_ds_[mode]));
      auto dyn_constraint = std::make_shared<planning::DynamicsConstraint>(
          rom,
          set_zero_accel ? idx_constant_rom_vel_during_double_support
                         : empty_idx_set_,
          "rom_dyn_" + std::to_string(mode) + "_" + std::to_string(j));
      DRAKE_DEMAND(static_cast<int>(dyn_constraint->num_constraints()) ==
                   num_states());
      dyn_constraint->SetConstraintScaling(rom_dyn_constraint_scaling_);
      int time_index = mode_start_[mode] + j;
      AddConstraint(
          dyn_constraint,
          {state_vars_by_mode(mode, j),
           u_vars().segment(time_index * num_inputs(), num_inputs()),
           state_vars_by_mode(mode, j + 1),
           u_vars().segment((time_index + 1) * num_inputs(), num_inputs()),
           h_vars().segment(time_index, 1)});
    }

    // Add discrete map constraint
    PrintStatus("Adding constraint -- ROM reset map");
    // Variables:
    //   y = COM pos rt stance foot
    //   discrete_input = swing foot touchdown pos rt stance foot
    //   new_y = COM pos rt the *new* stance foot
    // Therefore, the constraint is
    //   new_y = y - discrete_input
    // =>  y - new_y - discrete_input = 0
    AddLinearEqualityConstraint(
        A_discrete_map_3d, 0,
        {z_f_pre.segment<1>(0), z_f_post.segment<1>(0), touchdown_foot_var_x});
    AddLinearEqualityConstraint(
        A_discrete_map_3d, 0,
        {z_f_pre.segment<1>(1), z_f_post.segment<1>(1), touchdown_foot_var_y});
    AddLinearEqualityConstraint(
        A_discrete_map_2d, 0, {z_f_pre.segment<1>(2), z_f_post.segment<1>(2)});

    /// Constraints on foot steps
    // Step region (rt CoM) constraint
    PrintStatus(
        "Adding constraint -- step region (end of mode; pre and post event)");
    // Variables:
    //   y_pre = COM pos rt stance foot right before the discrete event
    //   y_post = COM pos rt the *new* stance foot (i.e. right after the
    //    discrete event )
    // Constraints:
    //     lb <= swing foot pos rt pelvis before event <= ub
    //     lb <= swing foot pos rt pelvis after event <= ub
    // =>  lb <= -y_pre <= ub
    //     lb <= -y_post <= ub
    // =>  -lb >= y_pre >= -ub
    //     -lb >= y_post >= -ub

    double lb_swing_x = back_limit_wrt_pelvis;
    double ub_swing_x = front_limit_wrt_pelvis;
    double lb_swing_y =
        left_stance ? -left_limit_wrt_pelvis : right_limit_wrt_pelvis;
    double ub_swing_y =
        left_stance ? -right_limit_wrt_pelvis : left_limit_wrt_pelvis;
    // 1. pre discrete event
    if (mode != 0) {
      // do nothing for the very fist mode (mode == 0), because the model is
      // passive in continuous time and cannot change stance leg length
      //      AddBoundingBoxConstraint(lb_swing_x, ub_swing_x, -z_f_pre(0));
      //      AddBoundingBoxConstraint(lb_swing_y, ub_swing_y, -z_f_pre(1));
      AddBoundingBoxConstraint(-ub_swing_x, -lb_swing_x, z_f_pre(0));
      AddBoundingBoxConstraint(-ub_swing_y, -lb_swing_y, z_f_pre(1));
    }
    // 2. post discrete event
    AddBoundingBoxConstraint(lb_swing_x, ub_swing_x, z_f_post(0));
    AddBoundingBoxConstraint(lb_swing_y, ub_swing_y, z_f_post(1));

    // Foot collision avoidance and step length
    PrintStatus("Adding constraint -- swing collision avoidance");
    // Variables:
    //   discrete_input = swing foot touchdown pos rt stance foot
    // Constraints:
    //     lb <= foot step rt stance foot <= ub
    // =>  lb <= discrete_input <= ub

    double lb_swing_rt_stance_x = -max_step_length;
    double ub_swing_rt_stance_x = max_step_length;
    double lb_swing_rt_stance_y =
        left_stance ? -left_limit_wrt_stance_ft : right_limit_wrt_stance_ft;
    double ub_swing_rt_stance_y =
        left_stance ? -right_limit_wrt_stance_ft : left_limit_wrt_stance_ft;
    AddBoundingBoxConstraint(lb_swing_rt_stance_x, ub_swing_rt_stance_x,
                             touchdown_foot_var_x);
    AddBoundingBoxConstraint(lb_swing_rt_stance_y, ub_swing_rt_stance_y,
                             touchdown_foot_var_y);

    // TODO: I dont' have a constraint on the distance between the current swing
    //  foot location and the desired touchdown location (but this might not be
    //  necessary)

    left_stance = !left_stance;
  }
}

void HybridRomTrajOpt::AddConstraintAndCostForLastFootStep(
    double w_predict_lipm_v, const Eigen::VectorXd& des_predicted_xy_vel,
    double stride_period, double com_height) {
  predicted_com_vel_var_ = NewContinuousVariables(2, "predicted_com_vel");

  // Constraint for LIP dynamics to get `predicted_com_vel_var_`
  // Velocity at the end of mode after horizon. (Given the initial position and
  // velocity, we can get the solution to the LIPM dynamics. Hence, the velocity
  // at the end of step)

  // Constraints (for either x or y):
  //     v_end_of_mode = CoM_rt_stnace_foot * omega_sinh + CoM_dot * cosh
  // =>  predicted_com_vel_var_ = y * omega_sinh + y_dot * cosh
  // =>  omega_sinh * y +  cosh * y_dot - predicted_com_vel_var_ = 0
  // =>  [omega_sinh, cosh, -1]^T * [y, y_dot, predicted_com_vel_var_] = 0
  double omega = std::sqrt(9.81 / com_height);

  Eigen::RowVectorXd A(3);
  A << omega * std::sinh(omega * stride_period),
      std::cosh(omega * stride_period), -1;
  VectorXDecisionVariable z_f_post = state_vars_by_mode(num_modes_, 0);
  AddLinearEqualityConstraint(A, 0,
                              {z_f_post.segment<1>(0), z_f_post.segment<1>(3),
                               predicted_com_vel_var_.segment<1>(0)});
  AddLinearEqualityConstraint(A, 0,
                              {z_f_post.segment<1>(1), z_f_post.segment<1>(4),
                               predicted_com_vel_var_.segment<1>(1)});

  // Initial guess for the new variable
  this->SetInitialGuess(predicted_com_vel_var_, des_predicted_xy_vel);

  // Desired value for predicted velocity
  // 20211109: It looks like the constraint version is solved much faster
  // 20220420: Cannot use the constraint version because it sometimes caused
  // overconstraining (see 20220420 folder of mpc improvement). Also, the speed
  // doesn't seem to be affected in some solves, so maybe it's not too bad.
  // 1. via cost
  /*PrintStatus("Adding cost -- predicted com vel one step after horizon");
  predict_lipm_v_bindings_.push_back(
      AddQuadraticErrorCost(w_predict_lipm_v * MatrixXd::Identity(2, 2),
                            des_predicted_xy_vel, predicted_com_vel_var_));*/
  // 2. via constraint
  /*PrintStatus("Adding constraint -- predicted com vel one step after
  horizon"); AddBoundingBoxConstraint(des_predicted_xy_vel,
  des_predicted_xy_vel, predicted_com_vel_var_);*/

  // 3. half half
  PrintStatus("Adding cost -- predicted com vel one step after horizon");
  predict_lipm_v_bindings_.push_back(AddQuadraticErrorCost(
      w_predict_lipm_v * MatrixXd::Identity(1, 1),
      des_predicted_xy_vel.tail<1>(), predicted_com_vel_var_.tail<1>()));
  PrintStatus("Adding constraint -- predicted com vel one step after horizon");
  AddBoundingBoxConstraint(des_predicted_xy_vel.head<1>(),
                           des_predicted_xy_vel.head<1>(),
                           predicted_com_vel_var_.head<1>());
}

void HybridRomTrajOpt::AddCascadedLipmMPC(
    double w_predict_lipm_p, double w_predict_lipm_v,
    const std::vector<Eigen::VectorXd>& des_xy_pos,
    const std::vector<Eigen::VectorXd>& des_xy_vel, int n_step_lipm,
    double stride_period, double max_step_length, double min_step_width) {
  DRAKE_DEMAND(n_step_lipm > 0);
  double height = 0.85;  // approximation

  // stance foot for after the planner's horizon
  bool left_stance = ((num_modes_ % 2 == 0) && start_with_left_stance_) ||
                     ((num_modes_ % 2 == 1) && !start_with_left_stance_);

  // Declare lipm state and input (foot location)
  x_lipm_vars_ = NewContinuousVariables(4 * (n_step_lipm + 1), "x_lipm_vars");
  u_lipm_vars_ = NewContinuousVariables(2 * n_step_lipm, "u_lipm_vars");

  // Last step lipm mapping function
  PrintStatus("Adding constraint -- lipm mapping for the last pose");
  // Constraint:
  //   [CoM rt stance foot, CoM dot] = x_lipm_vars(0)
  // TODO!!: the ROM system code set the current pelvis to origin, but we use
  //  the stance foot in this trajopt code
  Eigen::RowVectorXd A_pos_map = Eigen::RowVectorXd::Ones(2);
  A_pos_map << 1, -1;
  AddLinearEqualityConstraint(
      A_pos_map, 0,
      {y_end_of_last_mode_rt_init_stance_foot_var_.segment<1>(0),
       x_lipm_vars_.segment<1>(0)});
  AddLinearEqualityConstraint(
      A_pos_map, 0,
      {y_end_of_last_mode_rt_init_stance_foot_var_.segment<1>(1),
       x_lipm_vars_.segment<1>(1)});
  VectorXDecisionVariable z_ff_post = state_vars_by_mode(num_modes_, 0);
  AddLinearEqualityConstraint(
      A_pos_map, 0, {z_ff_post.segment<1>(3), x_lipm_vars_.segment<1>(2)});
  AddLinearEqualityConstraint(
      A_pos_map, 0, {z_ff_post.segment<1>(4), x_lipm_vars_.segment<1>(3)});
  // Add constraint for the last ROM position rt init stance foot
  // Constraint:
  //    u_lipm_vars(0) = last foot pose (originally the full state =
  //      step_1 + ... + step_n
  // => u_lipm_var(0) = sum(discrete_swing_foot_pos_rt_stance_foot_vars_)
  // => 0 = -u_lipm_var(0) + sum(discrete_swing_foot_pos_rt_stance_foot_vars_)
  Eigen::RowVectorXd A_last_pos = Eigen::RowVectorXd::Ones(num_modes_ + 1);
  A_last_pos(0) = -1;
  AddLinearEqualityConstraint(A_last_pos, 0,
                              {u_lipm_vars_.segment<1>(0),
                               discrete_swing_foot_pos_rt_stance_foot_x_vars_});
  AddLinearEqualityConstraint(A_last_pos, 0,
                              {u_lipm_vars_.segment<1>(1),
                               discrete_swing_foot_pos_rt_stance_foot_y_vars_});

  // Add LIPM dynamics constraint
  double omega = std::sqrt(9.81 / height);
  double cosh_wT = std::cosh(omega * stride_period);
  double sinh_wT = std::sinh(omega * stride_period);
  Eigen::Matrix<double, 2, 2> A;
  A << cosh_wT, sinh_wT / omega, omega * sinh_wT, cosh_wT;
  Eigen::Matrix<double, 2, 1> B;
  B << 1 - cosh_wT, -omega * sinh_wT;
  Eigen::MatrixXd I2 = Eigen::MatrixXd::Identity(2, 2);
  Eigen::Matrix<double, 2, 5> A_lin;
  A_lin << A, B, -I2;
  for (int i = 0; i < n_step_lipm; i++) {
    VectorXDecisionVariable x_i = x_lipm_vars_by_idx(i);
    VectorXDecisionVariable u_i = u_lipm_vars_by_idx(i);
    VectorXDecisionVariable x_i_post = x_lipm_vars_by_idx(i + 1);
    // x axis
    AddLinearEqualityConstraint(
        A_lin, VectorXd::Zero(2),
        {x_i.segment<1>(0), x_i.segment<1>(2), u_i.segment<1>(0),
         x_i_post.segment<1>(0), x_i_post.segment<1>(2)});
    // y axis
    AddLinearEqualityConstraint(
        A_lin, VectorXd::Zero(2),
        {x_i.segment<1>(1), x_i.segment<1>(3), u_i.segment<1>(1),
         x_i_post.segment<1>(1), x_i_post.segment<1>(3)});
  }

  // Add step size kinematics constraint
  // TODO: I copied and pasted this block of code from lipm_mpc.cc. Need to
  //  check if it's correct
  Eigen::Matrix<double, 1, 2> A_lin_kin;
  A_lin_kin << 1, -1;
  Eigen::Matrix<double, 1, 1> ub_x;
  ub_x << max_step_length;
  Eigen::Matrix<double, 1, 1> lb_x;
  lb_x << -max_step_length;
  Eigen::Matrix<double, 1, 1> ub_y;
  ub_y << max_step_length;
  Eigen::Matrix<double, 1, 1> lb_y;
  lb_y << min_step_width;
  for (int i = 1; i <= n_step_lipm; i++) {
    VectorXDecisionVariable x_i = x_lipm_vars_by_idx(i);
    // 1. end of mode
    VectorXDecisionVariable u_i = u_lipm_vars_by_idx(i - 1);
    AddLinearConstraint(A_lin_kin, lb_x, ub_x,
                        {x_i.segment<1>(0), u_i.head<1>()});
    AddLinearConstraint(A_lin_kin, left_stance ? -ub_y : lb_y,
                        left_stance ? -lb_y : ub_y,
                        {x_i.segment<1>(1), u_i.tail<1>()});

    // 2. start of next mode
    if (i != n_step_lipm) {
      VectorXDecisionVariable u_i_post = u_lipm_vars_by_idx(i);
      AddLinearConstraint(A_lin_kin, lb_x, ub_x,
                          {x_i.segment<1>(0), u_i_post.head<1>()});
      AddLinearConstraint(A_lin_kin, left_stance ? lb_y : -ub_y,
                          left_stance ? ub_y : -lb_y,
                          {x_i.segment<1>(1), u_i_post.tail<1>()});
    }
    left_stance = !left_stance;
  }

  // Add cost
  PrintStatus("Adding cost -- lipm com pos/vel tracking");
  //  cout << "add lipm tracking\n des_xy_pos = \n";
  for (int i = 0; i < n_step_lipm; i++) {
    //    cout << des_xy_pos.at(num_modes_ + i + 1).transpose() << endl;
    predict_lipm_p_bindings_.push_back(AddQuadraticErrorCost(
        w_predict_lipm_p * I2, des_xy_pos.at(num_modes_ + i + 1),
        x_lipm_vars_by_idx(i + 1).head<2>()));
    predict_lipm_v_bindings_.push_back(AddQuadraticErrorCost(
        w_predict_lipm_v * I2, des_xy_vel.at(num_modes_ + i),
        x_lipm_vars_by_idx(i + 1).tail<2>()));
  }
}

/*void addConstraintScaling(std::unordered_map<int, double>* map,
                          vector<int> idx_vec, vector<double> s_vec) {
  DRAKE_DEMAND(idx_vec.size() == s_vec.size());
  for (int i = 0; i < idx_vec.size(); i++) {
    int idx = idx_vec[i];
    double s = s_vec[i];

    DRAKE_DEMAND(0 <= idx);
    DRAKE_DEMAND(0 < s);
    if (map->find(idx) != map->end()) {
      // Update the scaling factor
      (*map)[idx] = s;
    } else {
      // Add a new scaling factor
      map->insert(std::pair<int, double>(idx, s));
    }
  }
}

std::vector<int> CreateIdxVector(int size) {
  vector<int> ret(size);
  for (int i = 0; i < size; i++) {
    ret[i] = i;
  }
  return ret;
}

void HybridRomTrajOpt::SetScalingForLIPM() {
  addConstraintScaling(
      &rom_dyn_constraint_scaling_, CreateIdxVector(6),
      {0.02775672892501, 0.02775672892501, 0.027777777777778, 0.005674724775848,
       0.006428925019448, 0.027777777777778});
  addConstraintScaling(
      &rom_fom_mapping_constraint_scaling_, CreateIdxVector(6),
      {0.600254507911354, 0.600254507911354, 1, 0.277406361482681,
       0.127149946660597, 0.324725931313971});
}*/

void HybridRomTrajOpt::AddTimeStepConstraint(
    std::vector<double> minimum_timestep, std::vector<double> maximum_timestep,
    bool fix_duration, bool equalize_timestep_size, double first_mode_duration,
    double remaining_mode_duration_per_mode) {
  if (fix_duration && equalize_timestep_size) {
    // 1. The first mode
    if (use_double_support_mode_in_planner_) {
      // a. if there is double support phase
      double dt_ss =
          (first_mode_duration - double_support_duration_) /
          (mode_lengths_[0] -
           num_time_samples_ds_[0]);  // It's fine to be negative, since we are
                                      // not going to use this variable when
                                      // it's negative anyway
      double dt_ds =
          start_in_double_support_phase_
              ? first_mode_duration / (num_time_samples_ds_[0] - 1)
              : double_support_duration_ / (num_time_samples_ds_[0] - 1);
      for (int i = 0; i < mode_lengths_[0] - 1; i++) {
        if (i < mode_lengths_[0] - num_time_samples_ds_[0]) {
          // In single support
          AddBoundingBoxConstraint(dt_ss, dt_ss, timestep(i));
          this->SetInitialGuess(timestep(i)(0), dt_ss);
        } else {
          // In double support
          AddBoundingBoxConstraint(dt_ds, dt_ds, timestep(i));
          this->SetInitialGuess(timestep(i)(0), dt_ds);
        }
      }
    } else {
      // b. if there is only single support phase
      double dt_first_mode = first_mode_duration / (mode_lengths_[0] - 1);
      PrintStatus("Fix all timestep size in the first mode " +
                  std::to_string(dt_first_mode));
      for (int i = 0; i < mode_lengths_[0] - 1; i++) {
        AddBoundingBoxConstraint(dt_first_mode, dt_first_mode, timestep(i));
        this->SetInitialGuess(timestep(i)(0), dt_first_mode);
      }
    }

    // 2. Rest of the modes
    if (num_modes_ > 1) {
      if (use_double_support_mode_in_planner_) {
        // a. if there is double support phase
        double dt_ss = single_support_duration_ /
                       (mode_lengths_[1] - num_time_samples_ds_[1]);
        double dt_ds = double_support_duration_ / (num_time_samples_ds_[1] - 1);

        for (int i = 1; i < num_modes_; i++) {
          for (int j = 0; j < mode_lengths_[i] - 1; j++) {
            int time_idx = mode_start_[i] + j;
            if (j < mode_lengths_[1] - num_time_samples_ds_[1]) {
              // in single support
              AddBoundingBoxConstraint(dt_ss, dt_ss, timestep(time_idx));
              this->SetInitialGuess(timestep(time_idx)(0), dt_ss);
            } else {
              // in double support
              AddBoundingBoxConstraint(dt_ds, dt_ds, timestep(time_idx));
              this->SetInitialGuess(timestep(time_idx)(0), dt_ds);
            }
          }
        }
      } else {
        // b. if there is only single support phase
        double dt_rest_of_modes =
            remaining_mode_duration_per_mode / (mode_lengths_[1] - 1);
        PrintStatus("Fix all timestep size in the rest of the modes to " +
                    std::to_string(dt_rest_of_modes));
        for (int i = mode_lengths_[0] - 1; i < this->N() - 1; i++) {
          AddBoundingBoxConstraint(dt_rest_of_modes, dt_rest_of_modes,
                                   timestep(i));
          this->SetInitialGuess(timestep(i)(0), dt_rest_of_modes);
        }
      }
    }
  } else {
    // TODO: I didn't finish modify minimum_timestep for the case when we use
    //  double support phase
    DRAKE_UNREACHABLE();

    for (int i = 0; i < num_modes_; i++) {
      // Set timestep bounds
      for (int j = 0; j < mode_lengths_[i] - 1; j++) {
        AddBoundingBoxConstraint(minimum_timestep[i], maximum_timestep[i],
                                 timestep(mode_start_[i] + j));
      }

      // all timesteps within a mode must be equal
      for (int j = 0; j < mode_lengths_[i] - 2; j++) {
        // TODO: improve the construction efficiency by using more specific API
        AddLinearConstraint(timestep(mode_start_[i] + j) ==
                            timestep(mode_start_[i] + j + 1));
      }
    }

    // Duration bound
    if (fix_duration) {
      double duration = first_mode_duration +
                        remaining_mode_duration_per_mode * (num_modes_ - 1);
      PrintStatus("Fix time duration: total duration = " +
                  std::to_string(duration));
      AddDurationBounds(duration, duration);
    }

    // Make the timesteps between modes the same (except the first one)
    if (equalize_timestep_size) {
      PrintStatus("Equalize time steps between modes (except the first one)");
      for (int i = 2; i < num_modes_; i++) {
        if (mode_start_[i] > 0) {
          // TODO: improve the construction efficiency by using more specific
          // API
          AddLinearConstraint(timestep(mode_start_[i] - 1) ==
                              timestep(mode_start_[i]));
        }
      }
    }
  }
}

const Eigen::VectorBlock<const VectorXDecisionVariable>
HybridRomTrajOpt::z_post_impact_vars_by_mode(int mode) const {
  return z_post_impact_vars_.segment(mode * n_z_, n_z_);
}

const Eigen::VectorBlock<const VectorXDecisionVariable>
HybridRomTrajOpt::x_lipm_vars_by_idx(int idx) const {
  return x_lipm_vars_.segment(idx * 4, 4);
}
const Eigen::VectorBlock<const VectorXDecisionVariable>
HybridRomTrajOpt::u_lipm_vars_by_idx(int idx) const {
  return u_lipm_vars_.segment(idx * 2, 2);
}

VectorX<Expression> HybridRomTrajOpt::SubstitutePlaceholderVariables(
    const VectorX<Expression>& f, int interval_index) const {
  VectorX<Expression> ret(f.size());
  for (int i = 0; i < f.size(); i++) {
    ret(i) =
        MultipleShooting::SubstitutePlaceholderVariables(f(i), interval_index);
  }
  return ret;
}

// Eigen::VectorBlock<const VectorXDecisionVariable>
// HybridRomTrajOpt::state_vars_by_mode(int mode, int
// time_index)  {
VectorXDecisionVariable HybridRomTrajOpt::state_vars_by_mode(
    int mode, int time_index) const {
  if (time_index == 0 && mode > 0) {
    return z_post_impact_vars_by_mode(mode - 1);
  } else {
    VectorXDecisionVariable ret(num_states());
    return x_vars().segment((mode_start_[mode] + time_index) * num_states(),
                            num_states());
    // std::cout << Eigen::VectorBlock<VectorXDecisionVariable>(ret, 0,
    //                  num_states())  << std::endl;
    // return Eigen::VectorBlock<VectorXDecisionVariable>(ret, 0, num_states());
  }
}

drake::solvers::VectorXDecisionVariable
HybridRomTrajOpt::discrete_swing_foot_pos_rt_stance_foot_x_vars(
    int mode) const {
  return discrete_swing_foot_pos_rt_stance_foot_x_vars_.segment<1>(mode);
}
drake::solvers::VectorXDecisionVariable
HybridRomTrajOpt::discrete_swing_foot_pos_rt_stance_foot_y_vars(
    int mode) const {
  return discrete_swing_foot_pos_rt_stance_foot_y_vars_.segment<1>(mode);
}
drake::solvers::VectorXDecisionVariable
HybridRomTrajOpt::y_end_of_last_mode_rt_init_stance_foot_var() const {
  return y_end_of_last_mode_rt_init_stance_foot_var_;
}

// TODO: need to configure this to handle the hybrid discontinuities properly
void HybridRomTrajOpt::DoAddRunningCost(const drake::symbolic::Expression& g) {
  // Trapezoidal integration:
  //    sum_{i=0...N-2} h_i/2.0 * (g_i + g_{i+1}), or
  // g_0*h_0/2.0 + [sum_{i=1...N-2} g_i*(h_{i-1} + h_i)/2.0] +
  // g_{N-1}*h_{N-2}/2.0.

  AddCost(MultipleShooting::SubstitutePlaceholderVariables(g, 0) * h_vars()(0) /
          2);
  for (int i = 1; i <= N() - 2; i++) {
    AddCost(MultipleShooting::SubstitutePlaceholderVariables(g, i) *
            (h_vars()(i - 1) + h_vars()(i)) / 2);
  }
  AddCost(MultipleShooting::SubstitutePlaceholderVariables(g, N() - 1) *
          h_vars()(N() - 2) / 2);
}

// TODO: need to configure this to handle the hybrid discontinuities properly
void HybridRomTrajOpt::DoAddRunningCost(const drake::symbolic::Expression& g,
                                        std::vector<Binding<Cost>>* bindings) {
  // Trapezoidal integration:
  //    sum_{i=0...N-2} h_i/2.0 * (g_i + g_{i+1}), or
  // g_0*h_0/2.0 + [sum_{i=1...N-2} g_i*(h_{i-1} + h_i)/2.0] +
  // g_{N-1}*h_{N-2}/2.0.

  bindings->push_back(
      AddCost(MultipleShooting::SubstitutePlaceholderVariables(g, 0) *
              h_vars()(0) / 2));
  for (int i = 1; i <= N() - 2; i++) {
    bindings->push_back(
        AddCost(MultipleShooting::SubstitutePlaceholderVariables(g, i) *
                (h_vars()(i - 1) + h_vars()(i)) / 2));
  }
  bindings->push_back(
      AddCost(MultipleShooting::SubstitutePlaceholderVariables(g, N() - 1) *
              h_vars()(N() - 2) / 2));
}

void HybridRomTrajOpt::GetStateAndDerivativeSamples(
    const drake::solvers::MathematicalProgramResult& result,
    std::vector<Eigen::MatrixXd>* state_samples,
    std::vector<Eigen::MatrixXd>* derivative_samples,
    std::vector<Eigen::VectorXd>* state_breaks) const {
  DRAKE_ASSERT(state_samples->empty());
  DRAKE_ASSERT(derivative_samples->empty());
  DRAKE_ASSERT(state_breaks->empty());

  VectorXd times(GetSampleTimes(result));

  for (int i = 0; i < num_modes_; i++) {
    MatrixXd states_i(num_states(), mode_lengths_[i]);
    MatrixXd derivatives_i(num_states(), mode_lengths_[i]);
    VectorXd times_i(mode_lengths_[i]);
    for (int j = 0; j < mode_lengths_[i]; j++) {
      int k_data = mode_start_[i] + j;

      VectorX<double> zk = result.GetSolution(state_vars_by_mode(i, j));
      VectorX<double> tauk = result.GetSolution(input(k_data));

      // z = [y; ydot]
      // Calculate zdot.
      // Copied from: examples/goldilocks_models/planning/dynamics_constraint.h
      VectorX<double> zdot(n_z_);
      zdot << zk.tail(n_y_),
          rom_.EvalDynamicFunc(zk.head(n_y_), zk.tail(n_y_), tauk);

      states_i.col(j) = drake::math::DiscardGradient(zk);
      derivatives_i.col(j) = drake::math::DiscardGradient(zdot);
      times_i(j) = times(k_data);
    }
    state_samples->push_back(states_i);
    derivative_samples->push_back(derivatives_i);
    state_breaks->push_back(times_i);
  }
}

void HybridRomTrajOpt::GetStateSamples(
    const drake::solvers::MathematicalProgramResult& result,
    std::vector<Eigen::MatrixXd>* state_samples,
    std::vector<Eigen::VectorXd>* state_breaks) const {
  DRAKE_ASSERT(state_samples->empty());
  DRAKE_ASSERT(state_breaks->empty());

  VectorXd times(GetSampleTimes(result));

  for (int i = 0; i < num_modes_; i++) {
    MatrixXd states_i(num_states(), mode_lengths_[i]);
    VectorXd times_i(mode_lengths_[i]);
    for (int j = 0; j < mode_lengths_[i]; j++) {
      int k_data = mode_start_[i] + j;

      VectorX<double> zk = result.GetSolution(state_vars_by_mode(i, j));

      states_i.col(j) = drake::math::DiscardGradient(zk);
      times_i(j) = times(k_data);
    }
    state_samples->push_back(states_i);
    state_breaks->push_back(times_i);
  }
}

PiecewisePolynomial<double> HybridRomTrajOpt::ReconstructInputTrajectory(
    const MathematicalProgramResult& result) const {
  Eigen::VectorXd times = GetSampleTimes(result);
  vector<double> times_vec(N());
  vector<Eigen::MatrixXd> inputs(N());
  for (int i = 0; i < N(); i++) {
    times_vec[i] = times(i);
    inputs[i] = result.GetSolution(input(i));
  }
  return PiecewisePolynomial<double>::FirstOrderHold(times_vec, inputs);
}

PiecewisePolynomial<double> HybridRomTrajOpt::ReconstructStateTrajectory(
    const MathematicalProgramResult& result) const {
  VectorXd times_all(GetSampleTimes(result));
  VectorXd times(N() + num_modes_ - 1);

  MatrixXd states(num_states(), N() + num_modes_ - 1);
  MatrixXd inputs(num_inputs(), N() + num_modes_ - 1);
  MatrixXd derivatives(num_states(), N() + num_modes_ - 1);

  for (int i = 0; i < num_modes_; i++) {
    for (int j = 0; j < mode_lengths_[i]; j++) {
      int k = mode_start_[i] + j + i;
      int k_data = mode_start_[i] + j;
      times(k) = times_all(k_data);

      // False timestep to match velocities
      if (i > 0 && j == 0) {
        times(k) += +1e-6;
      }
      VectorX<double> yk = result.GetSolution(state_vars_by_mode(i, j));
      VectorX<double> tauk = result.GetSolution(input(k_data));
      states.col(k) = yk;
      inputs.col(k) = tauk;

      // TODO(yminchen): need to modify the following code
      /*auto context = multibody::createContext(plant_, yk, tauk);
      constraints_[i]->updateData(*context, result.GetSolution(force(i, j)));
      derivatives.col(k) =
        drake::math::DiscardGradient(constraints_[i]->getXDot());*/
      DRAKE_UNREACHABLE();  // Put it here to test where we are using this.
    }
  }
  // return PiecewisePolynomial<double>::CubicHermite(times, states,
  // derivatives);
  return PiecewisePolynomial<double>::FirstOrderHold(times, states);
}

HybridRomTrajOptCassie::HybridRomTrajOptCassie(
    const std::vector<int>& num_time_samples, const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R, const ReducedOrderModel& rom,
    const VectorXd& init_rom_state,
    const std::vector<double>& max_swing_distance, bool start_with_left_stance,
    bool zero_touchdown_impact, const std::set<int>& relax_index,
    const PlannerSetting& param, const std::vector<int>& num_time_samples_ds,
    bool start_in_double_support_phase,
    const std::set<int>& idx_constant_rom_vel_during_double_support,
    bool print_status)
    : HybridRomTrajOpt(
          num_time_samples, Q, R, rom, init_rom_state, max_swing_distance,
          start_with_left_stance, zero_touchdown_impact, relax_index, param,
          num_time_samples_ds, start_in_double_support_phase,
          idx_constant_rom_vel_during_double_support, print_status) {}

void HybridRomTrajOptCassie::AddRomRegularizationCost(
    const Eigen::VectorXd& h_guess, const Eigen::MatrixXd& y_guess,
    const Eigen::MatrixXd& dy_guess, const Eigen::MatrixXd& tau_guess,
    int fisrt_mode_phase_index, double w_reg) {
  PrintStatus("Adding cost -- regularization for ROM state ...");

  MatrixXd z_guess(y_guess.rows() + dy_guess.rows(), y_guess.cols());
  z_guess << y_guess, dy_guess;

  MatrixXd I_h = w_reg * MatrixXd::Identity(1, 1);
  MatrixXd I_z = w_reg * MatrixXd::Identity(n_z_, n_z_);
  MatrixXd I_tau = w_reg * MatrixXd::Identity(rom_.n_tau(), rom_.n_tau());

  for (int i = 0; i < num_modes_; i++) {
    // Time steps
    // for (int j = 0; j < mode_lengths_[i] - 1; j++) {
    //  rom_regularization_cost_bindings_.push_back(AddQuadraticErrorCost(
    //      I_h, h_guess.segment(1, 1), timestep(mode_start_[i] + j)));
    //}

    // Rom states and inputs
    for (int j = 0; j < mode_lengths_[i]; j++) {
      // The initial state might start in the middle of the stride
      int j_guess = (i == 0) ? fisrt_mode_phase_index + j : j;

      rom_regularization_cost_bindings_.push_back(AddQuadraticErrorCost(
          I_z, z_guess.col(j_guess), state_vars_by_mode(i, j)));
      if (rom_.n_tau() > 0) {
        int time_index = mode_start_[i] + j;
        rom_regularization_cost_bindings_.push_back(AddQuadraticErrorCost(
            I_tau, tau_guess.col(j_guess),
            u_vars().segment(time_index * rom_.n_tau(), rom_.n_tau())));
      }
    }
  }
}

void HybridRomTrajOptCassie::SetHeuristicInitialGuess(
    const PlannerSetting& param, const Eigen::VectorXd& h_guess,
    const Eigen::MatrixXd& y_guess, const Eigen::MatrixXd& dy_guess,
    const Eigen::MatrixXd& tau_guess,
    const std::vector<Eigen::VectorXd>& reg_x_FOM, int fisrt_mode_phase_index,
    int starting_mode_index) {
  // PrintStatus("Adding initial guess ...");

  MatrixXd z_guess(y_guess.rows() + dy_guess.rows(), y_guess.cols());
  z_guess << y_guess, dy_guess;

  for (int i = starting_mode_index; i < num_modes_; i++) {
    // Time steps
    for (int j = 0; j < mode_lengths_[i] - 1; j++) {
      SetInitialGuess(timestep(mode_start_[i] + j), h_guess.segment<1>(1));
    }
    // Rom states and inputs
    for (int j = 0; j < mode_lengths_[i]; j++) {
      // The initial state might start in the middle of the stride
      int j_guess = (i == 0) ? fisrt_mode_phase_index + j : j;

      SetInitialGuess(state_vars_by_mode(i, j), z_guess.col(j_guess));
      int time_index = mode_start_[i] + j;
      SetInitialGuess(u_vars().segment(time_index * rom_.n_tau(), rom_.n_tau()),
                      tau_guess.col(j_guess));
    }

    // Full state
    //    auto x_preimpact = xf_vars_by_mode(i);
    //    auto x_postimpact = x0_vars_by_mode(i + 1);
    //    const VectorXd& x_guess_pre = reg_x_FOM.at(2 * i);
    //    const VectorXd& x_guess_post = reg_x_FOM.at(2 * i + 1);
    //    SetInitialGuess(x_preimpact, x_guess_pre);
    //    SetInitialGuess(x_postimpact, x_guess_post);

    // TODO: warm start the following variables
    //    discrete_swing_foot_pos_rt_stance_foot_x_vars_
    //    discrete_swing_foot_pos_rt_stance_foot_y_vars_
    //    y_end_of_last_mode_rt_init_stance_foot_var_
  }
}

void HybridRomTrajOptCassie::SetHeuristicInitialGuessForCascadedLipm(
    const PlannerSetting& param, const std::vector<Eigen::VectorXd>& des_xy_pos,
    const std::vector<Eigen::VectorXd>& des_xy_vel) {
  // LIPM state
  // TODO: Currently we initialize all vars, but we only need for those that's
  //  not warm-started by the previous solution
  // TODO: the foot placement guess should consider the kinematics constraint
  DRAKE_DEMAND(param.n_step > 0);  // because of des_xy_vel's size
  if (param.n_step_lipm > 1) {
    for (int i = 0; i < param.n_step_lipm; i++) {
      SetInitialGuess(x_lipm_vars_by_idx(i).head<2>(),
                      des_xy_pos.at(i + param.n_step));
      SetInitialGuess(x_lipm_vars_by_idx(i).tail<2>(),
                      des_xy_vel.at(i + param.n_step - 1));
      SetInitialGuess(u_lipm_vars_by_idx(i),
                      (des_xy_pos.at(i + param.n_step) +
                       des_xy_pos.at(i + param.n_step + 1)) /
                          2);
    }
    SetInitialGuess(x_lipm_vars_by_idx(param.n_step_lipm).head<2>(),
                    des_xy_pos.at(param.n_step_lipm + param.n_step));
    SetInitialGuess(x_lipm_vars_by_idx(param.n_step_lipm).tail<2>(),
                    des_xy_vel.at(param.n_step_lipm + param.n_step - 1));
  }
}

}  // namespace goldilocks_models
}  // namespace dairlib
