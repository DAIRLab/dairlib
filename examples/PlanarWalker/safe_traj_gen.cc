#include "examples/PlanarWalker/safe_traj_gen.h"
#include "matplotlibcpp.h"

#include <math.h>
#include <string>
#include <new>

#include "drake/solvers/solve.h"

using std::cout;
using std::endl;
using std::string;

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::systems::BasicVector;
using dairlib::systems::OutputVector;

using drake::solvers::MathematicalProgram;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

// TODO(nanda): Remove matplotlibcpp related files after testing
namespace plt = matplotlibcpp;

namespace dairlib {

SafeTrajGenerator::SafeTrajGenerator(
    const RigidBodyTree<double>& tree, const LIPMSwingLeg<double>& lipm_model,
    LoadLyapunovPolynomial& polynomial_loader, int left_foot_idx,
    Eigen::Vector3d pt_on_left_foot, int right_foot_idx,
    Eigen::Vector3d pt_on_right_foot, double mid_foot_height,
    double desired_final_foot_height,
    double desired_final_vertical_foot_velocity, bool add_extra_control)
    : tree_(tree),
      lipm_model_(lipm_model),
      polynomial_loader_(polynomial_loader),
      left_foot_idx_(left_foot_idx),
      pt_on_left_foot_(pt_on_left_foot),
      right_foot_idx_(right_foot_idx),
      pt_on_right_foot_(pt_on_right_foot),
      mid_foot_height_(mid_foot_height),
      desired_final_foot_height_(desired_final_foot_height),
      desired_final_vertical_foot_velocity_(
          desired_final_vertical_foot_velocity),
      add_extra_control_(add_extra_control) {
  this->set_name("safe_lipm_traj");

  state_port_ = this
                    ->DeclareVectorInputPort(OutputVector<double>(
                        tree_.get_num_positions(), tree_.get_num_velocities(),
                        tree_.get_num_actuators()))
                    .get_index();

  fsm_port_ = this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();

  // Load lyapunov polynomials
  x.resize(3);
  std::vector<Polynomiald> V(9, 0);  // V0, V1, W, partial_V0, partial_V1
  polynomial_loader_.load(x, V);
  V0_ = V[0];
  V1_ = V[1];
  W0_ = V[2];

  partial_V0_.resize(3);
  partial_V0_ = {V[3], V[4], V[5]};
  partial_V1_.resize(3);
  partial_V1_ = {V[6], V[7], V[8]};

  quadprog_ = std::make_unique<drake::solvers::MathematicalProgram>();

  // Variables
  input_ =
      quadprog_->NewContinuousVariables(lipm_model_.get_num_inputs(), "input");
  dx_ = quadprog_->NewContinuousVariables(lipm_model_.get_num_states(), "dx");
  rho_ = quadprog_->NewContinuousVariables(1, "rho");

  // Cost Matrices
  R_ = 2 * 0.1 * Matrix2d::Identity();
  Q_ = 2 * MatrixXd::Identity(1, 1);

  S_ = MatrixXd::Identity(1, 1);
  P_ = 10 * MatrixXd::Identity(1, 1);
  dt_ = 0.01;

  // Cost
  quadcost_input_ = quadprog_->AddQuadraticCost(R_, Vector2d::Zero(), input_)
                        .evaluator()
                        .get();

  quadcost_acceleration_ =
      quadprog_->AddQuadraticCost(Q_, VectorXd::Zero(1), dx_.block<1, 1>(1, 0))
          .evaluator()
          .get();

  // This cost will be modified in the solveQP function
  quadcost_swing_leg_ =
      quadprog_
          ->AddQuadraticCost(2 * P_ * dt_ * dt_, -2 * P_ * dt_ * 0,
                             input_.block<1, 1>(1, 0))
          .evaluator()
          .get();

  // This cost gets modified in the solveQP function
  lincost_rho_ = quadprog_->AddLinearCost(S_, 0, rho_).evaluator().get();

  // Constraints
  input_constraint_ = quadprog_
                          ->AddLinearConstraint(MatrixXd::Identity(2, 2),
                                                -1 * VectorXd::Ones(2),
                                                VectorXd::Ones(2), input_)
                          .evaluator()
                          .get();

  // This constraint will be modified in the solveQP function
  acceleration_constraint_ =
      quadprog_
          ->AddLinearEqualityConstraint(MatrixXd::Identity(1, 3),
                                        MatrixXd::Identity(1, 1),
                                        {input_, dx_.block<1, 1>(1, 0)})
          .evaluator()
          .get();

  // This constraint will be modified in the solveQP function
  barrier_constraint_ =
      quadprog_
          ->AddLinearConstraint(MatrixXd::Identity(1, 3), 0,
                                std::numeric_limits<double>::infinity(),
                                {rho_, input_})
          .evaluator()
          .get();

  // Output port
  PiecewisePolynomial<double> pp_com_traj((Vector3d()));

  Trajectory<double>& com_traj_inst = pp_com_traj;
  this->DeclareAbstractOutputPort("safe_lipm_traj", com_traj_inst,
                                  &SafeTrajGenerator::CalcTraj);

  PiecewisePolynomial<double> pp_swing_traj((Vector3d()));
  Trajectory<double>& swing_traj_inst = pp_swing_traj;
  this->DeclareAbstractOutputPort("swing_foot_traj", swing_traj_inst,
                                  &SafeTrajGenerator::CalcSwingTraj);

  DeclarePerStepDiscreteUpdateEvent(&SafeTrajGenerator::DiscreteVariableUpdate);

  // The time of the last touch down
  prev_td_time_idx_ = this->DeclareDiscreteState(0 * VectorXd::Ones(1));
  // Duration of stance
  duration_of_stance_idx_ = this->DeclareDiscreteState(0.5 * VectorXd::Ones(1));
  // The last state of FSM
  prev_fsm_state_idx_ = this->DeclareDiscreteState(-1 * VectorXd::Ones(1));
  // The swing foot position in the beginning of the swing phase
  prev_td_swing_foot_idx_ = this->DeclareDiscreteState(3);

  foot_position_idx_ = this->DeclareDiscreteState(VectorXd::Zero(3));
  last_calculation_time_idx_ =
      this->DeclareDiscreteState(-1 * VectorXd::Zero(1));

  // Check if the model is floating based
  is_quaternion_ = multibody::IsFloatingBase(tree);

  // Testing variables
  time_hist_ = std::make_unique<std::vector<double>>();
  CoM_hist_x_ = std::make_unique<std::vector<double>>();
  CoM_hist_z_ = std::make_unique<std::vector<double>>();
  desired_CoM_hist_x_ = std::make_unique<std::vector<double>>();
  desired_CoM_hist_z_ = std::make_unique<std::vector<double>>();
  swing_pos_x_hist_ = std::make_unique<std::vector<double>>();
  swing_pos_y_hist_ = std::make_unique<std::vector<double>>();
  swing_pos_z_hist_ = std::make_unique<std::vector<double>>();
  desired_swing_pos_x_hist_ = std::make_unique<std::vector<double>>();
  desired_swing_pos_y_hist_ = std::make_unique<std::vector<double>>();
  desired_swing_pos_z_hist_ = std::make_unique<std::vector<double>>();
  stance_toe_angle_hist_ = std::make_unique<std::vector<double>>();
  swing_toe_angle_hist_ = std::make_unique<std::vector<double>>();

  // Test private methods here
}

EventStatus SafeTrajGenerator::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {

  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);

  const BasicVector<double>* fsm_output =
      (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);

  VectorXd fsm_state = fsm_output->get_value();
  // std::cout << "fsm_state:" << std::endl;
  // std::cout << fsm_state << std::endl;

  auto prev_td_time =
      discrete_state->get_mutable_vector(prev_td_time_idx_).get_mutable_value();
  auto prev_fsm_state = discrete_state->get_mutable_vector(prev_fsm_state_idx_)
                            .get_mutable_value();

  double timestamp = robot_output->get_timestamp();
  double current_time = static_cast<double>(timestamp);

  std::cout << "current_time:" << current_time << std::endl;
  std::cout << fsm_state(0) << std::endl;
  std::cout << prev_fsm_state(0) << std::endl;

  // TODO(nanda): Modify this if condition to incorporate time
  if (fsm_state(0) != prev_fsm_state(0)) {
    cout << "Inside change of fsm state in safe_traj_gen!" << endl;
    prev_fsm_state(0) = fsm_state(0);
    prev_td_time(0) = current_time;
    auto swing_foot_pos_td =
        discrete_state->get_mutable_vector(prev_td_swing_foot_idx_)
            .get_mutable_value();

    /* Generate Swing Foot End Leg Position */
    /* Save it in a variable and keep track of when the variable was changed */
    /* Redo this computation if sufficient time has passed */
    auto last_calc_time =
        discrete_state->get_vector(last_calculation_time_idx_).get_value();
    // std::cout << "last_calc_time: " << last_calc_time << std::endl;
    if (current_time - last_calc_time(0) > 0.05) {
      KinematicsCache<double> cache = tree_.CreateKinematicsCache();
      VectorXd q = robot_output->GetPositions();

      VectorXd v = robot_output->GetVelocities();
      if (is_quaternion_) {
        multibody::SetZeroQuaternionToIdentity(&q);
      }
      cout << "q and v found" << endl;

      cache.initialize(q);
      tree_.doKinematics(cache);

      int stance_foot_idx = (fsm_state(0) == right_stance_state_)
                                ? right_foot_idx_
                                : left_foot_idx_;
      Vector3d pt_on_stance_foot = (fsm_state(0) == right_stance_state_)
                                       ? pt_on_right_foot_
                                       : pt_on_left_foot_;
      int swing_foot_idx = (fsm_state(0) == right_stance_state_)
                               ? left_foot_idx_
                               : right_foot_idx_;
      Vector3d pt_on_swing_foot = (fsm_state(0) == left_stance_state_)
                                      ? pt_on_left_foot_
                                      : pt_on_right_foot_;

      Vector3d CoM = tree_.centerOfMass(cache);
      MatrixXd J = tree_.centerOfMassJacobian(cache);
      Vector3d dCoM = J * v;

      cout << "Cache computed" << endl;

      Vector3d stance_foot_pos =
          tree_.transformPoints(cache, pt_on_stance_foot, stance_foot_idx, 0);
      Vector3d swing_foot_pos =
          tree_.transformPoints(cache, pt_on_swing_foot, swing_foot_idx, 0);
      swing_foot_pos(0) -= stance_foot_pos(0);

      Vector3d CoM_wrt_foot = CoM - stance_foot_pos;
      const double CoM_wrt_foot_x = CoM(0) - stance_foot_pos(0);
      const double CoM_wrt_foot_y = CoM(1) - stance_foot_pos(1);
      const double CoM_wrt_foot_z = CoM(2) - stance_foot_pos(2);
      DRAKE_DEMAND(CoM_wrt_foot_z > 0);
      cout << "Passed the drake demand" << endl;

      double stance_location;
      double step_duration = 0;
      Vector3d reduced_order_state;
      reduced_order_state << CoM_wrt_foot(0), dCoM(0),
          (swing_foot_pos - stance_foot_pos)[0];
      find_next_stance_location(reduced_order_state,
                                stance_location, step_duration);

      std::cout << "step_duration: " << step_duration << std::endl;
      std::cout << "stance_location: " << stance_location << std::endl;
      Vector3d stance_pos;
      swing_foot_pos << stance_foot_pos(0) + stance_location, 0, 0;
      discrete_state->get_mutable_vector(duration_of_stance_idx_)
              .get_mutable_value()
          << step_duration;
      discrete_state->get_mutable_vector(foot_position_idx_).get_mutable_value()
          << stance_foot_pos;
      discrete_state->get_mutable_vector(last_calculation_time_idx_)
              .get_mutable_value()
          << current_time;
      // exit(EXIT_FAILURE);
    }
  }

  return EventStatus::Succeeded();
}

Vector3d SafeTrajGenerator::solveQP(const Vector3d& reduced_order_state) const {
  std::map<Polynomiald::VarType, double> var_values;
  var_values[x[0].GetSimpleVariable()] = reduced_order_state(0);
  var_values[x[1].GetSimpleVariable()] = reduced_order_state(1);
  var_values[x[2].GetSimpleVariable()] = reduced_order_state(2);

  double V0_val = V0_.EvaluateMultivariate(var_values);
  double V1_val = V1_.EvaluateMultivariate(var_values);

  double multiplication_factor_S = 1;
  if (V0_val <= 1.0 || V1_val <= 1.0) {
    multiplication_factor_S *= 10;
  } else {
    multiplication_factor_S *= 0.1;
  }
  // cout << "multiplication_factor_S: " << multiplication_factor_S << endl;
  // cout << "Reduced-order state: " << reduced_order_state.transpose() << endl;

  // Swing foot cost
  double omega = lipm_model_.get_omega();
  // cout << "Omega: " << omega << endl;
  VectorXd desired_swing_foot_position(1);
  desired_swing_foot_position
      << reduced_order_state(0) + (reduced_order_state(1) / omega) - reduced_order_state(2);
  // std::cout << "swing_foot_pos: " << swing_foot_pos << std::endl;
  // std::cout << "Capture point: "  << desired_swing_foot_position << std::endl;
  quadcost_swing_leg_->UpdateCoefficients(
      2 * P_ * dt_ * dt_, -2 * P_ * dt_ * desired_swing_foot_position);

  // rho cost
  lincost_rho_->UpdateCoefficients(S_ * multiplication_factor_S, 0);

  // Acceleration constraint
  VectorXd f = Vector3d::Zero();
  MatrixXd g = MatrixXd::Zero(3, 2);
  lipm_model_.controlAffineDynamics(0, reduced_order_state, f, g);
  MatrixXd A_eq(1, 3);
  A_eq(0, 0) = g(1, 0);
  A_eq(0, 1) = g(1, 1);
  A_eq(0, 2) = -1;
  acceleration_constraint_->UpdateCoefficients(A_eq, -f.segment<1>(1));

  // Barrier constraint
  // cout << "V0_val: " << " " << V0_val << endl;
  // cout << "V1_val: " << " " << V1_val << endl;
  MatrixXd A_ineq(1, 3);
  MatrixXd lb_ineq(1, 1);
  if (V0_val <= 1) {
    // cout << "Inside 0-step!" << endl;
    MatrixXd partial_V0_val(1, 3);
    partial_V0_val << partial_V0_[0].EvaluateMultivariate(var_values),
        partial_V0_[1].EvaluateMultivariate(var_values),
        partial_V0_[2].EvaluateMultivariate(var_values);
    // cout << "partial_V0_val: " << endl;
    // cout << partial_V0_val << endl;

    auto dV_g = partial_V0_val * g;
    A_ineq << 1, -dV_g;
    auto dV_f = partial_V0_val * f;
    lb_ineq << dV_f;
    // cout << "dV_g: " << dV_g << endl;
    // cout << "dV_f: " << dV_f << endl;
  } else {
    // cout << "Inside 1-step!" << endl;
    MatrixXd partial_V1_val(1, 3);
    partial_V1_val << partial_V1_[0].EvaluateMultivariate(var_values),
        partial_V1_[1].EvaluateMultivariate(var_values),
        partial_V1_[2].EvaluateMultivariate(var_values);
    // cout << "partial_V1_val: " << endl;
    // cout << partial_V1_val << endl;

    auto dV_g = partial_V1_val * g;
    A_ineq << 1, -dV_g;
    auto dV_f = partial_V1_val * f;
    lb_ineq << dV_f;
    // cout << "dV_g: " << dV_g << endl;
    // cout << "dV_f: " << dV_f << endl;
  }
  MatrixXd ub_ineq(1, 1);
  ub_ineq << std::numeric_limits<double>::infinity();
  barrier_constraint_->UpdateCoefficients(
      A_ineq, lb_ineq, ub_ineq);

  auto constraints = quadprog_->GetAllConstraints();
  // cout << "Number of Constraint: " << endl;
  // cout << constraints.size() << endl;
  auto costs = quadprog_->GetAllCosts();
  // cout << "Number of Costs: " << endl;
  // cout << costs.size() << endl;
  drake::solvers::MathematicalProgramResult result =
      drake::solvers::Solve(*quadprog_);

  if (result.is_success()) {
    VectorXd input_val = result.GetSolution(input_);
    // std::cout << "input_val: " << input_val.transpose() << std::endl;

    VectorXd state_dot = f + g * input_val;
    // std::cout << "state_dot: " << state_dot.transpose() << std::endl;

    return state_dot;
  }
  cout << result.get_solution_result() << endl;

  std::cout << "SolveQP failed!" << endl;
  exit(EXIT_FAILURE);
  return Vector3d::Zero();
}

SteppingResults SafeTrajGenerator::should_step(double current_time, double prev_td_time,
                                    Vector3d reduced_order_state) const {
  std::map<Polynomiald::VarType, double> var_values;
  var_values[x[0].GetSimpleVariable()] = reduced_order_state(0);
  var_values[x[1].GetSimpleVariable()] = reduced_order_state(1);
  var_values[x[2].GetSimpleVariable()] = reduced_order_state(2);

  double V0_val = V0_.EvaluateMultivariate(var_values);
  double V1_val = V1_.EvaluateMultivariate(var_values);
  double W0_val = W0_.EvaluateMultivariate(var_values);
  // std::cout << "V0_val: " << V0_val << std::endl;
  // std::cout << "V1_val: " << V1_val << std::endl;
  // std::cout << "W0_val: " << W0_val << std::endl;
  // std::cout << "*******************" << std::endl;
  if (W0_val <= 1 && V0_val > 1) {
    cout << "reduced_order_state: " << endl;
    cout << reduced_order_state << endl;
    std::cout << "Step!" << endl;
    return SteppingResults::step;
  } else if (W0_val <= 1 && V0_val <= 1) {
    cout << "reduced_order_state: " << endl;
    cout << reduced_order_state << endl;
    std::cout << "Step (from within 0-step)!" << endl;
    return SteppingResults::continue_balancing;
  }
  return SteppingResults::dont_step;
}

// Use values of V here to see when the while loop can be stopped
// Runs for a prohibitively long time - depends on the initial condition
void SafeTrajGenerator::find_next_stance_location(
    const Eigen::Vector3d& reduced_order_state, double& next_stance_loc,
    double& t) const {
  cout << "Inside find_next_stance_location" << endl;
  double dt = 0.01;
  double total_time = 0;
  Vector3d state_dot;
  Vector3d next_state;
  next_state << reduced_order_state(0), reduced_order_state(1),
      reduced_order_state(2);
  cout << "Current state: " << next_state.transpose() << endl;

  if (should_step(total_time, 0, next_state) ==
      SteppingResults::continue_balancing) {
    cout << "Continue Balancing!" << endl;
    t = 0.5;
    next_state = Vector3d::Zero();
    cout << "At the end of find_next_stance_location" << endl;
    return;
  }

  int num_steps = 0;
  SteppingResults stepping_result;
  while (true) {
    state_dot = solveQP(next_state);

    // Euler Integration
    next_state += state_dot * dt;
    total_time += dt;
    num_steps++;

    // Change time here
    stepping_result = should_step(total_time, 0, next_state);
    if (stepping_result == SteppingResults::step) {
      break;
    }
    if (num_steps > 200) {
      break;
    }
  }
  next_stance_loc = next_state(2);
  if (num_steps > 200 &&
      stepping_result == SteppingResults::continue_balancing) {
    t = 0.5;
    cout << "Continue balancing!" << endl;
    // Think of what the state should be in this condition
  } else if (stepping_result == SteppingResults::dont_step) {
    t = -1;
    cout << "Can't balance!" << endl;
  } else {
    t = total_time;
    cout << "Can balance!" << endl;
  }
  cout << "At the end of find_next_stance_location" << endl;
  return;
}

void SafeTrajGenerator::CalcTraj(const Context<double>& context,
                                 Trajectory<double>* traj) const {
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  VectorXd v = robot_output->GetVelocities();

  const BasicVector<double>* fsm_output =
      (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
  VectorXd fsm_state = fsm_output->get_value();

  const auto prev_td_time =
      context.get_discrete_state(prev_td_time_idx_).get_value();
  const double stance_duration =
      context.get_discrete_state(duration_of_stance_idx_).get_value()(0);

  double timestamp = robot_output->get_timestamp();
  double current_time = static_cast<double>(timestamp);

  KinematicsCache<double> cache = tree_.CreateKinematicsCache();
  VectorXd q = robot_output->GetPositions();

  if (is_quaternion_) {
    multibody::SetZeroQuaternionToIdentity(&q);
  }

  cache.initialize(q);
  tree_.doKinematics(cache);

  int stance_foot_idx =
      (fsm_state(0) == right_stance_state_) ? right_foot_idx_ : left_foot_idx_;
  Vector3d pt_on_stance_foot = (fsm_state(0) == right_stance_state_)
                                   ? pt_on_right_foot_
                                   : pt_on_left_foot_;
  int swing_foot_idx =
      (fsm_state(0) == right_stance_state_) ? left_foot_idx_ : right_foot_idx_;
  Vector3d pt_on_swing_foot = (fsm_state(0) == left_stance_state_)
                                  ? pt_on_left_foot_
                                  : pt_on_right_foot_;

  Vector3d CoM = tree_.centerOfMass(cache);
  MatrixXd J = tree_.centerOfMassJacobian(cache);
  Vector3d dCoM = J * v;

  Vector3d stance_foot_pos =
      tree_.transformPoints(cache, pt_on_stance_foot, stance_foot_idx, 0);
  Vector3d swing_foot_pos =
      tree_.transformPoints(cache, pt_on_swing_foot, swing_foot_idx, 0);

  Vector3d CoM_wrt_foot = CoM - stance_foot_pos;
  const double CoM_wrt_foot_x = CoM_wrt_foot(0);
  const double CoM_wrt_foot_z = CoM_wrt_foot(2);
  DRAKE_DEMAND(CoM_wrt_foot_z > 0);


  const std::vector<double> breaks = {
      current_time, prev_td_time(0) + stance_duration + 0.002};  // Think about this line

  Vector3d reduced_order_state;
  reduced_order_state << CoM_wrt_foot(0), dCoM(0), swing_foot_pos(0);
  Vector3d state_dot = solveQP(reduced_order_state);

  Polynomiald t("t");
  drake::MatrixX<Polynomiald> polynomial_matrix(3, 1);
  polynomial_matrix(0, 0) =
      CoM(0) + dCoM(0) * t + state_dot(1) * pow(t, 2);
  polynomial_matrix(1, 0) = stance_foot_pos(1) + 0 * t + 0 * pow(t, 2);

  std::vector<MatrixXd> Y(breaks.size(), MatrixXd::Zero(1, 1));
  Y[0](0, 0) = lipm_model_.get_desired_com_height();
  Y[1](0, 0) = lipm_model_.get_desired_com_height();
  // cout << "Actual height vs desired height: " << endl;
  // cout << CoM(2) << endl;
  // cout << lipm_model_.get_desired_com_height() << endl;

  MatrixXd Y_dot_start = MatrixXd::Zero(1, 1);
  Y_dot_start(1, 0) = 0.0;
  MatrixXd Y_dot_end = MatrixXd::Zero(1, 1);
  Y_dot_end(1, 0) = 0.0; // Remove this maybe?

  PiecewisePolynomial<double> pp_part =
      PiecewisePolynomial<double>::Cubic(breaks, Y, Y_dot_start, Y_dot_end);

  polynomial_matrix(2, 0) = pp_part.getPolynomial(0, 0, 0);

  const std::vector<drake::MatrixX<Polynomiald>> polynomials = {
      polynomial_matrix};

  PiecewisePolynomial<double>* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  *casted_traj = PiecewisePolynomial<double>(polynomials, breaks);

  // plt::figure(0);
  // plt::clf();
  // Vector3d com_pos = casted_traj->value(current_time);
  // Vector3d com_ddpos = casted_traj->derivative(2).value(current_time);

  // time_hist_->push_back(current_time);
  // desired_CoM_hist_x_->push_back(com_pos(0));
  // desired_CoM_hist_z_->push_back(com_pos(2));
  // CoM_hist_x_->push_back(CoM(0));
  // CoM_hist_z_->push_back(CoM(2));

  // plt::named_plot("Center of Mass x", *time_hist_, *CoM_hist_x_);
  // plt::named_plot("Desired Center of Mass x", *time_hist_, *desired_CoM_hist_x_);
  // plt::quiver(std::vector<double>{(*time_hist_)[time_hist_->size() - 1]},
  //             std::vector<double>{CoM_wrt_foot_x}, std::vector<double>{0},
  //             std::vector<double>{com_ddpos(0)});
  // plt::legend();
  // plt::pause(0.01);

  // plt::figure(1);
  // plt::clf();

  // plt::named_plot("Center of Mass z", *time_hist_, *CoM_hist_z_);
  // plt::named_plot("Desired Center of Mass z", *time_hist_, *desired_CoM_hist_z_);
  // plt::quiver(std::vector<double>{(*time_hist_)[time_hist_->size() - 1]},
  //             std::vector<double>{CoM(2)}, std::vector<double>{0},
  //             std::vector<double>{com_ddpos(2)});
  // plt::legend();
  // plt::pause(0.01);
}

PiecewisePolynomial<double> SafeTrajGenerator::createSplineForSwingFoot(
    const double start_time_of_this_interval,
    const double end_time_of_this_interval, const Vector3d& init_swing_foot_pos,
    const Vector3d& CP) const {

  // Two segment of cubic polynomial with velocity constraints
  std::vector<double> T_waypoint = {
      start_time_of_this_interval,
      (start_time_of_this_interval + end_time_of_this_interval) / 2,
      end_time_of_this_interval};

  std::vector<MatrixXd> Y(T_waypoint.size(), MatrixXd::Zero(3, 1));
  // x
  Y[0](0, 0) = init_swing_foot_pos(0);
  Y[1](0, 0) = (init_swing_foot_pos(0) + CP(0)) / 2;
  Y[2](0, 0) = CP(0);
  // y
  Y[0](1, 0) = init_swing_foot_pos(1);
  Y[1](1, 0) = (init_swing_foot_pos(1) + CP(1)) / 2;
  Y[2](1, 0) = CP(1);
  // z
  Y[0](2, 0) = init_swing_foot_pos(2);
  Y[1](2, 0) = mid_foot_height_;
  Y[2](2, 0) = desired_final_foot_height_;

  double duration_of_stance =
      end_time_of_this_interval - start_time_of_this_interval;
  std::vector<MatrixXd> Y_dot(T_waypoint.size(), MatrixXd::Zero(3, 1));
  // x
  Y_dot[0](0, 0) = 0;
  Y_dot[1](0, 0) = (CP(0) - init_swing_foot_pos(0)) / duration_of_stance;
  Y_dot[2](0, 0) = 0;
  // y
  Y_dot[0](1, 0) = 0;
  Y_dot[1](1, 0) = (CP(1) - init_swing_foot_pos(1)) / duration_of_stance;
  Y_dot[2](1, 0) = 0;
  // z
  Y_dot[0](2, 0) = 0;
  Y_dot[1](2, 0) = 0;
  Y_dot[2](2, 0) = desired_final_vertical_foot_velocity_;
  PiecewisePolynomial<double> swing_foot_spline =
      PiecewisePolynomial<double>::Cubic(T_waypoint, Y, Y_dot);

  return swing_foot_spline;
}

void SafeTrajGenerator::CalcSwingTraj(const Context<double>& context,
                                      Trajectory<double>* traj) const {

  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);

  const BasicVector<double>* fsm_output =
      (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
  VectorXd fsm_state = fsm_output->get_value();


  // Get discrete states
  const auto swing_foot_pos_td =
      context.get_discrete_state(prev_td_swing_foot_idx_).get_value();
  const auto prev_td_time =
      context.get_discrete_state(prev_td_time_idx_).get_value();
  auto duration_of_stance =
      context.get_discrete_state(duration_of_stance_idx_).get_value();

  double stance_duration = duration_of_stance(0);
  if (stance_duration < 0) {
    cout << "There is no solution!" << endl;
    stance_duration = 0.5;
  }

  // Get current time
  double timestamp = robot_output->get_timestamp();
  double current_time = static_cast<double>(timestamp);

  // Get the start time and the end time of the current stance phase
  double start_time_of_this_interval = prev_td_time(0);
  double end_time_of_this_interval = prev_td_time(0) + stance_duration;

  // Ensure current_time < end_time_of_this_interval to avoid error in creating
  // trajectory.
  if ((end_time_of_this_interval <= current_time + 0.001)) {
    end_time_of_this_interval = current_time + 0.002;
  }

  /* Read the position of next stance leg */
  Vector3d next_stance_pos =
      context.get_discrete_state(foot_position_idx_).get_value();

  // Swing foot position at touchdown
  Vector3d init_swing_foot_pos = swing_foot_pos_td;

  // Assign traj
  PiecewisePolynomial<double>* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  PiecewisePolynomial<double> pp_traj = createSplineForSwingFoot(
      start_time_of_this_interval, end_time_of_this_interval,
      init_swing_foot_pos, next_stance_pos);
  *casted_traj = pp_traj;

  // KinematicsCache<double> cache = tree_.CreateKinematicsCache();
  // VectorXd q = robot_output->GetPositions();

  // cache.initialize(q);
  // tree_.doKinematics(cache);

  // int stance_foot_idx =
  //     (fsm_state(0) == right_stance_state_) ? right_foot_idx_ : left_foot_idx_;
  // Vector3d pt_on_stance_foot = (fsm_state(0) == right_stance_state_)
  //                                  ? pt_on_right_foot_
  //                                  : pt_on_left_foot_;
  // int swing_foot_idx =
  //     (fsm_state(0) == right_stance_state_) ? left_foot_idx_ : right_foot_idx_;
  // Vector3d pt_on_swing_foot = (fsm_state(0) == left_stance_state_)
  //                                 ? pt_on_left_foot_
  //                                 : pt_on_right_foot_;
  // Vector3d stance_foot_pos =
  //     tree_.transformPoints(cache, pt_on_stance_foot, stance_foot_idx, 0);
  // Vector3d swing_foot_pos =
  //     tree_.transformPoints(cache, pt_on_swing_foot, swing_foot_idx, 0);

  // plt::clf();
  // time_hist_->push_back(current_time);
  // swing_pos_x_hist_->push_back(swing_foot_pos[0] - stance_foot_pos[0]);
  // swing_pos_y_hist_->push_back(swing_foot_pos[1] - stance_foot_pos[1]);
  // swing_pos_z_hist_->push_back(swing_foot_pos[2] - stance_foot_pos[2]);

  // Vector3d desired_swing_pos = casted_traj->value(current_time);
  // desired_swing_pos_x_hist_->push_back(desired_swing_pos[0]);
  // desired_swing_pos_y_hist_->push_back(desired_swing_pos[1]);
  // desired_swing_pos_z_hist_->push_back(desired_swing_pos[2]);

  // plt::named_plot("Desred swing leg x", *time_hist_, *desired_swing_pos_x_hist_);
  // plt::named_plot("Desired swing leg y", *time_hist_, *desired_swing_pos_y_hist_);
  // plt::named_plot("Desired swing leg z", *time_hist_, *desired_swing_pos_z_hist_);
  // plt::named_plot("Swing leg x", *time_hist_, *swing_pos_x_hist_);
  // plt::named_plot("Swing leg y", *time_hist_, *swing_pos_y_hist_);
  // plt::named_plot("Swing leg z", *time_hist_, *swing_pos_z_hist_);
  // plt::legend();
  // plt::pause(0.01);

  // plt::clf();
  // time_hist_->push_back(current_time);
  // stance_toe_angle_hist_->push_back(robot_output->GetPositionAtIndex(stance_foot_idx));
  // swing_toe_angle_hist_->push_back(robot_output->GetPositionAtIndex(swing_foot_idx));

  // plt::named_plot("Stance toe", *time_hist_, *stance_toe_angle_hist_);
  // plt::named_plot("Swing toe", *time_hist_, *swing_toe_angle_hist_);
  // plt::legend();
  // plt::pause(0.01);
}

}  // namespace dairlib
