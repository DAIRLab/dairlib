#include "examples/PlanarWalker/safe_traj_gen.h"

#include <math.h>
#include <string>

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
  R_ = 0.1 * Matrix2d::Identity();
  Q_ = MatrixXd::Identity(1, 1);

  S_ = MatrixXd::Identity(1, 1);
  P_ = 10 * MatrixXd::Identity(1, 1);

  // Cost
  quadcost_input_ = quadprog_->AddQuadraticCost(R_, Vector2d::Zero(), input_)
                        .evaluator()
                        .get();

  quadcost_acceleration_ =
      quadprog_->AddQuadraticCost(Q_, VectorXd::Zero(1), dx_.block<1, 1>(1, 0))
          .evaluator()
          .get();

  // This cost will be modified in the solveQP function
  quadcost_swing_leg_ = quadprog_
                            ->AddQuadraticErrorCost(P_, VectorXd::Zero(1),
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
          ->AddLinearEqualityConstraint(MatrixXd::Identity(1, 2),
                                        MatrixXd::Identity(1, 1), input_)
          .evaluator()
          .get();

  // This constraint will be modified in the solveQP function
  // barrier_constraint_ =
  //     quadprog_
  //         ->AddLinearConstraint((MatrixXd::Identity(1, 2) * input_).array()
  //         <=
  //                               MatrixXd::Identity(1, 1).array())
  //         .evaluator()
  //         .get();

  // Output port
  PiecewisePolynomial<double> pp_traj(VectorXd(0));

  Trajectory<double>& traj_inst = pp_traj;
  this->DeclareAbstractOutputPort("safe_lipm_traj", traj_inst,
                                  &SafeTrajGenerator::CalcTraj);

  this->DeclareAbstractOutputPort("swing_foot_traj", traj_inst,
                                  &SafeTrajGenerator::CalcSwingTraj);

  DeclarePerStepDiscreteUpdateEvent(&SafeTrajGenerator::DiscreteVariableUpdate);

  // The time of the last touch down
  prev_td_time_idx_ = this->DeclareDiscreteState(0 * VectorXd::Ones(1));
  // Duration of stance
  duration_of_stance_idx_ = this->DeclareDiscreteState(0.35 * VectorXd::Ones(1));
  // The last state of FSM
  prev_fsm_state_idx_ = this->DeclareDiscreteState(0 * VectorXd::Ones(1));
  // The swing foot position in the beginning of the swing phase
  prev_td_swing_foot_idx_ = this->DeclareDiscreteState(3);

  foot_position_idx_ = DeclareDiscreteState(VectorXd::Zero(3));
  last_calculation_time_idx_ = DeclareDiscreteState(0);

  // Check if the model is floating based
  is_quaternion_ = multibody::IsFloatingBase(tree);
  std::cout << "SafeTrajGenerator is defined!" << std::endl;
}

EventStatus SafeTrajGenerator::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  std::cout << "Inside DeclareDiscreteState" << std::endl;

  const BasicVector<double>* fsm_output =
      (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);

  VectorXd fsm_state = fsm_output->get_value();
  std::cout << "fsm_state:" << std::endl;
  std::cout << fsm_state << std::endl;

  auto prev_td_time =
      discrete_state->get_mutable_vector(prev_td_time_idx_).get_mutable_value();
  auto prev_fsm_state = discrete_state->get_mutable_vector(prev_fsm_state_idx_)
                            .get_mutable_value();

  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);

  double timestamp = robot_output->get_timestamp();
  double current_time = static_cast<double>(timestamp);

  std::cout << "current_time:" << std::endl;
  std::cout << current_time << std::endl;

  if (fsm_state(0) != prev_fsm_state(0)) {
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
    if (current_time - last_calc_time(0) > 0.1) {
      KinematicsCache<double> cache = tree_.CreateKinematicsCache();
      VectorXd q = robot_output->GetPositions();

      VectorXd v = robot_output->GetVelocities();
      if (is_quaternion_) {
        multibody::SetZeroQuaternionToIdentity(&q);
      }

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

      Vector3d stance_foot_pos =
          tree_.transformPoints(cache, pt_on_stance_foot, stance_foot_idx, 0);

      Vector3d CoM_wrt_foot = CoM - stance_foot_pos;
      const double CoM_wrt_foot_x = CoM(0) - stance_foot_pos(0);
      const double CoM_wrt_foot_y = CoM(1) - stance_foot_pos(1);
      const double CoM_wrt_foot_z = CoM(2) - stance_foot_pos(2);
      DRAKE_DEMAND(CoM_wrt_foot_z > 0);

      Vector3d stance_location = Vector3d::Zero();
      double step_duration = 0;
      find_next_stance_location(CoM_wrt_foot, dCoM, pt_on_swing_foot,
                                stance_location, step_duration);

      discrete_state->get_mutable_vector(duration_of_stance_idx_)
              .get_mutable_value()
          << step_duration;
      discrete_state->get_mutable_vector(foot_position_idx_).get_mutable_value()
          << stance_location;
      discrete_state->get_mutable_vector(last_calculation_time_idx_)
              .get_mutable_value()
          << current_time;
    }
  }
  std::cout << "At the end of DiscreteVariableUpdate" << std::endl;

  return EventStatus::Succeeded();
}

Vector3d SafeTrajGenerator::solveQP(Vector3d CoM_wrt_foot, Vector3d dCoM,
                                    Vector3d swing_foot_pos) const {
  std::cout << "At the start of SolveQP" << std::endl;
  std::map<Polynomiald::VarType, double> var_values;
  var_values[x[0].GetSimpleVariable()] = CoM_wrt_foot(0);
  var_values[x[1].GetSimpleVariable()] = dCoM(0);
  var_values[x[2].GetSimpleVariable()] = swing_foot_pos(0);

  double V0_val = V0_.EvaluateMultivariate(var_values);
  double V1_val = V1_.EvaluateMultivariate(var_values);

  double multiplication_factor_S = 1;
  if (V0_val <= 1 || V1_val <= 1) {
    multiplication_factor_S *= 10;
  } else {
    multiplication_factor_S *= 0.1;
  }

  // Swing foot cost
  double omega = lipm_model_.get_omega();
  double capture_point = CoM_wrt_foot(0) + dCoM(0) / omega;
  VectorXd desired_swing_foot_position(1);
  desired_swing_foot_position << capture_point - swing_foot_pos(0);
  std::cout << desired_swing_foot_position << std::endl;
  quadcost_swing_leg_->UpdateCoefficients(P_, desired_swing_foot_position);

  // rho cost
  lincost_rho_->UpdateCoefficients(S_ * multiplication_factor_S, 0);

  // Acceleration constraint
  VectorXd x_reduced_order(3);
  x_reduced_order << CoM_wrt_foot(0), dCoM(0), swing_foot_pos(0);
  VectorXd f = Vector3d::Zero();
  MatrixXd g = MatrixXd::Zero(3, 2);
  lipm_model_.controlAffineDynamics(0, x_reduced_order, f, g);
  acceleration_constraint_->UpdateCoefficients(g.block<1, 2>(1, 0),
                                               f.segment<1>(1));

  // Barrier constraint
  if (V0_val <= 1) {
    MatrixXd partial_V0_val(1, 3);
    partial_V0_val << partial_V0_[0].EvaluateMultivariate(var_values),
        partial_V0_[1].EvaluateMultivariate(var_values),
        partial_V0_[2].EvaluateMultivariate(var_values);

    auto dV_g = partial_V0_val * g;
    auto dV_f = partial_V0_val * f;
    quadprog_->AddLinearConstraint((dV_f + dV_g * input_).array() <=
                                   rho_.array());
  } else {
    MatrixXd partial_V1_val(1, 3);
    partial_V1_val << partial_V1_[0].EvaluateMultivariate(var_values),
        partial_V1_[1].EvaluateMultivariate(var_values),
        partial_V1_[2].EvaluateMultivariate(var_values);
    auto dV_g = partial_V1_val * g;
    auto dV_f = partial_V1_val * f;
    quadprog_->AddLinearConstraint((dV_f + dV_g * input_).array() <=
                                   rho_.array());
  }

  drake::solvers::MathematicalProgramResult result =
      drake::solvers::Solve(*quadprog_);

  if (result.is_success()) {
    VectorXd input_val = result.GetSolution(input_);
    std::cout << "input_val: " << std::endl;
    std::cout << input_val << std::endl;

    VectorXd state_dot = f + g * input_val;

    std::cout << "At the end of SolveQP" << std::endl;
    return state_dot;
  }

  std::cout << "SolveQP failed!" << endl;
  std::cout << "At the end of SolveQP" << std::endl;
  DRAKE_ASSERT(0 > 1);
  return Vector3d::Zero();
}

bool SafeTrajGenerator::should_step(Vector3d reduced_order_state) const {
  std::cout << "At the start of should_step" << std::endl;
  std::map<Polynomiald::VarType, double> var_values;
  var_values[x[0].GetSimpleVariable()] = reduced_order_state(0);
  var_values[x[1].GetSimpleVariable()] = reduced_order_state(1);
  var_values[x[2].GetSimpleVariable()] = reduced_order_state(2);

  double V0_val = V0_.EvaluateMultivariate(var_values);
  double V1_val = V1_.EvaluateMultivariate(var_values);
  double W0_val = W0_.EvaluateMultivariate(var_values);
  std::cout << "At the end of should_step" << std::endl;
  if (W0_val <= 1 && V0_val > 1) return true;
  return false;
}

void SafeTrajGenerator::find_next_stance_location(
    Eigen::Vector3d CoM_wrt_foot, Eigen::Vector3d dCoM,
    Eigen::Vector3d swing_foot_pos, Eigen::Vector3d& next_stance_pos,
    double& t) const {
  std::cout << "At the start of find_next_stance_location" << std::endl;
  double dt = 0.01;
  double total_time = 0;
  Vector3d state_dot;
  Vector3d next_state;
  next_state << CoM_wrt_foot(0), dCoM(0), swing_foot_pos(0);
  int num_steps = 0;
  while (true) {
    state_dot = solveQP(CoM_wrt_foot, dCoM, swing_foot_pos);

    // Euler Integration
    next_state += state_dot * dt;
    total_time += dt;
    CoM_wrt_foot(0) = next_state(0);
    dCoM(0) = next_state(1);
    swing_foot_pos(0) = next_state(2);

    if (should_step(next_state)) {
      break;
    }
    if (num_steps > 200) {
      break;
    }
  }
  next_stance_pos = swing_foot_pos;
  if (num_steps > 200) {
    t = -1;
  } else {
    t = total_time;
  }
  std::cout << "At the end of find_next_stance_location" << std::endl;
}

void SafeTrajGenerator::CalcTraj(const Context<double>& context,
                                 Trajectory<double>* traj) const {
  std::cout << "At the start of CalcTraj" << std::endl;
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  VectorXd v = robot_output->GetVelocities();

  const BasicVector<double>* fsm_output =
      (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
  VectorXd fsm_state = fsm_output->get_value();

  const auto prev_td_time =
      context.get_discrete_state(prev_td_time_idx_).get_value();

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

  Vector3d CoM_wrt_foot = CoM - stance_foot_pos;
  const double CoM_wrt_foot_x = CoM(0) - stance_foot_pos(0);
  const double CoM_wrt_foot_y = CoM(1) - stance_foot_pos(1);
  const double CoM_wrt_foot_z = CoM(2) - stance_foot_pos(2);
  std::cout << "CoM_wrt_foot_x: " << std::endl;
  std::cout << CoM_wrt_foot_x << std::endl;
  std::cout << "CoM_wrt_foot_z: " << std::endl;
  std::cout << CoM_wrt_foot_z << std::endl;
  DRAKE_DEMAND(CoM_wrt_foot_z > 0);

  const std::vector<double> breaks = {
      current_time, current_time + 0.35};  // Think about this line

  Vector3d state_dot = solveQP(CoM_wrt_foot, dCoM, pt_on_swing_foot);

  Polynomiald t("t");
  drake::MatrixX<Polynomiald> polynomial_matrix(3, 1);
  polynomial_matrix(0, 0) = CoM_wrt_foot_x + dCoM(0) * t + state_dot(1) * t * t;
  polynomial_matrix(1, 0) = stance_foot_pos(1) + 0 * t + 0 * t * t;

  std::vector<MatrixXd> Y(breaks.size(), MatrixXd::Zero(1, 1));
  Y[0](0, 0) = CoM(2);
  Y[1](0, 0) = lipm_model_.get_desired_com_height();

  MatrixXd Y_dot_start = MatrixXd::Zero(1, 1);
  Y_dot_start(1, 0) = dCoM(2);
  MatrixXd Y_dot_end = MatrixXd::Zero(1, 1);

  PiecewisePolynomial<double> pp_part =
      PiecewisePolynomial<double>::Cubic(breaks, Y, Y_dot_start, Y_dot_end);

  polynomial_matrix(2, 0) = pp_part.getPolynomial(0, 0, 0);
  std::cout << "GetPolynomial!" << std::endl;

  const std::vector<drake::MatrixX<Polynomiald>> polynomials = {
      polynomial_matrix};
  std::cout << "Polynomials!" << std::endl;

  PiecewisePolynomial<double>* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  std::cout << "Casted traj created!" << std::endl;
  *casted_traj = PiecewisePolynomial<double>(polynomials, breaks);
  std::cout << "At the end of CalcTraj" << std::endl;
}

PiecewisePolynomial<double> SafeTrajGenerator::createSplineForSwingFoot(
    const double start_time_of_this_interval,
    const double end_time_of_this_interval, const Vector3d& init_swing_foot_pos,
    const Vector3d& CP) const {
  std::cout << "At the start of createSplineForSwingFoot" << std::endl;

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
  std::cout << "At the end of createSplineForSwingFoot" << std::endl;
  return swing_foot_spline;
}

void SafeTrajGenerator::CalcSwingTraj(const Context<double>& context,
                                      Trajectory<double>* traj) const {
  std::cout << "At the start of CalcSwingTraj" << std::endl;

  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);

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
    stance_duration = 0.35;
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
  *casted_traj = createSplineForSwingFoot(start_time_of_this_interval,
                                          end_time_of_this_interval,
                                          init_swing_foot_pos, next_stance_pos);
  std::cout << "At the end of CalcSwingTraj" << std::endl;
}

}  // namespace dairlib
