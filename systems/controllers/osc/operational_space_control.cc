#include "systems/controllers/osc/operational_space_control.h"
#include "attic/multibody/rigidbody_utils.h"

using std::cout;
using std::endl;

using std::vector;
using std::string;
using std::numeric_limits;

using Eigen::Vector2d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

using drake::systems::Context;
using drake::systems::BasicVector;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::ExponentialPlusPiecewisePolynomial;

using drake::solvers::MathematicalProgram;
using drake::solvers::Solve;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::SolutionResult;

namespace dairlib {
namespace systems {
namespace controllers {

using multibody::GetBodyIndexFromName;
using multibody::makeNameToVelocitiesMap;

OperationalSpaceControl::OperationalSpaceControl(
    const RigidBodyTree<double>& tree_w_spr,
    const RigidBodyTree<double>& tree_wo_spr,
    bool used_with_finite_state_machine,
    bool print_tracking_info) :
        tree_w_spr_(tree_w_spr),
        tree_wo_spr_(tree_wo_spr),
        used_with_finite_state_machine_(used_with_finite_state_machine),
        print_tracking_info_(print_tracking_info) {
  this->set_name("OSC");

  n_q_ = tree_wo_spr.get_num_positions();
  n_v_ = tree_wo_spr.get_num_velocities();
  n_u_ = tree_wo_spr.get_num_actuators();

  int n_q_w_spr = tree_w_spr.get_num_positions();
  int n_v_w_spr = tree_w_spr.get_num_velocities();
  int n_u_w_spr = tree_w_spr.get_num_actuators();

  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(
      OutputVector<double>(n_q_w_spr, n_v_w_spr, n_u_w_spr)).get_index();
  this->DeclareVectorOutputPort(TimestampedVector<double>(n_u_w_spr),
                                &OperationalSpaceControl::CalcOptimalInput);
  if (used_with_finite_state_machine) {
    fsm_port_ = this->DeclareVectorInputPort(
                  BasicVector<double>(1)).get_index();

    // Discrete update to record the last state event time
    DeclarePerStepDiscreteUpdateEvent(
        &OperationalSpaceControl::DiscreteVariableUpdate);
    prev_fsm_state_idx_ = this->DeclareDiscreteState(-0.1 * VectorXd::Ones(1));
    prev_event_time_idx_ = this->DeclareDiscreteState(VectorXd::Zero(1));
  }

  // Initialize the mapping from spring to no spring
  map_position_from_spring_to_no_spring_ = MatrixXd::Zero(n_q_, n_q_w_spr);
  for (int i = 0; i < n_q_; i++) {
    bool successfully_added = false;
    for (int j = 0; j < n_q_w_spr; j++) {
      std::string name_wo_spr = tree_wo_spr_.get_position_name(i);
      std::string name_w_spr = tree_w_spr_.get_position_name(j);
      if (name_wo_spr.compare(0, name_wo_spr.size(), name_w_spr) == 0) {
        map_position_from_spring_to_no_spring_(i, j) = 1;
        successfully_added = true;
      }
    }
    DRAKE_DEMAND(successfully_added);
  }
  map_velocity_from_spring_to_no_spring_ = MatrixXd::Zero(n_v_, n_v_w_spr);
  for (int i = 0; i < n_v_; i++) {
    bool successfully_added = false;
    for (int j = 0; j < n_v_w_spr; j++) {
      std::string name_wo_spr = tree_wo_spr_.get_velocity_name(i);
      std::string name_w_spr = tree_w_spr_.get_velocity_name(j);
      if (name_wo_spr.compare(0, name_wo_spr.size(), name_w_spr) == 0) {
        map_velocity_from_spring_to_no_spring_(i, j) = 1;
        successfully_added = true;
      }
    }
    DRAKE_DEMAND(successfully_added);
  }

  // Get input limits
  VectorXd u_min(n_u_);
  VectorXd u_max(n_u_);
  for (int i = 0; i < n_u_; i++) {
    u_min(i) = tree_wo_spr_.actuators[i].effort_limit_min_;
    u_max(i) = tree_wo_spr_.actuators[i].effort_limit_max_;
  }
  u_min_ = u_min;
  u_max_ = u_max;

  // Check if the model is floating based
  is_quaternion_ = multibody::IsFloatingBase(tree_w_spr);
}

// Cost methods
void OperationalSpaceControl::AddAccelerationCost(std::string joint_vel_name,
                                                  double w) {
  if (W_joint_accel_.size() == 0) {
    W_joint_accel_ = Eigen::MatrixXd::Zero(n_v_, n_v_);
  }
  int idx = makeNameToVelocitiesMap(tree_wo_spr_).at(joint_vel_name);
  W_joint_accel_(idx, idx) += w;
}

// Constraint methods
void OperationalSpaceControl::AddContactPoint(std::string body_name,
    VectorXd pt_on_body, double mu_low_friction, double period_of_low_friction){
  body_index_.push_back(GetBodyIndexFromName(tree_wo_spr_, body_name));
  pt_on_body_.push_back(pt_on_body);
  mu_low_friction_.push_back(mu_low_friction);
  period_of_low_friction_.push_back(period_of_low_friction);
}
void OperationalSpaceControl::AddStateAndContactPoint(int state,
    std::string body_name, VectorXd pt_on_body,
    double mu_low_friction, double period_of_low_friction) {
  fsm_state_when_active_.push_back(state);
  AddContactPoint(body_name, pt_on_body,
      mu_low_friction, period_of_low_friction);
}

// Tracking data methods
void OperationalSpaceControl::AddTrackingData(OscTrackingData* tracking_data,
    double t_lb, double t_ub) {
  tracking_data_vec_->push_back(tracking_data);
  fixed_position_vec_.push_back(VectorXd::Zero(0));
  t_s_vec_.push_back(t_lb);
  t_e_vec_.push_back(t_ub);

  // Construct input ports and add element to traj_name_to_port_index_map_
  string traj_name = tracking_data->GetName();
  PiecewisePolynomial<double> pp = PiecewisePolynomial<double>();
  int port_index = this->DeclareAbstractInputPort(traj_name,
      drake::Value<drake::trajectories::Trajectory<double>> (pp)).get_index();
  traj_name_to_port_index_map_[traj_name] = port_index;
}
void OperationalSpaceControl::AddConstTrackingData(
    OscTrackingData* tracking_data, VectorXd v, double t_lb, double t_ub) {
  tracking_data_vec_->push_back(tracking_data);
  fixed_position_vec_.push_back(v);
  t_s_vec_.push_back(t_lb);
  t_e_vec_.push_back(t_ub);
}

// Osc checkers and constructor
void OperationalSpaceControl::CheckCostSettings() {
  if (W_input_.size() != 0) {
    DRAKE_DEMAND((W_input_.rows() == n_u_) && (W_input_.cols() == n_u_));
  }
  if (W_joint_accel_.size() != 0) {
    DRAKE_DEMAND((W_joint_accel_.rows() == n_v_) &&
                 (W_joint_accel_.cols() == n_v_));
  }
}
void OperationalSpaceControl::CheckConstraintSettings() {
  if (!body_index_.empty()) {
    DRAKE_DEMAND(mu_ != -1);
    DRAKE_DEMAND(body_index_.size() == pt_on_body_.size());
  }
  if (!fsm_state_when_active_.empty()) {
    DRAKE_DEMAND(fsm_state_when_active_.size() == body_index_.size());
    DRAKE_DEMAND(fsm_state_when_active_.size() == pt_on_body_.size());
  }
}

void OperationalSpaceControl::Build() {
  // Checker
  CheckCostSettings();
  CheckConstraintSettings();
  for (auto tracking_data : *tracking_data_vec_) {
    tracking_data->CheckOscTrackingData();
  }

  // Construct QP
  n_h_ = tree_wo_spr_.getNumPositionConstraints();
  n_c_ = 3 * body_index_.size();
  prog_ = std::make_unique<MathematicalProgram>();

  // Add decision variables
  dv_ = prog_->NewContinuousVariables(n_v_, "dv");
  u_ = prog_->NewContinuousVariables(n_u_, "u");
  lambda_c_ = prog_->NewContinuousVariables(n_c_, "lambda_contact");
  lambda_h_ = prog_->NewContinuousVariables(n_h_, "lambda_holonomic");
  epsilon_ = prog_->NewContinuousVariables(n_c_, "epsilon");

  // Add constraints
  // 1. Dynamics constraint
  dynamics_constraint_ = prog_->AddLinearEqualityConstraint(
                           MatrixXd::Zero(n_v_, n_v_ + n_c_ + n_h_ + n_u_),
                           VectorXd::Zero(n_v_),
                           {dv_, lambda_c_, lambda_h_, u_}).
                         evaluator().get();
  // 2. Holonomic constraint
  holonomic_constraint_ = prog_->AddLinearEqualityConstraint(
                            MatrixXd::Zero(n_h_, n_v_),
                            VectorXd::Zero(n_h_), dv_).
                          evaluator().get();
  // 3. Contact constraint
  if (body_index_.size() > 0) {
    if (w_soft_constraint_ <= 0) {
      contact_constraints_ = prog_->AddLinearEqualityConstraint(
                               MatrixXd::Zero(n_c_, n_v_),
                               VectorXd::Zero(n_c_), dv_).
                             evaluator().get();
    } else {
      // Relaxed version:
      contact_constraints_ = prog_->AddLinearEqualityConstraint(
                               MatrixXd::Zero(n_c_, n_v_ + n_c_),
                               VectorXd::Zero(n_c_), {dv_, epsilon_}).
                             evaluator().get();
    }
  }
  // 4. Friction constraint (approximated firction cone)
  if (body_index_.size() > 0) {
    VectorXd mu_minus1(2); mu_minus1 << mu_, -1;
    VectorXd mu_plus1(2); mu_plus1 << mu_, 1;
    VectorXd one(1); one << 1;
    for (unsigned int j = 0; j < body_index_.size(); j++) {
      friction_constraints_.push_back(prog_->AddLinearConstraint(
                                        mu_minus1.transpose(),
                                        0, numeric_limits<double>::infinity(),
          {lambda_c_.segment(3 * j + 2, 1), lambda_c_.segment(3 * j + 0, 1)}).
          evaluator().get());

      friction_constraints_.push_back(prog_->AddLinearConstraint(
                                        mu_plus1.transpose(),
                                        0, numeric_limits<double>::infinity(),
          {lambda_c_.segment(3 * j + 2, 1), lambda_c_.segment(3 * j + 0, 1)}).
          evaluator().get());
      friction_constraints_.push_back(prog_->AddLinearConstraint(
                                        mu_minus1.transpose(),
                                        0, numeric_limits<double>::infinity(),
          {lambda_c_.segment(3 * j + 2, 1), lambda_c_.segment(3 * j + 1, 1)}).
          evaluator().get());
      friction_constraints_.push_back(prog_->AddLinearConstraint(
                                        mu_plus1.transpose(),
                                        0, numeric_limits<double>::infinity(),
          {lambda_c_.segment(3 * j + 2, 1), lambda_c_.segment(3 * j + 1, 1)}).
          evaluator().get());
      friction_constraints_.push_back(prog_->AddLinearConstraint(one.transpose(),
                                      0, numeric_limits<double>::infinity(),
                                      lambda_c_.segment(3 * j + 2, 1)).
                                      evaluator().get());
    }
  }
  // 5. Input constraint
  if (with_input_constraints_) {
    prog_->AddLinearConstraint(
        MatrixXd::Identity(n_u_, n_u_), u_min_, u_max_, u_);
  }
  // No joint position constraint in this implementation

  // Add costs
  // 1. input cost
  if (W_input_.size() > 0) {
    prog_->AddQuadraticCost(W_input_,
                            VectorXd::Zero(n_u_), u_);
  }
  // 2. acceleration cost
  if (W_joint_accel_.size() > 0) {
    prog_->AddQuadraticCost(W_joint_accel_,
        VectorXd::Zero(n_v_), dv_);
  }
  // 3. Soft constraint cost
  if (w_soft_constraint_ > 0) {
    // HACK (NOT TO BE COMMITTED TO MASTER): sets cost associated with linearly
    // dependent contact constraint violations to be zero
    MatrixXd weight = w_soft_constraint_*MatrixXd::Identity(n_c_, n_c_);
    weight(5, 5) = 0;
    weight(11, 11) = 0;
    prog_->AddQuadraticCost(weight,
        VectorXd::Zero(n_c_), epsilon_);
  }
  // 4. Tracking cost
  for (unsigned int i = 0; i < tracking_data_vec_->size(); i++) {
    tracking_cost_.push_back(prog_->AddQuadraticCost(
                               MatrixXd::Identity(n_v_, n_v_),
                               VectorXd::Zero(n_v_), dv_).
                             evaluator().get());
  }
}

std::vector<bool> OperationalSpaceControl::CalcActiveContactIndices(
    int fsm_state) const {
  std::vector<bool> active_contact_flags;
  for (unsigned int i = 0; i < body_index_.size(); i++) {
    if (fsm_state_when_active_.empty()) {
      active_contact_flags.push_back(true);
    } else {
      if (fsm_state_when_active_[i] == fsm_state) {
        active_contact_flags.push_back(true);
      } else {
        active_contact_flags.push_back(false);
      }
    }
  }
  return active_contact_flags;
}

drake::systems::EventStatus OperationalSpaceControl::DiscreteVariableUpdate(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {
  const BasicVector<double>* fsm_output = (BasicVector<double>*)
                                          this->EvalVectorInput(context, fsm_port_);
  VectorXd fsm_state = fsm_output->get_value();
  const OutputVector<double>* robot_output = (OutputVector<double>*)
      this->EvalVectorInput(context, state_port_);
  double timestamp = robot_output->get_timestamp();

  auto prev_fsm_state = discrete_state->get_mutable_vector(
                          prev_fsm_state_idx_).get_mutable_value();
  if (fsm_state(0) != prev_fsm_state(0)) {
    prev_fsm_state(0) = fsm_state(0);

    discrete_state->get_mutable_vector(
        prev_event_time_idx_).get_mutable_value() << timestamp;
  }
  return drake::systems::EventStatus::Succeeded();
}


VectorXd OperationalSpaceControl::SolveQp(
    VectorXd x_w_spr, VectorXd x_wo_spr,
    const drake::systems::Context<double>& context, double t,
    int fsm_state, double time_since_last_state_switch) const {

  vector<bool> active_contact_flags = CalcActiveContactIndices(fsm_state);

  // Get Kinematics Cache
  KinematicsCache<double> cache_w_spr = tree_w_spr_.doKinematics(
      x_w_spr.head(tree_w_spr_.get_num_positions()),
      x_w_spr.tail(tree_w_spr_.get_num_velocities()));
  KinematicsCache<double> cache_wo_spr = tree_wo_spr_.doKinematics(
      x_wo_spr.head(n_q_), x_wo_spr.tail(n_v_));

  // Get M, f_cg, B matrices of the manipulator equation
  MatrixXd B = tree_wo_spr_.B;
  MatrixXd M = tree_wo_spr_.massMatrix(cache_wo_spr);
  const RigidBodyTree<double>::BodyToWrenchMap no_external_wrenches;
  VectorXd bias = tree_wo_spr_.dynamicsBiasTerm(cache_wo_spr,
                  no_external_wrenches);

  // Get J and JdotV for holonomic constraint
  MatrixXd J_h = tree_wo_spr_.positionConstraintsJacobian(cache_wo_spr, false);
  VectorXd JdotV_h = tree_wo_spr_.positionConstraintsJacDotTimesV(cache_wo_spr);

  // Get J and JdotV for contact constraint
  MatrixXd J_c = MatrixXd::Zero(n_c_, n_v_);
  VectorXd JdotV_c = VectorXd::Zero(n_c_);
  for (unsigned int i = 0; i < active_contact_flags.size(); i++) {
    if (active_contact_flags[i]) {
      J_c.block(3 * i, 0, 3, n_v_) = tree_wo_spr_.transformPointsJacobian(
                                       cache_wo_spr, pt_on_body_[i],
                                       body_index_[i], 0, false);
      JdotV_c.segment(3 * i, 3) = tree_wo_spr_.transformPointsJacobianDotTimesV(
                                    cache_wo_spr, pt_on_body_[i],
                                    body_index_[i], 0);
    }
  }

  // Update constraints
  // 1. Dynamics constraint
  ///    M*dv + bias == J_c^T*lambda_c + J_h^T*lambda_h + B*u
  /// -> M*dv - J_c^T*lambda_c - J_h^T*lambda_h - B*u == - bias
  /// -> [M, -J_c^T, -J_h^T, -B]*[dv, lambda_c, lambda_h, u]^T = - bias
  MatrixXd A_dyn = MatrixXd::Zero(n_v_, n_v_ + n_c_ + n_h_ + n_u_);
  A_dyn.block(0, 0, n_v_, n_v_) = M;
  A_dyn.block(0, n_v_, n_v_, n_c_) = -J_c.transpose();
  A_dyn.block(0, n_v_ + n_c_ , n_v_, n_h_) = -J_h.transpose();
  A_dyn.block(0, n_v_ + n_c_ + n_h_, n_v_, n_u_) = -B;
  dynamics_constraint_->UpdateCoefficients(A_dyn, -bias);
  // 2. Holonomic constraint
  ///    JdotV_h + J_h*dv == 0
  /// -> J_h*dv == -JdotV_h
  holonomic_constraint_->UpdateCoefficients(J_h, -JdotV_h);
  // 3. Contact constraint
  if (w_soft_constraint_ <= 0) {
    ///    JdotV_c + J_c*dv == 0
    /// -> J_c*dv == -JdotV_c
    contact_constraints_->UpdateCoefficients(J_c, -JdotV_c);
  }
  else {
    // Relaxed version:
    ///    JdotV_c + J_c*dv == -epsilon
    /// -> J_c*dv + I*epsilon == -JdotV_c
    /// -> [J_c, I]* [dv, epsilon]^T == -JdotV_c
    MatrixXd A_c = MatrixXd::Zero(n_c_, n_v_ + n_c_);
    A_c.block(0, 0, n_c_, n_v_) = J_c;
    A_c.block(0, n_v_, n_c_, n_c_) = MatrixXd::Identity(n_c_, n_c_);
    contact_constraints_->UpdateCoefficients(A_c, -JdotV_c);
  }
  // 4. Friction constraint (approximated firction cone)
  /// For i = active contact indices
  ///     mu_*lambda_c(3*i+2) >= lambda_c(3*i+0)
  ///    -mu_*lambda_c(3*i+2) <= lambda_c(3*i+0)
  ///     mu_*lambda_c(3*i+2) >= lambda_c(3*i+1)
  ///    -mu_*lambda_c(3*i+2) <= lambda_c(3*i+1)
  ///         lambda_c(3*i+2) >= 0
  /// ->
  ///     mu_*lambda_c(3*i+2) - lambda_c(3*i+0) >= 0
  ///     mu_*lambda_c(3*i+2) + lambda_c(3*i+0) >= 0
  ///     mu_*lambda_c(3*i+2) - lambda_c(3*i+1) >= 0
  ///     mu_*lambda_c(3*i+2) + lambda_c(3*i+1) >= 0
  ///                           lambda_c(3*i+2) >= 0
  if (body_index_.size() > 0) {
    VectorXd mu_minus1(2);
    VectorXd mu_plus1(2);
    VectorXd inf_vectorxd(1); inf_vectorxd << numeric_limits<double>::infinity();
    for (unsigned int i = 0; i < active_contact_flags.size(); i++) {
      // If the contact is inactive, we assign zeros to A matrix. (lb<=Ax<=ub)
      if (active_contact_flags[i]) {
        if (time_since_last_state_switch < period_of_low_friction_[i]) {
          mu_minus1 << mu_low_friction_[i], -1;
          mu_plus1 << mu_low_friction_[i], 1;
        } else {
          mu_minus1 << mu_, -1;
          mu_plus1 << mu_, 1;
        }
        friction_constraints_.at(5 * i)->UpdateCoefficients(
            mu_minus1.transpose(), VectorXd::Zero(1), inf_vectorxd);
        friction_constraints_.at(5 * i + 1)->UpdateCoefficients(
            mu_plus1.transpose(), VectorXd::Zero(1), inf_vectorxd);
        friction_constraints_.at(5 * i + 2)->UpdateCoefficients(
            mu_minus1.transpose(), VectorXd::Zero(1), inf_vectorxd);
        friction_constraints_.at(5 * i + 3)->UpdateCoefficients(
            mu_plus1.transpose(), VectorXd::Zero(1), inf_vectorxd);
        friction_constraints_.at(5 * i + 4)->UpdateLowerBound(VectorXd::Zero(1));
      } else {
        friction_constraints_.at(5 * i)->UpdateCoefficients(
            Vector2d::Zero().transpose(), VectorXd::Zero(1), inf_vectorxd);
        friction_constraints_.at(5 * i + 1)->UpdateCoefficients(
            Vector2d::Zero().transpose(), VectorXd::Zero(1), inf_vectorxd);
        friction_constraints_.at(5 * i + 2)->UpdateCoefficients(
            Vector2d::Zero().transpose(), VectorXd::Zero(1), inf_vectorxd);
        friction_constraints_.at(5 * i + 3)->UpdateCoefficients(
            Vector2d::Zero().transpose(), VectorXd::Zero(1), inf_vectorxd);
        friction_constraints_.at(5 * i + 4)->UpdateLowerBound(-inf_vectorxd);
      }
    }
  }

  // Update costs
  // 4. Tracking cost
  for (unsigned int i = 0; i < tracking_data_vec_->size(); i++) {
    auto tracking_data = tracking_data_vec_->at(i);

    // Check whether or not it is a constant trajectory, and update TrackingData
    if (fixed_position_vec_.at(i).size() > 0){
      // Create constant trajectory and update
      tracking_data->Update(x_w_spr, cache_w_spr,
          x_wo_spr, cache_wo_spr,
          PiecewisePolynomial<double>(fixed_position_vec_.at(i)), t, fsm_state);
    } else {
      // Read in traj from input port
      string traj_name = tracking_data->GetName();
      int port_index = traj_name_to_port_index_map_.at(traj_name);
      const drake::AbstractValue* traj_intput =
          this->EvalAbstractInput(context, port_index);
      DRAKE_DEMAND(traj_intput != nullptr);
      const drake::trajectories::Trajectory<double> & traj =
          traj_intput->get_value<drake::trajectories::Trajectory<double>>();
      // Update
      tracking_data->Update(x_w_spr, cache_w_spr,
                          x_wo_spr, cache_wo_spr,
                          traj, t,
                          fsm_state);
    }

    if (tracking_data->GetTrackOrNot() &&
        time_since_last_state_switch >= t_s_vec_.at(i) &&
        time_since_last_state_switch <= t_e_vec_.at(i)) {
      VectorXd ddy_t = tracking_data->GetCommandOutput();
      MatrixXd W = tracking_data->GetWeight();
      MatrixXd J_t = tracking_data->GetJ();
      VectorXd JdotV_t = tracking_data->GetJdotTimesV();
      // The tracking cost is
      // 0.5 * (J_*dv + JdotV - y_command)^T * W * (J_*dv + JdotV - y_command).
      // We ignore the constant term
      // 0.5 * (JdotV - y_command)^T * W * (JdotV - y_command),
      // since it doesn't change the result of QP.
      tracking_cost_.at(i)->UpdateCoefficients(J_t.transpose()* W * J_t,
          J_t.transpose()* W * (JdotV_t - ddy_t));
    } else {
      tracking_cost_.at(i)->UpdateCoefficients(MatrixXd::Zero(n_v_, n_v_),
                               VectorXd::Zero(n_v_));
    }
  }

  // Solve the QP
  const MathematicalProgramResult result = Solve(*prog_);
  SolutionResult solution_result = result.get_solution_result();
  if (print_tracking_info_) {
    cout << "\n" << to_string(solution_result) <<  endl;
  }

  // Extract solutions
  VectorXd u_sol = result.GetSolution(u_);
  VectorXd lambda_c_sol = result.GetSolution(lambda_c_);
  VectorXd lambda_h_sol = result.GetSolution(lambda_h_);
  VectorXd dv_sol = result.GetSolution(dv_);
  VectorXd epsilon_sol = result.GetSolution(epsilon_);
  if (print_tracking_info_) {
    cout << "**********************\n";
    cout << "u_sol = " << u_sol.transpose() << endl;
    cout << "lambda_c_sol = " << lambda_c_sol.transpose() << endl;
    cout << "lambda_h_sol = " << lambda_h_sol.transpose() << endl;
    cout << "dv_sol = " << dv_sol.transpose() << endl;
    cout << "epsilon_sol = " << epsilon_sol.transpose() << endl;
  }

  // Print QP result
  if (print_tracking_info_) {
    cout << "**********************\n";
    // 1. input cost
    if (W_input_.size() > 0) {
      cout << "input cost = " << 0.5 * u_sol.transpose()*W_input_*u_sol << endl;
    }
    // 2. acceleration cost
    if (W_joint_accel_.size() > 0) {
      cout << "acceleration cost = " <<
           0.5 * dv_sol.transpose()*W_joint_accel_*dv_sol << endl;
    }
    // 3. Soft constraint cost
    if (w_soft_constraint_ > 0) {
      cout << "soft constraint cost = " <<
           0.5 * w_soft_constraint_*epsilon_sol.transpose()*epsilon_sol << endl;
    }
    // 4. Tracking cost
    for (unsigned int i = 0; i < tracking_data_vec_->size(); i++) {
      auto tracking_data = tracking_data_vec_->at(i);
      if (tracking_data->GetTrackOrNot()) {
        VectorXd ddy_t = tracking_data->GetCommandOutput();
        MatrixXd W = tracking_data->GetWeight();
        MatrixXd J_t = tracking_data->GetJ();
        VectorXd JdotV_t = tracking_data->GetJdotTimesV();
        // Note that the following cost also includes the constant term, so that
        // the user can differentiate which error norm is bigger. The constant
        // term was not added to the QP since it doesn't change the result.
        cout << "Tracking cost (" << tracking_data->GetName() << ") = " <<
             0.5 * (J_t * dv_sol + JdotV_t - ddy_t).transpose() * W *
             (J_t * dv_sol + JdotV_t - ddy_t) << endl;
      }
    }

    // Target acceleration
    cout << "**********************\n";
    for (auto tracking_data : *tracking_data_vec_) {
      if (tracking_data->GetTrackOrNot()) {
        tracking_data->PrintFeedbackAndDesiredValues(dv_sol);
      }
    }
    cout << "**********************\n\n";
  }

  return u_sol;
}


void OperationalSpaceControl::CalcOptimalInput(
    const drake::systems::Context<double>& context,
    systems::TimestampedVector<double>* control) const {
  // Read in current time and state
  const OutputVector<double>* robot_output = (OutputVector<double>*)
      this->EvalVectorInput(context, state_port_);
  double timestamp = robot_output->get_timestamp();
  double current_time = static_cast<double>(timestamp);
  // cout << "\n\ncurrent_time = " << current_time << endl;
  if (print_tracking_info_) {
    cout << "\n\ncurrent_time = " << current_time << endl;
  }

  VectorXd u_sol(n_u_);
  if (current_time == 0) {
    u_sol = VectorXd::Zero(n_u_);
  } else {
    VectorXd q_w_spr = robot_output->GetPositions();
    if (is_quaternion_) {
      multibody::SetZeroQuaternionToIdentity(&q_w_spr);
    }
    VectorXd v_w_spr = robot_output->GetVelocities();
    VectorXd x_w_spr(tree_w_spr_.get_num_positions() +
                     tree_w_spr_.get_num_velocities());
    x_w_spr << q_w_spr, v_w_spr;

    VectorXd x_wo_spr(n_q_ + n_v_);
    x_wo_spr << map_position_from_spring_to_no_spring_ * q_w_spr,
             map_velocity_from_spring_to_no_spring_ * v_w_spr;

    if (used_with_finite_state_machine_) {
      // Read in finite state machine
      const BasicVector<double>* fsm_output = (BasicVector<double>*)
          this->EvalVectorInput(context, fsm_port_);
      VectorXd fsm_state = fsm_output->get_value();

      // Get discrete states
      const auto prev_event_time = context.get_discrete_state(
                                     prev_event_time_idx_).get_value();

      u_sol = SolveQp(x_w_spr, x_wo_spr,
                      context, current_time,
                      fsm_state(0), current_time - prev_event_time(0));
    } else {
      u_sol = SolveQp(x_w_spr, x_wo_spr,
                      context, current_time,
                      -1, current_time);
    }
  }

  // cout << "u_sol = " << u_sol.transpose() << endl;
  // Assign the control input
  control->SetDataVector(u_sol);
  control->set_timestamp(robot_output->get_timestamp());
}

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib


