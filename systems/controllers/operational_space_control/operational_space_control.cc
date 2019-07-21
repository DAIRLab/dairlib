#include "systems/controllers/operational_space_control/operational_space_control.h"
#include "attic/multibody/rigidbody_utils.h"
#include "common/math_utils.h"

#include <chrono>   // measuring runtime

using std::chrono::high_resolution_clock;
using std::chrono::microseconds;
using std::chrono::duration_cast;

using std::cout;
using std::endl;

using std::vector;
using std::string;
using std::numeric_limits;

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

OperationalSpaceControl::OperationalSpaceControl(
  RigidBodyTree<double>* tree_w_spr,
  RigidBodyTree<double>* tree_wo_spr,
  bool used_with_finite_state_machine,
  bool print_tracking_info) :
  tree_w_spr_(tree_w_spr),
  tree_wo_spr_(tree_wo_spr),
  used_with_finite_state_machine_(used_with_finite_state_machine),
  print_tracking_info_(print_tracking_info) {
  this->set_name("OSC");

  n_q_ = tree_wo_spr->get_num_positions();
  n_v_ = tree_wo_spr->get_num_velocities();
  n_u_ = tree_wo_spr->get_num_actuators();
  cout << "n_q_ = " << n_q_ << endl;
  cout << "n_v_ = " << n_v_ << endl;
  cout << "n_u_ = " << n_u_ << endl;

  int n_q_w_spr = tree_w_spr->get_num_positions();
  int n_v_w_spr = tree_w_spr->get_num_velocities();
  int n_u_w_spr = tree_w_spr->get_num_actuators();
  cout << "n_q_w_spr = " << n_q_w_spr << endl;
  cout << "n_v_w_spr = " << n_v_w_spr << endl;
  cout << "n_u_w_spr = " << n_u_w_spr << endl;

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
    prev_fsm_state_idx_ = this->DeclareDiscreteState(1);
    prev_event_time_idx_ = this->DeclareDiscreteState(VectorXd::Zero(1));
  }

  // Initialize the mapping from spring to no spring
  map_position_from_spring_to_no_spring_ = MatrixXd::Zero(n_q_, n_q_w_spr);
  for (int i = 0; i < n_q_; i++)
    for (int j = 0; j < n_q_w_spr; j++) {
      std::string name_wo_spr = tree_wo_spr_->get_position_name(i);
      std::string name_w_spr = tree_w_spr_->get_position_name(j);
      if (name_wo_spr.compare(0, name_wo_spr.size(), name_w_spr) == 0) {
        map_position_from_spring_to_no_spring_(i, j) = 1;
      }
    }
  // cout<<"map_position_from_spring_to_no_spring_ = \n"<<map_position_from_spring_to_no_spring_<<"\n";
  map_velocity_from_spring_to_no_spring_ = MatrixXd::Zero(n_v_, n_v_w_spr);
  for (int i = 0; i < n_v_; i++)
    for (int j = 0; j < n_v_w_spr; j++) {
      std::string name_wo_spr = tree_wo_spr_->get_velocity_name(i);
      std::string name_w_spr = tree_w_spr_->get_velocity_name(j);
      if (name_wo_spr.compare(0, name_wo_spr.size(), name_w_spr) == 0) {
        map_velocity_from_spring_to_no_spring_(i, j) = 1;
      }
    }
  // cout<<"map_position_from_spring_to_no_spring_ = \n"<<map_position_from_spring_to_no_spring_<<"\n";

  // Get input limits
  VectorXd u_min(n_u_);
  VectorXd u_max(n_u_);
  for (int i = 0; i < n_u_; i++) {
    u_min(i) = tree_wo_spr_->actuators[i].effort_limit_min_;
    u_max(i) = tree_wo_spr_->actuators[i].effort_limit_max_;
  }
  u_min_ = u_min;
  u_max_ = u_max;

  // Check if the model is floating based
  is_quaternion_ = multibody::CheckFloatingBase(tree_w_spr);
}

// Cost methods
void OperationalSpaceControl::AddAccelerationCost(int joint_vel_idx, double w) {
  if (W_joint_accel_.size() == 0) {
    W_joint_accel_ = Eigen::MatrixXd::Zero(n_v_, n_v_);
  }
  W_joint_accel_(joint_vel_idx, joint_vel_idx) += w;
}

// Constraint methods
void OperationalSpaceControl::AddContactPoint(int body_index,
    Eigen::VectorXd pt_on_body) {
  body_index_.push_back(body_index);
  pt_on_body_.push_back(pt_on_body);
}
void OperationalSpaceControl::AddStateAndContactPoint(int state,
    int body_index, Eigen::VectorXd pt_on_body) {
  state_.push_back(state);
  AddContactPoint(body_index, pt_on_body);
}
void OperationalSpaceControl::AddContactPoint(std::vector<int> body_index,
    std::vector<Eigen::VectorXd> pt_on_body) {
  body_index_.insert(body_index_.end(), body_index.begin(), body_index.end());
  pt_on_body_.insert(pt_on_body_.end(), pt_on_body.begin(), pt_on_body.end());
}
void OperationalSpaceControl::AddStateAndContactPoint(std::vector<int> state,
    std::vector<int> body_index,
    std::vector<Eigen::VectorXd> pt_on_body) {
  state_.insert(state_.end(), state.begin(), state.end());
  AddContactPoint(body_index, pt_on_body);
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
  if (!state_.empty()) {
    DRAKE_DEMAND(state_.size() == body_index_.size());
  }
}

void OperationalSpaceControl::ConstructOSC() {
  // Checker
  CheckCostSettings();
  CheckConstraintSettings();
  for (auto tracking_data : *tracking_data_vec_) {
    tracking_data->CheckOscTrackingData();
  }

  // Construct traj_name_to_port_index_map_
  for (auto tracking_data : *tracking_data_vec_) {
    string traj_name = tracking_data->GetName();
    int port_index;
    if (tracking_data->TrajHasExp()) {
      port_index = this->DeclareAbstractInputPort(traj_name,
                   drake::Value<ExponentialPlusPiecewisePolynomial<double>> {}).get_index();
    } else {
      port_index = this->DeclareAbstractInputPort(traj_name,
                   drake::Value<PiecewisePolynomial<double>> {}).get_index();
    }
    traj_name_to_port_index_map_[traj_name] = port_index;
  }

  // TODO: construct the QP here. (will do so after the controller is working)
}

std::vector<int> OperationalSpaceControl::CalcActiveContactIndices(
  int fsm_state) const {
  std::vector<int> active_contact_idx;
  for (unsigned int i = 0; i < body_index_.size(); i++) {
    if (state_.empty()) {
      active_contact_idx.push_back(i);
    } else {
      if (state_[i] == fsm_state) {
        active_contact_idx.push_back(i);
      }
    }
  }
  return active_contact_idx;
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


// TODO(yminchen): currently construct the QP in every loop. Will modify this
// once the code is working.
VectorXd OperationalSpaceControl::SolveQp(
  VectorXd x_w_spr, VectorXd x_wo_spr,
  const drake::systems::Context<double>& context, double t,
  int fsm_state, double time_since_last_state_switch) const {

  vector<int> active_contact_idx = CalcActiveContactIndices(fsm_state);

  int n_h = tree_wo_spr_->getNumPositionConstraints();
  int n_c = 3 * active_contact_idx.size();
  // TODO: will need to use   int n_c = 3 * body_index_.size();
  // when we construct the QP in OSC constructor
  // cout << "n_c = " << n_c << endl;

  // Get Kinematics Cache
  KinematicsCache<double> cache_w_spr = tree_w_spr_->doKinematics(
                                          x_w_spr.head(tree_w_spr_->get_num_positions()),
                                          x_w_spr.tail(tree_w_spr_->get_num_velocities()));
  KinematicsCache<double> cache_wo_spr = tree_wo_spr_->doKinematics(
      x_wo_spr.head(n_q_), x_wo_spr.tail(n_v_));

  // Get M, f_cg, B matrices of the manipulator equation
  MatrixXd B = tree_wo_spr_->B;
  MatrixXd M = tree_wo_spr_->massMatrix(cache_wo_spr);
  // cout<<"Matrix M dimension = "<< M.rows()<<" "<< M.cols()<<"\n\n";
  // cout<< "M.row(18) = " << M.row(18) <<  endl;
  const RigidBodyTree<double>::BodyToWrenchMap no_external_wrenches;
  VectorXd bias = tree_wo_spr_->dynamicsBiasTerm(cache_wo_spr,
                  no_external_wrenches);

  // Get J and JdotV for holonomic constraint
  MatrixXd J_h = tree_wo_spr_->positionConstraintsJacobian(cache_wo_spr, false);
  VectorXd JdotV_h = tree_wo_spr_->positionConstraintsJacDotTimesV(cache_wo_spr);

  // Get J and JdotV for contact constraint
  MatrixXd J_c = MatrixXd::Zero(n_c, n_v_);
  VectorXd JdotV_c = VectorXd::Zero(n_c);
  for (unsigned int i = 0; i < active_contact_idx.size(); i++) {
    J_c.block(3 * i, 0, 3, n_v_) = tree_wo_spr_->transformPointsJacobian(
                                     cache_wo_spr, pt_on_body_[active_contact_idx[i]],
                                     body_index_[active_contact_idx[i]], 0, false);
    JdotV_c.segment(3 * i, 3) = tree_wo_spr_->transformPointsJacobianDotTimesV(
                                  cache_wo_spr, pt_on_body_[active_contact_idx[i]],
                                  body_index_[active_contact_idx[i]], 0);
  }

  // Construct QP
  // cout << "Constructing QP\n";
  MathematicalProgram prog;

  // Add decision variables
  auto u = prog.NewContinuousVariables(n_u_, "u");
  auto lambda_c = prog.NewContinuousVariables(n_c,
                  "lambda_contact");
  auto lambda_h = prog.NewContinuousVariables(n_h, "lambda_holonomic");
  auto dv = prog.NewContinuousVariables(n_v_, "dv");
  auto epsilon = prog.NewContinuousVariables(n_c, "epsilon");

  // Add constraints
  // 1. Dynamics constraint
  ///    M*dv + bias == J_c^T*lambda_c + J_h^T*lambda_h + B*u
  /// -> M*dv - J_c^T*lambda_c - J_h^T*lambda_h - B*u == - bias
  /// -> [M, -J_c^T, -J_h^T, -B]*[dv, lambda_c, lambda_h, u]^T = - bias
  // cout<< "size of (M, -J_c.transpose(), -J_h.transpose(), -B):\n"
  //       << M.rows() << ", " << M.cols() << "\n"
  //       << J_c.transpose().rows() << ", " << J_c.transpose().cols() << "\n"
  //       << J_h.transpose().rows() << ", " << J_h.transpose().cols() << "\n"
  //       << B.rows() << ", " << B.cols() << "\n";
  // cout << "Add dynamic constraint\n";
  MatrixXd A_dyn = MatrixXd::Zero(n_v_, n_v_ + n_c + J_h.rows() + n_u_);
  A_dyn.block(0, 0, n_v_, n_v_) = M;
  A_dyn.block(0, n_v_, n_v_, n_c) = -J_c.transpose();
  A_dyn.block(0, n_v_ + n_c , n_v_, J_h.rows()) = -J_h.transpose();
  A_dyn.block(0, n_v_ + n_c + J_h.rows(), n_v_, n_u_) = -B;
  // cout<< "M # of rows and cols = " <<M.rows()<<", "<<M.cols()<< endl;
  // cout<< "J_c.transpose() # of rows and cols = " <<J_c.transpose().rows()<<", "<<J_c.transpose().cols()<< endl;
  // cout<< "J_h.transpose() # of rows and cols = " <<J_h.transpose().rows()<<", "<<J_h.transpose().cols()<< endl;
  // cout<< "B # of rows and cols = " <<B.rows()<<", "<<B.cols()<< endl;
  // cout<<"A_dyn = \n"<<A_dyn<<"\n";
  prog.AddLinearConstraint(A_dyn, -bias, -bias, {dv, lambda_c, lambda_h, u});

  // 2. Holonomic constraint
  ///    JdotV_h + J_h*dv == 0
  /// -> J_h*dv == -JdotV_h
  // cout << "Add holonomic constraint\n";
  // cout<< "J_h # of rows and cols = " <<J_h.rows()<<", "<<J_h.cols()<< endl;
  // cout<< "JdotV_h # of rows and cols = " <<JdotV_h.rows()<<", "<<JdotV_h.cols()<< endl;
  prog.AddLinearConstraint(J_h, -JdotV_h, -JdotV_h, dv);

  // 3. Contact constraint
  if (active_contact_idx.size() > 0) {
    if (w_soft_constraint_ <= 0) {
      ///    JdotV_c + J_c*dv == 0
      /// -> J_c*dv == -JdotV_c
      // cout << "Add contact constraint\n";
      // cout<< "J_c # of rows and cols = " <<J_c.rows()<<", "<<J_c.cols()<< endl;
      // cout<< "JdotV_c # of rows and cols = " <<JdotV_c.rows()<<", "<<JdotV_c.cols()<< endl;
      prog.AddLinearConstraint(J_c, -JdotV_c, -JdotV_c, dv);
    }
    else {
      // Relaxed version:
      ///    JdotV_c + J_c*dv == -epsilon
      /// -> J_c*dv + I*epsilon == -JdotV_c
      /// -> [J_c, I]* [dv, epsilon]^T == -JdotV_c
      // cout << "Add soft contact constraint\n";
      MatrixXd A_c = MatrixXd::Zero(n_c, n_v_ + n_c);
      A_c.block(0, 0, n_c, n_v_) = J_c;
      A_c.block(0, n_v_, n_c, n_c) = MatrixXd::Identity(n_c, n_c);
      // cout<<"A_c: \n"<<A_c<<"\n";
      prog.AddLinearConstraint(A_c,
                               -JdotV_c, -JdotV_c, {dv, epsilon});
    }
  }

  // 4. Friction cone constraint (approximation)
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
    // cout << "Add friction constraint\n";
    VectorXd mu_minus1(2); mu_minus1 << mu_, -1;
    VectorXd mu_plus1(2); mu_plus1 << mu_, 1;
    VectorXd one(1); one << 1;
    for (unsigned int j = 0; j < active_contact_idx.size(); j++) {
      int i = active_contact_idx[j];
      prog.AddLinearConstraint(mu_minus1.transpose(),
                               0, numeric_limits<double>::infinity(),
      {lambda_c.segment(3 * i + 2, 1), lambda_c.segment(3 * i + 0, 1)});
      prog.AddLinearConstraint(mu_plus1.transpose(),
                               0, numeric_limits<double>::infinity(),
      {lambda_c.segment(3 * i + 2, 1), lambda_c.segment(3 * i + 0, 1)});
      prog.AddLinearConstraint(mu_minus1.transpose(),
                               0, numeric_limits<double>::infinity(),
      {lambda_c.segment(3 * i + 2, 1), lambda_c.segment(3 * i + 1, 1)});
      prog.AddLinearConstraint(mu_plus1.transpose(),
                               0, numeric_limits<double>::infinity(),
      {lambda_c.segment(3 * i + 2, 1), lambda_c.segment(3 * i + 1, 1)});
      prog.AddLinearConstraint(one.transpose(),
                               0, numeric_limits<double>::infinity(),
                               lambda_c.segment(3 * i + 2, 1));
    }
  }

  // 5. Input constraint
  if (with_input_constraints_) {
    // cout << "Add input constraint\n";
    prog.AddLinearConstraint(MatrixXd::Identity(n_u_, n_u_), u_min_, u_max_, u);
  }

  // No joint position constraint in this implementation

  // Add costs
  // 1. input cost
  if (W_input_.size() > 0) {
    // cout << "Add input cost\n";
    prog.AddQuadraticCost(W_input_, VectorXd::Zero(n_u_), u);
  }

  // 2. acceleration cost
  if (W_joint_accel_.size() > 0) {
    // cout << "Add joint accelration cost\n";
    prog.AddQuadraticCost(W_joint_accel_, VectorXd::Zero(n_v_), dv);
  }

  // 3. Soft constraint cost
  if (w_soft_constraint_ > 0) {
    // cout << "Add soft constraint cost\n";
    prog.AddQuadraticCost(w_soft_constraint_ * MatrixXd::Identity(n_c, n_c),
                          VectorXd::Zero(n_c),
                          epsilon );
  }

  // 4. Tracking cost
  for (auto tracking_data : *tracking_data_vec_) {
    string traj_name = tracking_data->GetName();
    // cout << "Add cost for " << traj_name << endl;
    int port_index = traj_name_to_port_index_map_.at(traj_name);
    // cout << "port_index = " << port_index << endl;
    // Read in traj
    const drake::AbstractValue* traj_intput =
      this->EvalAbstractInput(context, port_index);
    DRAKE_DEMAND(traj_intput != nullptr);
    const drake::trajectories::Trajectory<double> & traj =
      (tracking_data->TrajHasExp()) ?
      traj_intput->get_value<ExponentialPlusPiecewisePolynomial<double>>() :
      traj_intput->get_value<PiecewisePolynomial<double>>();

    bool track_or_not = tracking_data->Update(x_w_spr,
                        cache_w_spr, tree_w_spr_,
                        x_wo_spr,
                        cache_wo_spr, tree_wo_spr_,
                        traj, t,
                        fsm_state, time_since_last_state_switch);
    if (track_or_not) {
      VectorXd ddy_t = tracking_data->GetCommandOutput();
      // cout << "ddy_t = \n" << ddy_t << endl;
      MatrixXd W = tracking_data->GetWeight();
      // cout << "W = \n" << W << endl;
      MatrixXd J_t = tracking_data->GetJ();
      // cout << "J_t = \n" << J_t << endl;
      VectorXd JdotV_t = tracking_data->GetJdotTimesV();
      // cout << "JdotV_t = \n" << JdotV_t << endl;
      //TODO: later, if track_or_not = false, we can just update W
      // The tracking cost is
      // 0.5 * (J_*dv + JdotV - y_command)^T * W * (J_*dv + JdotV - y_command).
      // We ignore the constant term
      // 0.5 * (JdotV - y_command)^T * W * (JdotV - y_command),
      // since it doesn't change the result of QP.
      prog.AddQuadraticCost(J_t.transpose()* W * J_t,
                            J_t.transpose()* W * (JdotV_t - ddy_t),
                            dv);
    }
  }

  // Solve the QP
  const MathematicalProgramResult result = Solve(prog);
  SolutionResult solution_result = result.get_solution_result();
  if (print_tracking_info_) {
    cout << "\n" << to_string(solution_result) <<  endl;
  }

  // Extract solutions
  VectorXd u_sol = result.GetSolution(u);
  VectorXd lambda_c_sol = result.GetSolution(lambda_c);
  VectorXd lambda_h_sol = result.GetSolution(lambda_h);
  VectorXd dv_sol = result.GetSolution(dv);
  VectorXd epsilon_sol = result.GetSolution(epsilon);
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
    for (auto tracking_data : *tracking_data_vec_) {
      if (tracking_data->GetTrackOrNot()) {
        VectorXd ddy_t = tracking_data->GetCommandOutput();
        MatrixXd W = tracking_data->GetWeight();
        MatrixXd J_t = tracking_data->GetJ();
        VectorXd JdotV_t = tracking_data->GetJdotTimesV();
        //TODO: later, if track_or_not = false, we can just update W
        // Note that the following cost also include the constant term, so that
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
        //TODO: later, if track_or_not = false, we can just update W
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

  high_resolution_clock::time_point t1 = high_resolution_clock::now();


  // Read in current state and simulation time
  const OutputVector<double>* robot_output = (OutputVector<double>*)
      this->EvalVectorInput(context, state_port_);
  VectorXd q_w_spr = robot_output->GetPositions();
  if (is_quaternion_) {
    q_w_spr.segment(3, 4) = NormalizeQuaternion(q_w_spr.segment(3, 4));
  }
  // cout << "q_w_spr = " << q_w_spr.transpose() << endl;
  VectorXd v_w_spr = robot_output->GetVelocities();
  VectorXd x_w_spr(tree_w_spr_->get_num_positions() +
                   tree_w_spr_->get_num_velocities());
  x_w_spr << q_w_spr, v_w_spr;

  double timestamp = robot_output->get_timestamp();
  double current_time = static_cast<double>(timestamp);
  if (print_tracking_info_) {
    cout << "\n\ncurrent_time = " << current_time << endl;
  }

  // TODO(yminchen): currently construct the QP in every loop. Will modify this
  // once the code is working.
  // Set up the QP
  VectorXd x_wo_spr(n_q_ + n_v_);
  x_wo_spr << map_position_from_spring_to_no_spring_ * q_w_spr,
           map_velocity_from_spring_to_no_spring_ * v_w_spr;
  // cout << "x_w_spr = " << x_w_spr.transpose() << endl;
  // cout << "x_wo_spr = " << x_wo_spr.transpose() << endl;

  VectorXd u_sol(n_u_);
  if (used_with_finite_state_machine_) {
    // Read in finite state machine
    const BasicVector<double>* fsm_output = (BasicVector<double>*)
                                            this->EvalVectorInput(context, fsm_port_);
    VectorXd fsm_state = fsm_output->get_value();

    // Get discrete states
    const auto prev_event_time = context.get_discrete_state(
                                   prev_event_time_idx_).get_value();
    // cout << "prev_event_time = " << prev_event_time << endl;

    u_sol = SolveQp(x_w_spr, x_wo_spr,
                    context, current_time,
                    fsm_state(0), current_time - prev_event_time(0));
  } else {
    u_sol = SolveQp(x_w_spr, x_wo_spr,
                    context, current_time,
                    -1, -1);
  }

  // Assign the control input
  control->SetDataVector(u_sol);
  control->set_timestamp(robot_output->get_timestamp());


  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  auto duration = duration_cast<microseconds>( t2 - t1 ).count();
  cout << "it took " << duration / 1000.0 << " (ms).\n";

}

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib


