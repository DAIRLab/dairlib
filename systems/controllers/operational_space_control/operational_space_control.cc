#include "systems/controllers/operational_space_control/operational_space_control.h"

using std::vector;
using std::string;
using Eigen::VectorXd;
using Eigen::MatrixXd;

using drake::systems::Context;
using drake::systems::BasicVector;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::ExponentialPlusPiecewisePolynomial;

using drake::solvers::MathematicalProgram;
using drake::solvers::Solve;

namespace dairlib {
namespace systems {
namespace controllers {


OperationalSpaceControl::OperationalSpaceControl(
  RigidBodyTree<double>* tree_w_sprs,
  RigidBodyTree<double>* tree_wo_spr) :
  tree_w_sprs_(tree_w_sprs),
  tree_wo_spr_(tree_wo_spr) {
  this->set_name("OSC");

  n_q_ = tree_w_sprs->get_num_positions();
  n_v_ = tree_w_sprs->get_num_velocities();
  n_u_ = tree_w_sprs->get_num_actuators();

  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(
                  OutputVector<double>(n_q_, n_v_, n_u_)).get_index();
  fsm_port_ = this->DeclareVectorInputPort(
                BasicVector<double>(1)).get_index();
  this->DeclareVectorOutputPort(TimestampedVector<double>(n_u_),
                                &OperationalSpaceControl::CalcOptimalInput);

  // Discrete update to record the last state event time
  DeclarePerStepDiscreteUpdateEvent(
    &OperationalSpaceControl::DiscreteVariableUpdate);
  prev_fsm_state_idx_ = this->DeclareDiscreteState(1);
  prev_event_time_idx_ = this->DeclareDiscreteState(VectorXd::Zero(1));
}

// Cost methods
void OperationalSpaceControl::AddAccelerationCost(int joint_vel_idx, double w) {
  if (W_joint_accel_.size() == 0) {
    W_joint_accel_ = Eigen::MatrixXd::Zero(n_v_, n_v_);
  }
  W_joint_accel_(joint_vel_idx, joint_vel_idx) = w;
}

// Constraint methods
void OperationalSpaceControl::AddContactPoint(int body_index,
    Eigen::VectorXd pt_on_body) {
  body_index_.push_back(body_index);
  pt_on_body_.push_back(pt_on_body);
}
void OperationalSpaceControl::AddContactPoints(std::vector<int> body_index,
    std::vector<Eigen::VectorXd> pt_on_body) {
  body_index_.insert(body_index_.end(), body_index.begin(), body_index.end());
  pt_on_body_.insert(pt_on_body_.end(), pt_on_body.begin(), pt_on_body.end());
}

// Osc checkers and constructor
void OperationalSpaceControl::CheckCostSettings() {
  if (W_input_.size() != 0) {
    DRAKE_DEMAND((W_input_.rows() == n_u_) && (W_input_.cols() == n_u_));
  }
  if (W_joint_accel_.size() != 0) {
    DRAKE_DEMAND((W_joint_accel_.rows() == n_u_) &&
                 (W_joint_accel_.cols() == n_u_));
  }
}
void OperationalSpaceControl::CheckConstraintSettings() {
  if (!body_index_.empty()) {
    DRAKE_DEMAND(mu_ != -1);
    DRAKE_DEMAND(body_index_.size() == pt_on_body_.size());
  }
}

void OperationalSpaceControl::ConstructOSC() {
  // Checker
  CheckCostSettings();
  CheckConstraintSettings();

  // Construct traj_name_to_port_index_map_
  for (auto tracking_data : *tracking_data_vec_) {
    string traj_name = tracking_data->GetName();
    int port_index;
    if (tracking_data->DoesTrajHasExp()) {
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

drake::systems::EventStatus OperationalSpaceControl::DiscreteVariableUpdate(
  const drake::systems::Context<double>& context,
  drake::systems::DiscreteValues<double>* discrete_state) const {
  const TimestampedVector<double>* fsm_output = (TimestampedVector<double>*)
      this->EvalVectorInput(context, fsm_port_);
  VectorXd fsm_state = fsm_output->get_data();
  double tiemstamp = fsm_output->get_timestamp();

  auto prev_fsm_state = discrete_state->get_mutable_vector(
                          prev_fsm_state_idx_).get_mutable_value();

  if (fsm_state(0) != prev_fsm_state(0)) {
    prev_fsm_state(0) = fsm_state(0);

    discrete_state->get_mutable_vector(
      prev_event_time_idx_).get_mutable_value() << tiemstamp;
  }
  return drake::systems::EventStatus::Succeeded();
}


// TODO(yminchen): currently construct the QP in every loop. Will modify this
// once the code is working.
MathematicalProgram OperationalSpaceControl::SetUpQp(VectorXd q_w_spr,
    VectorXd v_w_spr, VectorXd q_wo_spr, VectorXd v_wo_spr) const {

  // Get data needed
  int n_h = tree_wo_spr_->getNumPositionConstraints();
  int n_c = body_index_.size();

  // Get Kinematics Cache
  KinematicsCache<double> cache_w_spr = tree_w_spr_->doKinematics(
                                          q_w_spr, v_w_spr);
  KinematicsCache<double> cache_wo_spr = tree_wo_spr_->doKinematics(
      q_wo_spr, v_wo_spr);

  // Get M, f_cg, B matrices of the manipulator equation
  MatrixXd B = tree_wo_spr_->B;
  MatrixXd M = tree_wo_spr_->massMatrix(cache_wo_spr);
  // std::cout<<"Matrix M dimension = "<< M.rows()<<" "<< M.cols()<<"\n\n";
  // std::cout<< "M.row(18) = " << M.row(18) << std::endl;
  const RigidBodyTree<double>::BodyToWrenchMap no_external_wrenches;
  VectorXd bias = tree_wo_spr_->dynamicsBiasTerm(cache_wo_spr,
                  no_external_wrenches);

  // Construct QP
  MathematicalProgram prog;

  // Add decision variables
  auto u = prog.NewContinuousVariables(n_u_, "u");
  auto lambda_c = prog.NewContinuousVariables(3 * n_c,
                  "lambda_contact");
  auto lambda_h = prog.NewContinuousVariables(n_h, "lambda_holonomic");
  auto dv = prog.NewContinuousVariables(n_v_, "dv");
  auto epsilon = prog.NewContinuousVariables(3 * body_index_.size(), "epsilon");

  // Add constraints
  // 1. Dynamics constraint
  //    M*dv + bias == J_c^T*lambda_c + J_h^T*lambda_fourbar + B*u
  // -> M*dv - J_c^T*lambda_c - J_h^T*lambda_fourbar - B*u == - bias
  // -> [M, -J_c^T, -J_h^T, -B]*[dv, lambda_c, lambda_fourbar, u]^T = - bias
  // std::cout<< "size of (M, -J_c.transpose(), -J_h.transpose(), -B):\n"
  //       << M.rows() << ", " << M.cols() << "\n"
  //       << J_c.transpose().rows() << ", " << J_c.transpose().cols() << "\n"
  //       << J_h.transpose().rows() << ", " << J_h.transpose().cols() << "\n"
  //       << B.rows() << ", " << B.cols() << "\n";
  MatrixXd A_dyn = MatrixXd::Zero(n_v_, n_v_ + 3 * n_c + J_h.rows() + n_u_);
  A_dyn.block(0, 0, n_v_, n_v_) = M;
  A_dyn.block(0, n_v_, n_v_, 3 * n_c) = -J_c.transpose();
  A_dyn.block(0, n_v_ + 3 * n_c , n_v_, J_h.rows()) = -J_h.transpose();
  A_dyn.block(0, n_v_ + 3 * n_c + J_h.rows(), n_v_, n_u_) = -B;
  // std::cout<< "M # of rows and cols = " <<M.rows()<<", "<<M.cols()<<std::endl;
  // std::cout<< "J_c.transpose() # of rows and cols = " <<J_c.transpose().rows()<<", "<<J_c.transpose().cols()<<std::endl;
  // std::cout<< "J_h.transpose() # of rows and cols = " <<J_h.transpose().rows()<<", "<<J_h.transpose().cols()<<std::endl;
  // std::cout<< "B # of rows and cols = " <<B.rows()<<", "<<B.cols()<<std::endl;
  // std::cout<<"A_dyn = \n"<<A_dyn<<"\n";
  prog.AddLinearConstraint(A_dyn, -bias, -bias, {dv, lambda_c, lambda_fourbar, u});

  // 2. Holonomic constraint
  //    JDotTimesV_h + J_h*dv == 0
  // -> J_h*dv == -JDotTimesV_h
  prog.AddLinearConstraint(J_h,
                           -JDotTimesV_h,
                           -JDotTimesV_h,
                           dv);
  // std::cout<< "J_h # of rows and cols = " <<J_h.rows()<<", "<<J_h.cols()<<std::endl;
  // std::cout<< "JDotTimesV_h # of rows and cols = " <<JDotTimesV_h.rows()<<", "<<JDotTimesV_h.cols()<<std::endl;


    // 3. Contact constraint
      if (body_index_.size() > 0) {
    if (w_soft_constraint_ == -1) {
        //    JDotTimesV_contact_constraint + J_contact_constraint*dv == 0
        // -> J_contact_constraint*dv == -JDotTimesV_contact_constraint
        prog.AddLinearConstraint( J_contact_constraint,
                                  -JDotTimesV_contact_constraint,
                                  -JDotTimesV_contact_constraint,
                                  dv);
        // std::cout<< "J_contact_constraint # of rows and cols = " <<J_contact_constraint.rows()<<", "<<J_contact_constraint.cols()<<std::endl;
        // std::cout<< "JDotTimesV_contact_constraint # of rows and cols = " <<JDotTimesV_contact_constraint.rows()<<", "<<JDotTimesV_contact_constraint.cols()<<std::endl;
    }
    else {
      // Relaxed version:
      //    JDotTimesV_contact_constraint + J_contact_constraint*dv == -epsilon
      // -> J_contact_constraint*dv + I*epsilon == -JDotTimesV_contact_constraint
      // -> [J_contact_constraint, I]* [dv, epsilon]^T == -JDotTimesV_contact_constraint
      MatrixXd A_StFtConstraint = MatrixXd::Zero(3 * num_collision * num_stance_foot, n_v_ + 3 * num_collision * num_stance_foot);
      A_StFtConstraint.block(0, 0, 3 * num_collision*num_stance_foot, n_v_) = J_contact_constraint;
      A_StFtConstraint.block(0, n_v_, 3 * num_collision*num_stance_foot, 3 * num_collision*num_stance_foot) = MatrixXd::Identity(3 * num_collision * num_stance_foot, 3 * num_collision * num_stance_foot);
      // std::cout<<"A_StFtConstraint: \n"<<A_StFtConstraint<<"\n";
      prog.AddLinearConstraint( A_StFtConstraint,
                                -JDotTimesV_contact_constraint,
                                -JDotTimesV_contact_constraint,
      {dv, epsilon});
}
      }


  return prog;
}


void OperationalSpaceControl::CalcOptimalInput(
  const drake::systems::Context<double>& context,
  systems::TimestampedVector<double>* control) const {
  // Read in current state and simulation time
  const OutputVector<double>* robot_output = (OutputVector<double>*)
      this->EvalVectorInput(context, state_port_);
  VectorXd current_state = robot_output->GetState();
  double timestamp = robot_output->get_timestamp();
  double current_sim_time = static_cast<double>(timestamp);

  // TODO(yminchen): currently construct the QP in every loop. Will modify this
  // once the code is working.

  // Set up the QP
  // MathematicalProgram prog = SetUpQp();



  VectorXd u;
  // Assign the control input
  control->SetDataVector(u);
  control->set_timestamp(robot_output->get_timestamp());
}

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib


