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
  RigidBodyTree<double>* tree_with_springs,
  RigidBodyTree<double>* tree_without_springs) :
  tree_with_springs_(tree_with_springs),
  tree_without_springs_(tree_without_springs) {
  this->set_name("OSC");

  n_q_ = tree_with_springs->get_num_positions();
  n_v_ = tree_with_springs->get_num_velocities();
  n_u_ = tree_with_springs->get_num_actuators();

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


// MathematicalProgram OperationalSpaceControl::SetUpQp() const {
//   MathematicalProgram prog;

//   // Add decision variables
//   auto u = prog.NewContinuousVariables(n_u_, "u");
//   auto lambda_contact = prog.NewContinuousVariables(3 * num_collision, "lambda_contact");
//   auto lambda_fourbar = prog.NewContinuousVariables(J_fourbar.rows(), "lambda_fourbar");
//   auto dv = prog.NewContinuousVariables(n_v_, "dv");
//   auto epsilon = prog.NewContinuousVariables(3 * num_collision, "epsilon");

//   return prog;
// }

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


