#include "examples/PlanarWalker/state_based_fsm.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

using Eigen::VectorXd;

using drake::systems::Context;
using dairlib::systems::OutputVector;
using dairlib::systems::BasicVector;
using drake::systems::EventStatus;
using drake::systems::DiscreteValues;

using drake::trajectories::Trajectory;
using drake::trajectories::PiecewisePolynomial;

namespace dairlib {
StateBasedFiniteStateMachine::StateBasedFiniteStateMachine(
    const RigidBodyTree<double>& tree, double left_foot_idx,
    Eigen::Vector3d pt_on_left_foot, double right_foot_idx,
    Eigen::Vector3d pt_on_right_foot, double time_shift)
    : tree_(tree),
      left_foot_idx_(left_foot_idx),
      pt_on_left_foot_(pt_on_left_foot),
      right_foot_idx_(right_foot_idx),
      pt_on_right_foot_(pt_on_right_foot) {
  state_port_ = this
                    ->DeclareVectorInputPort(systems::OutputVector<double>(
                        tree_.get_num_positions(), tree_.get_num_velocities(),
                        tree_.get_num_actuators()))
                    .get_index();

  PiecewisePolynomial<double> pp = PiecewisePolynomial<double>();
  trajectory_port_ = this->DeclareAbstractInputPort(
                             "swing_traj", drake::Value<Trajectory<double>>(pp))
                         .get_index();

  this->DeclareVectorOutputPort(systems::BasicVector<double>(1),
                                &StateBasedFiniteStateMachine::CopyFiniteState,
                                {this->all_state_ticket()});

  this->DeclarePerStepDiscreteUpdateEvent(
      &StateBasedFiniteStateMachine::DiscreteVariableUpdate);

  fsm_state_index_ = this->DeclareDiscreteState(1);
  prev_fsm_change_time_idx_ = this->DeclareDiscreteState(1);

  std::cout << "State based FSM is defined!" << std::endl;
}

EventStatus StateBasedFiniteStateMachine::DiscreteVariableUpdate(
    const drake::systems::Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);

  const drake::AbstractValue* traj_input =
      this->EvalAbstractInput(context, trajectory_port_);
  const drake::trajectories::Trajectory<double>& traj =
      traj_input->get_value<drake::trajectories::Trajectory<double>>();

  Eigen::VectorXd current_finite_state(1);
  current_finite_state << discrete_state->get_mutable_vector(fsm_state_index_)
                              .get_mutable_value();

  const double prev_change_time =
      discrete_state->get_vector(prev_fsm_change_time_idx_).get_value()(0);

  // Currently this class is a time-based FSM
  double timestamp = robot_output->get_timestamp();
  double current_time = static_cast<double>(timestamp);

  // if(int((current_time - fmod(current_time, 0.5))/0.5) % 2 == 0) {
  //   current_finite_state << left_stance_state_;
  // } else {
  //   current_finite_state << right_stance_state_;
  // }
  std::cout << "PP end time: " << std::endl;
  std::cout << traj.end_time() << std::endl;
  if (current_time >= traj.end_time() - 0.002 &&
      current_time >= 0.002 + prev_change_time) {
    std::cout << "Inside the loop" << std::endl;
    std::cout << "Current finite state is: " << std::endl;
    std::cout << current_finite_state(0) << std::endl;
    current_finite_state(0) = (current_finite_state(0) == right_stance_state_)
                                  ? left_stance_state_
                                  : right_stance_state_;
    std::cout << "Next finite state would be:" << std::endl;
    std::cout << current_finite_state(0) << std::endl;

    discrete_state->get_mutable_vector(prev_fsm_change_time_idx_)
                              .get_mutable_value() << current_time;
  }

  discrete_state->get_mutable_vector(fsm_state_index_).get_mutable_value()
      << current_finite_state;
  std::cout << "End of discrete update in the FSM block!" << std::endl;
  return EventStatus::Succeeded();
}

void StateBasedFiniteStateMachine::CopyFiniteState(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {

  // Assign
  output->get_mutable_value()
      << context.get_discrete_state(fsm_state_index_).get_value()(0);
}

}  // namespace dairlib
