#include "examples/PlanarWalker/state_based_fsm.h"

using Eigen::VectorXd;

using drake::systems::Context;
using dairlib::systems::OutputVector;
using dairlib::systems::BasicVector;

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

  this->DeclareVectorOutputPort(systems::BasicVector<double>(1),
                                &StateBasedFiniteStateMachine::CalcFiniteState);

  std::cout << "State based FSM is being defined!" << std::endl;
}

void StateBasedFiniteStateMachine::CalcFiniteState(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* fsm_state) const {
  std::cout << "From StateBasedFiniteStateMachine: " << std::endl;
  // const OutputVector<double>* robot_output =
  //     (OutputVector<double>*)this->EvalVectorInput(context, state_port_);

  Eigen::VectorXd current_finite_state(1);
  current_finite_state << initial_state_number_;
  std::cout << "Current state: " << current_finite_state << std::endl;

  fsm_state->get_mutable_value() = current_finite_state;
  std::cout << "End of StateBasedFiniteStateMachine" << std::endl;
}

}  // namespace dairlib
