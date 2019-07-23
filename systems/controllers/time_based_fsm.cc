#include "systems/controllers/time_based_fsm.h"

using std::cout;
using std::endl;

using std::string;
using Eigen::VectorXd;
using drake::systems::Context;
using drake::systems::BasicVector;

namespace dairlib {
namespace systems {

TimeBasedFiniteStateMachine::TimeBasedFiniteStateMachine(
    const RigidBodyTree<double>& tree,
    int first_state_number,
    int second_state_number,
    int initial_state_number,
    double duration_per_state,
    double time_shift) :
      first_state_number_(first_state_number),
      second_state_number_(second_state_number),
      initial_state_number_(initial_state_number),
      duration_per_state_(duration_per_state),
      time_shift_(time_shift) {
  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(
                  OutputVector<double>(tree.get_num_positions(),
                                       tree.get_num_velocities(),
                                       tree.get_num_actuators())).get_index();
  this->DeclareVectorOutputPort(BasicVector<double>(1),
                                &TimeBasedFiniteStateMachine::CalcFiniteState);
}

void TimeBasedFiniteStateMachine::CalcFiniteState(
    const Context<double>& context,
    BasicVector<double>* fsm_state) const {
  // Read in current state and simulation time
  const OutputVector<double>* robot_output = (OutputVector<double>*)
      this->EvalVectorInput(context, state_port_);
  VectorXd currentState = robot_output->GetState();
  double timestamp = robot_output->get_timestamp();
  double current_sim_time = static_cast<double>(timestamp);

  const double period = duration_per_state_ * 2;

  double m = floor((current_sim_time - time_shift_) / period);
  double remainder = (current_sim_time - time_shift_) - m * period;
  double phase = remainder / duration_per_state_;  // within [0,2]

  // Get current finite tate
  VectorXd current_finite_state(1);
  current_finite_state << initial_state_number_;
  if (current_sim_time >= time_shift_) {
    if (phase < 1)
      current_finite_state(0) = first_state_number_;
    else if (phase < 2)
      current_finite_state(0) = second_state_number_;
  }

  // Assign fsm_state
  fsm_state->get_mutable_value() = current_finite_state;
}

}  // namespace systems
}  // namespace dairlib


