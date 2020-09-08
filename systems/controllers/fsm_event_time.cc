#include "systems/controllers/fsm_event_time.h"

#include <limits>

using std::cout;
using std::endl;
using std::string;

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;

namespace dairlib {
namespace systems {

FiniteStateMachineEventTime::FiniteStateMachineEventTime(
    std::vector<int> fsm_states_of_interest)
    : fsm_states_of_interest_(fsm_states_of_interest) {
  this->set_name("fsm_event_time");

  // Input/Output Setup
  fsm_port_ = this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();
  start_time_port_ =
      this->DeclareVectorOutputPort(
              BasicVector<double>(1),
              &FiniteStateMachineEventTime::AssignStartTimeOfCurrentState)
          .get_index();
  if (!fsm_states_of_interest.empty()) {
    start_time_of_interest_port_ =
        this->DeclareVectorOutputPort(
                BasicVector<double>(1),
                &FiniteStateMachineEventTime::AssignStartTimeOfStateOfInterest)
            .get_index();
  }

  // Per-step update to record the previous state and the previous event time
  DeclarePerStepDiscreteUpdateEvent(
      &FiniteStateMachineEventTime::DiscreteVariableUpdate);
  // The start time of the current fsm state
  prev_time_idx_ = this->DeclareDiscreteState(1);
  // The start time of the most recent fsm state that we are interested
  prev_time_of_state_of_interest_idx_ = this->DeclareDiscreteState(1);
  // The last state of FSM
  prev_fsm_state_idx_ = this->DeclareDiscreteState(
      -std::numeric_limits<double>::infinity() * VectorXd::Ones(1));
}

EventStatus FiniteStateMachineEventTime::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  // Read in current finite state machine state
  auto fsm_output =
      (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
  VectorXd fsm_state = fsm_output->get_value();
  // Read in previous finite state machine state
  auto prev_fsm_state = discrete_state->get_mutable_vector(prev_fsm_state_idx_)
                            .get_mutable_value();

  // Check if the current state is of our interest
  auto it = find(fsm_states_of_interest_.begin(), fsm_states_of_interest_.end(),
                 int(fsm_state(0)));
  bool is_state_of_interest = it != fsm_states_of_interest_.end();

  // when entering a new state which is in fsm_states_of_interest
  if (fsm_state(0) != prev_fsm_state(0)) {
    prev_fsm_state(0) = fsm_state(0);

    // Record time
    discrete_state->get_mutable_vector(prev_time_idx_)
        .get_mutable_value() << context.get_time();

    if (is_state_of_interest) {
      // Record time
      discrete_state->get_mutable_vector(prev_time_of_state_of_interest_idx_)
          .get_mutable_value() << context.get_time();
    }
  }

  return EventStatus::Succeeded();
}

void FiniteStateMachineEventTime::AssignStartTimeOfCurrentState(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* current_state_start_time) const {
  current_state_start_time->get_mutable_value() =
      context.get_discrete_state(prev_time_idx_).get_value();
}
void FiniteStateMachineEventTime::AssignStartTimeOfStateOfInterest(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* state_start_time) const {
  state_start_time->get_mutable_value() =
      context.get_discrete_state(prev_time_of_state_of_interest_idx_)
          .get_value();
}

}  // namespace systems
}  // namespace dairlib
