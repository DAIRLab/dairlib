#pragma once

#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

/// FiniteStateMachineEventTime outputs the latest switching time of a finite
/// state machine. If the user provides `fsm_states_of_interest` to the
/// constructor, FiniteStateMachineEventTime creates a second output port that
/// sends out the start time of the state that the user is interested in.
class FiniteStateMachineEventTime : public drake::systems::LeafSystem<double> {
 public:
  FiniteStateMachineEventTime(std::vector<int> fsm_states_of_interest = {});

  const drake::systems::InputPort<double>& get_input_port_fsm() const {
    return this->get_input_port(fsm_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_event_time() const {
    return this->get_output_port(start_time_port_);
  }
  const drake::systems::OutputPort<double>&
  get_output_port_event_time_of_interest() const {
    return this->get_output_port(start_time_of_interest_port_);
  }

 private:
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;
  void AssignStartTimeOfCurrentState(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* current_state_start_time) const;
  void AssignStartTimeOfStateOfInterest(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* state_start_time) const;

  // Port index
  int fsm_port_;
  int start_time_port_;
  int start_time_of_interest_port_;

  // Discrete state index
  int prev_fsm_state_idx_;
  int prev_time_idx_;
  int prev_time_of_state_of_interest_idx_;

  std::vector<int> fsm_states_of_interest_;
};

}  // namespace systems
}  // namespace dairlib
