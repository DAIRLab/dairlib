#pragma once

#include <limits>

#include "systems/framework/output_vector.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

/// FiniteStateMachineEventTime outputs the latest switching time of a finite
/// state machine. If the user provides `fsm_states_of_interest` to the
/// constructor, FiniteStateMachineEventTime creates a second output port that
/// sends out the start time of the state that the user is interested in.
class FiniteStateMachineEventTime : public drake::systems::LeafSystem<double> {
 public:
  FiniteStateMachineEventTime(
      const drake::multibody::MultibodyPlant<double>& plant,
      std::vector<int> fsm_states_of_interest = {});

  const drake::systems::InputPort<double>& get_input_port_fsm() const {
    return this->get_input_port(fsm_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(robot_output_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_event_time() const {
    return this->get_output_port(start_time_port_);
  }
  const drake::systems::OutputPort<double>&
  get_output_port_event_time_of_interest() const {
    return this->get_output_port(start_time_of_interest_port_);
  }

 private:
  void AssignStartTimeOfCurrentState(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* current_state_start_time) const;
  void AssignStartTimeOfStateOfInterest(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* state_start_time) const;

  // Port index
  int fsm_port_;
  int robot_output_port_;
  int start_time_port_;
  int start_time_of_interest_port_;

  std::vector<int> fsm_states_of_interest_;

  // Internal states of this leafsystem
  // prev_fsm_state0_ is used for AssignStartTimeOfCurrentState()
  // prev_fsm_state1_ is used for AssignStartTimeOfStateOfInterest()
  mutable double prev_fsm_state0_ = -std::numeric_limits<double>::infinity();
  mutable double prev_fsm_state1_ = -std::numeric_limits<double>::infinity();
  mutable double prev_time_;
  mutable double prev_time_of_state_of_interest_;
};

}  // namespace systems
}  // namespace dairlib
