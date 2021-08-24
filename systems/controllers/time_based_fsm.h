#pragma once

#include <string>
#include "systems/framework/output_vector.h"

#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

// Time-based Finite State Machine
/// TimeBasedFiniteStateMachine is a time-triggered finite state machine. Given
/// n number of states with n number of durations, TimeBasedFiniteStateMachine
/// goes through each state in order, and the time it stays in each state for is
/// specified by the corresponding duration.

/// Constructor:
///  @param plant, multibody plant
///  @param states, integer representation of each state
///  @param state_durations, duration of each state
///  @param t0, time offset of the whole finite state machine

/// Assigment of the state:
/// If t < t0,
///     state = `init_state_`.
/// else
///     If t is within [`t0`,
///                     `t0` + state_durations[0])
///         state = states[0].
///     else if t is within [`t0`, + state_durations[0]
///                          `t0` + state_durations[0] + + state_durations[1])
///         state = states[1]
///     else if ... (recursively)
///
class TimeBasedFiniteStateMachine : public drake::systems::LeafSystem<double> {
 public:
  TimeBasedFiniteStateMachine(
      const drake::multibody::MultibodyPlant<double>& plant,
      const std::vector<int>& states,
      const std::vector<double>& state_durations, double t0 = 0);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_fsm() const {
    return this->get_output_port(fsm_port_);
  }

 private:
  void CalcFiniteState(const drake::systems::Context<double>& context,
                       drake::systems::BasicVector<double>* fsm_state) const;

  int state_port_;
  int fsm_port_;

  std::vector<int> states_;
  std::vector<double> state_durations_;
  int initial_state_ = -1;
  double t0_;

  std::vector<double> accu_state_durations_;
  double period_;
};

}  // namespace systems
}  // namespace dairlib
