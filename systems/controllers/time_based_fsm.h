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

 protected:
  drake::systems::InputPortIndex state_port_;
  drake::systems::OutputPortIndex fsm_port_;

 private:
  void CalcFiniteState(const drake::systems::Context<double>& context,
                       drake::systems::BasicVector<double>* fsm_state) const;

  std::vector<int> states_;
  int initial_state_ = -1;
  double t0_;

  std::vector<double> accu_state_durations_;
  double period_;

  double eps_ = 1e-12;
};


class TimeBasedFiniteStateMachineWithTrigger :
    public drake::systems::LeafSystem<double> {
 public:
  TimeBasedFiniteStateMachineWithTrigger(
      const drake::multibody::MultibodyPlant<double>& plant,
      const std::vector<int>& states,
      const std::vector<double>& state_durations,
      bool with_trigger_input_port, double one_stride_period);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_trigger() const {
    return this->get_input_port(trigger_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_fsm() const {
    return this->get_output_port(fsm_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_global_fsm_idx()
  const {
    return this->get_output_port(global_fsm_idx_port_);
  }

 private:
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;
  void CalcFiniteState(const drake::systems::Context<double>& context,
                       drake::systems::BasicVector<double>* fsm_state) const;
  void CalcGlobalFsmIdx(const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* global_fsm_idx) const;

  int state_port_;
  int trigger_port_;

  int fsm_port_;
  int global_fsm_idx_port_;

  mutable bool trigged_ = false;
  mutable double t0_ = -1;
  double one_stride_period_;

  std::vector<int> states_;
  std::vector<double> accu_state_durations_;
  double period_;

  double eps_ = 1e-12;
};


}  // namespace systems
}  // namespace dairlib
