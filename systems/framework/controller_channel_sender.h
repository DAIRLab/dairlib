#pragma once

#include <string>

#include "dairlib/lcmt_controller_switch.hpp"
#include "systems/framework/output_vector.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {

/// ControllerChannelSender is a LeafSystem that assigns a string
/// to the lcm message MessageType, and outputs the lcm message.
///
/// The class is extended so that the output starts when
/// `TimeBasedFiniteStateMachine` switches to a new state. This requires that
/// the users provides two (or three) more arguments to the constructors besides
/// `channel_name`:
///   - n_state_switch, the number of state-switch after the start of the
///   simulator containing this diagram
///   - period, has to be the same as `duration_per_state` in
///   `TimeBasedFiniteStateMachine`
///   - (optional) fsm_offset, has to be the same as `time_shift` in
///   `TimeBasedFiniteStateMachine`
///
/// Let t_init be the time of the simulator/robot when we started running the
/// diagram containing this class. And let t_current be the current diagram
/// time. This class outputs the channel name only when
///   t_current >= (floor(t_init/period) + n_state_switch) * period +
///   fsm_offset.
/// That is, `ControllerChannelSender` starts outputting the new channel name
/// when `TimeBasedFiniteStateMachine` switches to a new state for the
/// `n_state_switch`-th times after we start running the simulator that contains
/// `ControllerChannelSender`.

template <typename MessageType>
class ControllerChannelSender : public drake::systems::LeafSystem<double> {
 public:
  explicit ControllerChannelSender(const std::string& channel_name,
                                   int n_state_switch = -1, double period = -1,
                                   double fsm_offset = 0);

 private:
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void Output(const drake::systems::Context<double>& context,
              MessageType* output) const;
  std::string channel_name_;
  drake::systems::DiscreteStateIndex time_idx_;
  int n_state_switch_;
  double period_;
  double fsm_offset_;
};

}  // namespace dairlib
