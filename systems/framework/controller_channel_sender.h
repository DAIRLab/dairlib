#pragma once

#include <string>

#include "dairlib/lcmt_controller_switch.hpp"
#include "systems/framework/output_vector.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {

/// ControllerChannelSender is a single-output block that assigns a string
/// to the lcm message, MessageType.
/// The user can choose to publish the string with or without a delay. The delay
/// is added so that we can sync the publish with the TIME-BASED finite state
/// machine in the controller.
/// If no delay, this class outputs the channel name all the time.
/// In the case of delay, the user has to provide
///   - n_fsm_period, the number of period with which the user wants to delay
///   the publish
///   - period, the period of the time-based fsm
///   - (optional) fsm_offset, which is the offset of the time-based fsm.
/// Let t_init be the time of the simulator/robot when we started running the
/// diagram containing this class. And let t_current be the current diagram
/// time. This class outputs the channel name only when
///   t_current >= (floor(t_init/period) + n_fsm_period) * period + fsm_offset.
/// That is, the current time has to be bigger than the initial diagram
/// time plus the delay.

template <typename MessageType>
class ControllerChannelSender : public drake::systems::LeafSystem<double> {
 public:
  explicit ControllerChannelSender(const std::string& channel_name,
                                   int n_fsm_period = -1,
                                   double period = -1,
                                   double fsm_offset = 0);

 private:
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void Output(const drake::systems::Context<double>& context,
              MessageType* output) const;
  std::string channel_name_;
  drake::systems::DiscreteStateIndex time_idx_;
  int n_fsm_period_;
  double period_;
  double fsm_offset_;
};

}  // namespace dairlib
