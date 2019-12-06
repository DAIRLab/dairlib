#pragma once

#include <string>

#include "dairlib/lcmt_controller_switch.hpp"
#include "systems/framework/output_vector.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {

/// ControllerChannelSender is a single-output block that assigns a string
/// to the lcm message, MessageType.
/// The user can choose to publish the string with or without a delay. If no
/// delay, this class outputs the channel name all the time. In the case of
/// delay, the user has to provide a time delay t_delay. Let the time of the
/// simulator/robot be t_init when we started running the diagram containing
/// this class. And let t_current be the current diagram time. This class
/// outputs the channel name only when t_current >= t_init + t_delay. That is,
/// the current time has to be bigger than the initial diagram time plus the
/// delay.

template <typename MessageType>
class ControllerChannelSender : public drake::systems::LeafSystem<double> {
 public:
  explicit ControllerChannelSender(const std::string& channel_name,
                                   double t_delay = -1);

 private:
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void Output(const drake::systems::Context<double>& context,
              MessageType* output) const;
  std::string channel_name_;
  drake::systems::DiscreteStateIndex time_idx_;
  double t_delay_;
};

}  // namespace dairlib
