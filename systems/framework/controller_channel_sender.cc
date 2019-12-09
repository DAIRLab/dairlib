#include "systems/framework/controller_channel_sender.h"
#include <math.h>

namespace dairlib {

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::systems::LeafSystem;
using Eigen::VectorXd;
using std::string;
using systems::OutputVector;

template <typename SwitchMessageType>
ControllerChannelSender<SwitchMessageType>::ControllerChannelSender(
    const string& channel_name, int n_state_switch, double period,
    double fsm_offset)
    : channel_name_(channel_name),
      n_state_switch_(n_state_switch),
      period_(period),
      fsm_offset_(fsm_offset) {
  // Ensure that if (n_state_switch >= 0), then (period > 0).
  DRAKE_DEMAND((n_state_switch < 0) || (period > 0));
  // offset has to be positive
  DRAKE_DEMAND(fsm_offset >= 0);

  // Create output
  this->DeclareAbstractOutputPort(&ControllerChannelSender::Output);

  if (n_state_switch >= 0) {
    // Create per-step update
    DeclarePerStepDiscreteUpdateEvent(
        &ControllerChannelSender<SwitchMessageType>::DiscreteVariableUpdate);
    // A state to store initial time
    time_idx_ = DeclareDiscreteState(-VectorXd::Ones(1));
  }
}

template <typename SwitchMessageType>
EventStatus ControllerChannelSender<SwitchMessageType>::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  // Update the initial time if it has not been initialized
  if (discrete_state->get_vector(time_idx_).get_value()(0) < 0) {
    discrete_state->get_mutable_vector(time_idx_).get_mutable_value()
        << context.get_time();
  }
  return EventStatus::Succeeded();
}

template <typename SwitchMessageType>
void ControllerChannelSender<SwitchMessageType>::Output(
    const Context<double>& context, SwitchMessageType* msg) const {
  auto t_init = context.get_discrete_state(time_idx_).get_value();

  // If n_state_switch is not set (that is, the user is not using the
  // delay-publish feature), then we publish the switch channel name all the
  // time.
  if (n_state_switch_ < 0) {
    msg->channel = channel_name_;
  } else {
    // If the t_init is initialized and the current time is bigger than
    // (floor(t_init/period) + n_state_switch) * period + fsm_offset,
    // then we publish the switch channel name.
    if ((t_init(0) >= 0) &&
        (context.get_time() >=
         (floor(t_init(0) / period_) + n_state_switch_) * period_ +
             fsm_offset_)) {
      msg->channel = channel_name_;
    } else {
      msg->channel = "";
    }
  }
}

template class ControllerChannelSender<dairlib::lcmt_controller_switch>;

}  // namespace dairlib
