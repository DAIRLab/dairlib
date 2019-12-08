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

template <typename MessageType>
ControllerChannelSender<MessageType>::ControllerChannelSender(
    const string& channel_name, int n_fsm_period, double period,
    double fsm_offset)
    : channel_name_(channel_name),
      n_fsm_period_(n_fsm_period),
      period_(period),
      fsm_offset_(fsm_offset) {
  // Ensure that if (n_fsm_period >= 0), then (period > 0).
  DRAKE_DEMAND((n_fsm_period < 0) || (period > 0));
  // offset has to be positive
  DRAKE_DEMAND(fsm_offset >= 0);

  // Create output
  this->DeclareAbstractOutputPort(&ControllerChannelSender::Output);

  if (n_fsm_period >= 0) {
    // Create per-step update
    DeclarePerStepDiscreteUpdateEvent(
        &ControllerChannelSender<MessageType>::DiscreteVariableUpdate);
    // A state to store initial time
    time_idx_ = DeclareDiscreteState(-VectorXd::Ones(1));
  }
}

template <typename MessageType>
EventStatus ControllerChannelSender<MessageType>::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  // Update the initial time if it has not been initialized
  if (discrete_state->get_vector(time_idx_).get_value()(0) < 0) {
    discrete_state->get_mutable_vector(time_idx_).get_mutable_value()
        << context.get_time();
  }
  return EventStatus::Succeeded();
}

template <typename MessageType>
void ControllerChannelSender<MessageType>::Output(
    const Context<double>& context, MessageType* msg) const {
  auto t_init = context.get_discrete_state(time_idx_).get_value();

  // If n_fsm_period is not set (that is, the user is not using the
  // delay-publish feature), then we publish the switch channel name all the
  // time.
  if (n_fsm_period_ < 0) {
    msg->channel = channel_name_;
  } else {
    // If the t_init is initialized and the current time is bigger than
    // (floor(t_init/period) + n_fsm_period) * period + fsm_offset,
    // then we publish the switch channel name.
    if ((t_init(0) >= 0) &&
        (context.get_time() >=
         (floor(t_init(0) / period_) + n_fsm_period_) * period_ +
             fsm_offset_)) {
      msg->channel = channel_name_;
    } else {
      msg->channel = "";
    }
  }
}

template class ControllerChannelSender<dairlib::lcmt_controller_switch>;

}  // namespace dairlib
