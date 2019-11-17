#include "systems/framework/controller_channel_sender.h"

namespace dairlib {

using std::string;
using drake::systems::Context;
using drake::systems::LeafSystem;

template <typename MessageType>
ControllerChannelSender<MessageType>::ControllerChannelSender(
    const string& channel_name)
    : channel_name_(channel_name) {
  this->DeclareAbstractOutputPort(&ControllerChannelSender::Output);
}

template <typename MessageType>
void ControllerChannelSender<MessageType>::Output(const Context<double>& context,
                                                  MessageType* msg) const {
  msg->channel = channel_name_;
}

template class ControllerChannelSender<dairlib::lcmt_controller_switch>;

}  // namespace dairlib
