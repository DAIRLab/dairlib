#include "examples/Cassie/controller_channel_sender.h"

namespace dairlib {

using std::string;
using drake::systems::Context;
using drake::systems::LeafSystem;

ControllerChannelSender::ControllerChannelSender(const string& channel_name)
    : channel_name_(channel_name) {
  this->DeclareAbstractOutputPort(&ControllerChannelSender::Output);
}

void ControllerChannelSender::Output(const Context<double>& context,
                                     dairlib::lcmt_controller_switch* msg) const {
  msg->channel = channel_name_;
}

}  // namespace dairlib
