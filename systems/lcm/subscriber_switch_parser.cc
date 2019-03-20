#include "systems/lcm/subscriber_switch_parser.h"

namespace dairlib {
namespace systems {

using drake::systems::Context;

SubscriberSwitchParser::SubscriberSwitchParser() {
  this->DeclareAbstractInputPort("lcmt_subscriber_switch",
    drake::Value<dairlib::lcmt_subscriber_switch>{});
  this->DeclareAbstractOutputPort(&SubscriberSwitchParser::CopyOutput);
}

void SubscriberSwitchParser::CopyOutput(const Context<double>& context,
    std::string* output) const {
  const auto& msg = this->EvalAbstractInput(context, 0)->
      get_value<dairlib::lcmt_subscriber_switch>();
  *output = msg.channel;
}

}  // namespace systems
}  // namespace dairlib
