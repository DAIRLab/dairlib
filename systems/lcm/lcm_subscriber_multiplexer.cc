#include "systems/lcm/lcm_subscriber_multiplexer.h"

namespace dairlib {
namespace systems {
namespace lcm {

using std::vector;
using std::unique_ptr;
using std::string;
using drake::AbstractValue;
using drake::Value;
using drake::lcm::DrakeLcmInterface;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::InputPort;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::SerializerInterface;
using drake::systems::UnrestrictedUpdateEvent;
using drake::systems::CompositeEventCollection;
using drake::systems::State;

namespace {
const int kChannelSelectorPortIndex = 0;
}  // namespace

LcmSubscriberMultiplexer::LcmSubscriberMultiplexer(
    const vector<string>& channels,
    vector<LcmSubscriberSystem*> lcm_subscribers,
    unique_ptr<SerializerInterface> serializer,
    string default_channel) :
    serializer_(std::move(serializer)),
    subscribers_(lcm_subscribers),
    default_channel_(default_channel) {
  DRAKE_ASSERT(channels.size() == lcm_subscribers.size());
  DRAKE_THROW_UNLESS(
      (std::find(channels.begin(), channels.end(), default_channel) !=
      channels.end()));

  const auto& channel_input_port =
      DeclareAbstractInputPort("active_channel", Value<string>{});
  DRAKE_ASSERT(kChannelSelectorPortIndex == channel_input_port.get_index());

  DeclareAbstractOutputPort(
      [this]() {
        return this->serializer_->CreateDefaultValue();
      },
      [this](const Context<double>& context, AbstractValue* out) {
          this->Output(context, out);
      });

  for (unsigned int i = 0; i < channels.size(); i++) {
    // Initialize map
    subscriber_map_[channels.at(i)] = lcm_subscribers.at(i);

    // Create input ports from LCM subscribers
    const auto& port = DeclareAbstractInputPort(channels.at(i),
        *serializer_->CreateDefaultValue());
    port_index_map_[channels.at(i)] = port.get_index();
  }
}

const InputPort<double>& LcmSubscriberMultiplexer::get_channel_input_port() {
  return get_input_port(kChannelSelectorPortIndex);
}

const InputPort<double>& LcmSubscriberMultiplexer::GetSubscriberPort(
    const string channel) {
  return get_input_port(GetSubscriberPortIndex(channel));
}

const int& LcmSubscriberMultiplexer::GetSubscriberPortIndex(
    const string channel) {
  // return the default value if unavailable
  if (port_index_map_.count(channel) != 0) {
    return port_index_map_.at(channel);
  } else {
    return port_index_map_.at(default_channel_);
  }
}

void LcmSubscriberMultiplexer::Output(
    const Context<double>& context, AbstractValue* out) {
  auto active_channel = EvalAbstractInput(context,
      kChannelSelectorPortIndex)->get_value<string>();
  out->SetFrom(*EvalAbstractInput(context,
      GetSubscriberPortIndex(active_channel)));
}

const LcmSubscriberSystem* LcmSubscriberMultiplexer::get_active_subscriber(
    const Context<double>& context) const {
  auto active_channel = EvalAbstractInput(context,
      kChannelSelectorPortIndex)->get_value<string>();
  if (port_index_map_.count(active_channel) != 0) {
    return subscriber_map_.at(active_channel);
  } else {
    return subscriber_map_.at(default_channel_);
  }
}

const LcmSubscriberSystem* LcmSubscriberMultiplexer::get_subscriber(
    const int index) const {
  return subscribers_.at(index);
}



}  // namespace lcm
}  // namespace systems
}  // namespace dairlib
