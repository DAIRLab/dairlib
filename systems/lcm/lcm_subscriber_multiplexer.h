#pragma once

#include <map>
#include <string>
#include <vector>

#include "drake/systems/lcm/serializer.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace dairlib {
namespace systems {
namespace lcm {

class LcmSubscriberMultiplexer : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LcmSubscriberMultiplexer)

  /// A factory method that constructs both the LcmSubscriberMultiplexer
  /// and its constituent subscribers
  template <typename LcmMessage>
  static LcmSubscriberMultiplexer* MakeMuxAndSubscribers(
      drake::systems::DiagramBuilder<double>* builder,
      const std::vector<std::string>& channels,
      drake::lcm::DrakeLcmInterface* lcm,
      const std::string default_channel = "") {
    using drake::systems::lcm::LcmSubscriberSystem;

    // Create subscribers
    std::vector<LcmSubscriberSystem*> subscribers;
    for (const std::string& channel :  channels) {
      LcmSubscriberSystem* sub = builder->AddSystem(
          LcmSubscriberSystem::Make<LcmMessage>(channel, lcm));
      subscribers.push_back(sub);
    }

    auto mux = builder->AddSystem(
        std::make_unique<LcmSubscriberMultiplexer>(channels, subscribers,
            std::make_unique<drake::systems::lcm::Serializer<LcmMessage>>(),
            (default_channel.size() == 0) ? channels.at(0) : default_channel));
    for (unsigned int i = 0; i < channels.size(); i++) {
      builder->Connect(subscribers.at(i)->get_output_port(),
                       mux->GetSubscriberPort(channels.at(i)));
    }
    return mux;
  }

  /// The main constructor for a LcmSubscriberMultiplexer. The preferred usage,
  /// however, is LcmSubscriberMultiplexer::Make<>(). Use this constructor with
  /// care, as it assumes, but does not check, that the channel vector
  /// implicitly corresponds with the vector of subscribers, and that the
  /// serializer is correct for all subscribers.
  LcmSubscriberMultiplexer(
    const std::vector<std::string>& channels,
    std::vector<drake::systems::lcm::LcmSubscriberSystem*>
        subscribers,
    std::unique_ptr<drake::systems::lcm::SerializerInterface> serializer,
    const std::string default_channel);


  const drake::systems::InputPort<double>& get_channel_input_port();

  const drake::systems::InputPort<double>& GetSubscriberPort(
      const std::string channel);

  const int& GetSubscriberPortIndex(const std::string channel);

  const drake::systems::lcm::LcmSubscriberSystem* get_active_subscriber(
      const drake::systems::Context<double>& context) const;

  const drake::systems::lcm::LcmSubscriberSystem* get_subscriber(
      const int index) const;

 private:
  const std::unique_ptr<drake::systems::lcm::SerializerInterface> serializer_;

  void Output(const drake::systems::Context<double>& context,
              drake::AbstractValue* out);

  std::vector<drake::systems::lcm::LcmSubscriberSystem*> subscribers_;

  std::map<std::string, drake::systems::lcm::LcmSubscriberSystem*>
      subscriber_map_;

  std::map<std::string, int> port_index_map_;

  std::string default_channel_;
};

}  // namespace lcm
}  // namespace systems
}  // namespace dairlib
