#include "examples/Cassie/networking/cassie_udp_publisher.h"

#include <cstdint>
#include <utility>
#include <vector>

#include "drake/common/text_logging.h"
#include "drake/systems/framework/fixed_input_port_value.h"

namespace dairlib {
namespace systems {

using drake::systems::TriggerType;

namespace {
const int kPortIndex = 0;
}  // namespace

CassieUDPPublisher::CassieUDPPublisher(const std::string& address,
      const int port, const UDPTriggerTypes& publish_triggers,
      double publish_period)
    : address_(address),
      port_(port),
      serializer_(std::move(std::make_unique<CassieUDPInSerializer>())) {
  DRAKE_DEMAND(publish_period >= 0.0);
  DRAKE_DEMAND(!publish_triggers.empty());

  // Check that publish_triggers does not contain an unsupported trigger
  for (const auto& trigger : publish_triggers) {
      DRAKE_DEMAND((trigger == TriggerType::kForced) ||
        (trigger == TriggerType::kPeriodic) ||
        (trigger == TriggerType::kPerStep));
  }

  // Creating socket file descriptor
  socket_ = socket(AF_INET, SOCK_DGRAM, 0);

  memset(&server_address_, 0, sizeof(server_address_));

  // Filling server information
  inet_aton(address_.c_str(), &server_address_.sin_addr);
  server_address_.sin_family = AF_INET;  // IPv4
  server_address_.sin_port = htons(port);

  // Declare a forced publish so that any time Publish(.) is called on this
  // system (or a Diagram containing it), a message is emitted.
  if (publish_triggers.find(TriggerType::kForced) != publish_triggers.end()) {
    this->DeclareForcedPublishEvent(
      &CassieUDPPublisher::PublishInputAsUDPMessage);
  }

  DeclareAbstractInputPort("cassie_user_in_t",
      *serializer_->CreateDefaultValue());

  set_name(make_name(address, port));

  if (publish_triggers.find(TriggerType::kPeriodic) != publish_triggers.end()) {
    DRAKE_DEMAND(publish_period > 0);
    const double offset = 0.0;
    this->DeclarePeriodicPublishEvent(
        publish_period, offset,
        &CassieUDPPublisher::PublishInputAsUDPMessage);
  } else {
    DRAKE_DEMAND(publish_period == 0);
  }
  if (publish_triggers.find(TriggerType::kPerStep) != publish_triggers.end()) {
    this->DeclarePerStepEvent(
        drake::systems::PublishEvent<double>([this](
            const drake::systems::Context<double>& context,
            const drake::systems::PublishEvent<double>&) {
          this->PublishInputAsUDPMessage(context);
        }));
  }
}

CassieUDPPublisher::CassieUDPPublisher(const std::string& address,
      const int port, double publish_period)
    : CassieUDPPublisher(address, port,
      (publish_period > 0) ?
      UDPTriggerTypes({TriggerType::kForced, TriggerType::kPeriodic}) :
      UDPTriggerTypes({TriggerType::kForced, TriggerType::kPerStep}),
      publish_period) {}

CassieUDPPublisher::~CassieUDPPublisher() {}

std::string CassieUDPPublisher::make_name(const std::string& address,
                                          const int port) {
  return "CassieUDPPublisher(" + address + ":" + std::to_string(port) + ")";
}

// Takes the cassie_user_in_t from the input port of the context and publishes
// it as a UDP message. This function is called automatically, as
// necessary, at the requisite publishing period (if a positive publish period
// was passed to the constructor) or per a simulation step (if no publish
// period or publish period = 0.0 was passed to the constructor).
drake::systems::EventStatus CassieUDPPublisher::PublishInputAsUDPMessage(
    const drake::systems::Context<double>& context) const {
  SPDLOG_TRACE(drake::log(), "Publishing UDP {} message", address_);

  // Converts the input into message bytes.
  const drake::AbstractValue* const input_value =
      this->EvalAbstractInput(context, kPortIndex);
  DRAKE_ASSERT(input_value != nullptr);
  std::vector<uint8_t> message_bytes;

  serializer_->Serialize(*input_value, &message_bytes);

  int result = sendto(socket_, message_bytes.data(),
      message_bytes.size(), 0,
      (struct sockaddr *)&server_address_, sizeof(server_address_));
  DRAKE_THROW_UNLESS(result >= 0);
  return drake::systems::EventStatus::Succeeded();
}

}  // namespace systems
}  // namespace dairlib
