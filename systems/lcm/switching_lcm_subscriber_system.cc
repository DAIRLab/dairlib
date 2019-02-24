#include "systems/lcm/switching_lcm_subscriber_system.h"

#include <functional>
#include <iostream>
#include <utility>
#include <functional>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/systems/framework/basic_vector.h"

namespace dairlib {
namespace systems {
namespace lcm {

using drake::lcm::DrakeLcmInterface;
using drake::systems::Context;
using drake::systems::AbstractValues;
using drake::AbstractValue;
using drake::systems::UnrestrictedUpdateEvent;
using drake::systems::CompositeEventCollection;
using drake::systems::EventCollection;
using drake::systems::lcm::SerializerInterface;
using drake::systems::State;
using drake::systems::lcm::Serializer;
using std::make_unique;

namespace {
constexpr int kStateIndexMessage = 0;
constexpr int kStateIndexMessageCount = 1;
}  // namespace


SwitchingLcmSubscriberSystem::SwitchingLcmSubscriberSystem(
    const std::string& channel,
    const std::string& switching_channel,
    std::unique_ptr<SerializerInterface> serializer,
    drake::lcm::DrakeLcmInterface* lcm)
    : serializer_(std::move(serializer)),
      switching_serializer_(
          std::make_unique<Serializer<lcmt_subscriber_switch>>()),
      lcm_(lcm) {
  DRAKE_DEMAND(serializer_ != nullptr);
  DRAKE_DEMAND(lcm);

  DeclareAbstractOutputPort(
      [this]() {
        return this->AllocateSerializerOutputValue();
      },
      [this](const Context<double>& context, AbstractValue* out) {
        this->CalcSerializerOutputValue(context, out);
      });

  lcm_->Subscribe(switching_channel,
      [this] (const void* buffer, int size) {
        this->SwitchingLcmSubscriberSystem::HandleSwitch(buffer, size);
      });

  SubscribeAndSetChannel(channel);

  static_assert(kStateIndexMessage == 0, "");
  this->DeclareAbstractState(AllocateSerializerOutputValue());
  static_assert(kStateIndexMessageCount == 1, "");
  this->DeclareAbstractState(AbstractValue::Make<int>(0));
}

SwitchingLcmSubscriberSystem::~SwitchingLcmSubscriberSystem() {}

void SwitchingLcmSubscriberSystem::CopyLatestMessageInto(State<double>* state)
    const {
  ProcessMessageAndStoreToAbstractState(&state->get_mutable_abstract_state());
}

void SwitchingLcmSubscriberSystem::SubscribeAndSetChannel(std::string channel) {
  if (subscribed_channels_.find(channel) == subscribed_channels_.end()) {
    subscribed_channels_.insert(channel);

    // Bind a function to HandleMessage where the channel name is fixed
    auto handle_function =
        std::bind(&SwitchingLcmSubscriberSystem::HandleMessage,
            this, channel, std::placeholders::_1, std::placeholders::_2);

    lcm_->Subscribe(channel, handle_function);
  }

  std::cout << "Subscribing to: " << channel << std::endl;

  active_channel_ = channel;
}

void SwitchingLcmSubscriberSystem::ProcessMessageAndStoreToAbstractState(
    AbstractValues* abstract_state) const {
  DRAKE_ASSERT(serializer_ != nullptr);

  std::lock_guard<std::mutex> lock(received_message_mutex_);
  if (!received_message_.empty()) {
    serializer_->Deserialize(
        received_message_.data(), received_message_.size(),
        &abstract_state->get_mutable_value(kStateIndexMessage));
  }
  abstract_state->get_mutable_value(kStateIndexMessageCount)
      .GetMutableValue<int>() = received_message_count_;
}

int SwitchingLcmSubscriberSystem::GetMessageCount(
      const Context<double>& context) const {
  // Gets the last message count from either abstract state or discrete state.
  int last_message_count;
  last_message_count =
      context.get_abstract_state<int>(kStateIndexMessageCount);
  return last_message_count;
}

void SwitchingLcmSubscriberSystem::DoCalcNextUpdateTime(
    const Context<double>& context,
    CompositeEventCollection<double>* events, double* time) const {
  // We do not support events other than our own message timing events.
  LeafSystem<double>::DoCalcNextUpdateTime(context, events, time);
  DRAKE_THROW_UNLESS(events->HasEvents() == false);
  DRAKE_THROW_UNLESS(std::isinf(*time));

  // Do nothing unless we have a new message.
  const int last_message_count = GetMessageCount(context);
  const int received_message_count = [this]() {
    std::unique_lock<std::mutex> lock(received_message_mutex_);
    return received_message_count_;
  }();
  if (last_message_count == received_message_count) {
    return;
  }

  // Schedule an update event at the current time.
  *time = context.get_time();
  EventCollection<UnrestrictedUpdateEvent<double>>& uu_events =
      events->get_mutable_unrestricted_update_events();
  uu_events.add_event(
      std::make_unique<UnrestrictedUpdateEvent<double>>(
          drake::systems::TriggerType::kTimed));
}

// This is only called if our output port is abstract-valued, because we are
// using a serializer.
std::unique_ptr<AbstractValue>
SwitchingLcmSubscriberSystem::AllocateSerializerOutputValue() const {
  DRAKE_DEMAND(serializer_ != nullptr);
  return serializer_->CreateDefaultValue();
}

void SwitchingLcmSubscriberSystem::CalcSerializerOutputValue(
    const Context<double>& context, AbstractValue* output_value) const {
  DRAKE_DEMAND(serializer_ != nullptr);
  output_value->SetFrom(
      context.get_abstract_state().get_value(kStateIndexMessage));
}

void SwitchingLcmSubscriberSystem::HandleSwitch(const void* buffer, int size) {
  auto value =
      AbstractValue::Make<lcmt_subscriber_switch>(lcmt_subscriber_switch());
  switching_serializer_->Deserialize(buffer, size, value.get());
  SubscribeAndSetChannel(value->GetValue<lcmt_subscriber_switch>().channel);
}

void SwitchingLcmSubscriberSystem::HandleMessage(std::string handle_channel,
                                        const void* buffer, int size) {
  // Ignore message
  if (active_channel_ != handle_channel) {
    return;
  }

  std::cout << "Handling message: " << handle_channel << std::endl;

  const uint8_t* const rbuf_begin = static_cast<const uint8_t*>(buffer);
  const uint8_t* const rbuf_end = rbuf_begin + size;
  std::lock_guard<std::mutex> lock(received_message_mutex_);
  received_message_.clear();
  received_message_.insert(received_message_.begin(), rbuf_begin, rbuf_end);
  received_message_count_++;
  received_message_condition_variable_.notify_all();
}

int SwitchingLcmSubscriberSystem::WaitForMessage(
    int old_message_count, AbstractValue* message) const {
  DRAKE_ASSERT(serializer_ != nullptr);

  // The message buffer and counter are updated in HandleMessage(), which is
  // a callback function invoked by a different thread owned by the
  // drake::lcm::DrakeLcmInterface instance passed to the constructor. Thus,
  // for thread safety, these need to be properly protected by a mutex.
  std::unique_lock<std::mutex> lock(received_message_mutex_);

  // This while loop is necessary to guard for spurious wakeup:
  // https://en.wikipedia.org/wiki/Spurious_wakeup
  while (old_message_count >= received_message_count_) {
    // When wait returns, lock is atomically acquired. So it's thread safe to
    // read received_message_count_.
    received_message_condition_variable_.wait(lock);
  }
  int new_message_count = received_message_count_;
  if (message) {
      serializer_->Deserialize(
          received_message_.data(), received_message_.size(), message);
  }
  lock.unlock();

  return new_message_count;
}

int SwitchingLcmSubscriberSystem::GetInternalMessageCount() const {
  std::unique_lock<std::mutex> lock(received_message_mutex_);
  return received_message_count_;
}


}  // namespace lcm
}  // namespace systems
}  // namespace dairlib
