#include "examples/Cassie/networking/cassie_udp_subscriber.h"
#include <poll.h>
#include <sys/ioctl.h>
#include <functional>
#include <iostream>
#include <utility>

#include "examples/Cassie/networking/udp_serializer.h"
#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"

namespace dairlib {
namespace systems {

using drake::systems::Context;
using drake::systems::AbstractValue;
using drake::systems::AbstractValues;
using drake::systems::State;
using drake::systems::CompositeEventCollection;
using drake::systems::TriggerType;
using drake::systems::UnrestrictedUpdateEvent;
using drake::systems::EventCollection;
using std::make_unique;

namespace {
constexpr int kStateIndexMessage = 0;
constexpr int kStateIndexMessageCount = 1;
}  // namespace

CassieUDPSubscriber::CassieUDPSubscriber(const std::string& address,
    const int port)
    : address_(address),
      port_(port),
      serializer_(std::move(make_unique<CassieUDPSerializer>())) {

  // Creating socket file descriptor
  socket_ = socket(AF_INET, SOCK_DGRAM, 0);
  DRAKE_THROW_UNLESS(socket_ >= 0);

  memset(&server_address_, 0, sizeof(server_address_));
  memset(&client_address_, 0, sizeof(client_address_));

  // Filling server information
  inet_aton(address.c_str(), &server_address_.sin_addr);
  server_address_.sin_family = AF_INET;  // IPv4
  server_address_.sin_port = htons(port);

  // Bind the socket with the server address
  DRAKE_THROW_UNLESS(bind(socket_, (const struct sockaddr*) &server_address_,
      sizeof(server_address_)) >= 0);

  // Use the "advanced" method to construct explicit non-member functors
  // to deal with the unusual methods we have available.
  DeclareAbstractOutputPort(
    [this]() {
        return this->AllocateSerializerOutputValue();
      },
      [this](const Context<double>& context, AbstractValue* out) {
        this->CalcSerializerOutputValue(context, out);
      });

  // Declare our two states (message_value, message_count).
  static_assert(kStateIndexMessage == 0, "");
  this->DeclareAbstractState(AllocateSerializerOutputValue());
  static_assert(kStateIndexMessageCount == 1, "");
  this->DeclareAbstractState(AbstractValue::Make<int>(0));

  set_name(make_name(address, port));

  polling_thread_ = std::thread(&CassieUDPSubscriber::Poll, this,
      [this](const void* buffer, int size) {
        this->HandleMessage(buffer, size);
      });
}

CassieUDPSubscriber::~CassieUDPSubscriber() {}

void CassieUDPSubscriber::Poll(HandlerFunction handler) {
  // Create cassie input/output structs
  cassie_out_t cassie_out;
  char receive_buffer[2 + CASSIE_OUT_T_LEN];

  ssize_t des_len = (sizeof receive_buffer);

  // Poll for a new packet of the correct length
  ssize_t nbytes = 0;
  struct pollfd fd = {.fd = socket_, .events = POLLIN, .revents = 0};
  do {
      poll(&fd, 1, -1);
      // Get newest valid packet in RX buffer
      // Does not use sequence number for determining newest packet
      ioctl(socket_, FIONREAD, &nbytes);
      //  std::cout <<  "ioctl: " << nbytes << std::endl;
      if (des_len <= nbytes) {
        nbytes = recv(socket_, receive_buffer, des_len, 0);
      } else {
        recv(socket_, receive_buffer, 0, 0);  // Discard packet
      }
  } while (des_len != nbytes);

  // Split header and data
  const void *data_in =
    reinterpret_cast<const unsigned char *>(&receive_buffer[2]);

  handler(data_in, nbytes - 2);

  // Unpack received data into cassie output struct
  unpack_cassie_out_t(reinterpret_cast<const unsigned char *>(data_in),
                      &cassie_out);
}

void CassieUDPSubscriber::CopyLatestMessageInto(State<double>* state) const {
  ProcessMessageAndStoreToAbstractState(&state->get_mutable_abstract_state());
}

void CassieUDPSubscriber::ProcessMessageAndStoreToAbstractState(
    AbstractValues* abstract_state) const {
  std::lock_guard<std::mutex> lock(received_message_mutex_);
  if (!received_message_.empty()) {
    // TODO(mposa): deserialize/serialie
    serializer_->Deserialize(
        received_message_.data(), received_message_.size(),
        &abstract_state->get_mutable_value(kStateIndexMessage));
  }
  abstract_state->get_mutable_value(kStateIndexMessageCount)
      .GetMutableValue<int>() = received_message_count_;
}

int CassieUDPSubscriber::GetMessageCount(const Context<double>& context) const {
  // Gets the last message count from abstract state
  return context.get_abstract_state<int>(kStateIndexMessageCount);
}

void CassieUDPSubscriber::DoCalcNextUpdateTime(
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
      std::make_unique<systems::UnrestrictedUpdateEvent<double>>(
          TriggerType::kTimed));
}

std::string CassieUDPSubscriber::make_name(const std::string& address,
                                           const int port) {
  return "CassieUDPSubscriber(" + address + ":" + std::to_string(port) + ")";
}

std::unique_ptr<systems::AbstractValue>
CassieUDPSubscriber::AllocateSerializerOutputValue() const {
  return serializer_->CreateDefaultValue();
}

void CassieUDPSubscriber::CalcSerializerOutputValue(
    const Context<double>& context, AbstractValue* output_value) const {
  output_value->SetFrom(
      context.get_abstract_state().get_value(kStateIndexMessage));
}

void CassieUDPSubscriber::HandleMessage(const void* buffer, int size) {
  SPDLOG_TRACE(drake::log(), "Receiving CASSIE message");

  const uint8_t* const rbuf_begin = static_cast<const uint8_t*>(buffer);
  const uint8_t* const rbuf_end = rbuf_begin + size;
  std::lock_guard<std::mutex> lock(received_message_mutex_);
  received_message_.clear();
  received_message_.insert(received_message_.begin(), rbuf_begin, rbuf_end);
  received_message_count_++;
  received_message_condition_variable_.notify_all();
}

int CassieUDPSubscriber::WaitForMessage(
    int old_message_count, AbstractValue* message) const {

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

int CassieUDPSubscriber::GetInternalMessageCount() const {
  std::unique_lock<std::mutex> lock(received_message_mutex_);
  return received_message_count_;
}

}  // namespace systems
}  // namespace dairlib
