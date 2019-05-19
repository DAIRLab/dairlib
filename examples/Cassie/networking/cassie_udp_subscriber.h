#pragma once

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/drake_throw.h"
#include "drake/systems/framework/leaf_system.h"
#include "examples/Cassie/networking/udp_serializer.h"

namespace dairlib {
namespace systems {
/**
 * AS OF 5-17-2019, THIS CLASS IS DEPRECATED
 *
 * Receives UDP messages from Cassie outputs them to a System<double>'s port
 * as a cassie_out_t struct. This class stores the most recently processed
 * message in the State. When a message arrives asynchronously, an update
 * event is scheduled to process the message and store it in the State at the
 * earliest possible simulation time. The output is always consistent with the
 * State.
 *
 * To process a message, CalcNextUpdateTime() needs to be called first to
 * check for new messages and schedule a callback event if a new message
 * has arrived. The message is then processed and stored in the Context by
 * CalcUnrestrictedUpdate(). When this system is evaluated by the Simulator,
 * all these operations are taken care of by the Simulator. On the other hand,
 * the user needs to manually replicate this process without the Simulator.
 *
 * @ingroup message_passing
 */
class CassieUDPSubscriber : public drake::systems::LeafSystem<double> {
  /**
   * A callback used by Poll(), with arguments:
   * - `message_buffer` A pointer to the byte vector that is the serial
   *   representation of the UDP message.
   * - `message_size` The size of `message_buffer`.
   */
  using HandlerFunction = std::function<void(const void*, int)>;


 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CassieUDPSubscriber)

  /**
   * Factory method that returns a subscriber System that provides
   *
   * @param address the IP address to subscribe to
   *
   * @param port the port to listen on
   */
  static std::unique_ptr<CassieUDPSubscriber> Make(const std::string& address,
      const int port) {
    return std::make_unique<CassieUDPSubscriber>(
        address, port);
  }

  /**
   * Constructor that returns a subscriber System that provides message objects
   * on its sole abstract-valued output port.  
   *
   * @param address the IP address to subscribe to
   *
   * @param port the port to listen on
   */
  CassieUDPSubscriber(const std::string& address, const int port);

  ~CassieUDPSubscriber() override;

  void StopPolling();

  /// Main polling function.
  /// @param handler for handling received messages
  void Poll(HandlerFunction handler);

  /// Returns the sole output port.
  const drake::systems::OutputPort<double>& get_output_port() const {
    DRAKE_THROW_UNLESS(this->num_output_ports() == 1);
    return drake::systems::LeafSystem<double>::get_output_port(0);
  }

  // Don't use the indexed overload; use the no-arg overload.
  void get_output_port(int index) = delete;

  // This system has no input ports.
  void get_input_port(int) = delete;

  // Gets the last time, in microseconds, of the most recently received message
  // Counts from the time this subscriber was initialized, which seems
  // safe because there should only ever be one such subscriber in a process
  // Needed for UDPDrivenLoop
  int get_message_utime(const drake::systems::Context<double>& context) const;

  /**
   * Blocks the caller until its internal message count exceeds
   * `old_message_count`.
   * @param old_message_count Internal message counter.
   * @param message If non-null, will return the received message.
   * @pre If `message` is specified, this system must be abstract-valued.
   */
  int WaitForMessage(int old_message_count,
      drake::AbstractValue* message = nullptr) const;

  /**
   * (Advanced.) Writes the most recently received message (and message count)
   * into @p state.  If no messages have been received, only the message count
   * is updated.  This is primarily useful for unit testing.
   */
  void CopyLatestMessageInto(drake::systems::State<double>* state) const;

  /**
   * Returns the internal message counter. Meant to be used with
   * `WaitForMessage`.
   */
  int GetInternalMessageCount() const;

  /**
   * Returns the message counter stored in @p context.
   */
  int GetMessageCount(const drake::systems::Context<double>& context) const;

 protected:
  void DoCalcNextUpdateTime(const drake::systems::Context<double>& context,
    drake::systems::CompositeEventCollection<double>* events,
    double* time) const override;

  void DoCalcUnrestrictedUpdate(
      const drake::systems::Context<double>&,
      const std::vector<
          const drake::systems::UnrestrictedUpdateEvent<double>*>&,
      drake::systems::State<double>* state) const override {
    ProcessMessageAndStoreToAbstractState(&state->get_mutable_abstract_state());
  }

 private:
  void ProcessMessageAndStoreToAbstractState(
      drake::systems::AbstractValues* abstract_state) const;

  // Callback entry point from LCM into this class.
  void HandleMessage(const void*, int);

  std::string make_name(const std::string& address, const int port);

  // This pair of methods is used for the output port when we're using a
  // serializer.
  std::unique_ptr<drake::AbstractValue> AllocateSerializerOutputValue()
      const;
  void CalcSerializerOutputValue(const drake::systems::Context<double>& context,
      drake::AbstractValue* output_value) const;

  // The channel on which to receive messages.
  const std::string address_;

  // The port on which to receive messages
  const int port_;

  // The mutex that guards received_message_ and received_message_count_.
  mutable std::mutex received_message_mutex_;

  // A condition variable that's signaled every time the handler is called.
  mutable std::condition_variable received_message_condition_variable_;

  // The bytes of the most recently received LCM message.
  std::vector<uint8_t> received_message_;

  // A message counter that's incremented every time the handler is called.
  int received_message_count_{0};

  int socket_;
  struct sockaddr_in server_address_;
  std::thread polling_thread_;

  const std::unique_ptr<CassieUDPOutSerializer> serializer_;

  std::chrono::time_point<std::chrono::steady_clock> start_;

  bool keep_polling_;
};

}  // namespace systems
}  // namespace dairlib
