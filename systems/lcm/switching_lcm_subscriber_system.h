#pragma once

#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <set>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/drake_throw.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_translator_dictionary.h"
#include "drake/systems/lcm/serializer.h"

namespace dairlib {
namespace systems {
namespace lcm {

/**
 * This class is based on drake::systems::LcmSubscriberSystem, but is modified
 * so that the subscription channel can be changed via an LCM message on a 
 * separate channel.
 *
 * SwitchingLcmSubscriberSystem:
 * Receives LCM messages from a given channel and outputs them to a
 * System<double>'s port. This class stores the most recently processed LCM
 * message in the State. When a LCM message arrives asynchronously, an update
 * event is scheduled to process the message and store it in the State at the
 * earliest possible simulation time. The output is always consistent with the
 * State.
 *
 * To process a LCM message, CalcNextUpdateTime() needs to be called first to
 * check for new messages and schedule a callback event if a new LCM message
 * has arrived. The message is then processed and stored in the Context by
 * CalcUnrestrictedUpdate(). When this system is evaluated by the Simulator,
 * all these operations are taken care of by the Simulator. On the other hand,
 * the user needs to manually replicate this process without the Simulator.
 *
 * If LCM service in use is a drake::lcm::DrakeLcmLog (not live operation),
 * then see drake::systems::lcm::LcmLogPlaybackSystem for a helper to advance
 * the log cursor in concert with the simulation.
 *
 * @ingroup message_passing
 */
class SwitchingLcmSubscriberSystem : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SwitchingLcmSubscriberSystem)

  /**
   * Factory method that returns a subscriber System that provides
   * Value<LcmMessage> message objects on its sole abstract-valued output port.
   *
   * @tparam LcmMessage message type to deserialize, e.g., lcmt_drake_signal.
   *
   * @param[in] channel The LCM channel on which to subscribe.
   *
   * @param lcm A non-null pointer to the LCM subsystem to subscribe on.
   */
  template <typename LcmMessage>
  static std::unique_ptr<SwitchingLcmSubscriberSystem> Make(
      const std::string& channel, const std::string& switching_channel,
      drake::lcm::DrakeLcmInterface* lcm) {
    return std::make_unique<SwitchingLcmSubscriberSystem>(channel,
        switching_channel,
        std::make_unique<drake::systems::lcm::Serializer<LcmMessage>>(), lcm);
  }


  /**
   * Constructor that returns a subscriber System that provides message objects
   * on its sole abstract-valued output port.  The type of the message object is
   * determined by the @p serializer.
   *
   * @param[in] channel The LCM channel on which to subscribe.
   *
   * @param[in] serializer The serializer that converts between byte vectors
   * and LCM message objects.
   *
   * @param lcm A non-null pointer to the LCM subsystem to subscribe on.
   */
  SwitchingLcmSubscriberSystem(const std::string& channel,
      const std::string& switching_channel,
      std::unique_ptr<drake::systems::lcm::SerializerInterface> serializer,
      drake::lcm::DrakeLcmInterface* lcm);
  /// Returns the sole output port.
  const drake::systems::OutputPort<double>& get_output_port() const {
    DRAKE_THROW_UNLESS(this->get_num_output_ports() == 1);
    return drake::systems::LeafSystem<double>::get_output_port(0);
  }

  ~SwitchingLcmSubscriberSystem() override;

  // Don't use the indexed overload; use the no-arg overload.
  void get_output_port(int index) = delete;

  // This system has no input ports.
  void get_input_port(int) = delete;

  /**
   * Blocks the caller until its internal message count exceeds
   * `old_message_count`.
   * @param old_message_count Internal message counter.
   * @param message If non-null, will return the received message.
   * @pre If `message` is specified, this system must be abstract-valued.
   */
  int WaitForMessage(
      int old_message_count, drake::AbstractValue* message = nullptr) const;

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
      const drake::systems::Context<double>&, const
      std::vector<const drake::systems::UnrestrictedUpdateEvent<double>*>&,
      drake::systems::State<double>* state) const override {
    ProcessMessageAndStoreToAbstractState(&state->get_mutable_abstract_state());
  }

 private:
  void SubscribeAndSetChannel(std::string channel);

  void ProcessMessageAndStoreToAbstractState(
      drake::systems::AbstractValues* abstract_state) const;

  void HandleSwitch(const void*, int);

  // Callback entry point from LCM into this class.
  void HandleMessage(std::string handle_channel, const void*, int);

  // This pair of methods is used for the output port when we're using a
  // serializer.
  std::unique_ptr<drake::AbstractValue> AllocateSerializerOutputValue() const;
  void CalcSerializerOutputValue(const drake::systems::Context<double>& context,
                                 drake::AbstractValue* output_value) const;


  // Converts LCM message bytes to Value<LcmMessage> objects.
  // Will be non-null iff our output port is abstract-valued.
  const std::unique_ptr<drake::systems::lcm::SerializerInterface> serializer_;

  // The mutex that guards received_message_ and received_message_count_.
  mutable std::mutex received_message_mutex_;

  // A condition variable that's signaled every time the handler is called.
  mutable std::condition_variable received_message_condition_variable_;

  const std::unique_ptr<drake::systems::lcm::SerializerInterface> 
      switching_serializer_;

  // The bytes of the most recently received LCM message.
  std::vector<uint8_t> received_message_;

  // A message counter that's incremented every time the handler is called.
  int received_message_count_{0};

  drake::lcm::DrakeLcmInterface* lcm_;

  // current active channel name
  std::string active_channel_;

  // list of subscribed channels
  std::set<std::string> subscribed_channels_;
};

}  // namespace lcm
}  // namespace systems
}  // namespace dairlib
