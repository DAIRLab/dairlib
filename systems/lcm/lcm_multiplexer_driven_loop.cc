#include "systems/lcm/lcm_multiplexer_driven_loop.h"

namespace dairlib {
namespace systems {
namespace lcm {

using drake::systems::Context;
using drake::systems::System;
using drake::systems::Diagram;
using drake::AbstractValue;
using drake::systems::Simulator;

LcmMultiplexerDrivenLoop::LcmMultiplexerDrivenLoop(
    const Diagram<double>& diagram,
    const LcmSubscriberMultiplexer& driving_mux,
    std::unique_ptr<Context<double>> context, drake::lcm::DrakeLcm* lcm,
    std::unique_ptr<LcmMessageToTimeInterface> time_converter)
    : diagram_(diagram),
      driving_mux_(driving_mux),
      lcm_(lcm),
      time_converter_(std::move(time_converter)),
      stepper_(
          std::make_unique<Simulator<double>>(diagram_, std::move(context))) {
  DRAKE_DEMAND(lcm != nullptr);
  DRAKE_DEMAND(time_converter_ != nullptr);

  // Allocates extra context and output just for the driving subscriber, so
  // that this can explicitly query the message.
  const auto first_sub = driving_mux.get_subscriber(0);
  sub_context_ = first_sub->CreateDefaultContext();
  sub_output_ = first_sub->AllocateOutput();
  sub_swap_state_ = sub_context_->CloneState();
  sub_events_ = first_sub->AllocateCompositeEventCollection();

  // Disables simulator's publish on its internal time step.
  stepper_->set_publish_every_time_step(false);
  stepper_->set_publish_at_initialization(false);

  stepper_->Initialize();

  // Starts the subscribing thread.
  lcm_->StartReceiveThread();
}

const AbstractValue& LcmMultiplexerDrivenLoop::WaitForMessage(
      const Context<double>& context) {
  auto active_sub = driving_mux_.get_active_subscriber(context);

  active_sub->WaitForMessage(active_sub->GetMessageCount(*sub_context_));

  active_sub->CalcNextUpdateTime(*sub_context_, sub_events_.get());

  // If active_sub->WaitForMessage() returned, a message should be received
  // and an event should be queued by active_sub->CalcNextUpdateTime().
  if (sub_events_->HasDiscreteUpdateEvents()) {
    active_sub->CalcDiscreteVariableUpdates(
        *sub_context_, sub_events_->get_discrete_update_events(),
        &sub_swap_state_->get_mutable_discrete_state());
  } else if (sub_events_->HasUnrestrictedUpdateEvents()) {
    active_sub->CalcUnrestrictedUpdate(*sub_context_,
        sub_events_->get_unrestricted_update_events(), sub_swap_state_.get());
  } else {
    DRAKE_DEMAND(false);
  }
  sub_context_->get_mutable_state().CopyFrom(*sub_swap_state_);

  active_sub->CalcOutput(*sub_context_, sub_output_.get());
  return *(sub_output_->get_data(0));
}

void LcmMultiplexerDrivenLoop::RunToSecondsAssumingInitialized(
    double stop_time) {
  double msg_time;

  while (true) {
    const auto& context =
        diagram_.GetSubsystemContext(driving_mux_, stepper_->get_context());
    msg_time = time_converter_->GetTimeInSeconds(WaitForMessage(context));
    if (msg_time >= stop_time) break;

    stepper_->StepTo(msg_time);

    // Explicitly publish after we are done with all the intermediate
    // computation.
    if (publish_on_every_received_message_) {
      diagram_.Publish(stepper_->get_context());
    }
  }
}

}  // namespace lcm
}  // namespace systems
}  // namespace dairlib
