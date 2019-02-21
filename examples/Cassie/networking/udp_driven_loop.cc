#include "examples/Cassie/networking/udp_driven_loop.h"

namespace dairlib {
namespace systems {

using drake::systems::Context;
using drake::systems::System;
using drake::systems::SystemOutput;
using drake::systems::State;
using drake::systems::CompositeEventCollection;
using drake::systems::Simulator;
using drake::AbstractValue;

UDPDrivenLoop::UDPDrivenLoop(
    const System<double>& system, const CassieUDPSubscriber& driving_subscriber,
    std::unique_ptr<Context<double>> context)
    : system_(system),
      driving_sub_(driving_subscriber),
      stepper_(
          std::make_unique<Simulator<double>>(system_, std::move(context))) {
  // Allocates extra context and output just for the driving subscriber, so
  // that this can explicitly query the message.
  sub_context_ = driving_sub_.CreateDefaultContext();
  sub_output_ = driving_sub_.AllocateOutput();
  sub_swap_state_ = sub_context_->CloneState();
  sub_events_ = driving_sub_.AllocateCompositeEventCollection();

  // Disables simulator's publish on its internal time step.
  stepper_->set_publish_at_initialization(false);
  stepper_->set_publish_every_time_step(false);
  stepper_->Initialize();
}

const AbstractValue& UDPDrivenLoop::WaitForMessage() {
  driving_sub_.WaitForMessage(driving_sub_.GetMessageCount(*sub_context_));

  driving_sub_.CalcNextUpdateTime(*sub_context_, sub_events_.get());

  // If driving_sub_.WaitForMessage() returned, a message should be received
  // and an event should be queued by driving_sub_.CalcNextUpdateTime().
  driving_sub_.CalcUnrestrictedUpdate(*sub_context_,
      sub_events_->get_unrestricted_update_events(), sub_swap_state_.get());
  sub_context_->get_mutable_state().CopyFrom(*sub_swap_state_);

  driving_sub_.CalcOutput(*sub_context_, sub_output_.get());
  return *(sub_output_->get_data(0));
}

void UDPDrivenLoop::RunToSecondsAssumingInitialized(double stop_time) {
  double msg_time;

  while (true) {
    // std::cout << "UDPDrivenLoop::WaitForMessage." << std::endl;
    WaitForMessage();
    msg_time = driving_sub_.get_message_utime(*sub_context_)/1.0e6;
    if (msg_time >= stop_time) break;
    // std::cout << "UDPDrivenLoop::Starting step." << std::endl;
    stepper_->StepTo(msg_time);
    // std::cout << "UDPDrivenLoop::Starting publish" << std::endl; 
    // Explicitly publish after we are done with all the intermediate
    // computation.
    if (publish_on_every_received_message_) {
      system_.Publish(stepper_->get_context());
    }
  }
}


}  // namespace systems
}  // namespace dairlib
