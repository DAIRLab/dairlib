#include "systems/framework/lcm_driven_loop.h"

namespace dairlib {
namespace systems {

using std::vector;
using std::string;
using std::unique_ptr;
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

using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::Context;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::TriggerType;
using drake::lcm::Subscriber;

LcmDrivenLoop::LcmDrivenLoop(
    drake::lcm::DrakeLcm* lcm_local,
    drake::systems::DiagramBuilder<double>* builder,
    const drake::systems::LeafSystem<double>* first_leafsystem)
    : lcm_local_(lcm_local),
      builder_(builder),
      first_leafsystem_(first_leafsystem),
      input_switch_sub_(drake::lcm::Subscriber<lcmt_controller_switch>(
          lcm_local,
          "TEMP")) {
  auto owned_diagram = builder->Build();
  diagram_ = owned_diagram.get();
  simulator_ = std::make_unique<drake::systems::Simulator<double>>(
      std::move(owned_diagram));
}

void LcmDrivenLoop::SetSwitchChannelName(std::string switch_channel) {
  // Create subscribers
  input_switch_sub_ = drake::lcm::Subscriber<lcmt_controller_switch>(lcm_local_,
                                                                     switch_channel);
}

//void LcmDrivenLoop::SetSingleInputChannelName(string input_channel) {
//
//  active_channel_ = input_channel;
//
//  // Create subscribers
//    std::cout << "Constructing subscriber for " << input_channel << std::endl;
//    name_to_sub_map_.insert(std::make_pair(input_channel,
//                                          drake::lcm::Subscriber<lcmt_robot_input>(
//                                              lcm_local_,
//                                              input_channel)));
//}


void LcmDrivenLoop::SetMultipleInputChannelName(vector<string> input_channels) {
  active_channel_ = input_channels.at(0);

  // Create subscribers
  for (auto name : input_channels) {
    std::cout << "Constructing subscriber for " << name << std::endl;
    name_to_sub_map_.insert(std::make_pair(name,
                                           drake::lcm::Subscriber<
                                               lcmt_robot_input>(
                                               lcm_local_,
                                               name)));
  }
}

void LcmDrivenLoop::Simulate(double end_time) {
  // Get a reference of the diagram
//  const auto& diagram = simulator_->get_system();
  auto& diagram_context = simulator_->get_mutable_context();
  auto& first_leafsystem_context =
      diagram_->GetMutableSubsystemContext(*first_leafsystem_, &diagram_context);

  // Wait for the first message.
  drake::log()->info("Waiting for first lcmt_robot_input");
  LcmHandleSubscriptionsUntil(lcm_local_, [&]() {
    return name_to_sub_map_.at(active_channel_).count() > 0;
  });

  // Initialize the context based on the first message.
  const double t0 = name_to_sub_map_.at(active_channel_).message().utime * 1e-6;
  diagram_context.SetTime(t0);
  auto& command_value = first_leafsystem_->get_input_port(0).FixValue(
      &first_leafsystem_context,
      name_to_sub_map_.at(active_channel_).message());

  // Store robot input
  lcmt_robot_input previous_input =
      name_to_sub_map_.at(active_channel_).message();

  // "Simulator" time
  double time = 0; // initialize the current time to 0
  double message_time;

  drake::log()->info("dispatcher_robot_in started");
  while (time < end_time) {
    // Update active channel name
    if (input_switch_sub_.count() > 0) {
      // Check if the channel name is a key of the map
      if (name_to_sub_map_.count(input_switch_sub_.message().channel) == 1) {
        active_channel_ = input_switch_sub_.message().channel;
      } else {
        std::cout << input_switch_sub_.message().channel << " doesn't exist\n";
      }
      input_switch_sub_.clear();
    }

    // Wait for an lcmt_robot_input or an lcmt_controller_switch message.
    name_to_sub_map_.at(active_channel_).clear();
    LcmHandleSubscriptionsUntil(
        lcm_local_,
        [&]() { return (name_to_sub_map_.at(active_channel_).count() > 0); });

    // Write the lcmt_robot_input message into the context.
    command_value.GetMutableData()->set_value(
        name_to_sub_map_.at(active_channel_).message());

    // Get message time from the active channel to advance
    message_time = name_to_sub_map_.at(active_channel_).message().utime * 1e-6;
    // We cap the time from bottom just in case the message time is older around
    // the instant when we switch to a different controller
    if (message_time >= time) {
      time = message_time;
    }

    // Check if we are very far ahead or behind
    // (likely due to a restart of the driving clock)
    if (time > simulator_->get_context().get_time() + 1.0 ||
        time < simulator_->get_context().get_time()) {
      std::cout << "Dispatcher time is " << simulator_->get_context().get_time()
                << ", but stepping to " << time << std::endl;
      std::cout << "Difference is too large, resetting dispatcher time." <<
                std::endl;
      simulator_->get_mutable_context().SetTime(time);
    }

    simulator_->AdvanceTo(time);
    // Force-publish via the diagram
    diagram_->Publish(diagram_context);
  }
}

}  // namespace systems
}  // namespace dairlib