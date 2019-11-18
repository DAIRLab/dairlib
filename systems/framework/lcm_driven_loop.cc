#include "systems/framework/lcm_driven_loop.h"

#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_cassie_out.hpp"

namespace dairlib {
namespace systems {

using std::vector;
using std::string;
using std::unique_ptr;

using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::lcm::Subscriber;

template <typename InputMessageType, typename SwitchMessageType>
LcmDrivenLoop<InputMessageType, SwitchMessageType>::LcmDrivenLoop(
    drake::lcm::DrakeLcm* drake_lcm,
    std::unique_ptr<drake::systems::Diagram<double>> diagram,
    const drake::systems::LeafSystem<double>* first_leafsystem)
    : drake_lcm_(drake_lcm),
      first_leafsystem_(first_leafsystem) {
  diagram_ptr_ = diagram.get();
  simulator_ = std::make_unique<drake::systems::Simulator<double>>(
      std::move(diagram));
}

template <typename InputMessageType, typename SwitchMessageType>
LcmDrivenLoop<InputMessageType, SwitchMessageType>::LcmDrivenLoop(
    drake::lcm::DrakeLcm* drake_lcm,
    std::unique_ptr<drake::systems::Diagram<double>> diagram,
    const drake::systems::LeafSystem<double>* first_leafsystem,
    const std::string& input_channel)
    : LcmDrivenLoop(drake_lcm, std::move(diagram), first_leafsystem) {
  is_multiple_inputs_ = false;

  // Create subscribers for inputs
  std::cout << "Constructing subscriber for " << input_channel << std::endl;
  name_to_input_sub_map_.insert(std::make_pair(input_channel,
                                               drake::lcm::Subscriber<
                                                   InputMessageType>(
                                                   drake_lcm_,
                                                   input_channel)));

  active_channel_ = input_channel;
}

template <typename InputMessageType, typename SwitchMessageType>
LcmDrivenLoop<InputMessageType, SwitchMessageType>::LcmDrivenLoop(
    drake::lcm::DrakeLcm* drake_lcm,
    std::unique_ptr<drake::systems::Diagram<double>> diagram,
    const drake::systems::LeafSystem<double>* first_leafsystem,
    const std::string& switch_channel,
    vector<string> input_channels)
    : LcmDrivenLoop(drake_lcm, std::move(diagram), first_leafsystem) {
  DRAKE_DEMAND(!input_channels.empty());
  is_multiple_inputs_ = true;

  // Create subscriber for the switch
  switch_sub_ =
      std::make_unique<drake::lcm::Subscriber<SwitchMessageType>>(
          drake_lcm_,
          switch_channel);

  // Create subscribers for inputs
  for (const auto& name : input_channels) {
    std::cout << "Constructing subscriber for " << name << std::endl;
    name_to_input_sub_map_.insert(std::make_pair(name,
                                                 drake::lcm::Subscriber<
                                                     InputMessageType>(
                                                     drake_lcm_,
                                                     name)));
  }

  // Default initial active channel
  active_channel_ = input_channels.at(0);
}

template <typename InputMessageType, typename SwitchMessageType>
void LcmDrivenLoop<InputMessageType, SwitchMessageType>::SetInitActiveChannel(
    const std::string& init_channel) {
  active_channel_ = init_channel;
}

template <typename InputMessageType, typename SwitchMessageType>
void LcmDrivenLoop<InputMessageType, SwitchMessageType>::SetName(
    const std::string& diagram_name) {
  diagram_name_ = diagram_name;
}

template <typename InputMessageType, typename SwitchMessageType>
void LcmDrivenLoop<InputMessageType, SwitchMessageType>::Simulate(
    double end_time) {
  // Get mutable contexts
  auto& diagram_context = simulator_->get_mutable_context();
  auto& first_leafsystem_context =
      diagram_ptr_->GetMutableSubsystemContext(*first_leafsystem_,
                                               &diagram_context);

  // Wait for the first message.
  drake::log()->info("Waiting for first lcm input message");
  LcmHandleSubscriptionsUntil(drake_lcm_, [&]() {
    return name_to_input_sub_map_.at(active_channel_).count() > 0;
  });

  // Initialize the context based on the first message.
  const double t0 =
      name_to_input_sub_map_.at(active_channel_).message().utime * 1e-6;
  diagram_context.SetTime(t0);
  auto& input_value = first_leafsystem_->get_input_port(0).FixValue(
      &first_leafsystem_context,
      name_to_input_sub_map_.at(active_channel_).message());

  // "Simulator" time
  double time = 0;  // initialize the current time with 0
  double message_time;

  drake::log()->info(diagram_name_ + " started");
  // Run the simulation until end_time
  while (time < end_time) {
    // Update the name of the active channel if there are multiple inputs
    if (is_multiple_inputs_) {
      if (switch_sub_->count() > 0) {
        // Check if the channel name is a key of the map. If it is, we update
        // the active channel name and clear switch_sub_'s message. If it is not
        // we do not update the active channel name.
        if (name_to_input_sub_map_.count(switch_sub_->message().channel) == 1) {
          active_channel_ = switch_sub_->message().channel;
        } else {
          std::cout << switch_sub_->message().channel
                    << " doesn't exist\n";
        }
        switch_sub_->clear();
      }
    }

    // Wait for an InputMessageType message.
    name_to_input_sub_map_.at(active_channel_).clear();
    LcmHandleSubscriptionsUntil(
        drake_lcm_,
        [&]() {
          return (name_to_input_sub_map_.at(active_channel_).count() > 0);
        });

    // Write the InputMessageType message into the context.
    input_value.GetMutableData()->set_value(
        name_to_input_sub_map_.at(active_channel_).message());

    // Get message time from the active channel to advance
    message_time =
        name_to_input_sub_map_.at(active_channel_).message().utime * 1e-6;
    // We cap the time from below just in case after we switch to a different
    // input channel, the message time from the new channel is smaller then the
    // current diagram time
    if (message_time >= time) {
      time = message_time;
    }

    // Check if we are very far ahead or behind
    // (likely due to a restart of the driving clock)
    if (time > simulator_->get_context().get_time() + 1.0 ||
        time < simulator_->get_context().get_time()) {
      std::cout << diagram_name_ + " time is "
                << simulator_->get_context().get_time()
                << ", but stepping to " << time << std::endl;
      std::cout << "Difference is too large, resetting " +
          diagram_name_ + " time.\n";
      simulator_->get_mutable_context().SetTime(time);
    }

    simulator_->AdvanceTo(time);
    // Force-publish via the diagram
    diagram_ptr_->Publish(diagram_context);
  }
}

template class LcmDrivenLoop<dairlib::lcmt_robot_input,
                             dairlib::lcmt_controller_switch>;

template class LcmDrivenLoop<dairlib::lcmt_robot_output>;
template class LcmDrivenLoop<dairlib::lcmt_cassie_out>;

}  // namespace systems
}  // namespace dairlib