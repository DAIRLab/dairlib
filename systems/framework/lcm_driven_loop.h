#pragma once

#include <map>
#include <string>
#include <vector>

#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/serializer.h"

#include "dairlib/lcmt_controller_switch.hpp"

namespace dairlib {
namespace systems {

/// LcmDrivenLoop runs the simulation of a diagram (the whole system) of which
/// the update is triggered by the incoming lcm messages.
/// It can handle single and multiple incoming lcm message types.
/// In the case of single message type, the diagram is triggered by this lcm
/// trivially.
/// In the case of multiple message types (we call these messages the input
/// messages), the user has to provide one extra lcm message type (we call this
/// message the switch message) which tells LcmDrivenLoop the channel that it
/// should listen to for the simulation update.

/// WARNING: Currently, we update the value of the first InputPort of
/// `lcm_parser` with the incoming lcm message. This would cause a problem if
/// the user has a `lcm_parser` with multiple InputPort's.

/// Procedures to use LcmDrivenLoop:
/// 1. construct LcmDrivenLoop
/// 2. (if it's multi-input) the user can set the initial channel that
///    LcmDrivenLoop listens to by calling SetInitActiveChannel().
/// 3. run Simulate()

/// Note that we implement the class only in the header file because we don't
/// know what MessageTypes are beforehand.

// We set a default value for SwitchMessageType so that we can generalize this
// to both single and multi inputs.
template <typename InputMessageType,
          typename SwitchMessageType = dairlib::lcmt_controller_switch>
class LcmDrivenLoop {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LcmDrivenLoop)

  /// Constructor for single-input LcmDrivenLoop
  ///     @param drake_lcm DrakeLcm
  ///     @param diagram A Drake diagram
  ///     @param lcm_parser The LeafSystem of the diagram that parses the
  ///     incoming lcm message
  ///     @param input_channel The name of the input channel
  LcmDrivenLoop(drake::lcm::DrakeLcm* drake_lcm,
                std::unique_ptr<drake::systems::Diagram<double>> diagram,
                const drake::systems::LeafSystem<double>* lcm_parser,
                const std::string& input_channel)
      : LcmDrivenLoop(drake_lcm, std::move(diagram), lcm_parser,
                      std::vector<std::string>(1, input_channel), input_channel,
                      ""){};

  /// Constructor for multi-input LcmDrivenLoop
  ///     @param drake_lcm DrakeLcm
  ///     @param diagram A Drake diagram
  ///     @param lcm_parser The LeafSystem of the diagram that parses the
  ///     incoming lcm message
  ///     @param input_channels The names of the input channels
  ///     @param active_channel The name of the initial active input channel
  ///     @param switch_channel The name of the switch channel
  LcmDrivenLoop(drake::lcm::DrakeLcm* drake_lcm,
                std::unique_ptr<drake::systems::Diagram<double>> diagram,
                const drake::systems::LeafSystem<double>* lcm_parser,
                std::vector<std::string> input_channels,
                const std::string& active_channel,
                const std::string& switch_channel)
      : drake_lcm_(drake_lcm), lcm_parser_(lcm_parser) {
    // Move simulator
    if (!diagram->get_name().empty()) {
      diagram_name_ = diagram->get_name();
    }
    diagram_ptr_ = diagram.get();
    simulator_ =
        std::make_unique<drake::systems::Simulator<double>>(std::move(diagram));

    // Create subscriber for the switch (in the case of multi-input)
    DRAKE_DEMAND(!input_channels.empty());
    if (input_channels.size() > 1) {
      DRAKE_DEMAND(!switch_channel.empty());
      switch_sub_ = std::make_unique<drake::lcm::Subscriber<SwitchMessageType>>(
          drake_lcm_, switch_channel);
    }

    // Create subscribers for inputs
    for (const auto& name : input_channels) {
      std::cout << "Constructing subscriber for " << name << std::endl;
      name_to_input_sub_map_.insert(std::make_pair(
          name, drake::lcm::Subscriber<InputMessageType>(drake_lcm_, name)));
    }

    // Make sure input_channels contains active_channel, and then set initial
    // active channel
    bool is_name_match = false;
    for (const auto& name : input_channels) {
      if (name.compare(active_channel) == 0) {
        is_name_match = true;
        break;
      }
    }
    DRAKE_DEMAND(is_name_match);

    active_channel_ = active_channel;
  };

  // Start simulating the diagram
  void Simulate(double end_time = std::numeric_limits<double>::infinity()) {
    // Get mutable contexts
    auto& diagram_context = simulator_->get_mutable_context();
    auto& lcm_parser_context = diagram_ptr_->GetMutableSubsystemContext(
        *lcm_parser_, &diagram_context);

    // Wait for the first message.
    drake::log()->info("Waiting for first lcm input message");
    LcmHandleSubscriptionsUntil(drake_lcm_, [&]() {
      return name_to_input_sub_map_.at(active_channel_).count() > 0;
    });

    // Initialize the context based on the first message.
    const double t0 =
        name_to_input_sub_map_.at(active_channel_).message().utime * 1e-6;
    diagram_context.SetTime(t0);
    auto& input_value = lcm_parser_->get_input_port(0).FixValue(
        &lcm_parser_context,
        name_to_input_sub_map_.at(active_channel_).message());

    // "Simulator" time
    double time = 0;  // initialize the current time with 0
    double message_time;

    drake::log()->info(diagram_name_ + " started");
    // Run the simulation until end_time
    while (time < end_time) {
      // Update the name of the active channel if there are multiple inputs
      if (switch_sub_ != nullptr) {
        if (switch_sub_->count() > 0) {
          // Check if the channel name is a key of the map. If it is, we update
          // the active channel name and clear switch_sub_'s message. If it is
          // not we do not update the active channel name.
          if (name_to_input_sub_map_.count(switch_sub_->message().channel) ==
              1) {
            active_channel_ = switch_sub_->message().channel;
          } else {
            std::cout << switch_sub_->message().channel << " doesn't exist\n";
          }
          switch_sub_->clear();
        }
      }

      // Wait for new InputMessageType messages and SwitchMessageType messages.
      bool is_new_input_message = false;
      bool is_new_switch_message = false;
      name_to_input_sub_map_.at(active_channel_).clear();
      LcmHandleSubscriptionsUntil(drake_lcm_, [&]() {
        if (name_to_input_sub_map_.at(active_channel_).count() > 0) {
          is_new_input_message = true;
        }
        if (switch_sub_ != nullptr) {
          if (switch_sub_->count() > 0) {
            is_new_switch_message = true;
          }
        }
        return is_new_input_message || is_new_switch_message;
      });

      // Update the diagram context when there is new input message
      if (is_new_input_message) {
        // Write the InputMessageType message into the context.
        input_value.GetMutableData()->set_value(
            name_to_input_sub_map_.at(active_channel_).message());

        // Get message time from the active channel to advance
        message_time =
            name_to_input_sub_map_.at(active_channel_).message().utime * 1e-6;
        // We cap the time from below just in case after we switch to a different
        // input channel, the message time from the new channel is smaller then
        // the current diagram time
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
          std::cout << "Difference is too large, resetting " + diagram_name_ +
              " time.\n";
          simulator_->get_mutable_context().SetTime(time);
        }

        simulator_->AdvanceTo(time);
        // Force-publish via the diagram
        diagram_ptr_->Publish(diagram_context);
      }
    }
  };

 private:
  drake::lcm::DrakeLcm* drake_lcm_;
  drake::systems::Diagram<double>* diagram_ptr_;
  const drake::systems::LeafSystem<double>* lcm_parser_;
  std::unique_ptr<drake::systems::Simulator<double>> simulator_;

  std::string diagram_name_ = "diagram";
  std::string active_channel_;
  std::unique_ptr<drake::lcm::Subscriber<SwitchMessageType>> switch_sub_ =
      nullptr;
  std::map<std::string, drake::lcm::Subscriber<InputMessageType>>
      name_to_input_sub_map_;
};

}  // namespace systems
}  // namespace dairlib