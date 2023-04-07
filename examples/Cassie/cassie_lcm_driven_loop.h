#pragma once

#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "dairlib/lcmt_controller_switch.hpp"

#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/serializer.h"

namespace dairlib {

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

/// Notice that diagram.ForcedPublish() is for dispatching the publish of
/// TriggerType::kForced type. In LcmPublisherSystem, both periodic and
/// per-step publishes are also forced publish.
/// https://github.com/RobotLocomotion/drake/blob/03fe7e4/systems/lcm/lcm_publisher_system.h#L54-L56
/// Therefore, in the case of periodic and per-step publishes, we should
/// not run diagram.ForcedPublish() after AdvanceTo(). Otherwise, we double
/// publish.
/// With the above things being said, the parameter is_forced_publish_ should be
/// set to true only when LcmPublisher is of TriggerType::kForced type and NOT
/// other types.

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
class CassieLcmDrivenLoop {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CassieLcmDrivenLoop)

  /// Constructor for single-input LcmDrivenLoop
  ///     @param drake_lcm DrakeLcm
  ///     @param diagram A Drake diagram
  ///     @param lcm_parser The LeafSystem of the diagram that parses the
  ///     incoming lcm message
  ///     @param input_channel The name of the input channel
  ///     @param is_forced_publish A flag which enables publishing via diagram.
  CassieLcmDrivenLoop(drake::lcm::DrakeLcm* drake_lcm,
                      std::unique_ptr<drake::systems::Diagram<double>> diagram,
                      const drake::systems::LeafSystem<double>* lcm_parser,
                      const std::string& input_channel, bool is_forced_publish)
      : CassieLcmDrivenLoop(drake_lcm, std::move(diagram), lcm_parser,
                            std::vector<std::string>(1, input_channel),
                            input_channel, "", is_forced_publish){};

  /// Constructor for multi-input LcmDrivenLoop
  ///     @param drake_lcm DrakeLcm
  ///     @param diagram A Drake diagram
  ///     @param lcm_parser The LeafSystem of the diagram that parses the
  ///     incoming lcm message
  ///     @param input_channels The names of the input channels
  ///     @param active_channel The name of the initial active input channel
  ///     @param switch_channel The name of the switch channel
  ///     @param is_forced_publish A flag which enables publishing via diagram.
  CassieLcmDrivenLoop(drake::lcm::DrakeLcm* drake_lcm,
                      std::unique_ptr<drake::systems::Diagram<double>> diagram,
                      const drake::systems::LeafSystem<double>* lcm_parser,
                      std::vector<std::string> input_channels,
                      const std::string& active_channel,
                      const std::string& switch_channel, bool is_forced_publish,
                      const std::string& backup_drive_channel = "")
      : drake_lcm_(drake_lcm),
        lcm_parser_(lcm_parser),
        is_forced_publish_(is_forced_publish) {
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

    if (!backup_drive_channel.empty()) {
      state_sub_ = std::make_unique<drake::lcm::Subscriber<lcmt_robot_output>>(
          drake_lcm_, backup_drive_channel);
      failure_sub_ =
          std::make_unique<drake::lcm::Subscriber<lcmt_controller_failure>>(
              drake_lcm_, "CONTROLLER_ERROR");
    }
  };

  /// Constructor for single-input LcmDrivenLoop without lcm_parser
  ///     @param drake_lcm DrakeLcm
  ///     @param diagram A Drake diagram
  ///     @param input_channel The name of the input channel
  ///     @param is_forced_publish A flag which enables publishing via diagram.
  /// The use case is that the user only need the time from lcm message.
  CassieLcmDrivenLoop(drake::lcm::DrakeLcm* drake_lcm,
                      std::unique_ptr<drake::systems::Diagram<double>> diagram,
                      const std::string& input_channel, bool is_forced_publish)
      : CassieLcmDrivenLoop(drake_lcm, std::move(diagram), nullptr,
                            std::vector<std::string>(1, input_channel),
                            input_channel, "", is_forced_publish){};

  // Getters for diagram and its context
  drake::systems::Diagram<double>* get_diagram() { return diagram_ptr_; }
  drake::systems::Context<double>& get_diagram_mutable_context() {
    return simulator_->get_mutable_context();
  }

  // Start simulating the diagram
  void Simulate(double end_time = std::numeric_limits<double>::infinity()) {
    // Get mutable contexts
    auto& diagram_context = simulator_->get_mutable_context();

    // Wait for the first message.
    drake::log()->info("Waiting for first lcm input message");
    LcmHandleSubscriptionsUntil(drake_lcm_, [&]() {
      return name_to_input_sub_map_.at(active_channel_).count() > 0;
    });

    // Initialize the context time.
    const double t0 =
        name_to_input_sub_map_.at(active_channel_).message().utime * 1e-6;
    diagram_context.SetTime(t0);

    // "Simulator" time
    double time = 0;  // initialize the current time with 0
    // Variable needed for the driven loop
    std::string previous_active_channel_name = active_channel_;

    // Run the simulation until end_time
    /// Structure of the code:
    ///  While() {
    ///    Wait for new InputMessageType and SwitchMessageType messages.
    ///
    ///    if(there is new InputMessageType message) {
    ///      Update diagram context and advance time.
    ///      Clear input channel messages.
    ///    }
    ///
    ///    if(there is new SwitchMessageType message) {
    ///      Update active input channel name.
    ///      Clear switch channel messages.
    ///      if(switch to a new input channel) {
    ///         Clear message in new active input channel.
    ///      }
    ///    }
    ///  }
    drake::log()->info(diagram_name_ + " started");
    while (time < end_time) {
      // Wait for new InputMessageType messages and SwitchMessageType messages.
      bool is_new_input_message = false;
      bool is_new_switch_message = false;
      bool is_new_state_message = false;
      LcmHandleSubscriptionsUntil(drake_lcm_, [&]() {
        if (name_to_input_sub_map_.at(active_channel_).count() > 0) {
          is_new_input_message = true;
        }
        if (switch_sub_ != nullptr) {
          if (switch_sub_->count() > 0) {
            is_new_switch_message = true;
          }
        }
        // Check the backup signal used to drive the system
        if (state_sub_ != nullptr) {
          if (state_sub_->count() > 0) {
            if (state_sub_->message().utime * 1e-6 - last_input_msg_time_ >
                0.1) {
              too_long_between_input_messages_ = true;
            }
            is_new_state_message = too_long_between_input_messages_;
          }
        }
        if (failure_sub_ != nullptr) {
          if (failure_sub_->count() > 0) {
            if (failure_sub_->message().controller_channel ==
                    "GLOBAL_CHANNEL" &&
                failure_sub_->message().error_code != 0) {
              too_long_between_input_messages_ = true;
            }
          }
        }

        return is_new_input_message || is_new_switch_message ||
               is_new_state_message;
      });

      // Update the diagram context when there is new input message
      if (is_new_input_message || too_long_between_input_messages_) {
        // Write the InputMessageType message into the context if lcm_parser is
        // provided
        if (lcm_parser_ != nullptr) {
          lcm_parser_->get_input_port(0).FixValue(
              &(diagram_ptr_->GetMutableSubsystemContext(*lcm_parser_,
                                                         &diagram_context)),
              name_to_input_sub_map_.at(active_channel_).message());
        }

        // Get message time from the active channel to advance
        if (is_new_input_message) {
          time =
              name_to_input_sub_map_.at(active_channel_).message().utime * 1e-6;
          last_input_msg_time_ = time;
        } else {
          // use the robot state to advance
          if (state_sub_->message().utime * 1e-6 - last_input_msg_time_ >
              0.005) {
            time = state_sub_->message().utime * 1e-6;
          }
          state_sub_->clear();
        }

        // Check if we are very far ahead or behind
        // (likely due to a restart of the driving clock)
        if (time > simulator_->get_context().get_time() + 1.0 ||
            time < simulator_->get_context().get_time()) {
          if (time > simulator_->get_context().get_time() + 1.0) {
            std::cout << "input message time greater than driven loop time"
                      << std::endl;
          }
          if (time < simulator_->get_context().get_time()) {
            std::cout << "input message time less than driven loop time"
                      << std::endl;
          }
          std::cout << diagram_name_ + " time is "
                    << simulator_->get_context().get_time()
                    << ", but stepping to " << time << std::endl;
          std::cout << "Difference is too large, resetting " + diagram_name_ +
                           " time.\n";
          simulator_->get_mutable_context().SetTime(time);
          simulator_->Initialize();
        }

        simulator_->AdvanceTo(time);
        diagram_ptr_->CalcForcedUnrestrictedUpdate(
            diagram_context, &diagram_context.get_mutable_state());
        diagram_ptr_->CalcForcedDiscreteVariableUpdate(
            diagram_context, &diagram_context.get_mutable_discrete_state());
        if (is_forced_publish_) {
          // Force-publish via the diagram
          diagram_ptr_->ForcedPublish(diagram_context);
        }

        // Clear messages in the current input channel
        name_to_input_sub_map_.at(active_channel_).clear();
      }

      // Update the name of the active channel if there are multiple inputs and
      // there is new switch message
      if (is_new_switch_message) {
        // Check if the channel name is a key of the map. If it is, we update
        // the active channel name and clear switch_sub_'s message. If it is
        // not we do not update the active channel name.
        if (name_to_input_sub_map_.count(switch_sub_->message().channel) == 1) {
          active_channel_ = switch_sub_->message().channel;
        } else {
          std::cout << switch_sub_->message().channel << " doesn't exist\n";
        }

        // Advancing the simulator here ensure that the switch message is
        // received in the leaf systems without the encountering a race
        // condition when constructing a separate LcmSubscriberSystem that
        // listens to the same channel. Advancing the simulator, ensures that
        // the LCM message used here successfully arrives at the input port of
        // the other LcmSubscriberSystem
        simulator_->AdvanceTo(time);
        diagram_ptr_->CalcForcedUnrestrictedUpdate(
            diagram_context, &diagram_context.get_mutable_state());
        diagram_ptr_->CalcForcedDiscreteVariableUpdate(
            diagram_context, &diagram_context.get_mutable_discrete_state());
        if (is_forced_publish_) {
          // Force-publish via the diagram
          diagram_ptr_->ForcedPublish(diagram_context);
        }

        // Clear messages in the switch channel
        switch_sub_->clear();

        // Clear messages in the new input channel if we just switched input
        // channel in the current loop
        if (previous_active_channel_name.compare(active_channel_) != 0) {
          name_to_input_sub_map_.at(active_channel_).clear();
        }
      }
      previous_active_channel_name = active_channel_;
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
  std::unique_ptr<drake::lcm::Subscriber<lcmt_robot_output>> state_sub_;
  std::unique_ptr<drake::lcm::Subscriber<lcmt_controller_failure>> failure_sub_;

  bool is_forced_publish_;
  bool too_long_between_input_messages_ = false;
  double last_input_msg_time_;
};

}  // namespace dairlib