#include <string>
#include <gflags/gflags.h>
#include "dairlib/lcmt_controller_switch.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "systems/framework/lcm_driven_loop.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace dairlib {

using drake::systems::TriggerType;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::TriggerTypeSet;

DEFINE_string(channel_x, "CASSIE_STATE",
              "The name of the channel which receives state");
DEFINE_string(controller_channel, "PD_CONTROLLER",
              "The name of the lcm channel that dispatcher_in listens to");
DEFINE_int32(n_fsm_period, -1,
             "the number of state-switch after the start of the"
             "simulator containing this diagram"
             "Negative value means no time delay");
DEFINE_double(fsm_period, -1.0,
              "has to be the same as `duration_per_state` in "
              "`TimeBasedFiniteStateMachine`");
DEFINE_double(fsm_offset, 0.0,
              "has to be the same as `time_shift` in "
              "`TimeBasedFiniteStateMachine`");

/// This diagram publishes a string which tells dispatcher_robot_in which
/// channel to listen to.
/// This is a one-time-use switch. If the user wants to publish a different
/// channel name, then he/she has to rerun this program.

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  drake::systems::DiagramBuilder<double> builder;

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");

  // Create forced publisher
  auto name_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_controller_switch>(
          "INPUT_SWITCH", &lcm_local, TriggerTypeSet({TriggerType::kForced})));

  // Create the diagram
  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("switch publisher"));

  // Move simulator
  drake::systems::Diagram<double>* diagram_ptr_ = owned_diagram.get();
  drake::systems::Simulator<double> simulator_(std::move(owned_diagram));

  // Create subscriber
  using InputMessageType = dairlib::lcmt_robot_output;
  drake::lcm::Subscriber<InputMessageType> input_sub_(
          &lcm_local, FLAGS_channel_x);

  // Get mutable contexts
  auto& diagram_context = simulator_.get_mutable_context();

  // Wait for the first message.
  drake::log()->info("Waiting for first lcm input message");
  LcmHandleSubscriptionsUntil(&lcm_local,
                              [&]() { return input_sub_.count() > 0; });

  // Initialize the context time.
  const double t0 = input_sub_.message().utime * 1e-6;
  diagram_context.SetTime(t0);

  // Run the simulation until it publishes `n_publishes` times
  int n_publishes = 10;
  int total_pub = 0;
  drake::log()->info(diagram_ptr_->get_name() + " started");
  while (total_pub < n_publishes) {
    // Wait for input message.
    input_sub_.clear();
    LcmHandleSubscriptionsUntil(&lcm_local,
                                [&]() { return input_sub_.count() > 0; });

    // Get message time from the input channel
    double current_time = input_sub_.message().utime * 1e-6;
    if (current_time >=
        (floor(t0 / FLAGS_fsm_period) + FLAGS_n_fsm_period) * FLAGS_fsm_period +
            FLAGS_fsm_offset) {
      dairlib::lcmt_controller_switch msg;
      msg.channel = FLAGS_controller_channel;

      name_pub->get_input_port().FixValue(
          &(diagram_ptr_->GetMutableSubsystemContext(*name_pub,
                                                     &diagram_context)),
          msg);

      // Force-publish via the diagram
      /// We don't need AdvanceTo(time) because we manually force publish lcm
      /// message, and there is nothing in the diagram that needs to be updated.
      diagram_ptr_->Publish(diagram_context);

      total_pub++;
    }
  }

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }