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

/// This program is a one-time-use switch that tells dispatcher_robot_in which
/// channel to listen to. It publishes the lcm message, PublishMessageType,
/// which contains a string of the channel name.

/// The program is extended so that it could be used with
/// `TimeBasedFiniteStateMachine`. More specifically, the string assigment
/// starts when `TimeBasedFiniteStateMachine` switches to a new state (because
/// we don't want to switch to a new controller when the new controller is in a
/// middle of a discrete state). This requires that the users provide two (or
/// three) more arguments to the constructors besides `controller_channel`:
///   @param n_fsm_period, the number of state-switch after the start of the
///   simulator containing this diagram
///   @param fsm_period, has to be the same as `duration_per_state` in
///   `TimeBasedFiniteStateMachine`
///   @param fsm_offset, has to be the same as `time_shift` in
///   `TimeBasedFiniteStateMachine`
///
/// Let t0 be the time of the simulator/robot when we started running the
/// diagram containing this class. And let t_current be the current diagram
/// time. This class outputs the channel name only when
///   t_current >= (floor(t0/fsm_period) + n_fsm_period) * fsm_period +
///   fsm_offset.
/// That is, this thread starts outputting the new channel name when
/// `TimeBasedFiniteStateMachine` switches to a new state for the
/// `n_fsm_period`-th times after it starts running.

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Ensure that if (n_fsm_period >= 0), then (period > 0).
  DRAKE_DEMAND((FLAGS_n_fsm_period < 0) || (FLAGS_fsm_period > 0));
  // offset has to be positive
  DRAKE_DEMAND(FLAGS_fsm_offset >= 0);

  // Parameters
  using PublishMessageType = dairlib::lcmt_controller_switch;
  using TriggerMessageType = dairlib::lcmt_robot_output;
  std::string switch_channel = "INPUT_SWITCH";
  int n_publishes = 10;
  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");

  // Build the diagram
  drake::systems::DiagramBuilder<double> builder;
  auto name_pub =
      builder.AddSystem(LcmPublisherSystem::Make<PublishMessageType>(
          switch_channel, &lcm_local, TriggerTypeSet({TriggerType::kForced})));
  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("switch publisher"));

  // Create simulator
  drake::systems::Diagram<double>* diagram_ptr = owned_diagram.get();
  drake::systems::Simulator<double> simulator(std::move(owned_diagram));
  auto& diagram_context = simulator.get_mutable_context();

  // Create subscriber for lcm driven loop
  drake::lcm::Subscriber<TriggerMessageType> input_sub(&lcm_local,
                                                       FLAGS_channel_x);

  // Wait for the first message and initialize the context time..
  drake::log()->info("Waiting for first lcm input message");
  LcmHandleSubscriptionsUntil(&lcm_local,
                              [&]() { return input_sub.count() > 0; });
  const double t0 = input_sub.message().utime * 1e-6;
  diagram_context.SetTime(t0);

  // Set the threshold time
  double t_threshold = t0;
  if (FLAGS_n_fsm_period > 0) {
    t_threshold =
        (floor(t0 / FLAGS_fsm_period) + FLAGS_n_fsm_period) * FLAGS_fsm_period +
        FLAGS_fsm_offset;
  }
  // Create output message
  PublishMessageType msg;
  msg.channel = FLAGS_controller_channel;

  // Run the simulation until it publishes the channel name `n_publishes` times
  drake::log()->info(diagram_ptr->get_name() + " started");
  int pub_count = 0;
  while (pub_count < n_publishes) {
    // Wait for input message.
    input_sub.clear();
    LcmHandleSubscriptionsUntil(&lcm_local,
                                [&]() { return input_sub.count() > 0; });

    // Get message time from the input channel
    double t_current = input_sub.message().utime * 1e-6;
    if (t_current >= t_threshold) {
      name_pub->get_input_port().FixValue(
          &(diagram_ptr->GetMutableSubsystemContext(*name_pub,
                                                    &diagram_context)),
          msg);

      // Force-publish via the diagram
      /// We don't need AdvanceTo(time) because we manually force publish lcm
      /// message, and there is nothing in the diagram that needs to be updated.
      diagram_ptr->Publish(diagram_context);

      pub_count++;
    }
  }
  drake::log()->info(diagram_ptr->get_name() + " ended");

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }