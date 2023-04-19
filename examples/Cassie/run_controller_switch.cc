#include <iostream>
#include <string>

#include <drake/systems/lcm/lcm_interface_system.h>
#include <gflags/gflags.h>

#include "dairlib/lcmt_controller_switch.hpp"
#include "dairlib/lcmt_robot_output.hpp"

#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/serializer.h"

namespace dairlib {

using drake::systems::TriggerType;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::TriggerTypeSet;

DEFINE_string(channel_x, "CASSIE_STATE_DISPATCHER",
              "The name of the channel which receives state");
DEFINE_string(switch_channel, "INPUT_SWITCH",
              "The name of the channel which sends the channel name that "
              "dispatcher_in listens to");
DEFINE_string(new_channel, "PD_CONTROL",
              "The name of the new lcm channel that dispatcher_in listens to "
              "after switch");
DEFINE_int32(n_publishes, 1,
             "The simulation gets updated until it publishes the channel name "
             "n_publishes times");
DEFINE_int32(n_period_delay, -1,
             "the number of periods before we start publishing the new channel "
             "name. If the value is non-positive, the channel name is published"
             "right after the start of the program.");
DEFINE_double(fsm_period, -1.0, " the period of TimeBasedFiniteStateMachine");
DEFINE_double(fsm_offset, 0.0,
              "a constant that's used to determined the publish time see the "
              "documentation below for details");
DEFINE_double(blend_duration, 1.0,
              "Duration to blend efforts between previous and current "
              "controller command");

/// This program is a one-time-use switch that tells dispatcher_robot_in which
/// channel to listen to. It publishes the lcm message,
/// dairlib::lcmt_controller_switch, which contains a string of the channel
/// name.

/// The program is extended so that it could be used with
/// `TimeBasedFiniteStateMachine`. More specifically, we want the string
/// assigment starts at the moment when `TimeBasedFiniteStateMachine` switches
/// to a new state (because we don't want to switch to a new controller when the
/// new controller is still in a middle of a discrete state). This requires that
/// the users provide two (or three) more arguments to the constructors besides
/// `new_channel`:
///   @param n_period_delay, the number of periods before we start publishing
///   the new channel name
///   @param fsm_period, the period of `TimeBasedFiniteStateMachine`
///   @param fsm_offset, a constant that's used to determined the publish time
///   (see below for details)
///
/// Let t0 be the time of the simulator/robot when we started running the
/// simulation of the diagram in this program. And let t_current be the current
/// diagram time.
/// If n_period_delay is positive, this diagram starts publishing the channel
/// name after
///   t_current >= (floor(t0/fsm_period) + n_period_delay) * fsm_period +
///   fsm_offset.
/// If n_period_delay is not positive, then it starts publishing the name right
/// after the start of the program. i.e.
///   t_current >= t0

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Ensure that if (n_period_delay >= 0), then (period > 0).
  DRAKE_DEMAND((FLAGS_n_period_delay < 0) || (FLAGS_fsm_period > 0));
  // offset has to be positive
  DRAKE_DEMAND(FLAGS_fsm_offset >= 0);

  // Build the diagram
  drake::systems::DiagramBuilder<double> builder;
  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>("udpm://239.255.76.67:7667?ttl=0");
  auto name_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_controller_switch>(
          FLAGS_switch_channel, lcm, TriggerTypeSet({TriggerType::kForced})));
  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("switch publisher"));

  // Create simulator
  drake::systems::Diagram<double>* diagram_ptr = owned_diagram.get();
  drake::systems::Simulator<double> simulator(std::move(owned_diagram));
  auto& diagram_context = simulator.get_mutable_context();

  // Create subscriber for lcm driven loop
  drake::lcm::Subscriber<dairlib::lcmt_robot_output> input_sub(lcm,
                                                               FLAGS_channel_x);

  // Wait for the first message and initialize the context time..
  drake::log()->info("Waiting for first lcm input message");
  LcmHandleSubscriptionsUntil(lcm, [&]() { return input_sub.count() > 0; });
  const double t0 = input_sub.message().utime * 1e-6;
  diagram_context.SetTime(t0);

  // Set the threshold time
  double t_threshold = t0;
  if (FLAGS_n_period_delay > 0) {
    t_threshold = (floor(t0 / FLAGS_fsm_period) + FLAGS_n_period_delay) *
        FLAGS_fsm_period +
        FLAGS_fsm_offset;
  }
  // Create output message
  dairlib::lcmt_controller_switch msg;
  msg.channel = FLAGS_new_channel;
  msg.blend_duration = FLAGS_blend_duration;

  // Run the simulation until it publishes the channel name `n_publishes` times
  drake::log()->info(diagram_ptr->get_name() + " started");
  int pub_count = 0;
  while (pub_count < FLAGS_n_publishes) {
    // Wait for input message.
    input_sub.clear();
    LcmHandleSubscriptionsUntil(lcm, [&]() { return input_sub.count() > 0; });

    // Get message time from the input channel
    double t_current = input_sub.message().utime * 1e-6;
    if (t_current >= t_threshold) {
      std::cout << "publish at t = " << t_current << std::endl;

      msg.utime = (int)input_sub.message().utime;
      name_pub->get_input_port().FixValue(
          &(diagram_ptr->GetMutableSubsystemContext(*name_pub,
                                                    &diagram_context)),
          msg);

      // Force-publish via the diagram
      /// We don't need AdvanceTo(time) because we manually force publish lcm
      /// message, and there is nothing in the diagram that needs to be updated.
      diagram_ptr->ForcedPublish(diagram_context);

      pub_count++;
    }
  }
  drake::log()->info(diagram_ptr->get_name() + " ended");

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }
