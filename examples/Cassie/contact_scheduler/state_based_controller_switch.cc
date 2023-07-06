#include <iostream>
#include <string>

#include <gflags/gflags.h>
#include <dairlib/lcmt_contact_timing.hpp>

#include "dairlib/lcmt_controller_switch.hpp"
#include "dairlib/lcmt_robot_output.hpp"

#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/serializer.h"

namespace dairlib {

using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;

DEFINE_string(channel_x, "CASSIE_STATE_DISPATCHER",
              "The name of the channel which receives state");
DEFINE_string(switch_channel, "INPUT_SWITCH",
              "The name of the channel which sends the channel name that "
              "dispatcher_in listens to");
DEFINE_string(new_channel, "PD_CONTROL",
              "The name of the new lcm channel that dispatcher_in listens to "
              "after switch");
DEFINE_string(contact_schedule_channel, "CONTACT_TIMING",
              "The name of the lcm channel that publishes contact timings");
DEFINE_double(trigger_state, -1.0,
              " the period of TimeBasedFiniteStateMachine");
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
/// new controller is still in a middle of a discrete state). This requires that
/// the users provide two (or three) more arguments to the constructors besides
/// `new_channel`:
///   @param trigger_state, the FSM state to switch on`
///

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Parameters
  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");

  // Build the diagram
  drake::systems::DiagramBuilder<double> builder;
  auto name_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_controller_switch>(
          FLAGS_switch_channel, &lcm_local,
          TriggerTypeSet({TriggerType::kForced})));
  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("switch publisher"));

  // Create simulator
  drake::systems::Diagram<double>* diagram_ptr = owned_diagram.get();
  drake::systems::Simulator<double> simulator(std::move(owned_diagram));
  auto& diagram_context = simulator.get_mutable_context();

  // Create subscriber for lcm driven loop
  drake::lcm::Subscriber<dairlib::lcmt_robot_output> input_sub(&lcm_local,
                                                               FLAGS_channel_x);
  drake::lcm::Subscriber<dairlib::lcmt_contact_timing> timing_sub(&lcm_local,
                                                               FLAGS_contact_schedule_channel);

  // Wait for the first message and initialize the context time..
  drake::log()->info("Waiting for first lcm input message");
  LcmHandleSubscriptionsUntil(&lcm_local,
                              [&]() { return input_sub.count() > 0; });
  const double t0 = input_sub.message().utime * 1e-6;
  diagram_context.SetTime(t0);

  drake::log()->info(diagram_ptr->get_name() + " started");

  LcmHandleSubscriptionsUntil(&lcm_local,
                              [&]() { return timing_sub.count() > 0; });
  while(timing_sub.message().transition_states.back() != FLAGS_trigger_state){
    timing_sub.clear();
    LcmHandleSubscriptionsUntil(&lcm_local,
                                [&]() { return timing_sub.count() > 0; });
  }
  const double transition_time = timing_sub.message().transition_times.back() + FLAGS_fsm_offset;
  std::cout << "transition time: " << transition_time << std::endl;

  // Create output message
  dairlib::lcmt_controller_switch msg;
  msg.channel = FLAGS_new_channel;
  msg.blend_duration = FLAGS_blend_duration;


  int pub_count = 0;
  while (pub_count < 1) {
    // Wait for input message.
    input_sub.clear();
    LcmHandleSubscriptionsUntil(&lcm_local,
                                [&]() { return input_sub.count() > 0; });

    // Get message time from the input channel
    double t_current = input_sub.message().utime * 1e-6;
    if (t_current >= transition_time) {
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
