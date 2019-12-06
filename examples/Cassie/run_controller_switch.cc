#include <string>
#include <gflags/gflags.h>
#include "dairlib/lcmt_controller_switch.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "systems/framework/controller_channel_sender.h"
#include "systems/framework/lcm_driven_loop.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace dairlib {

using drake::systems::lcm::LcmPublisherSystem;

DEFINE_double(end_time, std::numeric_limits<double>::infinity(),
              "End time of simulation");
DEFINE_double(publish_rate, 1000, "Publishing frequency (Hz)");
DEFINE_string(channel_x, "CASSIE_STATE",
              "The name of the channel which receives state");
DEFINE_string(controller_channel, "PD_CONTROLLER",
              "The name of the lcm channel that dispatcher_in listens to");
DEFINE_int32(n_fsm_period, -1,
             "Number of period (of the time-based finite state machine) with "
             "which we delay to publish the switch lcm."
             "Negative value means no time delay");
DEFINE_double(period, -1.0,
              "The period of the time-based finite state machine");

/// This diagram publishes a string which tells dispatcher_robot_in which
/// channel to listen to.
/// This is a one-time-use switch. If the user wants to publish a different
/// channel name, then he/she has to rerun this program.

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  drake::systems::DiagramBuilder<double> builder;

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");

  // Create publisher
  auto channel_sender =
      builder
          .AddSystem<ControllerChannelSender<dairlib::lcmt_controller_switch>>(
              FLAGS_controller_channel, FLAGS_n_fsm_period, FLAGS_period);
  auto name_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_controller_switch>(
          "INPUT_SWITCH", &lcm_local, 1.0 / FLAGS_publish_rate));
  builder.Connect(channel_sender->get_output_port(0),
                  name_pub->get_input_port());

  // Create the diagram
  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("switch publisher"));

  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm_local, std::move(owned_diagram), FLAGS_channel_x);
  loop.Simulate();

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }