#include <string>

#include <gflags/gflags.h>
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

#include "systems/framework/controller_channel_sender.h"
#include "dairlib/lcmt_controller_switch.hpp"

namespace dairlib {

using drake::systems::lcm::LcmPublisherSystem;

DEFINE_double(end_time, std::numeric_limits<double>::infinity(),
              "End time of simulation");
DEFINE_double(publish_rate, 1000, "Publishing frequency (Hz)");
DEFINE_string(controller_channel, "PD_CONTROLLER",
              "The name of the lcm channel that dispatcher_in listens to");

/// This diagram publishes a string which tells dispatcher_robot_in which
/// channel to listen to.

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  drake::systems::DiagramBuilder<double> builder;

  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();

  // Create publisher
  auto channel_sender = builder.AddSystem<
      ControllerChannelSender<dairlib::lcmt_controller_switch>>(
          FLAGS_controller_channel);
  auto name_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_controller_switch>(
          "INPUT_SWITCH", lcm, 1.0 / FLAGS_publish_rate));
  builder.Connect(channel_sender->get_output_port(0),
                  name_pub->get_input_port());

  auto diagram = builder.Build();
  drake::systems::Simulator<double> simulator(*diagram);

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(1.0);
  simulator.Initialize();

  simulator.AdvanceTo(FLAGS_end_time);
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }