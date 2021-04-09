#include <string>

#include <gflags/gflags.h>

#include "dairlib/lcmt_target_standing_height.hpp"

#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/serializer.h"

namespace dairlib {

using drake::systems::TriggerType;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::TriggerTypeSet;

DEFINE_string(switch_channel, "MPC_SWITCH",
              "The name of the channel which sends the trigger value");
DEFINE_int32(n_publishes, 10,
             "The simulation gets updated until it publishes the channel name "
             "n_publishes times");


int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Parameters
  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");

  // Build the diagram
  drake::systems::DiagramBuilder<double> builder;
  auto name_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_target_standing_height>(
          FLAGS_switch_channel, &lcm_local,
          TriggerTypeSet({TriggerType::kForced})));
  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("switch publisher"));

  // Create simulator
  drake::systems::Diagram<double>* diagram_ptr = owned_diagram.get();
  drake::systems::Simulator<double> simulator(std::move(owned_diagram));
  auto& diagram_context = simulator.get_mutable_context();

  // Create output message
  dairlib::lcmt_target_standing_height msg;
  msg.timestamp = 0;  // doesn't matter
  msg.target_height = 1;  // greater 0.5

  for (int i = 0; i<FLAGS_n_publishes ; i++) {
    name_pub->get_input_port().FixValue(
        &(diagram_ptr->GetMutableSubsystemContext(*name_pub,
                                                  &diagram_context)),
        msg);
    diagram_ptr->Publish(diagram_context);
  }

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }
