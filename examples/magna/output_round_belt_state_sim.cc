#include <drake/systems/analysis/simulator.h>

#include "dairlib/lcmt_round_belt_state.hpp"
#include "examples/magna/systems/state_estimation/drake_deformable_state_to_round_belt_state_converter.h"
#include "gflags/gflags.h"
#include "systems/system_utils.h"

#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_viewer_link_data.hpp"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace dairlib {
namespace examples {
namespace magna {

using dairlib::examples::magna::systems::state_estimation::
    DrakeDeformableStateToRoundBeltStateConverter;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;

DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");
DEFINE_int32(publish_rate, 10, "Publish rate for round belt state in Hz");
int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::systems::DiagramBuilder<double> builder;
  drake::lcm::DrakeLcm drake_lcm(FLAGS_lcm_url);
  auto lcm =
      builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(&drake_lcm);
  auto deformable_state_sub = builder.AddSystem(
      drake::systems::lcm::LcmSubscriberSystem::Make<
          drake::lcmt_viewer_link_data>("DRAKE_VIEWER_DEFORMABLE", lcm));

  auto round_belt_state_pub =
      builder.AddSystem(drake::systems::lcm::LcmPublisherSystem::Make<
                        dairlib::lcmt_round_belt_state>(
          "ROUND_BELT_STATE", lcm, TriggerTypeSet({TriggerType::kPeriodic}),
          1.0 / FLAGS_publish_rate));
  auto converter =
      builder.AddSystem<DrakeDeformableStateToRoundBeltStateConverter>();
  builder.Connect(deformable_state_sub->get_output_port(),
                  converter->get_input_port_deformable_state());
  builder.Connect(converter->get_output_port_round_belt_state(),
                  round_belt_state_pub->get_input_port());

  // Build diagram
  auto diagram = builder.Build();
  diagram->set_name("output_round_belt_state_sim");
  DrawAndSaveDiagramGraph(*diagram);
  auto context = diagram->CreateDefaultContext();
  auto simulator = std::make_unique<drake::systems::Simulator<double>>(
      *diagram, std::move(context));
  simulator->Initialize();
  simulator->set_target_realtime_rate(1.0);
  simulator->AdvanceTo(std::numeric_limits<double>::infinity());
  return 0;
}
}  // namespace magna
}  // namespace examples
}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::examples::magna::DoMain(argc, argv);
}