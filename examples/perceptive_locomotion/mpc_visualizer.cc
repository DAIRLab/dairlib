#include <gflags/gflags.h>

#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_mpc_debug.hpp"
#include "examples/perceptive_locomotion/systems/alip_mpfc_meshcat_visualizer.h"
#include "systems/robot_lcm_systems.h"
#include "systems/plant_visualizer.h"

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace dairlib {

DEFINE_string(channel_x, "CASSIE_STATE_DISPATCHER", "State channel");
DEFINE_string(channel_mpc, "ALIP_MINLP_DEBUG", "mpc_debug_channel");
DEFINE_string(channel_terrain, "", "lcm channel with processed footholds from "
                                   "lcm translator");

using dairlib::systems::RobotOutputReceiver;
using dairlib::systems::SubvectorPassThrough;
using dairlib::systems::PlantVisualizer;
using dairlib::perceptive_locomotion::AlipMPFCMeshcatVisualizer;
using drake::multibody::MultibodyPlant;
using drake::systems::Simulator;
using drake::systems::lcm::LcmSubscriberSystem;

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  drake::systems::DiagramBuilder<double> builder;

  const std::string urdf{"examples/Cassie/urdf/cassie_v2_shells.urdf"};
  auto plant_visualizer = builder.AddSystem<PlantVisualizer>(urdf);

  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(
      "udpm://239.255.76.67:7667?ttl=0");

  // Create state receiver.
  auto state_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(
          FLAGS_channel_x, lcm)
  );
  auto mpc_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_mpc_debug>(
          FLAGS_channel_mpc, lcm)
  );
  auto state_receiver = builder.AddSystem<RobotOutputReceiver>(
      plant_visualizer->get_plant()
  );
  builder.Connect(*state_sub, *state_receiver);
  builder.Connect(*state_receiver, *plant_visualizer);

  auto foothold_vis = builder.AddSystem<AlipMPFCMeshcatVisualizer>(
      plant_visualizer->get_meshcat(), plant_visualizer->get_plant()
  );
  builder.Connect(mpc_sub->get_output_port(),
                  foothold_vis->get_input_port_mpc());
  builder.Connect(state_receiver->get_output_port(),
                  foothold_vis->get_input_port_state());

  if (not FLAGS_channel_terrain.empty()) {;
    auto foothold_subscriber = builder.AddSystem(
        LcmSubscriberSystem::Make<dairlib::lcmt_foothold_set>(
            FLAGS_channel_terrain, lcm));
    builder.Connect(foothold_subscriber->get_output_port(),
                    foothold_vis->get_input_port_terrain());
  }


  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  /// Initialize the lcm subscriber to avoid triggering runtime errors
  /// during initialization due to internal checks
  /// (unit quaternion check in MultibodyPositionToGeometryPose
  /// internal calculations)
  auto& state_sub_context = diagram->GetMutableSubsystemContext(
      *state_sub, context.get()
  );
  state_receiver->InitializeSubscriberPositions(
      plant_visualizer->get_plant(), state_sub_context
  );

  /// Use the simulator to drive at a fixed rate
  /// If set_publish_every_time_step is true, this publishes twice
  /// Set realtime rate. Otherwise, runs as fast as possible
  auto stepper =
      std::make_unique<Simulator<double>>(*diagram, std::move(context));
  stepper->set_publish_every_time_step(false);
  stepper->set_publish_at_initialization(false);
  stepper->set_target_realtime_rate(1.0);
  stepper->Initialize();

  drake::log()->info("visualizer started");

  stepper->AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }