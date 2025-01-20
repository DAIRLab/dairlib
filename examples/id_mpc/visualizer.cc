
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "systems/visualization/lcm_visualization_systems.h"
#include "systems/plant_visualizer.h"
#include "systems/robot_lcm_systems.h"

#include "drake/geometry/meshcat.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/analysis/simulator.h"

namespace dairlib::systems {

using drake::systems::Simulator;
using drake::systems::lcm::LcmSubscriberSystem;

int DoMain() {

  std::string state_channel = "CASSIE_STATE_SIMULATION";
  std::string mpc_channel = "ID_MPC";
  std::string urdf = "examples/Cassie/urdf/cassie_fixed_spring_conservative.urdf";

  drake::systems::DiagramBuilder<double> builder;

  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();

  auto state_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<lcmt_robot_output>(state_channel, lcm));
  auto mpc_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<lcmt_timestamped_saved_traj>(mpc_channel, lcm));
  auto plant_visualizer = builder.AddSystem<PlantVisualizer>(urdf);
  auto state_receiver = builder.AddSystem<RobotOutputReceiver>(
      plant_visualizer->get_plant());

  auto mpc_visualizer = builder.AddSystem<LcmConfigurationDrawer>(
      plant_visualizer->get_meshcat(), urdf, "q");

  builder.Connect(*state_sub, *state_receiver);
  builder.Connect(*state_receiver, *plant_visualizer);
  builder.Connect(*mpc_sub, *mpc_visualizer);

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  auto& subscriber_context = state_sub->GetMyMutableContextFromRoot(context.get());
  state_receiver->InitializeSubscriberPositions(plant_visualizer->get_plant(),
                                                subscriber_context);

  auto simulator =
  std::make_unique<Simulator<double>>(*diagram, std::move(context));
  simulator->set_publish_every_time_step(false);
  simulator->set_publish_at_initialization(false);
  simulator->set_target_realtime_rate(
      1.0);  // may need to change this to param.real_time_rate?
  simulator->Initialize();

  simulator->AdvanceTo(std::numeric_limits<double>::infinity());
  return 0;
}

}

int main(int argc, char**argv) {
  return dairlib::systems::DoMain();
}