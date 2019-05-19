#include <gflags/gflags.h>

#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/analysis/simulator.h"
#include "systems/primitives/subvector_pass_through.h"

#include "dairlib/lcmt_robot_output.hpp"
#include "systems/robot_lcm_systems.h"
#include "examples/Cassie/cassie_utils.h"

namespace dairlib {

DEFINE_bool(floating_base, true, "Fixed or floating base model");

using std::endl;
using std::cout;
using dairlib::systems::SubvectorPassThrough;
using drake::systems::Simulator;
using dairlib::systems::RobotOutputReceiver;

int doMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  RigidBodyTree<double> tree;

  if (!FLAGS_floating_base) {
    buildCassieTree(tree);
  } else {
    buildCassieTree(tree, "examples/Cassie/urdf/cassie_v2.urdf",
                    drake::multibody::joints::kQuaternion);
    const double terrain_size = 100;
    const double terrain_depth = 0.20;
    drake::multibody::AddFlatTerrainToWorld(&tree, terrain_size, terrain_depth);
  }

  drake::systems::DiagramBuilder<double> builder;
  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();

  const std::string channel_x = "CASSIE_STATE";

  // Create state receiver.
  auto state_sub = builder.AddSystem(
                     drake::systems::lcm::LcmSubscriberSystem::Make <
                     dairlib::lcmt_robot_output > (channel_x, lcm));
  auto state_receiver = builder.AddSystem<RobotOutputReceiver>(tree);
  builder.Connect(state_sub->get_output_port(),
                  state_receiver->get_input_port(0));

  auto publisher =
    builder.AddSystem<drake::systems::DrakeVisualizer>(tree, lcm);

  auto passthrough = builder.AddSystem<SubvectorPassThrough>(
                       state_receiver->get_output_port(0).size(),
                       0,
                       publisher->get_input_port(0).size());

  builder.Connect(state_receiver->get_output_port(0),
                  passthrough->get_input_port());
  builder.Connect(passthrough->get_output_port(),
                  publisher->get_input_port(0));

  publisher->set_publish_period(1.0 / 30.0);


  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  /// Use the simulator to drive at a fixed rate
  /// If set_publish_every_time_step is true, this publishes twice
  /// Set realtime rate. Otherwise, runs as fast as possible
  auto stepper = std::make_unique<Simulator<double>>(*diagram,
                 std::move(context));
  stepper->set_publish_every_time_step(false);
  stepper->set_publish_at_initialization(false);
  stepper->set_target_realtime_rate(1.0);
  stepper->Initialize();

  drake::log()->info("visualizer started");

  stepper->StepTo(std::numeric_limits<double>::infinity());

  return 0;
}


}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::doMain(argc, argv);
}


