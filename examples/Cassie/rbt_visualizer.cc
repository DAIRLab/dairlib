#include <gflags/gflags.h>

#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"

namespace dairlib {

using std::endl;
using std::cout;
using std::string;
using drake::systems::Simulator;
using dairlib::systems::SubvectorPassThrough;
using dairlib::systems::RobotOutputReceiver;
using drake::multibody::AddFlatTerrainToWorld;
using drake::multibody::joints::kFixed;
using drake::multibody::joints::kRollPitchYaw;

// RBT version of the cassie state visualizer.
// TODO: Remove this once multibody can support floating base simulations.

// Simulation flags
DEFINE_bool(floating_base, false, "Fixed or floating base model");
DEFINE_string(channel, "CASSIE_STATE", "LCM channel for receiving state.");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  RigidBodyTree<double> tree;
  string filename = "examples/Cassie/urdf/cassie_v2.urdf";

  if (FLAGS_floating_base) {
    buildCassieTree(tree, filename, kRollPitchYaw);
    AddFlatTerrainToWorld(&tree, 8, 0.05);
  } else {
    buildCassieTree(tree, filename, kFixed);
  }

  drake::systems::DiagramBuilder<double> builder;
  drake::lcm::DrakeLcm lcm;

  // Create state receiver.
  auto state_sub =
      builder.AddSystem(drake::systems::lcm::LcmSubscriberSystem::Make<
                        dairlib::lcmt_robot_output>(FLAGS_channel, &lcm));
  auto state_receiver = builder.AddSystem<RobotOutputReceiver>(tree);
  builder.Connect(state_sub->get_output_port(),
                  state_receiver->get_input_port(0));

  auto publisher =
      builder.AddSystem<drake::systems::DrakeVisualizer>(tree, &lcm);

  auto passthrough = builder.AddSystem<SubvectorPassThrough>(
      state_receiver->get_output_port(0).size(), 0,
      publisher->get_input_port(0).size());

  builder.Connect(state_receiver->get_output_port(0),
                  passthrough->get_input_port());
  builder.Connect(passthrough->get_output_port(), publisher->get_input_port(0));

  publisher->set_publish_period(1.0 / 30.0);

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  /// Use the simulator to drive at a fixed rate
  /// If set_publish_every_time_step is true, this publishes twice
  /// Set realtime rate. Otherwise, runs as fast as possible
  auto stepper =
      std::make_unique<Simulator<double>>(*diagram, std::move(context));
  stepper->set_publish_every_time_step(false);
  stepper->set_publish_at_initialization(false);
  stepper->set_target_realtime_rate(1.0);
  stepper->Initialize();

  lcm.StartReceiveThread();

  drake::log()->info("visualizer started");

  stepper->StepTo(std::numeric_limits<double>::infinity());

  return 0;
}
}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }
