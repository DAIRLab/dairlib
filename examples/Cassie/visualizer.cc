#include <gflags/gflags.h>

#include "drake/systems/framework/diagram_builder.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"
#include "systems/primitives/subvector_pass_through.h"

#include "dairlib/lcmt_robot_output.hpp"
#include "systems/robot_lcm_systems.h"
#include "examples/Cassie/cassie_utils.h"
#include "multibody/multibody_utils.h"

namespace dairlib {

DEFINE_bool(floating_base, true, "Fixed or floating base model");

using std::endl;
using std::cout;
using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using dairlib::systems::SubvectorPassThrough;
using drake::systems::Simulator;
using dairlib::systems::RobotOutputReceiver;
using drake::systems::rendering::MultibodyPositionToGeometryPose;
using drake::systems::lcm::LcmSubscriberSystem;

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  drake::systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  MultibodyPlant<double> plant;

  if (FLAGS_floating_base) {
    multibody::addFlatTerrain(&plant, &scene_graph, .8, .8);
  }

  addCassieMultibody(&plant, &scene_graph, FLAGS_floating_base);
  plant.Finalize();

  drake::lcm::DrakeLcm lcm;

  const std::string channel_x = "CASSIE_STATE";

  // Create state receiver.
  auto state_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(channel_x, &lcm));
  auto state_receiver = builder.AddSystem<RobotOutputReceiver>(plant);
  builder.Connect(*state_sub, *state_receiver);

  auto passthrough = builder.AddSystem<SubvectorPassThrough>(
    state_receiver->get_output_port(0).size(), 0, plant.num_positions());
  builder.Connect(*state_receiver, *passthrough);

  auto to_pose =
      builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant);
  builder.Connect(*passthrough, *to_pose);
  builder.Connect(to_pose->get_output_port(), scene_graph.get_source_pose_port(
    plant.get_source_id().value()));

  drake::geometry::ConnectDrakeVisualizer(&builder, scene_graph);

  // state_receiver->set_publish_period(1.0/30.0);  // framerate

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

  lcm.StartReceiveThread();

  drake::log()->info("visualizer started");

  stepper->StepTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::do_main(argc, argv);
}
