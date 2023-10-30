#include <gflags/gflags.h>

#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_mpc_debug.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/perceptive_locomotion/systems/alip_mpfc_meshcat_visualization_driver.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"

#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/meshcat_visualizer_params.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

namespace dairlib {

DEFINE_string(channel_x, "CASSIE_STATE_DISPATCHER", "State channel");
DEFINE_string(channel_mpc, "ALIP_MINLP_DEBUG", "mpc_debug_channel");
DEFINE_string(channel_terrain, "", "lcm channel with processed footholds from "
                                   "lcm translator");

using dairlib::systems::RobotOutputReceiver;
using dairlib::systems::SubvectorPassThrough;
using dairlib::perceptive_locomotion::AlipMPFCMeshcatVisualizationDriver;
using drake::geometry::SceneGraph;
using drake::math::RigidTransformd;
using drake::multibody::MultibodyPlant;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::multibody::UnitInertia;
using drake::systems::Simulator;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::rendering::MultibodyPositionToGeometryPose;

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  drake::systems::DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");
  MultibodyPlant<double> plant(0.0);
  AddCassieMultibody(&plant, &scene_graph, true, "examples/Cassie/urdf/cassie_v2_shells.urdf");
  plant.Finalize();

  /// Set visualizer lcm url to ttl=0 to avoid sending DrakeViewerDraw
  /// messages to Cassie
  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(
      "udpm://239.255.76.67:7667?ttl=0");

  // Create state receiver.
  auto state_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(
          FLAGS_channel_x, lcm));
  auto mpc_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_mpc_debug>(
          FLAGS_channel_mpc, lcm));
  auto state_receiver = builder.AddSystem<RobotOutputReceiver>(plant);
  builder.Connect(*state_sub, *state_receiver);

  auto passthrough = builder.AddSystem<SubvectorPassThrough>(
      state_receiver->get_output_port(0).size(), 0, plant.num_positions());
  builder.Connect(*state_receiver, *passthrough);

  auto to_pose =
      builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant);
  builder.Connect(*passthrough, *to_pose);
  builder.Connect(
      to_pose->get_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));

  drake::geometry::MeshcatVisualizerParams params;
  params.publish_period = 1.0/60.0;
  auto meshcat = std::make_shared<drake::geometry::Meshcat>();
  auto visualizer = &drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
      &builder, scene_graph, meshcat, std::move(params));
  auto foothold_vis = builder.AddSystem<AlipMPFCMeshcatVisualizationDriver>(meshcat, plant);
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
      *state_sub, context.get());
  state_receiver->InitializeSubscriberPositions(plant, state_sub_context);

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