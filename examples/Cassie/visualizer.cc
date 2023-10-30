#include <gflags/gflags.h>

#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "multibody/com_pose_system.h"
#include "multibody/multibody_utils.h"
#include "multibody/stepping_stone_utils.h"
#include "multibody/visualization_utils.h"
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

DEFINE_bool(floating_base, true, "Fixed or floating base model");
DEFINE_bool(com, true, "Visualize the COM as a sphere");
DEFINE_bool(
    com_ground, true,
    "If com=true, sets whether the COM should be shown on the ground (z=0)"
    " or at the correct height.");
DEFINE_string(channel, "CASSIE_STATE_DISPATCHER",
              "LCM channel for receiving state. "
              "Use CASSIE_STATE_SIMULATION to get state from simulator, and "
              "use CASSIE_STATE_DISPATCHER to get state from state estimator");
// Terrain
DEFINE_double(ground_incline, 0, "in radians. Positive is walking downhill");
DEFINE_string(stepping_stone_yaml, "",
              "yaml file with boxy stepping stones to visualize");

using dairlib::systems::RobotOutputReceiver;
using dairlib::systems::SubvectorPassThrough;
using drake::geometry::DrakeVisualizer;
using drake::geometry::SceneGraph;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::multibody::MultibodyPlant;
using drake::multibody::RigidBody;
using drake::systems::Simulator;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::rendering::MultibodyPositionToGeometryPose;

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  drake::systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  MultibodyPlant<double> plant(0.0);
  if (!FLAGS_stepping_stone_yaml.empty()) {
    multibody::AddSteppingStonesToSimFromYaml(
        &plant,
        &scene_graph,
        FLAGS_stepping_stone_yaml,
        1.0);
  }

  AddCassieMultibody(&plant, &scene_graph, FLAGS_floating_base,
                     "examples/Cassie/urdf/cassie_v2_shells.urdf");
  plant.Finalize();

  /// Set visualizer lcm url to ttl=0 to avoid sending DrakeViewerDraw
  /// messages to Cassie
  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(
      "udpm://239.255.76.67:7667?ttl=0");

  // Create state receiver.
  auto state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(
          FLAGS_channel, lcm));
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

  // *******Add COM visualization**********
  auto ball_plant = multibody::ConstructBallPlant(&scene_graph);
  if (FLAGS_com) {
    // connect
    auto q_passthrough = builder.AddSystem<SubvectorPassThrough>(
        state_receiver->get_output_port(0).size(), 0, plant.num_positions());
    builder.Connect(state_receiver->get_output_port(0),
                    q_passthrough->get_input_port());
    auto rbt_passthrough = builder.AddSystem<multibody::ComPoseSystem>(plant);

    auto ball_to_pose =
        builder.AddSystem<MultibodyPositionToGeometryPose<double>>(*ball_plant);
    builder.Connect(*q_passthrough, *rbt_passthrough);
    if (FLAGS_com_ground) {
      builder.Connect(rbt_passthrough->get_xy_com_output_port(),
                      ball_to_pose->get_input_port());
    } else {
      builder.Connect(rbt_passthrough->get_com_output_port(),
                      ball_to_pose->get_input_port());
    }
    builder.Connect(
        ball_to_pose->get_output_port(),
        scene_graph.get_source_pose_port(ball_plant->get_source_id().value()));
  }

  DrakeVisualizer<double>::AddToBuilder(&builder, scene_graph, lcm);

  // state_receiver->set_publish_period(1.0/30.0);  // framerate

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
