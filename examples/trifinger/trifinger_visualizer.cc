#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "dairlib/lcmt_object_state.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "multibody/com_pose_system.h"
#include "multibody/multibody_utils.h"
#include "parameters/trifinger_lcm_channels.h"
#include "parameters/trifinger_sim_params.h"
#include "systems/lcm_visualization_systems.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/meshcat_visualizer_params.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

DEFINE_string(sim_parameters,
              "examples/trifinger/parameters/trifinger_sim_params.yaml",
              "Filepath to simulation configs");

DEFINE_string(lcm_channels,
              "examples/trifinger/parameters/lcm_channels_hardware.yaml",
              "Filepath containing lcm channels");

namespace dairlib {
using dairlib::systems::ObjectStateReceiver;
using dairlib::systems::RobotOutputReceiver;
using dairlib::systems::SubvectorPassThrough;
using drake::geometry::SceneGraph;
using drake::math::RigidTransformd;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Multiplexer;
using drake::systems::Simulator;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::rendering::MultibodyPositionToGeometryPose;

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  auto sim_params =
      drake::yaml::LoadYamlFile<TrifingerSimParams>(FLAGS_sim_parameters);
  auto lcm_channels =
      drake::yaml::LoadYamlFile<TrifingerLcmChannels>(FLAGS_lcm_channels);

  drake::systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  MultibodyPlant<double> plant(0.0);
  multibody::AddFlatTerrain(&plant, &scene_graph, .8, .8, {0, 0, 1}, true);

  // Adds the URDF model to the trifinger plant.
  Parser parser(&plant, &scene_graph);
  drake::multibody::ModelInstanceIndex trifinger_index =
      parser.AddModels(FindResourceOrThrow(sim_params.trifinger_model))[0];
  drake::multibody::ModelInstanceIndex cube_index =
      parser.AddModels(FindResourceOrThrow(sim_params.cube_model))[0];

  // Fixes the trifinger base to the world frame.
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link"),
                   drake::math::RigidTransform<double>::Identity());
  plant.Finalize();

  /// Set visualizer lcm url to ttl=0 to avoid sending DrakeViewerDraw
  /// messages to Cassie
  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(
      "udpm://239.255.76.67:7667?ttl=1");

  // Create trifinger and cube state receiver.
  auto trifinger_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(
          lcm_channels.trifinger_state_channel, lcm));
  auto trifinger_state_receiver =
      builder.AddSystem<RobotOutputReceiver>(plant, trifinger_index);
  builder.Connect(*trifinger_state_sub, *trifinger_state_receiver);

  auto cube_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_object_state>(
          lcm_channels.cube_state_channel, lcm));
  auto cube_state_receiver =
      builder.AddSystem<ObjectStateReceiver>(plant, cube_index);
  builder.Connect(*cube_state_sub, *cube_state_receiver);

  auto trifinger_passthrough = builder.AddSystem<SubvectorPassThrough>(
      trifinger_state_receiver->get_output_port(0).size(), 0,
      plant.num_positions(trifinger_index));
  builder.Connect(*trifinger_state_receiver, *trifinger_passthrough);

  auto cube_passthrough = builder.AddSystem<SubvectorPassThrough>(
      cube_state_receiver->get_output_port(0).size(), 0,
      plant.num_positions(cube_index));
  builder.Connect(*cube_state_receiver, *cube_passthrough);

  std::vector<int> input_sizes = {plant.num_positions(trifinger_index),
                                  plant.num_positions(cube_index)};
  auto mux =
      builder.AddSystem<drake::systems::Multiplexer<double>>(input_sizes);

  builder.Connect(trifinger_passthrough->get_output_port(),
                  mux->get_input_port(0));
  builder.Connect(cube_passthrough->get_output_port(), mux->get_input_port(1));

  auto to_pose =
      builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant);
  builder.Connect(*mux, *to_pose);
  builder.Connect(
      to_pose->get_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));

  drake::geometry::MeshcatVisualizerParams params;
  params.publish_period = 1.0 / sim_params.visualizer_publish_rate;
  auto meshcat = std::make_shared<drake::geometry::Meshcat>();

  // set desired camera pose.
  Eigen::Vector3d camera_in_world(0.4, 0.4, 0.4);
  Eigen::Vector3d camera_target = Eigen::Vector3d::Zero();
  meshcat->SetCameraPose(camera_in_world, camera_target);

  [[maybe_unused]] auto visualizer =
      &drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
          &builder, scene_graph, meshcat, std::move(params));
  // trifinger_state_receiver->set_publish_period(1.0/30.0);  // framerate

  // Add target visualization system
  auto cube_target_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_object_state>(
          lcm_channels.cube_target_channel, lcm));
  auto target_drawer =
      builder.AddSystem<dairlib::systems::LcmCubeTargetDrawer>(meshcat);
  builder.Connect(cube_target_sub->get_output_port(0),
                  target_drawer->get_input_port(0));

  auto diagram = builder.Build();

  auto context = diagram->CreateDefaultContext();

  /// Initialize the lcm subscriber to avoid triggering runtime errors
  /// during initialization due to internal checks
  /// (unit quaternion check in MultibodyPositionToGeometryPose
  /// internal calculations)
  auto& trifinger_state_sub_context =
      diagram->GetMutableSubsystemContext(*trifinger_state_sub, context.get());
  auto& cube_state_sub_context =
      diagram->GetMutableSubsystemContext(*cube_state_sub, context.get());
  trifinger_state_receiver->InitializeSubscriberPositions(
      plant, trifinger_state_sub_context);
  cube_state_receiver->InitializeSubscriberPositions(plant,
                                                     cube_state_sub_context);

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

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
