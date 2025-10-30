#include <iostream>

#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "common/find_resource.h"
#include "dairlib/lcmt_robot_output.hpp"
#include "parameter_headers/lcm_channel_params.h"
#include "parameter_headers/visualizer_params.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "drake/common/find_resource.h"
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

namespace assembly {

static constexpr const char* kFrankaModel =
    "package://drake_models/franka_description/urdf/panda_arm_hand.urdf";
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::systems::RobotOutputReceiver;
using dairlib::systems::SubvectorPassThrough;
using drake::geometry::DrakeVisualizer;
using drake::geometry::SceneGraph;
using drake::math::RigidTransform;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;

using drake::math::RigidTransform;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::rendering::MultibodyPositionToGeometryPose;

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  BeltAssemblyVisualizerParams vis_params =
      drake::yaml::LoadYamlFile<BeltAssemblyVisualizerParams>(
          "examples/belt_assembly/parameters/visualizer_params.yaml");
  RoundBeltLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<RoundBeltLcmChannels>(
          "examples/belt_assembly/parameters/lcm_channels_simulation.yaml");

  drake::systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  // ----- Build the visualizer plant -----
  // TODO: This code is duplicated from the simulation plant. We should refactor
  // this.
  MultibodyPlant<double> plant(0.0);
  Parser parser(&plant, &scene_graph);
  parser.SetAutoRenaming(true);

  // Add the task scene model which includes the table and the franka mount.
  // Note: the task scene model has no collision geometry; only the pulley
  // models include it.
  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  ModelInstanceIndex round_belt_task_scene_index =
      parser.AddModels(dairlib::FindResourceOrThrow(
          "examples/belt_assembly/urdf/round_belt_task/"
          "round_belt_task_scene.urdf"))[0];
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("round_belt_task_scene",
                                        round_belt_task_scene_index),
                   X_WI);
  // Add pulley models
  ModelInstanceIndex task_board_index =
      parser.AddModels(dairlib::FindResourceOrThrow(
          "examples/belt_assembly/urdf/round_belt_task/round_belt_task_board/"
          "round_belt_task_board.sdf"))[0];
  RigidTransform<double> task_board_pose =
      RigidTransform<double>(drake::math::RollPitchYaw<double>(0, 0, 1.57079),
                             drake::Vector3<double>(0.68585, -0.192, 0.00543));
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("board", task_board_index),
                   task_board_pose);

  // Add franka arm model
  ModelInstanceIndex franka_index = parser.AddModelsFromUrl(kFrankaModel)[0];
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"),
                   X_WI);
  plant.Finalize();

  // ----- Construct LCM subscriber to the franka state -----
  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();
  auto franka_state_receiver =
      builder.AddSystem<RobotOutputReceiver>(plant, franka_index);

  auto franka_passthrough = builder.AddSystem<SubvectorPassThrough>(
      franka_state_receiver->get_output_port(0).size(), 0,
      plant.num_positions(franka_index));

  auto franka_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(
          lcm_channel_params.franka_state_channel, lcm));

  auto to_pose =
      builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant);

  drake::geometry::MeshcatVisualizerParams illustration_params;
  illustration_params.publish_period = 1.0 / vis_params.visualizer_publish_rate;
  illustration_params.role = drake::geometry::Role::kIllustration;
  illustration_params.prefix = "visualizer";

  drake::geometry::MeshcatVisualizerParams proximity_params;
  proximity_params.publish_period = 1.0 / vis_params.visualizer_publish_rate;
  proximity_params.role = drake::geometry::Role::kProximity;
  proximity_params.prefix = "proximity";
  proximity_params.visible_by_default = false;
  auto meshcat = std::make_shared<drake::geometry::Meshcat>();
  meshcat->SetCameraPose(vis_params.camera_pose, vis_params.camera_target);

  builder.Connect(franka_passthrough->get_output_port(),
                  to_pose->get_input_port());
  builder.Connect(
      to_pose->get_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(*franka_state_receiver, *franka_passthrough);
  builder.Connect(*franka_state_sub, *franka_state_receiver);

  drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
      &builder, scene_graph, meshcat, std::move(illustration_params));
  drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
      &builder, scene_graph, meshcat, std::move(proximity_params));
  auto diagram = builder.Build();
  diagram->set_name("round_belt_visualizer");
  dairlib::DrawAndSaveDiagramGraph(*diagram);
  auto context = diagram->CreateDefaultContext();

  auto& franka_state_sub_context =
      diagram->GetMutableSubsystemContext(*franka_state_sub, context.get());

  franka_state_receiver->InitializeSubscriberPositions(
      plant, franka_state_sub_context);

  /// Use the simulator to drive at a fixed rate
  /// If set_publish_every_time_step is true, this publishes twice
  auto simulator =
      std::make_unique<Simulator<double>>(*diagram, std::move(context));
  simulator->set_publish_every_time_step(false);
  simulator->set_publish_at_initialization(false);
  simulator->set_target_realtime_rate(1.0);
  simulator->Initialize();
  drake::log()->info("Visualizer started");
  simulator->AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}
}  // namespace assembly

int main(int argc, char* argv[]) { return assembly::DoMain(argc, argv); }
