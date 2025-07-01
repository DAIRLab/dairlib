#include <fstream>
#include <iostream>

#include <c3/lcmt_contact_forces.hpp>
#include <dairlib/lcmt_c3_state.hpp>
#include <dairlib/lcmt_timestamped_saved_traj.hpp>
#include <drake/common/find_resource.h>
#include <drake/common/yaml/yaml_io.h>
#include <drake/geometry/drake_visualizer.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/geometry/meshcat_visualizer_params.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/lcm/lcm_interface_system.h>
#include <drake/systems/lcm/lcm_subscriber_system.h>
#include <drake/systems/primitives/multiplexer.h>
#include <drake/systems/rendering/multibody_position_to_geometry_pose.h>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "common/find_resource.h"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/plate-balancing/parameters/lcm_channel_config.h"
#include "examples/plate-balancing/parameters/plate_balancing_config.h"
#include "examples/plate-balancing/parameters/simulation_config.h"
#include "examples/plate-balancing/parameters/simulation_scene_config.h"
#include "multibody/com_pose_system.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"
#include "systems/trajectory_optimization/lcm_trajectory_systems.h"
#include "systems/visualization/lcm_visualization_systems.h"

using drake::geometry::MeshcatVisualizer;
using drake::math::RigidTransform;
using drake::math::RigidTransformd;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::rendering::MultibodyPositionToGeometryPose;

using Eigen::Vector3d;

DEFINE_string(plate_balancing_config,
              "examples/plate-balancing/config/plate_balancing_config.yaml",
              "Path to the plate balancing configuration YAML file.");

namespace dairlib {
namespace examples {
namespace plate_balancing {

// Main function for the plate balancing visualizer.
int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Load configuration files.
  PlateBalancingConfig main_config =
      drake::yaml::LoadYamlFile<PlateBalancingConfig>(
          FLAGS_plate_balancing_config);
  SimulationConfig sim_params = drake::yaml::LoadYamlFile<SimulationConfig>(
      main_config.simulation_config_file);
  LcmChannelConfig lcm_channel_params =
      drake::yaml::LoadYamlFile<LcmChannelConfig>(
          main_config.lcm_simulation_settings_file);
  SimulationSceneConfig scene_params =
      drake::yaml::LoadYamlFile<SimulationSceneConfig>(
          main_config.get_simulation_scene_config_file());

  // Create a Drake diagram builder.
  DiagramBuilder<double> builder;

  // Add scene graph.
  auto& scene_graph = *builder.AddSystem<drake::geometry::SceneGraph>();
  scene_graph.set_name("scene_graph");

  // Add MultibodyPlant.
  MultibodyPlant<double> plant(0.0);

  // Create a parser for loading models.
  Parser parser(&plant, &scene_graph);
  parser.SetAutoRenaming(true);

  // Load models from URDF files.
  drake::multibody::ModelInstanceIndex franka_index =
      parser.AddModelsFromUrl(sim_params.franka_model)[0];
  drake::multibody::ModelInstanceIndex end_effector_index =
      parser.AddModels(FindResourceOrThrow(sim_params.end_effector_model))[0];
  drake::multibody::ModelInstanceIndex tray_index =
      parser.AddModels(FindResourceOrThrow(sim_params.tray_model))[0];
  drake::multibody::ModelInstanceIndex object_index =
      parser.AddModels(FindResourceOrThrow(sim_params.object_model))[0];
  multibody::AddFlatTerrain(&plant, &scene_graph, 1.0, 1.0);

  // Weld frames to position the robot and end effector.
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"),
                   RigidTransform<double>(drake::math::RotationMatrix<double>(),
                                          Eigen::VectorXd::Zero(3)));
  plant.WeldFrames(plant.GetFrameByName("panda_link7"),
                   plant.GetFrameByName("plate", end_effector_index),
                   RigidTransform<double>(drake::math::RotationMatrix<double>(),
                                          sim_params.tool_attachment_frame));

  // Load and weld environment models.
  std::vector<drake::multibody::ModelInstanceIndex> environment_model_indices;
  environment_model_indices.resize(scene_params.environment_models.size());
  for (int i = 0; i < scene_params.environment_models.size(); ++i) {
    environment_model_indices[i] = parser.AddModels(
        FindResourceOrThrow(scene_params.environment_models[i]))[0];
    RigidTransform<double> T_E_W =
        RigidTransform<double>(drake::math::RollPitchYaw<double>(
                                   scene_params.environment_orientations[i]),
                               scene_params.environment_positions[i]);
    plant.WeldFrames(plant.world_frame(),
                     plant.GetFrameByName("base", environment_model_indices[i]),
                     T_E_W);
  }

  // Finalize the MultibodyPlant.
  plant.Finalize();

  // Add LCM interface system.
  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();

  // Create LCM subscribers for robot and object states.
  auto franka_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(
          lcm_channel_params.franka_state_channel, lcm));
  auto tray_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.tray_state_channel, lcm));
  auto object_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.object_state_channel, lcm));

  // Create state receivers to convert LCM messages to MultibodyPlant states.
  auto franka_state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant, franka_index);
  auto tray_state_receiver =
      builder.AddSystem<systems::ObjectStateReceiver>(plant, tray_index);
  auto object_state_receiver =
      builder.AddSystem<systems::ObjectStateReceiver>(plant, object_index);

  // Create passthrough systems to extract relevant state vectors.
  auto franka_passthrough = builder.AddSystem<systems::SubvectorPassThrough>(
      franka_state_receiver->get_output_port(0).size(), 0,
      plant.num_positions(franka_index));
  auto robot_time_passthrough =
      builder.AddSystem<systems::SubvectorPassThrough>(
          franka_state_receiver->get_output_port(0).size(),
          franka_state_receiver->get_output_port(0).size() - 1, 1);
  auto tray_passthrough = builder.AddSystem<systems::SubvectorPassThrough>(
      tray_state_receiver->get_output_port(0).size(), 0,
      plant.num_positions(tray_index));
  auto object_passthrough = builder.AddSystem<systems::SubvectorPassThrough>(
      tray_state_receiver->get_output_port(0).size(), 0,
      plant.num_positions(object_index));

  // Create a multiplexer to combine the robot and object states.
  std::vector<int> input_sizes = {plant.num_positions(franka_index),
                                  plant.num_positions(tray_index),
                                  plant.num_positions(object_index)};
  auto mux =
      builder.AddSystem<drake::systems::Multiplexer<double>>(input_sizes);

  // Create LCM subscribers for trajectories and forces.
  auto trajectory_sub_actor = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.c3_actor_channel, lcm));
  auto trajectory_sub_tray = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.c3_object_channel, lcm));
  auto trajectory_sub_force =
      builder.AddSystem(LcmSubscriberSystem::Make<c3::lcmt_contact_forces>(
          lcm_channel_params.c3_force_channel, lcm));

  // Create LCM subscribers for C3 states.
  auto c3_state_actual_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_c3_state>(
          lcm_channel_params.c3_actual_state_channel, lcm));
  auto c3_state_target_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_c3_state>(
          lcm_channel_params.c3_target_state_channel, lcm));

  // Create a system to convert MultibodyPlant positions to geometry poses.
  auto to_pose =
      builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant);

  // Configure Meshcat visualizer.
  drake::geometry::MeshcatVisualizerParams params;
  params.publish_period = 1.0 / sim_params.visualizer_publish_rate;
  auto meshcat = std::make_shared<drake::geometry::Meshcat>();
  meshcat->SetCameraPose(scene_params.camera_pose, scene_params.camera_target);

  // Visualize workspace limits.
  if (sim_params.visualize_workspace) {
    double width = sim_params.world_x_limits[main_config.scene_index][1] -
                   sim_params.world_x_limits[main_config.scene_index][0];
    double depth = sim_params.world_y_limits[main_config.scene_index][1] -
                   sim_params.world_y_limits[main_config.scene_index][0];
    double height = sim_params.world_z_limits[main_config.scene_index][1] -
                    sim_params.world_z_limits[main_config.scene_index][0];
    Vector3d workspace_center = {
        0.5 * (sim_params.world_x_limits[main_config.scene_index][1] +
               sim_params.world_x_limits[main_config.scene_index][0]),
        0.5 * (sim_params.world_y_limits[main_config.scene_index][1] +
               sim_params.world_y_limits[main_config.scene_index][0]),
        0.5 * (sim_params.world_z_limits[main_config.scene_index][1] +
               sim_params.world_z_limits[main_config.scene_index][0])};
    meshcat->SetObject("c3_state/workspace",
                       drake::geometry::Box(width, depth, height),
                       {1, 0, 0, 0.2});
    meshcat->SetTransform("c3_state/workspace",
                          RigidTransformd(workspace_center));
  }

  // Visualize center of mass plan trajectories.
  if (sim_params.visualize_center_of_mass_plan) {
    auto trajectory_drawer_actor =
        builder.AddSystem<systems::LcmTrajectoryDrawer>(
            meshcat, "end_effector_position_target");
    auto trajectory_drawer_object =
        builder.AddSystem<systems::LcmTrajectoryDrawer>(
            meshcat, "object_position_target");
    trajectory_drawer_actor->SetLineColor(drake::geometry::Rgba({1, 0, 0, 1}));
    trajectory_drawer_object->SetLineColor(drake::geometry::Rgba({0, 0, 1, 1}));
    trajectory_drawer_actor->SetNumSamples(40);
    trajectory_drawer_object->SetNumSamples(40);
    builder.Connect(trajectory_sub_actor->get_output_port(),
                    trajectory_drawer_actor->get_input_port_trajectory());
    builder.Connect(trajectory_sub_tray->get_output_port(),
                    trajectory_drawer_object->get_input_port_trajectory());
  }

  // Visualize pose traces.
  if (sim_params.visualize_pose_trace) {
    auto object_pose_drawer = builder.AddSystem<systems::LcmPoseDrawer>(
        meshcat, FindResourceOrThrow("examples/plate-balancing/urdf/tray.sdf"),
        "object_position_target", "object_orientation_target");
    auto end_effector_pose_drawer = builder.AddSystem<systems::LcmPoseDrawer>(
        meshcat, FindResourceOrThrow(sim_params.end_effector_model),
        "end_effector_position_target", "end_effector_orientation_target");

    builder.Connect(trajectory_sub_tray->get_output_port(),
                    object_pose_drawer->get_input_port_trajectory());
    builder.Connect(trajectory_sub_actor->get_output_port(),
                    end_effector_pose_drawer->get_input_port_trajectory());
  }

  // Visualize C3 states.
  if (sim_params.visualize_c3_object_state ||
      sim_params.visualize_c3_end_effector_state) {
    auto c3_target_drawer = builder.AddSystem<systems::LcmC3TargetDrawer>(
        meshcat, sim_params.visualize_c3_object_state,
        sim_params.visualize_c3_end_effector_state);
    builder.Connect(c3_state_actual_sub->get_output_port(),
                    c3_target_drawer->get_input_port_c3_state_actual());
    builder.Connect(c3_state_target_sub->get_output_port(),
                    c3_target_drawer->get_input_port_c3_state_target());
  }

  // Visualize C3 forces.
  if (sim_params.visualize_c3_forces) {
    auto end_effector_force_drawer = builder.AddSystem<systems::LcmForceDrawer>(
        meshcat, "end_effector_position_target", "end_effector_force_target",
        "lcs_force_trajectory");
    builder.Connect(
        trajectory_sub_actor->get_output_port(),
        end_effector_force_drawer->get_input_port_actor_trajectory());
    builder.Connect(
        trajectory_sub_force->get_output_port(),
        end_effector_force_drawer->get_input_port_force_trajectory());
    builder.Connect(robot_time_passthrough->get_output_port(),
                    end_effector_force_drawer->get_input_port_robot_time());
  }

  // Connect systems.
  builder.Connect(franka_passthrough->get_output_port(),
                  mux->get_input_port(0));
  builder.Connect(tray_passthrough->get_output_port(), mux->get_input_port(1));
  builder.Connect(object_passthrough->get_output_port(),
                  mux->get_input_port(2));
  builder.Connect(*mux, *to_pose);
  builder.Connect(
      to_pose->get_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(*franka_state_receiver, *franka_passthrough);
  builder.Connect(*franka_state_receiver, *robot_time_passthrough);
  builder.Connect(*tray_state_receiver, *tray_passthrough);
  builder.Connect(*object_state_receiver, *object_passthrough);
  builder.Connect(*franka_state_sub, *franka_state_receiver);
  builder.Connect(*tray_state_sub, *tray_state_receiver);
  builder.Connect(*object_state_sub, *object_state_receiver);

  // Add Meshcat visualizer.
  auto visualizer = &MeshcatVisualizer<double>::AddToBuilder(
      &builder, scene_graph, meshcat, std::move(params));

  // Build the diagram.
  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  // Initialize state receivers with subscriber positions.
  auto& franka_state_sub_context =
      diagram->GetMutableSubsystemContext(*franka_state_sub, context.get());
  auto& tray_state_sub_context =
      diagram->GetMutableSubsystemContext(*tray_state_sub, context.get());
  auto& object_state_sub_context =
      diagram->GetMutableSubsystemContext(*object_state_sub, context.get());
  franka_state_receiver->InitializeSubscriberPositions(
      plant, franka_state_sub_context);
  tray_state_receiver->InitializeSubscriberPositions(plant,
                                                     tray_state_sub_context);
  object_state_receiver->InitializeSubscriberPositions(
      plant, object_state_sub_context);

  // Create and configure the simulator.
  auto simulator =
      std::make_unique<Simulator<double>>(*diagram, std::move(context));
  simulator->set_publish_every_time_step(false);
  simulator->set_publish_at_initialization(false);
  simulator->set_target_realtime_rate(1.0);
  simulator->Initialize();

  // Run the simulation.
  simulator->AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace plate_balancing
}  // namespace examples
}  // namespace dairlib

// Main entrypoint.
int main(int argc, char* argv[]) {
  return dairlib::examples::plate_balancing::do_main(argc, argv);
}
