#include <iostream>

#include <dairlib/lcmt_c3_forces.hpp>
#include <dairlib/lcmt_c3_state.hpp>
#include <dairlib/lcmt_timestamped_saved_traj.hpp>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/primitives/multiplexer.h>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "common/find_resource.h"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/franka/parameters/franka_lcm_channels.h"
#include "examples/franka/parameters/franka_sim_params.h"
#include "examples/franka/parameters/franka_sim_scene_params.h"
#include "multibody/com_pose_system.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"
#include "systems/trajectory_optimization/lcm_trajectory_systems.h"
#include "systems/visualization/lcm_visualization_systems.h"

#include "drake/common/find_resource.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/meshcat_visualizer_params.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

namespace dairlib {

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::systems::ObjectStateReceiver;
using dairlib::systems::RobotOutputReceiver;
using dairlib::systems::SubvectorPassThrough;
using drake::geometry::DrakeVisualizer;
using drake::geometry::SceneGraph;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::multibody::MultibodyPlant;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::multibody::UnitInertia;
using drake::systems::Simulator;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::rendering::MultibodyPositionToGeometryPose;

using drake::math::RigidTransform;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;

DEFINE_string(lcm_channels,
              "examples/franka/parameters/lcm_channels_simulation.yaml",
              "Filepath containing lcm channels");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  FrankaSimParams sim_params = drake::yaml::LoadYamlFile<FrankaSimParams>(
      "examples/franka/parameters/franka_sim_params.yaml");
  FrankaLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<FrankaLcmChannels>(FLAGS_lcm_channels);
  FrankaSimSceneParams scene_params =
      drake::yaml::LoadYamlFile<FrankaSimSceneParams>(
          sim_params.sim_scene_file[sim_params.scene_index]);

  drake::systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  MultibodyPlant<double> plant(0.0);

  Parser parser(&plant, &scene_graph);
  parser.SetAutoRenaming(true);
  drake::multibody::ModelInstanceIndex franka_index =
      parser.AddModelsFromUrl(sim_params.franka_model)[0];
  drake::multibody::ModelInstanceIndex end_effector_index =
      parser.AddModels(FindResourceOrThrow(sim_params.end_effector_model))[0];
  drake::multibody::ModelInstanceIndex tray_index =
      parser.AddModels(FindResourceOrThrow(sim_params.tray_model))[0];
  drake::multibody::ModelInstanceIndex object_index =
      parser.AddModels(FindResourceOrThrow(sim_params.object_model))[0];
  multibody::AddFlatTerrain(&plant, &scene_graph, 1.0, 1.0);

  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  Vector3d franka_origin = Eigen::VectorXd::Zero(3);

  RigidTransform<double> R_X_W = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), franka_origin);
  RigidTransform<double> T_EE_W = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), sim_params.tool_attachment_frame);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"),
                   R_X_W);
  plant.WeldFrames(plant.GetFrameByName("panda_link7"),
                   plant.GetFrameByName("plate", end_effector_index), T_EE_W);

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

  plant.Finalize();

  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();

  // Create state receiver.
  auto franka_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(
          lcm_channel_params.franka_state_channel, lcm));
  auto tray_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.tray_state_channel, lcm));
  auto object_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.object_state_channel, lcm));
  auto franka_state_receiver =
      builder.AddSystem<RobotOutputReceiver>(plant, franka_index);
  auto tray_state_receiver =
      builder.AddSystem<ObjectStateReceiver>(plant, tray_index);
  auto object_state_receiver =
      builder.AddSystem<ObjectStateReceiver>(plant, object_index);

  auto franka_passthrough = builder.AddSystem<SubvectorPassThrough>(
      franka_state_receiver->get_output_port(0).size(), 0,
      plant.num_positions(franka_index));
  auto robot_time_passthrough = builder.AddSystem<SubvectorPassThrough>(
      franka_state_receiver->get_output_port(0).size(),
      franka_state_receiver->get_output_port(0).size() - 1, 1);
  auto tray_passthrough = builder.AddSystem<SubvectorPassThrough>(
      tray_state_receiver->get_output_port(0).size(), 0,
      plant.num_positions(tray_index));
  auto object_passthrough = builder.AddSystem<SubvectorPassThrough>(
      tray_state_receiver->get_output_port(0).size(), 0,
      plant.num_positions(object_index));

  std::vector<int> input_sizes = {plant.num_positions(franka_index),
                                  plant.num_positions(tray_index),
                                  plant.num_positions(object_index)};
  auto mux =
      builder.AddSystem<drake::systems::Multiplexer<double>>(input_sizes);

  auto trajectory_sub_actor = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.c3_actor_channel, lcm));
  auto trajectory_sub_tray = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.c3_object_channel, lcm));
  auto trajectory_sub_force =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_c3_forces>(
          lcm_channel_params.c3_force_channel, lcm));

  auto c3_state_actual_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_c3_state>(
          lcm_channel_params.c3_actual_state_channel, lcm));
  auto c3_state_target_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_c3_state>(
          lcm_channel_params.c3_target_state_channel, lcm));
  auto to_pose =
      builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant);

  drake::geometry::MeshcatVisualizerParams params;
  params.publish_period = 1.0 / sim_params.visualizer_publish_rate;
  auto meshcat = std::make_shared<drake::geometry::Meshcat>();
  auto visualizer = &drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
      &builder, scene_graph, meshcat, std::move(params));
  meshcat->SetCameraPose(scene_params.camera_pose,
                         scene_params.camera_target);

  if (sim_params.visualize_workspace) {
    double width = sim_params.world_x_limits[sim_params.scene_index][1] -
                   sim_params.world_x_limits[sim_params.scene_index][0];
    double depth = sim_params.world_y_limits[sim_params.scene_index][1] -
                   sim_params.world_y_limits[sim_params.scene_index][0];
    double height = sim_params.world_z_limits[sim_params.scene_index][1] -
                    sim_params.world_z_limits[sim_params.scene_index][0];
    Vector3d workspace_center = {
        0.5 * (sim_params.world_x_limits[sim_params.scene_index][1] +
               sim_params.world_x_limits[sim_params.scene_index][0]),
        0.5 * (sim_params.world_y_limits[sim_params.scene_index][1] +
               sim_params.world_y_limits[sim_params.scene_index][0]),
        0.5 * (sim_params.world_z_limits[sim_params.scene_index][1] +
               sim_params.world_z_limits[sim_params.scene_index][0])};
    meshcat->SetObject("c3_state/workspace",
                       drake::geometry::Box(width, depth, height),
                       {1, 0, 0, 0.2});
    meshcat->SetTransform("c3_state/workspace",
                          RigidTransformd(workspace_center));
  }
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

  if (sim_params.visualize_pose_trace) {
    auto object_pose_drawer = builder.AddSystem<systems::LcmPoseDrawer>(
        meshcat,
        FindResourceOrThrow("examples/franka/urdf/tray.sdf"),
        "object_position_target", "object_orientation_target");
    auto end_effector_pose_drawer = builder.AddSystem<systems::LcmPoseDrawer>(
        meshcat, FindResourceOrThrow(sim_params.end_effector_model),
        "end_effector_position_target", "end_effector_orientation_target");

    builder.Connect(trajectory_sub_tray->get_output_port(),
                    object_pose_drawer->get_input_port_trajectory());
    builder.Connect(trajectory_sub_actor->get_output_port(),
                    end_effector_pose_drawer->get_input_port_trajectory());
  }

  if (sim_params.visualize_c3_object_state || sim_params.visualize_c3_end_effector_state) {
    auto c3_target_drawer =
        builder.AddSystem<systems::LcmC3TargetDrawer>(meshcat, sim_params.visualize_c3_object_state, sim_params.visualize_c3_end_effector_state);
    builder.Connect(c3_state_actual_sub->get_output_port(),
                    c3_target_drawer->get_input_port_c3_state_actual());
    builder.Connect(c3_state_target_sub->get_output_port(),
                    c3_target_drawer->get_input_port_c3_state_target());
  }

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

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

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

  /// Use the simulator to drive at a fixed rate
  /// If set_publish_every_time_step is true, this publishes twice
  /// Set realtime rate. Otherwise, runs as fast as possible
  auto stepper =
      std::make_unique<Simulator<double>>(*diagram, std::move(context));
  stepper->set_publish_every_time_step(false);
  stepper->set_publish_at_initialization(false);
  stepper->set_target_realtime_rate(
      1.0);  // may need to change this to param.real_time_rate?
  stepper->Initialize();

  drake::log()->info("visualizer started");

  stepper->AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }
