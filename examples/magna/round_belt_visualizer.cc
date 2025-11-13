#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "common/find_resource.h"
#include "dairlib/lcmt_c3_state.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "examples/magna/systems/franka_hand/franka_hand_state_receiver.h"
#include "examples/magna/systems/visualization/c3_belt_state_drawer.h"
#include "examples/magna/systems/visualization/deformable_drawer.h"
#include "parameter_headers/lcm_channel_params.h"
#include "parameter_headers/visualizer_params.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"
#include "systems/visualization/lcm_visualization_systems.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/meshcat_visualizer_params.h"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/lcmt_viewer_link_data.hpp"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

namespace dairlib {
namespace examples {
namespace magna {

static constexpr const char* kFrankaModelWithHandActuation =
    "package://drake_models/franka_description/urdf/"
    "panda_arm_hand_with_long_fingers.urdf";
static constexpr const char* kFrankaModelWithoutHandActuation =
    "package://drake_models/franka_description/urdf/"
    "panda_arm_hand_with_fixed_long_fingers.urdf";
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::examples::magna::systems::franka_hand::FrankaHandStateReceiver;
using dairlib::systems::RobotOutputReceiver;
using dairlib::systems::SubvectorPassThrough;
using drake::geometry::SceneGraph;
using drake::math::RigidTransform;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::systems::Multiplexer;

using dairlib::examples::magna::systems::visualization::DeformableDrawer;
using drake::math::RigidTransform;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::rendering::MultibodyPositionToGeometryPose;

DEFINE_bool(is_simulation, true, "True for simulation, false for hardware");
DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  MagnaVisualizerParams vis_params =
      drake::yaml::LoadYamlFile<MagnaVisualizerParams>(
          "examples/magna/parameters/visualizer_params.yaml");
  MagnaLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<MagnaLcmChannels>(
          "examples/magna/parameters/lcm_channels.yaml");

  // ----- Build the visualizer plant -----
  drake::systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  // TODO: This code is duplicated from the simulation plant. We should refactor
  // this.
  MultibodyPlant<double> plant_vis(0.0);
  Parser parser(&plant_vis, &scene_graph);
  parser.SetAutoRenaming(true);

  // Add the task scene model which includes the table and the franka mount.
  // Note: the task scene model has no collision geometry; only the pulley
  // models include it.
  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  ModelInstanceIndex round_belt_task_scene_index = parser.AddModels(
      dairlib::FindResourceOrThrow("examples/magna/urdf/round_belt_task/"
                                   "round_belt_task_scene.urdf"))[0];
  plant_vis.WeldFrames(plant_vis.world_frame(),
                       plant_vis.GetFrameByName("round_belt_task_scene",
                                                round_belt_task_scene_index),
                       X_WI);
  // Add pulley models
  ModelInstanceIndex task_board_index = parser.AddModels(
      dairlib::FindResourceOrThrow("examples/magna/urdf/round_belt_task/"
                                   "round_belt_task_board.sdf"))[0];
  RigidTransform<double> task_board_pose =
      RigidTransform<double>(drake::math::RollPitchYaw<double>(0, 0, 1.57079),
                             drake::Vector3<double>(0.68585, -0.192, 0.00543));
  plant_vis.WeldFrames(plant_vis.world_frame(),
                       plant_vis.GetFrameByName("board", task_board_index),
                       task_board_pose);

  // Add franka arm model
  ModelInstanceIndex franka_index =
      parser.AddModelsFromUrl(kFrankaModelWithHandActuation)[0];
  plant_vis.WeldFrames(plant_vis.world_frame(),
                       plant_vis.GetFrameByName("panda_link0"), X_WI);
  plant_vis.Finalize();

  // ----- Create plant for receiving state -----
  // - Simulation: state includes arm + hand (dim 9), use plant with hand
  // - Hardware: arm state (dim 7) and hand state (dim 2) are separate, use
  // plant without hand
  MultibodyPlant<double> plant_for_state_receiving(0.0);
  Parser parser_for_state_receiving(&plant_for_state_receiving, nullptr);
  parser_for_state_receiving.SetAutoRenaming(true);
  ModelInstanceIndex franka_state_receiving_index;
  franka_state_receiving_index = parser_for_state_receiving.AddModelsFromUrl(
      FLAGS_is_simulation ? kFrankaModelWithHandActuation
                          : kFrankaModelWithoutHandActuation)[0];
  plant_for_state_receiving.WeldFrames(
      plant_for_state_receiving.world_frame(),
      plant_for_state_receiving.GetFrameByName("panda_link0"), X_WI);
  plant_for_state_receiving.Finalize();

  // ----- Construct LCM subscriber to the franka state -----
  auto lcm =
      builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(FLAGS_lcm_url);
  auto franka_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(
          lcm_channel_params.franka_state_channel, lcm));
  auto franka_state_receiver = builder.AddSystem<RobotOutputReceiver>(
      plant_for_state_receiving, franka_state_receiving_index);
  builder.Connect(*franka_state_sub, *franka_state_receiver);

  // ----- Only for Hardware -----
  // Combine arm state (7 dims) and hand state (2 dims) for visualization
  SubvectorPassThrough<double>* franka_passthrough_for_vis = nullptr;
  if (!FLAGS_is_simulation) {
    auto franka_hand_state_sub = builder.AddSystem(
        LcmSubscriberSystem::Make<drake::lcmt_schunk_wsg_status>(
            lcm_channel_params.franka_hand_state_channel, lcm));
    auto franka_hand_state_receiver =
        builder.AddSystem<FrankaHandStateReceiver>();

    // Combine arm (7) + hand (2) = 9 for visualization
    auto franka_combined_mux =
        builder.AddSystem<Multiplexer<double>>(std::vector<int>{
            plant_for_state_receiving.num_positions(
                franka_state_receiving_index),
            franka_hand_state_receiver->get_output_port(0).size()});

    // Extract only positions from arm state for visualization (7 positions)
    auto franka_arm_positions_passthrough =
        builder.AddSystem<SubvectorPassThrough>(
            franka_state_receiver->get_output_port(0).size(), 0,
            plant_for_state_receiving.num_positions(
                franka_state_receiving_index));

    builder.Connect(franka_hand_state_sub->get_output_port(),
                    franka_hand_state_receiver->get_input_port());
    builder.Connect(*franka_state_receiver, *franka_arm_positions_passthrough);
    builder.Connect(franka_arm_positions_passthrough->get_output_port(),
                    franka_combined_mux->get_input_port(0));
    builder.Connect(franka_hand_state_receiver->get_output_port(),
                    franka_combined_mux->get_input_port(1));

    // Create passthrough for combined state to visualization
    franka_passthrough_for_vis =
        builder.AddSystem<SubvectorPassThrough<double>>(
            franka_combined_mux->get_output_port(0).size(), 0,
            plant_vis.num_positions(franka_index));
    builder.Connect(franka_combined_mux->get_output_port(),
                    franka_passthrough_for_vis->get_input_port());
  } else {
    franka_passthrough_for_vis = builder.AddSystem<SubvectorPassThrough>(
        franka_state_receiver->get_output_port(0).size(), 0,
        plant_vis.num_positions(franka_index));
    builder.Connect(*franka_state_receiver, *franka_passthrough_for_vis);
  }

  auto to_pose =
      builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant_vis);

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

  builder.Connect(franka_passthrough_for_vis->get_output_port(),
                  to_pose->get_input_port());
  builder.Connect(
      to_pose->get_output_port(),
      scene_graph.get_source_pose_port(plant_vis.get_source_id().value()));

  drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
      &builder, scene_graph, meshcat, std::move(illustration_params));
  drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
      &builder, scene_graph, meshcat, std::move(proximity_params));

  // Add deformable drawer
  auto deformable_drawer_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<drake::lcmt_viewer_link_data>(
          lcm_channel_params.deformable_geometry_channel, lcm));
  auto deformable_drawer = builder.AddSystem<DeformableDrawer>(
      meshcat, "deformable", "keypoints", std::vector<int>{104, 74},
      std::vector<std::pair<size_t, size_t>>{{0, 1}});
  builder.Connect(*deformable_drawer_sub, *deformable_drawer);

  // Add visualization of LCS actual state
  auto c3_state_actual_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_c3_state>(
          lcm_channel_params.c3_actual_state_channel, lcm));
  auto c3_actual_drawer = builder.AddSystem<
      dairlib::examples::magna::systems::visualization::C3BeltStateDrawer>(
      meshcat, 1, false);
  builder.Connect(c3_state_actual_sub->get_output_port(),
                  c3_actual_drawer->get_input_port_c3_state());

  // Add visualization of LCS target state
  auto c3_state_target_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_c3_state>(
          lcm_channel_params.c3_target_state_channel, lcm));
  auto c3_target_drawer = builder.AddSystem<
      dairlib::examples::magna::systems::visualization::C3BeltStateDrawer>(
      meshcat, 1, true);
  builder.Connect(c3_state_target_sub->get_output_port(),
                  c3_target_drawer->get_input_port_c3_state());

  // Add visualization of the planned MPC trajectory
  auto planned_ee_traj_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.tracking_trajectory_actor_channel, lcm));
  auto planned_obj_traj_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.planned_keypoints_trajectory_channel, lcm));
  Eigen::VectorXd color(3);
  color << 1, 0, 0;
  auto planned_keypoints_pose_drawer =
      builder.AddSystem<dairlib::systems::LcmPoseDrawer>(
          meshcat, vis_params.belt_keypoint_model, "planned_keypoints", "",
          "plans/keypoints_", 7, true, color);
  auto planned_ee_pose_drawer =
      builder.AddSystem<dairlib::systems::LcmPoseDrawer>(
          meshcat, vis_params.belt_keypoint_model,
          "end_effector_position_target", "end_effector_orientation_target",
          "plans/ee_", 7, true, color);
  builder.Connect(planned_obj_traj_sub->get_output_port(),
                  planned_keypoints_pose_drawer->get_input_port_trajectory());
  builder.Connect(planned_ee_traj_sub->get_output_port(),
                  planned_ee_pose_drawer->get_input_port_trajectory());

  // Build the diagram
  auto diagram = builder.Build();
  diagram->set_name("round_belt_visualizer");
  dairlib::DrawAndSaveDiagramGraph(*diagram);
  auto context = diagram->CreateDefaultContext();

  auto& franka_state_sub_context =
      diagram->GetMutableSubsystemContext(*franka_state_sub, context.get());

  franka_state_receiver->InitializeSubscriberPositions(
      plant_for_state_receiving, franka_state_sub_context);

  /// Use the simulator to drive at a fixed rate
  /// If set_publish_every_time_step is true, this publishes twice
  auto simulator =
      std::make_unique<Simulator<double>>(*diagram, std::move(context));
  simulator->set_publish_every_time_step(false);
  simulator->set_publish_at_initialization(false);
  simulator->set_target_realtime_rate(1.0);
  simulator->Initialize();
  drake::log()->info("Visualizer for {} setting started",
                     FLAGS_is_simulation ? "simulation" : "hardware");
  simulator->AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace magna
}  // namespace examples
}  // namespace dairlib
int main(int argc, char* argv[]) {
  return dairlib::examples::magna::DoMain(argc, argv);
}
