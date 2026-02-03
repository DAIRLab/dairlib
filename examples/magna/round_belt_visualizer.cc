#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "common/find_resource.h"
#include "dairlib/lcmt_c3_forces.hpp"
#include "dairlib/lcmt_c3_state.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_round_belt_state.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "examples/magna/systems/franka_hand/franka_hand_status_bridge_out.h"
#include "examples/magna/systems/visualization/c3_belt_state_drawer.h"
#include "examples/magna/systems/visualization/deformable_drawer.h"
#include "parameter_headers/lcm_channel_params.h"
#include "parameter_headers/round_belt_controller_params.h"
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

static constexpr const char* kFrankaModel =
    "package://drake_models/franka_description/urdf/"
    "panda_arm.urdf";
static constexpr const char* kFrankaHand =
    "package://drake_models/franka_description/urdf/"
    "panda_hand_with_long_fingers.urdf";
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::examples::magna::systems::franka_hand::FrankaHandStatusBridgeOut;
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

  RoundBeltControllerParams round_belt_controller_params =
      drake::yaml::LoadYamlFile<RoundBeltControllerParams>(
          FLAGS_is_simulation ? "examples/magna/parameters/"
                                "round_belt_controller_params_sim.yaml"
                              : "examples/magna/parameters/"
                                "round_belt_controller_params_hw.yaml");

  MagnaVisualizerParams vis_params =
      round_belt_controller_params.visualizer_params;
  MagnaLcmChannels lcm_channel_params =
      round_belt_controller_params.lcm_channels;

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
  ModelInstanceIndex scene_index =
      parser.AddModels(dairlib::FindResourceOrThrow(
          round_belt_controller_params.scene_model))[0];
  plant_vis.WeldFrames(
      plant_vis.world_frame(),
      plant_vis.GetFrameByName("round_belt_task_scene", scene_index), X_WI);
  // Add pulley models
  ModelInstanceIndex task_board_index =
      parser.AddModels(dairlib::FindResourceOrThrow(
          round_belt_controller_params.task_board_model))[0];
  RigidTransform<double> task_board_pose = RigidTransform<double>(
      drake::math::RollPitchYaw<double>(
          round_belt_controller_params.task_board_orientation[0],
          round_belt_controller_params.task_board_orientation[1],
          round_belt_controller_params.task_board_orientation[2]),
      drake::Vector3<double>(
          round_belt_controller_params.task_board_position[0],
          round_belt_controller_params.task_board_position[1],
          round_belt_controller_params.task_board_position[2]));
  plant_vis.WeldFrames(plant_vis.world_frame(),
                       plant_vis.GetFrameByName("board", task_board_index),
                       task_board_pose);

  // Add franka arm model
  ModelInstanceIndex franka_index = parser.AddModelsFromUrl(kFrankaModel)[0];
  plant_vis.WeldFrames(plant_vis.world_frame(),
                       plant_vis.GetFrameByName("panda_link0"), X_WI);

  // Add franka hand model and attach it to the franka arm
  RigidTransform<double> franka_hand_pose_wrt_panda_link8 =
      RigidTransform<double>(
          drake::math::RollPitchYaw<double>(0, 0, -0.785398163397),
          drake::Vector3<double>(0.0, 0.0, 0.0));
  ModelInstanceIndex franka_hand_index =
      parser.AddModelsFromUrl(kFrankaHand)[0];
  plant_vis.WeldFrames(
      plant_vis.GetFrameByName("panda_link8"),
      plant_vis.GetFrameByName("panda_hand", franka_hand_index),
      franka_hand_pose_wrt_panda_link8);
  plant_vis.Finalize();

  // ----- Construct LCM subscriber to the franka state -----
  auto lcm =
      builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(FLAGS_lcm_url);
  auto franka_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(
          lcm_channel_params.franka_state_channel, lcm));
  auto franka_state_receiver =
      builder.AddSystem<RobotOutputReceiver>(plant_vis, franka_index);
  builder.Connect(*franka_state_sub, *franka_state_receiver);
  auto robot_time_passthrough = builder.AddSystem<SubvectorPassThrough>(
      franka_state_receiver->get_output_port(0).size(),
      franka_state_receiver->get_output_port(0).size() - 1, 1);
  builder.Connect(franka_state_receiver->get_output_port(0),
                  robot_time_passthrough->get_input_port());

  auto franka_hand_state_receiver =
      builder.AddSystem<RobotOutputReceiver>(plant_vis, franka_hand_index);
  if (FLAGS_is_simulation) {
    auto franka_hand_state_sub =
        builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(
            lcm_channel_params.franka_hand_state_channel, lcm));
    builder.Connect(franka_hand_state_sub->get_output_port(),
                    franka_hand_state_receiver->get_input_port());
  } else {
    auto franka_hand_state_sub = builder.AddSystem(
        LcmSubscriberSystem::Make<drake::lcmt_schunk_wsg_status>(
            lcm_channel_params.franka_hand_state_channel, lcm));
    auto franka_hand_status_bridge_out =
        builder.AddSystem<FrankaHandStatusBridgeOut>();
    builder.Connect(franka_hand_state_sub->get_output_port(),
                    franka_hand_status_bridge_out->get_input_port());
    builder.Connect(franka_hand_status_bridge_out->get_output_port(),
                    franka_hand_state_receiver->get_input_port());
  }

  // Extract arm and hand positions for visualization (7 dims for arm, 2 dims
  // for hand), then combine them and send to the visualization system.
  auto franka_combined_mux = builder.AddSystem<Multiplexer<double>>(
      std::vector<int>{plant_vis.num_positions(franka_index),
                       plant_vis.num_positions(franka_hand_index)});
  auto franka_arm_positions_passthrough =
      builder.AddSystem<SubvectorPassThrough>(
          franka_state_receiver->get_output_port(0).size(), 0,
          plant_vis.num_positions(franka_index));
  auto franka_hand_positions_passthrough =
      builder.AddSystem<SubvectorPassThrough>(
          franka_hand_state_receiver->get_output_port(0).size(), 0,
          plant_vis.num_positions(franka_hand_index));
  builder.Connect(franka_state_receiver->get_output_port(),
                  franka_arm_positions_passthrough->get_input_port());
  builder.Connect(franka_hand_state_receiver->get_output_port(),
                  franka_hand_positions_passthrough->get_input_port());
  builder.Connect(franka_arm_positions_passthrough->get_output_port(),
                  franka_combined_mux->get_input_port(0));
  builder.Connect(franka_hand_positions_passthrough->get_output_port(),
                  franka_combined_mux->get_input_port(1));

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
  //   meshcat->SetCameraPose(vis_params.camera_pose, vis_params.camera_target);

  builder.Connect(franka_combined_mux->get_output_port(),
                  to_pose->get_input_port());
  builder.Connect(
      to_pose->get_output_port(),
      scene_graph.get_source_pose_port(plant_vis.get_source_id().value()));

  drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
      &builder, scene_graph, meshcat, std::move(illustration_params));
  drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
      &builder, scene_graph, meshcat, std::move(proximity_params));

  if (FLAGS_is_simulation) {
    // Add deformable drawer
    auto deformable_drawer_sub = builder.AddSystem(
        LcmSubscriberSystem::Make<drake::lcmt_viewer_link_data>(
            lcm_channel_params.deformable_geometry_channel, lcm));
    auto deformable_drawer = builder.AddSystem<DeformableDrawer>(
        meshcat, "deformable", "keypoints", std::vector<int>{104, 74},
        std::vector<std::pair<size_t, size_t>>{});
    builder.Connect(*deformable_drawer_sub, *deformable_drawer);
  }

  // Add visualization of LCS actual state
  auto c3_state_actual_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_c3_state>(
          lcm_channel_params.c3_actual_state_channel, lcm));
  auto task_relevant_keypoints_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_round_belt_state>(
          lcm_channel_params.round_belt_task_relevant_keypoints_channel, lcm));
  auto c3_actual_drawer = builder.AddSystem<
      dairlib::examples::magna::systems::visualization::C3BeltStateDrawer>(
      meshcat, 1, false, 6, 3, "c3_state",
      round_belt_controller_params.spring_stiffness,
      round_belt_controller_params.spring_rest_length);
  builder.Connect(c3_state_actual_sub->get_output_port(),
                  c3_actual_drawer->get_input_port_c3_state());
  builder.Connect(task_relevant_keypoints_sub->get_output_port(),
                  c3_actual_drawer->get_input_port_task_relevant_keypoints());

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
          lcm_channel_params.planned_trajectory_channel, lcm));
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

  // Add visualization of C3 forces
  auto c3_forces_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_c3_forces>(
          lcm_channel_params.c3_force_channel, lcm));
  auto c3_forces_drawer = builder.AddSystem<dairlib::systems::LcmForceDrawer>(
      meshcat, "end_effector_position_target", "end_effector_force_target",
      "lcs_force_trajectory");
  builder.Connect(planned_ee_traj_sub->get_output_port(),
                  c3_forces_drawer->get_input_port_actor_trajectory());
  builder.Connect(robot_time_passthrough->get_output_port(),
                  c3_forces_drawer->get_input_port_robot_time());
  builder.Connect(c3_forces_sub->get_output_port(),
                  c3_forces_drawer->get_input_port_force_trajectory());

  // Build the diagram
  auto diagram = builder.Build();
  diagram->set_name("round_belt_visualizer");
  dairlib::DrawAndSaveDiagramGraph(*diagram);
  auto context = diagram->CreateDefaultContext();

  // Initialize the franka state receiver
  auto& franka_state_sub_context =
      diagram->GetMutableSubsystemContext(*franka_state_sub, context.get());
  franka_state_receiver->InitializeSubscriberPositions(
      plant_vis, franka_state_sub_context);

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

}  // namespace magna
}  // namespace examples
}  // namespace dairlib
int main(int argc, char* argv[]) {
  return dairlib::examples::magna::DoMain(argc, argv);
}
