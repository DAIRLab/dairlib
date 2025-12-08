#include <drake/systems/primitives/constant_vector_source.h>
#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "dairlib/lcmt_c3_state.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "examples/magna/assembly_controller.h"
#include "examples/magna/parameter_headers/assembly_c3_options.h"
#include "examples/magna/parameter_headers/lcm_channel_params.h"
#include "examples/magna/parameter_headers/round_belt_controller_params.h"
#include "examples/magna/parameter_headers/target_poses.h"
#include "examples/magna/systems/force_elements/linear_spring_damper_no_compression.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/framework/state_vector.h"
#include "systems/franka_kinematics.h"
#include "systems/robot_lcm_systems.h"
#include "systems/senders/c3_state_sender.h"
#include "systems/system_utils.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace dairlib {
namespace examples {
namespace magna {

using dairlib::systems::StateVector;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::ConstantVectorSource;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using Eigen::VectorXd;

DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::lcm::DrakeLcm lcm(FLAGS_lcm_url);

  // --------------------- Load parameters ---------------------- //
  RoundBeltControllerParams round_belt_controller_params =
      drake::yaml::LoadYamlFile<RoundBeltControllerParams>(
          "examples/magna/parameters/round_belt_controller_params.yaml");
  // ------------------------------------------------------------- //

  DiagramBuilder<double> builder;

  // --------------------- Create LCS plant ---------------------- //
  DiagramBuilder<double> plant_lcs_builder;
  auto [plant_lcs, scene_graph] =
      AddMultibodyPlantSceneGraph(&plant_lcs_builder, 0.0);
  Parser parser(&plant_lcs, &scene_graph);
  parser.AddModels(
      dairlib::FindResourceOrThrow(round_belt_controller_params.ee_model));
  parser.AddModels(dairlib::FindResourceOrThrow(
      round_belt_controller_params.belt_element_model));
  parser.AddModels(dairlib::FindResourceOrThrow(
      round_belt_controller_params.task_board_model));

  plant_lcs.WeldFrames(plant_lcs.world_frame(),
                       plant_lcs.GetFrameByName("base_link"),
                       RigidTransform<double>::Identity());
  plant_lcs.WeldFrames(plant_lcs.world_frame(),
                       plant_lcs.GetFrameByName("belt_element_base_link"),
                       RigidTransform<double>::Identity());
  RigidTransform<double> task_board_pose = RigidTransform<double>(
      drake::math::RollPitchYaw<double>(
          round_belt_controller_params.task_board_orientation[0],
          round_belt_controller_params.task_board_orientation[1],
          round_belt_controller_params.task_board_orientation[2]),
      drake::Vector3<double>(
          round_belt_controller_params.task_board_position[0],
          round_belt_controller_params.task_board_position[1],
          round_belt_controller_params.task_board_position[2]));
  plant_lcs.WeldFrames(plant_lcs.world_frame(),
                       plant_lcs.GetFrameByName("board"), task_board_pose);
  plant_lcs.AddForceElement<
      magna::systems::force_elements::LinearSpringDamperNoCompression>(
      plant_lcs.GetBodyByName("end_effector_simple"),
      drake::Vector3<double>(0, 0, 0), plant_lcs.GetBodyByName("belt_element"),
      drake::Vector3<double>(0, 0, 0),
      round_belt_controller_params.spring_rest_length,
      round_belt_controller_params.spring_stiffness,
      round_belt_controller_params.spring_damping);
  plant_lcs.Finalize();

  // Create AutoDiff version of LCS plant
  std::unique_ptr<MultibodyPlant<drake::AutoDiffXd>> plant_lcs_ad =
      drake::systems::System<double>::ToAutoDiffXd(plant_lcs);
  auto plant_lcs_ad_context = plant_lcs_ad->CreateDefaultContext();

  auto plant_lcs_diagram = plant_lcs_builder.Build();
  std::unique_ptr<drake::systems::Context<double>> diagram_context =
      plant_lcs_diagram->CreateDefaultContext();
  auto& plant_lcs_context = plant_lcs_diagram->GetMutableSubsystemContext(
      plant_lcs, diagram_context.get());

  // contact_pairs variable holds the list of groups, each group is a list of
  // contact pairs we currently consider two groups of contact pairs:
  // belt-large-pulley and belt-small-pulley.
  std::vector<std::vector<drake::SortedPair<drake::geometry::GeometryId>>>
      contact_pairs;
  std::vector<SortedPair<GeometryId>> belt_large_pulley_contact_pairs;
  std::vector<SortedPair<GeometryId>> belt_small_pulley_contact_pairs;

  auto& ee_geom = plant_lcs.GetCollisionGeometriesForBody(
      plant_lcs.GetBodyByName("end_effector_simple"))[0];
  auto belt_large_pulley_geoms = plant_lcs.GetCollisionGeometriesForBody(
      plant_lcs.GetBodyByName("large_round_pulley"));
  auto belt_small_pulley_geoms = plant_lcs.GetCollisionGeometriesForBody(
      plant_lcs.GetBodyByName("small_round_pulley"));
  auto belt_element_geom = plant_lcs.GetCollisionGeometriesForBody(
      plant_lcs.GetBodyByName("belt_element"))[0];
  for (auto geom_id : belt_large_pulley_geoms) {
    belt_large_pulley_contact_pairs.emplace_back(belt_element_geom, geom_id);
  }
  for (auto geom_id : belt_small_pulley_geoms) {
    belt_small_pulley_contact_pairs.emplace_back(ee_geom, geom_id);
  }
  contact_pairs.emplace_back(belt_large_pulley_contact_pairs);
  //   contact_pairs.emplace_back(belt_small_pulley_contact_pairs);

  auto assembly_controller = builder.AddSystem<AssemblyController>(
      plant_lcs, &plant_lcs_context, *plant_lcs_ad, plant_lcs_ad_context.get(),
      contact_pairs, round_belt_controller_params);
  // ------------------------------------------------------------- //

  // ----- Construct plants for FrankaKinematics ----- //
  // FrankaKinematics is a LeafSystem that computes the LCS state from the
  // current states of the Franka robot and all associated objects.
  // The LCS state represents a simplified model in which the entire Franka arm
  // is abstracted by its end effector.

  // Create a Franka-only plant (no need to add walls to this).
  MultibodyPlant<double> plant_franka(0.0);
  Parser parser_franka(&plant_franka);
  [[maybe_unused]] ModelInstanceIndex franka_index =
      parser_franka.AddModelsFromUrl(
          round_belt_controller_params.franka_arm_hand_model)[0];
  plant_franka.WeldFrames(plant_franka.world_frame(),
                          plant_franka.GetFrameByName("panda_link0"),
                          RigidTransform<double>::Identity());
  plant_franka.Finalize();
  auto franka_context = plant_franka.CreateDefaultContext();

  // Create an object-only plant.
  MultibodyPlant<double> plant_object(0.0);
  Parser parser_object(&plant_object, nullptr);
  parser_object.AddModels(dairlib::FindResourceOrThrow(
      "examples/magna/urdf/round_belt_task/belt_element.urdf"));
  plant_object.WeldFrames(plant_object.world_frame(),
                          plant_object.GetFrameByName("belt_element_base_link"),
                          RigidTransform<double>::Identity());
  plant_object.Finalize();
  auto object_context = plant_object.CreateDefaultContext();
  std::vector<std::string> object_names = {"belt_element"};

  auto franka_kinematics =
      builder.AddSystem<dairlib::systems::FrankaKinematics>(
          plant_franka, franka_context.get(), plant_object,
          object_context.get(), "finger_tip", "belt_element", true,
          object_names);

  auto robot_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(
          round_belt_controller_params.lcm_channels.franka_state_channel,
          &lcm));
  auto robot_state_receiver =
      builder.AddSystem<dairlib::systems::RobotOutputReceiver>(plant_franka);

  // Wire up connections
  // Robot state -> Receiver -> Kinematics -> AssemblyController (LCS state)
  builder.Connect(robot_state_sub->get_output_port(),
                  robot_state_receiver->get_input_port(0));
  builder.Connect(robot_state_receiver->get_output_port(0),
                  franka_kinematics->get_input_port_franka_state());
  builder.Connect(franka_kinematics->get_output_port_lcs_state(),
                  assembly_controller->get_input_port_lcs_state());
  // ---------------------------------------------------  //

  // ----- Publish current/target LCS state via LCM messages ----- //

  // C3StateSender is a LeafSystem that converts the LCS state to a LCM message.
  int lcs_state_size = plant_lcs.num_positions() + plant_lcs.num_velocities();
  std::vector<std::string> lcs_state_names;
  for (int i = 0; i < lcs_state_size; i++) {
    lcs_state_names.push_back("x_lcs[" + std::to_string(i) + "]");
  }
  auto c3_state_sender = builder.AddSystem<dairlib::systems::C3StateSender>(
      lcs_state_size, lcs_state_names);
  auto lcs_state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_state>(
          round_belt_controller_params.lcm_channels.c3_actual_state_channel,
          &lcm, TriggerTypeSet({TriggerType::kForced})));
  auto target_lcs_state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_state>(
          round_belt_controller_params.lcm_channels.c3_target_state_channel,
          &lcm, TriggerTypeSet({TriggerType::kForced})));
  builder.Connect(franka_kinematics->get_output_port_lcs_state(),
                  c3_state_sender->get_input_port_actual_state());
  builder.Connect(c3_state_sender->get_output_port_actual_c3_state(),
                  lcs_state_pub->get_input_port(0));
  builder.Connect(c3_state_sender->get_output_port_target_c3_state(),
                  target_lcs_state_pub->get_input_port(0));

  // Crreate a constant source for the object state
  // `object` in this case is one keypoint on the belt.
  // TODO: we should subscribe and obtain state from a LCM channel.
  auto constant_object_state_vector = StateVector<double>(3, 3);
  VectorXd constant_positions = Eigen::Map<VectorXd>(
      round_belt_controller_params.fixed_keypoint_position.data(),
      round_belt_controller_params.fixed_keypoint_position.size());
  VectorXd constant_velocities = VectorXd::Zero(3);
  constant_object_state_vector.SetPositions(constant_positions);
  constant_object_state_vector.SetVelocities(constant_velocities);
  auto constant_source =
      builder.AddSystem<ConstantVectorSource>(constant_object_state_vector);

  builder.Connect(constant_source->get_output_port(),
                  *(franka_kinematics->get_input_ports_object_state()[0]));

  // Connect current target LCS state from assembly controller to
  // c3_state_sender
  builder.Connect(
      assembly_controller->get_output_port_current_target_lcs_state(),
      c3_state_sender->get_input_port_target_state());
  // ------------------------------------------------------------ //

  // ----- Publish gripper position command via LCM messages ----- //
  auto gripper_pos_command_pub = builder.AddSystem(
      LcmPublisherSystem::Make<drake::lcmt_schunk_wsg_command>(
          round_belt_controller_params.lcm_channels
              .franka_hand_target_position_channel,
          &lcm, TriggerTypeSet({TriggerType::kForced})));
  builder.Connect(assembly_controller->get_output_port_gripper_pos_command(),
                  gripper_pos_command_pub->get_input_port(0));
  // ------------------------------------------------------------- //

  // ----- Publish trajectory output via LCM messages ----- //
  auto traj_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          round_belt_controller_params.lcm_channels
              .tracking_trajectory_actor_channel,
          &lcm, TriggerTypeSet({TriggerType::kForced})));
  auto traj_planned_keypoints_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          round_belt_controller_params.lcm_channels
              .planned_keypoints_trajectory_channel,
          &lcm, TriggerTypeSet({TriggerType::kForced})));
  builder.Connect(assembly_controller->get_output_port_traj_execute(),
                  traj_pub->get_input_port(0));
  builder.Connect(assembly_controller->get_output_port_traj_planned_keypoints(),
                  traj_planned_keypoints_pub->get_input_port(0));
  // ------------------------------------------------------------- //

  // ----- Publish C3 forces via LCM messages ----- //
  auto c3_forces_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_forces>(
          round_belt_controller_params.lcm_channels.c3_force_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  builder.Connect(assembly_controller->get_output_port_c3_forces(),
                  c3_forces_pub->get_input_port(0));
  // ------------------------------------------------------------- //

  // Build diagram
  auto owned_diagram = builder.Build();
  std::shared_ptr<Diagram<double>> shared_diagram = std::move(owned_diagram);
  shared_diagram->set_name(("assembly_controller_diagram"));
  dairlib::DrawAndSaveDiagramGraph(*shared_diagram);
  // Run lcm-driven simulation
  dairlib::systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, shared_diagram, robot_state_receiver,
      round_belt_controller_params.lcm_channels.franka_state_channel, true);
  drake::log()->info("Assembly controller started");
  loop.Simulate(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace magna
}  // namespace examples
}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::examples::magna::DoMain(argc, argv);
}
