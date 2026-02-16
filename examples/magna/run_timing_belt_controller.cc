#include <drake/multibody/tree/revolute_joint.h>
#include <drake/multibody/tree/revolute_spring.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "dairlib/lcmt_c3_output.hpp"
#include "dairlib/lcmt_c3_state.hpp"
#include "dairlib/lcmt_osc_target_tracking_debug.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "examples/magna/parameter_headers/lcm_channel_params.h"
#include "examples/magna/parameter_headers/timing_belt_controller_params.h"
#include "examples/magna/timing_belt_controller.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/framework/state_vector.h"
#include "systems/franka_kinematics.h"
#include "systems/robot_lcm_systems.h"
#include "systems/senders/c3_state_sender.h"
#include "systems/system_utils.h"
#include "systems/trajectory_optimization/c3_output_systems.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace dairlib {
namespace examples {
namespace magna {

using dairlib::systems::StateVector;
using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::math::RigidTransform;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::RevoluteJoint;
using drake::multibody::RevoluteSpring;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using Eigen::VectorXd;

DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");
DEFINE_bool(is_simulation, true, "True for simulation, false for hardware");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::lcm::DrakeLcm lcm(FLAGS_lcm_url);
  drake::lcm::DrakeLcm local_lcm("udpm://239.255.76.67:7667?ttl=0");

  // --------------------- Load parameters ---------------------- //
  TimingBeltControllerParams timing_belt_controller_params =
      drake::yaml::LoadYamlFile<TimingBeltControllerParams>(
          FLAGS_is_simulation ? "examples/magna/parameters/"
                                "timing_belt_controller_params_sim.yaml"
                              : "examples/magna/parameters/"
                                "timing_belt_controller_params_hw.yaml");
  // ------------------------------------------------------------- //

  DiagramBuilder<double> builder;

  // --------------------- Create LCS plant ---------------------- //
  DiagramBuilder<double> plant_lcs_builder;
  auto [plant_lcs, scene_graph] =
      AddMultibodyPlantSceneGraph(&plant_lcs_builder, 0.001);
  const auto& scene_graph_inspector = scene_graph.model_inspector();
  Parser parser(&plant_lcs, &scene_graph);
  parser.AddModels(
      dairlib::FindResourceOrThrow(timing_belt_controller_params.ee_model));
  parser.AddModels(dairlib::FindResourceOrThrow(
      timing_belt_controller_params.task_board_model));

  plant_lcs.WeldFrames(plant_lcs.world_frame(),
                       plant_lcs.GetFrameByName("base_link"),
                       RigidTransform<double>::Identity());
  RigidTransform<double> task_board_pose = RigidTransform<double>(
      drake::math::RollPitchYaw<double>(
          timing_belt_controller_params.task_board_orientation[0],
          timing_belt_controller_params.task_board_orientation[1],
          timing_belt_controller_params.task_board_orientation[2]),
      drake::Vector3<double>(
          timing_belt_controller_params.task_board_position[0],
          timing_belt_controller_params.task_board_position[1],
          timing_belt_controller_params.task_board_position[2]));
  plant_lcs.WeldFrames(plant_lcs.world_frame(),
                       plant_lcs.GetFrameByName("board"), task_board_pose);

  ModelInstanceIndex timing_belt_index =
      parser.AddModels(dairlib::FindResourceOrThrow(
          timing_belt_controller_params.timing_belt_model))[0];
  plant_lcs.AddWeldConstraint(
      plant_lcs.GetBodyByName(
          timing_belt_controller_params.timing_belt_start_body_name),
      RigidTransform<double>::Identity(),
      plant_lcs.GetBodyByName(
          timing_belt_controller_params.timing_belt_end_body_name),
      RigidTransform<double>::Identity());
  int num_timing_belt_elements =
      timing_belt_controller_params.num_timing_belt_elements;
  for (int i = 0; i < num_timing_belt_elements - 1; i++) {
    std::string roll_joint_name = "joint_" + std::to_string(i) + "_roll";
    std::string yaw_joint_name = "joint_" + std::to_string(i) + "_yaw";
    plant_lcs.AddForceElement<RevoluteSpring>(
        plant_lcs.GetJointByName<RevoluteJoint>(roll_joint_name,
                                                timing_belt_index),
        0.0, timing_belt_controller_params.twisting_stiffness);
    plant_lcs.AddForceElement<RevoluteSpring>(
        plant_lcs.GetJointByName<RevoluteJoint>(yaw_joint_name,
                                                timing_belt_index),
        0.0, timing_belt_controller_params.bending_stiffness);
  }
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
  // belt-large-pulley and end-effector-small-pulley.
  std::vector<std::vector<drake::SortedPair<drake::geometry::GeometryId>>>
      contact_pairs;
  std::vector<SortedPair<GeometryId>> belt_top_plate_large_pulley_contact_pairs;
  std::vector<SortedPair<GeometryId>> belt_body_large_pulley_contact_pairs;
  std::vector<SortedPair<GeometryId>> belt_top_plate_small_pulley_contact_pairs;
  std::vector<SortedPair<GeometryId>> belt_body_small_pulley_contact_pairs;
  std::vector<SortedPair<GeometryId>> ee_top_plate_small_pulley_contact_pairs;
  std::vector<SortedPair<GeometryId>> ee_belt_contact_pairs;

  const GeometryId& ee_geom = plant_lcs.GetCollisionGeometriesForBody(
      plant_lcs.GetBodyByName("end_effector_simple"))[0];
  const std::vector<GeometryId>& large_pulley_geoms =
      plant_lcs.GetCollisionGeometriesForBody(
          plant_lcs.GetBodyByName("large_timing_pulley"));
  const std::vector<GeometryId>& small_pulley_geoms =
      plant_lcs.GetCollisionGeometriesForBody(
          plant_lcs.GetBodyByName("small_timing_pulley"));
  std::map<std::string, GeometryId> large_pulley_geoms_map;
  std::map<std::string, GeometryId> small_pulley_geoms_map;
  for (const GeometryId& geom_id : large_pulley_geoms) {
    std::string geom_name = scene_graph_inspector.GetName(geom_id);
    std::string geom_name_without_prefix =
        geom_name.substr(geom_name.find("::") + 2);
    large_pulley_geoms_map[geom_name_without_prefix] = geom_id;
  }
  for (const GeometryId& geom_id : small_pulley_geoms) {
    std::string geom_name = scene_graph_inspector.GetName(geom_id);
    std::string geom_name_without_prefix =
        geom_name.substr(geom_name.find("::") + 2);
    small_pulley_geoms_map[geom_name_without_prefix] = geom_id;
  }
  std::vector<GeometryId> timing_belt_segment_geoms;
  for (int i = 0; i <= num_timing_belt_elements; i++) {
    timing_belt_segment_geoms.push_back(plant_lcs.GetCollisionGeometriesForBody(
        plant_lcs.GetBodyByName("rod_" + std::to_string(i)))[0]);
  }
  auto large_pulley_top_plate_geom =
      large_pulley_geoms_map.at("large_timing_pulley_top_plate");
  auto large_pulley_body_geom =
      large_pulley_geoms_map.at("large_timing_pulley_body");
  auto small_pulley_top_plate_geom =
      small_pulley_geoms_map.at("small_timing_pulley_top_plate");
  auto small_pulley_body_geom =
      small_pulley_geoms_map.at("small_timing_pulley_body");

  for (const GeometryId& segment_geom_id : timing_belt_segment_geoms) {
    belt_top_plate_large_pulley_contact_pairs.emplace_back(
        large_pulley_top_plate_geom, segment_geom_id);
    belt_body_large_pulley_contact_pairs.emplace_back(large_pulley_body_geom,
                                                      segment_geom_id);
    belt_top_plate_small_pulley_contact_pairs.emplace_back(
        small_pulley_top_plate_geom, segment_geom_id);
    belt_body_small_pulley_contact_pairs.emplace_back(small_pulley_body_geom,
                                                      segment_geom_id);
    ee_belt_contact_pairs.emplace_back(ee_geom, segment_geom_id);
  }
  ee_top_plate_small_pulley_contact_pairs.emplace_back(
      ee_geom, small_pulley_top_plate_geom);

  if (timing_belt_controller_params.verbose) {
    drake::log()->info(
        "number of belt-top-plate-large-pulley contact pairs: {}",
        belt_top_plate_large_pulley_contact_pairs.size());
    drake::log()->info("number of belt-body-large-pulley contact pairs: {}",
                       belt_body_large_pulley_contact_pairs.size());
    drake::log()->info(
        "number of belt-top-plate-small-pulley contact pairs: {}",
        belt_top_plate_small_pulley_contact_pairs.size());
    drake::log()->info("number of belt-body-small-pulley contact pairs: {}",
                       belt_body_small_pulley_contact_pairs.size());
    drake::log()->info("number of ee-belt contact pairs: {}",
                       ee_belt_contact_pairs.size());
    drake::log()->info("number of ee-top-plate-small-pulley contact pairs: {}",
                       ee_top_plate_small_pulley_contact_pairs.size());
  }

  contact_pairs.emplace_back(belt_top_plate_large_pulley_contact_pairs);
  contact_pairs.emplace_back(belt_body_large_pulley_contact_pairs);
  contact_pairs.emplace_back(belt_top_plate_small_pulley_contact_pairs);
  contact_pairs.emplace_back(belt_body_small_pulley_contact_pairs);
  contact_pairs.emplace_back(ee_top_plate_small_pulley_contact_pairs);
  contact_pairs.emplace_back(ee_belt_contact_pairs);

  auto timing_belt_controller = builder.AddSystem<TimingBeltController>(
      plant_lcs, &plant_lcs_context, *plant_lcs_ad, plant_lcs_ad_context.get(),
      contact_pairs, timing_belt_controller_params);
  // ------------------------------------------------------------- //

  // ----- Construct plants for FrankaKinematics ----- //
  // FrankaKinematics is a LeafSystem that computes the LCS state from the
  // current states of the Franka robot and all associated objects.
  // The LCS state represents a simplified model in which the entire Franka arm
  // is abstracted by its end effector.

  MultibodyPlant<double> plant_franka(0.0);
  Parser parser_franka(&plant_franka);
  [[maybe_unused]] ModelInstanceIndex franka_index =
      parser_franka.AddModelsFromUrl(
          timing_belt_controller_params.franka_arm_hand_model)[0];
  plant_franka.WeldFrames(plant_franka.world_frame(),
                          plant_franka.GetFrameByName("panda_link0"),
                          RigidTransform<double>::Identity());
  plant_franka.Finalize();
  auto franka_context = plant_franka.CreateDefaultContext();

  // Create an object-only plant.
  MultibodyPlant<double> plant_object(0.0);
  Parser parser_object(&plant_object, nullptr);
  parser_object.AddModels(dairlib::FindResourceOrThrow(
      timing_belt_controller_params.timing_belt_model));
  plant_object.Finalize();
  auto object_context = plant_object.CreateDefaultContext();
  std::vector<std::string> object_names = {"rod_0"};

  auto franka_kinematics =
      builder.AddSystem<dairlib::systems::FrankaKinematics>(
          plant_franka, franka_context.get(), plant_object,
          object_context.get(), "finger_tip", "rod_0", true, object_names);

  auto robot_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(
          timing_belt_controller_params.lcm_channels.franka_state_channel,
          &lcm));
  auto robot_state_receiver =
      builder.AddSystem<dairlib::systems::RobotOutputReceiver>(plant_franka);

  auto object_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_object_state>(
          timing_belt_controller_params.lcm_channels.timing_belt_state_channel,
          &lcm));
  auto object_state_receiver =
      builder.AddSystem<dairlib::systems::ObjectStateReceiver>(plant_object);

  // Wire up connections
  // Robot state -> Receiver -> Kinematics -> AssemblyController (LCS state)
  builder.Connect(robot_state_sub->get_output_port(),
                  robot_state_receiver->get_input_port(0));
  builder.Connect(object_state_sub->get_output_port(),
                  object_state_receiver->get_input_port(0));
  builder.Connect(object_state_receiver->get_output_port(0),
                  *franka_kinematics->get_input_ports_object_state()[0]);
  builder.Connect(robot_state_receiver->get_output_port(0),
                  franka_kinematics->get_input_port_franka_state());
  builder.Connect(franka_kinematics->get_output_port_lcs_state(),
                  timing_belt_controller->get_input_port_lcs_state());
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
          timing_belt_controller_params.lcm_channels.c3_actual_state_channel,
          &local_lcm, TriggerTypeSet({TriggerType::kForced})));
  auto target_lcs_state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_state>(
          timing_belt_controller_params.lcm_channels.c3_target_state_channel,
          &local_lcm, TriggerTypeSet({TriggerType::kForced})));
  builder.Connect(franka_kinematics->get_output_port_lcs_state(),
                  c3_state_sender->get_input_port_actual_state());
  builder.Connect(c3_state_sender->get_output_port_actual_c3_state(),
                  lcs_state_pub->get_input_port(0));
  builder.Connect(c3_state_sender->get_output_port_target_c3_state(),
                  target_lcs_state_pub->get_input_port(0));

  // Connect current target LCS state from assembly controller to
  // c3_state_sender
  builder.Connect(
      timing_belt_controller->get_output_port_current_target_lcs_state(),
      c3_state_sender->get_input_port_target_state());
  // ------------------------------------------------------------ //

  // ----- Publish gripper position command via LCM messages ----- //
  auto gripper_pos_command_pub = builder.AddSystem(
      LcmPublisherSystem::Make<drake::lcmt_schunk_wsg_command>(
          timing_belt_controller_params.lcm_channels
              .franka_hand_target_position_channel,
          &local_lcm, TriggerTypeSet({TriggerType::kForced})));
  builder.Connect(timing_belt_controller->get_output_port_gripper_pos_command(),
                  gripper_pos_command_pub->get_input_port(0));
  // ------------------------------------------------------------- //

  // ----- Publish trajectory output via LCM messages ----- //
  auto traj_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          timing_belt_controller_params.lcm_channels
              .tracking_trajectory_actor_channel,
          &lcm, TriggerTypeSet({TriggerType::kForced})));
  auto traj_planned_keypoints_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          timing_belt_controller_params.lcm_channels
              .planned_keypoints_trajectory_channel,
          &local_lcm, TriggerTypeSet({TriggerType::kForced})));
  auto traj_planned_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          timing_belt_controller_params.lcm_channels.planned_trajectory_channel,
          &local_lcm, TriggerTypeSet({TriggerType::kForced})));
  builder.Connect(timing_belt_controller->get_output_port_traj_execute(),
                  traj_pub->get_input_port(0));
  builder.Connect(
      timing_belt_controller->get_output_port_traj_planned_keypoints(),
      traj_planned_keypoints_pub->get_input_port(0));
  builder.Connect(timing_belt_controller->get_output_port_traj_planned(),
                  traj_planned_pub->get_input_port(0));
  // ------------------------------------------------------------- //

  // ----- Publish C3 forces via LCM messages ----- //
  //   auto c3_forces_pub =
  //       builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_forces>(
  //           timing_belt_controller_params.lcm_channels.c3_force_channel,
  //           &local_lcm, TriggerTypeSet({TriggerType::kForced})));
  //   builder.Connect(timing_belt_controller->get_output_port_c3_forces(),
  //                   c3_forces_pub->get_input_port(0));
  // ------------------------------------------------------------- //

  // ----- Publish C3 output (solution + intermediates) via LCM ----- //
  //   auto c3_output_sender =
  //   builder.AddSystem<dairlib::systems::C3OutputSender>(); auto c3_output_pub
  //   =
  //       builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_output>(
  //           timing_belt_controller_params.lcm_channels.c3_output_channel,
  //           &local_lcm, TriggerTypeSet({TriggerType::kForced})));
  //   builder.Connect(timing_belt_controller->get_output_port_c3_solution(),
  //                   c3_output_sender->get_input_port_c3_solution());
  //   builder.Connect(timing_belt_controller->get_output_port_c3_intermediates(),
  //                   c3_output_sender->get_input_port_c3_intermediates());
  //   builder.Connect(c3_output_sender->get_output_port_c3_debug(),
  //                   c3_output_pub->get_input_port(0));
  // ------------------------------------------------------------- //

  // ----- Publish OSC target tracking debug via LCM ----- //
  //   auto osc_target_tracking_debug_pub = builder.AddSystem(
  //       LcmPublisherSystem::Make<dairlib::lcmt_osc_target_tracking_debug>(
  //           timing_belt_controller_params.lcm_channels
  //               .osc_target_tracking_debug_channel,
  //           &local_lcm, TriggerTypeSet({TriggerType::kForced})));
  //   builder.Connect(
  //       timing_belt_controller->get_output_port_osc_target_tracking_debug(),
  //       osc_target_tracking_debug_pub->get_input_port(0));
  // ------------------------------------------------------------- //

  // Build diagram
  auto owned_diagram = builder.Build();
  std::shared_ptr<Diagram<double>> shared_diagram = std::move(owned_diagram);
  shared_diagram->set_name(("timing_belt_controller_diagram"));
  dairlib::DrawAndSaveDiagramGraph(*shared_diagram);
  // Run lcm-driven simulation
  dairlib::systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, shared_diagram, robot_state_receiver,
      timing_belt_controller_params.lcm_channels.franka_state_channel, true);
  drake::log()->info("Timing belt controller started");
  loop.Simulate(std::numeric_limits<double>::infinity());
  return 0;
}

}  // namespace magna
}  // namespace examples
}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::examples::magna::DoMain(argc, argv);
}
