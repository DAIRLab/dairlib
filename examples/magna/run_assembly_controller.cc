#include <drake/systems/primitives/constant_vector_source.h>
#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "dairlib/lcmt_c3_state.hpp"
#include "dairlib/lcmt_franka_hand_target_position.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "examples/magna/assembly_controller.h"
#include "examples/magna/parameter_headers/assembly_c3_options.h"
#include "examples/magna/parameter_headers/lcm_channel_params.h"
#include "examples/magna/parameter_headers/target_poses.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/framework/state_vector.h"
#include "systems/franka_kinematics.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/linear_spring_damper.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_value_source.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace dairlib {
namespace examples {
namespace magna {

static constexpr const char* kFrankaModel =
    "package://drake_models/franka_description/urdf/"
    "panda_arm_hand_with_long_fingers.urdf";

using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::ConstantValueSource;
using drake::systems::ConstantVectorSource;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using Eigen::VectorXd;
using systems::StateVector;

DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::lcm::DrakeLcm lcm(FLAGS_lcm_url);

  // Load LCM channel parameters
  MagnaLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<MagnaLcmChannels>(
          "examples/magna/parameters/lcm_channels_simulation.yaml");

  AssemblyC3Options assembly_c3_options =
      drake::yaml::LoadYamlFile<AssemblyC3Options>(
          "examples/magna/parameters/assembly_c3_options.yaml");

  TargetPosesParams target_poses_params =
      drake::yaml::LoadYamlFile<TargetPosesParams>(
          "examples/magna/parameters/target_poses.yaml");

  DiagramBuilder<double> builder;

  // Create LCS plant
  DiagramBuilder<double> plant_lcs_builder;
  auto [plant_lcs, scene_graph] =
      AddMultibodyPlantSceneGraph(&plant_lcs_builder, 0.0);
  Parser parser(&plant_lcs, &scene_graph);
  parser.AddModels(dairlib::FindResourceOrThrow(
      "examples/magna/urdf/round_belt_task/end_effector_simple_model.urdf"));
  parser.AddModels(dairlib::FindResourceOrThrow(
      "examples/magna/urdf/round_belt_task/belt_element.urdf"));
  parser.AddModels(dairlib::FindResourceOrThrow(
      "examples/magna/urdf/round_belt_task/round_belt_task_board.sdf"));

  plant_lcs.WeldFrames(plant_lcs.world_frame(),
                       plant_lcs.GetFrameByName("base_link"),
                       RigidTransform<double>::Identity());
  plant_lcs.WeldFrames(plant_lcs.world_frame(),
                       plant_lcs.GetFrameByName("belt_element_base_link"),
                       RigidTransform<double>::Identity());
  RigidTransform<double> task_board_pose =
      RigidTransform<double>(drake::math::RollPitchYaw<double>(0, 0, 1.57079),
                             drake::Vector3<double>(0.68585, -0.192, 0.00543));
  plant_lcs.WeldFrames(plant_lcs.world_frame(),
                       plant_lcs.GetFrameByName("board"), task_board_pose);
  plant_lcs.AddForceElement<drake::multibody::LinearSpringDamper>(
      plant_lcs.GetBodyByName("end_effector_simple"),
      drake::Vector3<double>(0, 0, 0), plant_lcs.GetBodyByName("belt_element"),
      drake::Vector3<double>(0, 0, 0), 0.24, 1000, 0.1);
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
  for (auto geom_id : belt_large_pulley_geoms) {
    belt_large_pulley_contact_pairs.emplace_back(ee_geom, geom_id);
  }
  for (auto geom_id : belt_small_pulley_geoms) {
    belt_small_pulley_contact_pairs.emplace_back(ee_geom, geom_id);
  }
  contact_pairs.emplace_back(belt_large_pulley_contact_pairs);
  contact_pairs.emplace_back(belt_small_pulley_contact_pairs);

  // Create AssemblyController
  auto assembly_controller = builder.AddSystem<AssemblyController>(
      plant_lcs, &plant_lcs_context, *plant_lcs_ad, plant_lcs_ad_context.get(),
      contact_pairs, assembly_c3_options, target_poses_params);

  // ----- Construct plants for FrankaKinematics ----- //

  // Create a Franka-only plant (no need to add walls to this).
  MultibodyPlant<double> plant_franka(0.0);
  Parser parser_franka(&plant_franka);
  [[maybe_unused]] ModelInstanceIndex franka_index =
      parser_franka.AddModelsFromUrl(kFrankaModel)[0];
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
  // ---------------------------------------------------  //

  // LCM subscriber and receiver for target state
  // LCM subscriber for robot state (to get LCS state via kinematics)
  auto robot_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(
          lcm_channel_params.franka_state_channel, &lcm));
  auto robot_state_receiver =
      builder.AddSystem<dairlib::systems::RobotOutputReceiver>(plant_franka);

  // LCM publisher for trajectory output
  auto traj_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.tracking_trajectory_actor_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));

  // Wire up connections
  // Robot state -> Receiver -> Kinematics -> AssemblyController (LCS state)
  builder.Connect(robot_state_sub->get_output_port(),
                  robot_state_receiver->get_input_port(0));
  builder.Connect(robot_state_receiver->get_output_port(0),
                  franka_kinematics->get_input_port_franka_state());
  builder.Connect(franka_kinematics->get_output_port_lcs_state(),
                  assembly_controller->get_input_port_lcs_state());

  // TODO: Currently, I don't how to get state of a particular vertex of a
  // deformable object yet.
  auto constant_object_state_vector = StateVector<double>(3, 3);
  VectorXd constant_positions(3);
  constant_positions << 0.0, 0.0, 0.0;
  VectorXd constant_velocities = VectorXd::Zero(3);
  constant_object_state_vector.SetPositions(constant_positions);
  constant_object_state_vector.SetVelocities(constant_velocities);
  auto constant_source =
      builder.AddSystem<ConstantVectorSource>(constant_object_state_vector);

  builder.Connect(constant_source->get_output_port(),
                  *(franka_kinematics->get_input_ports_object_state()[0]));
  // AssemblyController -> Publisher (trajectory output)
  builder.Connect(assembly_controller->get_output_port_traj_execute(),
                  traj_pub->get_input_port(0));

  // Constant source for x_lcs_des
  VectorXd x_lcs_des =
      VectorXd::Zero(plant_lcs.num_positions() + plant_lcs.num_velocities());
  VectorXd x_lcs_des_positions = VectorXd::Zero(plant_lcs.num_positions());
  x_lcs_des_positions << 0.46, 0.198, 0.031, 0.5236, 0.0, 2.094, 0.48985, -0.1,
      0.03;
  x_lcs_des.segment(0, plant_lcs.num_positions()) = x_lcs_des_positions;
  auto x_lcs_des_source = builder.AddSystem<ConstantVectorSource>(x_lcs_des);
  builder.Connect(x_lcs_des_source->get_output_port(),
                  assembly_controller->get_input_port_target());

  // Publish C3+ target state
  auto lcmt_c3_state = dairlib::lcmt_c3_state();
  lcmt_c3_state.utime = 0;
  lcmt_c3_state.num_states = x_lcs_des.size();
  lcmt_c3_state.state = std::vector<float>(x_lcs_des.size());
  lcmt_c3_state.state_names = std::vector<std::string>(x_lcs_des.size());
  for (int i = 0; i < x_lcs_des.size(); i++) {
    lcmt_c3_state.state[i] = static_cast<float>(x_lcs_des(i));
    lcmt_c3_state.state_names[i] = "x_lcs_des[" + std::to_string(i) + "]";
  }
  auto x_lcs_des_abs_source = builder.AddSystem<ConstantValueSource<double>>(
      drake::Value<dairlib::lcmt_c3_state>(lcmt_c3_state));
  auto c3_state_target_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_state>(
          lcm_channel_params.c3_target_state_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  builder.Connect(x_lcs_des_abs_source->get_output_port(),
                  c3_state_target_pub->get_input_port(0));

  auto gripper_pos_command_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_franka_hand_target_position>(
          lcm_channel_params.franka_hand_target_position_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  builder.Connect(assembly_controller->get_output_port_gripper_pos_command(),
                  gripper_pos_command_pub->get_input_port(0));

  // Build diagram
  auto owned_diagram = builder.Build();
  std::shared_ptr<Diagram<double>> shared_diagram = std::move(owned_diagram);
  shared_diagram->set_name(("assembly_controller_diagram"));
  dairlib::DrawAndSaveDiagramGraph(*shared_diagram);
  // Run lcm-driven simulation
  dairlib::systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, shared_diagram, robot_state_receiver,
      lcm_channel_params.franka_state_channel, true);
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
