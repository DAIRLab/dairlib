#include <drake/systems/primitives/constant_vector_source.h>
#include <gflags/gflags.h>

#include "common/find_resource.h"
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
#include "drake/systems/primitives/constant_vector_source.h"

namespace dairlib {
namespace magna {

static constexpr const char* kFrankaModel =
    "package://drake_models/franka_description/urdf/"
    "panda_arm_hand_with_long_fingers.urdf";

using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::BasicVector;
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

  // Create plant for AssemblyController
  DiagramBuilder<double> builder;

  // Create LCS plant
  MultibodyPlant<double> plant(0.0);
  Parser parser(&plant, nullptr);
  parser.AddModels(dairlib::FindResourceOrThrow(
      "examples/magna/urdf/round_belt_task/end_effector_simple_model.urdf"));
  parser.AddModels(dairlib::FindResourceOrThrow(
      "examples/magna/urdf/round_belt_task/belt_element.urdf"));
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link"),
                   RigidTransform<double>::Identity());
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("belt_element_base_link"),
                   RigidTransform<double>::Identity());
  plant.AddForceElement<drake::multibody::LinearSpringDamper>(
      plant.GetBodyByName("end_effector_simple"),
      drake::Vector3<double>(0, 0, 0), plant.GetBodyByName("belt_element"),
      drake::Vector3<double>(0, 0, 0), 0.24, 1000, 0.1);
  plant.Finalize();
  std::cout << "plant.num_positions(): " << plant.num_positions() << std::endl;
  std::cout << "plant.num_velocities(): " << plant.num_velocities()
            << std::endl;
  auto plant_context = plant.CreateDefaultContext();

  // Create AutoDiff plant
  std::unique_ptr<MultibodyPlant<drake::AutoDiffXd>> plant_ad =
      drake::systems::System<double>::ToAutoDiffXd(plant);
  auto plant_ad_context = plant_ad->CreateDefaultContext();

  // Contact geometry pairs - empty for now
  std::vector<std::vector<drake::SortedPair<drake::geometry::GeometryId>>>
      contact_geoms;

  // Create AssemblyController
  auto assembly_controller = builder.AddSystem<AssemblyController>(
      plant, plant_context.get(), *plant_ad, plant_ad_context.get(),
      contact_geoms, assembly_c3_options, target_poses_params);

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
}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::magna::DoMain(argc, argv); }
