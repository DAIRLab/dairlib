#include <dairlib/lcmt_radio_out.hpp>
#include <drake/common/find_resource.h>
#include <drake/common/yaml/yaml_io.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake/systems/lcm/lcm_subscriber_system.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/multiplexer.h>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "common/find_resource.h"
#include "examples/plate-balancing/parameters/c3_scene_config.h"
#include "examples/plate-balancing/parameters/lcm_channel_config.h"
#include "examples/plate-balancing/parameters/plate_balancing_config.h"
#include "examples/plate-balancing/parameters/plate_balancing_target_config.h"
#include "examples/plate-balancing/systems/franka_kinematics.h"
#include "examples/plate-balancing/systems/plate_balancing_target.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/lcmt_generators/robot_state_generator.h"
#include "systems/primitives/radio_parser.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

DEFINE_string(plate_balancing_config,
              "examples/plate-balancing/config/plate_balancing_config.yaml",
              "Controller settings such as channels. Attempting to minimize "
              "number of gflags");
DEFINE_bool(simulation, true, "Running in simulation or hardware");

using drake::geometry::GeometryId;
using drake::math::RigidTransform;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::multibody::MakeNameToPositionsMap;
using dairlib::multibody::MakeNameToVelocitiesMap;
using dairlib::systems::LcmDrivenLoop;
using dairlib::systems::ObjectStateReceiver;
using dairlib::systems::RadioToVector;
using dairlib::systems::RobotOutputReceiver;
using dairlib::systems::lcmt_generators::RobotStateGenerator;

namespace dairlib {
namespace examples {
namespace plate_balancing {

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0");

  // Load parameters from YAML files
  drake::yaml::LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;

  PlateBalancingConfig main_config =
      drake::yaml::LoadYamlFile<PlateBalancingConfig>(
          FLAGS_plate_balancing_config);
  PlateBalancingTargetConfig target_config =
      drake::yaml::LoadYamlFile<PlateBalancingTargetConfig>(
          main_config.plate_balancing_target_config_file);
  C3SceneConfig scene_params = drake::yaml::LoadYamlFile<C3SceneConfig>(
      main_config.get_c3_scene_config_file());
  LcmChannelConfig lcm_channel_params =
      drake::yaml::LoadYamlFile<LcmChannelConfig>(
          FLAGS_simulation ? main_config.lcm_simulation_settings_file
                           : main_config.lcm_hardware_settings_file);

  // Build the MultibodyPlant for Franka
  DiagramBuilder<double> plant_builder;
  MultibodyPlant<double> plant_franka(0.0);
  Parser parser_franka(&plant_franka, nullptr);
  parser_franka.AddModelsFromUrl(scene_params.franka_model);
  drake::multibody::ModelInstanceIndex end_effector_index =
      parser_franka.AddModels(
          FindResourceOrThrow(scene_params.end_effector_model))[0];

  // Weld frames for Franka
  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant_franka.WeldFrames(plant_franka.world_frame(),
                          plant_franka.GetFrameByName("panda_link0"), X_WI);

  RigidTransform<double> T_EE_W =
      RigidTransform<double>(drake::math::RotationMatrix<double>(),
                             scene_params.tool_attachment_frame);
  plant_franka.WeldFrames(
      plant_franka.GetFrameByName("panda_link7"),
      plant_franka.GetFrameByName("plate", end_effector_index), T_EE_W);

  plant_franka.Finalize();
  auto franka_context = plant_franka.CreateDefaultContext();

  // Build the MultibodyPlant for the tray
  MultibodyPlant<double> plant_tray(0.0);
  Parser parser_tray(&plant_tray, nullptr);
  parser_tray.AddModels(scene_params.object_models[0]);
  plant_tray.Finalize();
  auto tray_context = plant_tray.CreateDefaultContext();

  // Build the overall diagram
  DiagramBuilder<double> builder;

  // Add LCM subscribers for tray and radio state
  auto tray_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.tray_state_channel, &lcm));
  auto radio_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
          lcm_channel_params.radio_channel, &lcm));

  // Add state receivers and kinematics system
  auto franka_state_receiver =
      builder.AddSystem<RobotOutputReceiver>(plant_franka);
  auto tray_state_receiver = builder.AddSystem<ObjectStateReceiver>(plant_tray);
  auto reduced_order_model_receiver =
      builder.AddSystem<systems::FrankaKinematics>(
          plant_franka, franka_context.get(), plant_tray, tray_context.get(),
          scene_params.end_effector_name, "tray",
          main_config.include_end_effector_orientation);

  // Add systems for radio parsing and C3 state handling
  auto radio_to_vector = builder.AddSystem<RadioToVector>();

  // Add plate balancing target generator
  auto plate_balancing_target =
      builder.AddSystem<systems::PlateBalancingTargetGenerator>(
          plant_tray, scene_params.end_effector_thickness,
          target_config.near_target_threshold);
  plate_balancing_target->SetRemoteControlParameters(
      target_config.first_target[main_config.scene_index],
      target_config.second_target[main_config.scene_index],
      target_config.third_target[main_config.scene_index],
      target_config.x_scale, target_config.y_scale, target_config.z_scale);

  // Add multiplexer for target state
  std::vector<int> input_sizes = {3, 7, 3, 6};
  auto target_state_mux =
      builder.AddSystem<drake::systems::Multiplexer>(input_sizes);

  // Add constant vector sources for zero velocities
  auto end_effector_zero_velocity_source =
      builder.AddSystem<drake::systems::ConstantVectorSource>(
          VectorXd::Zero(3));
  auto tray_zero_velocity_source =
      builder.AddSystem<drake::systems::ConstantVectorSource>(
          VectorXd::Zero(6));

  // Connect the systems
  builder.Connect(*radio_sub, *radio_to_vector);
  builder.Connect(tray_state_receiver->get_output_port(),
                  plate_balancing_target->get_input_port_tray_state());
  builder.Connect(plate_balancing_target->get_output_port_end_effector_target(),
                  target_state_mux->get_input_port(0));
  builder.Connect(plate_balancing_target->get_output_port_tray_target(),
                  target_state_mux->get_input_port(1));
  builder.Connect(tray_state_sub->get_output_port(),
                  tray_state_receiver->get_input_port());
  builder.Connect(radio_to_vector->get_output_port(),
                  plate_balancing_target->get_input_port_radio());
  builder.Connect(franka_state_receiver->get_output_port(),
                  reduced_order_model_receiver->get_input_port_franka_state());
  builder.Connect(tray_state_receiver->get_output_port(),
                  reduced_order_model_receiver->get_input_port_object_state());
  builder.Connect(end_effector_zero_velocity_source->get_output_port(),
                  target_state_mux->get_input_port(2));
  builder.Connect(tray_zero_velocity_source->get_output_port(),
                  target_state_mux->get_input_port(3));

  // Define state names for C3 state sender
  std::vector<std::string> state_names = {
      "end_effector_x",  "end_effector_y", "end_effector_z",  "tray_qw",
      "tray_qx",         "tray_qy",        "tray_qz",         "tray_x",
      "tray_y",          "tray_z",         "end_effector_vx", "end_effector_vy",
      "end_effector_vz", "tray_wx",        "tray_wy",         "tray_wz",
      "tray_vz",         "tray_vz",        "tray_vz",
  };
  RobotStateGenerator::AddLcmPublisherToBuilder(
      builder, state_names, false, target_state_mux->get_output_port(),
      lcm_channel_params.c3_target_state_channel, &lcm,
      TriggerTypeSet({TriggerType::kForced}));
  RobotStateGenerator::AddLcmPublisherToBuilder(
      builder, state_names, true,
      reduced_order_model_receiver->get_output_port_lcs_state(),
      lcm_channel_params.c3_actual_state_channel, &lcm,
      TriggerTypeSet({TriggerType::kForced}));

  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("franka_forward_kinematics"));

  // Run lcm-driven simulation
  LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, std::move(owned_diagram), franka_state_receiver,
      lcm_channel_params.franka_state_channel, true);
  DrawAndSaveDiagramGraph(*loop.get_diagram());

  LcmHandleSubscriptionsUntil(
      &lcm, [&]() { return tray_state_sub->GetInternalMessageCount() > 1; });
  loop.Simulate();
  return 0;
}

}  // namespace plate_balancing
}  // namespace examples
}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::examples::plate_balancing::DoMain(argc, argv);
}