#include "common/find_resource.h"
#include "dairlib/lcmt_fingertips_target_kinematics.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "gflags/gflags.h"
#include "parameters/trifinger_impedance_controller_params.h"
#include "parameters/trifinger_lcm_channels.h"
#include "systems/fingertips_target_kinematics_receiver.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"
#include "systems/trifinger_impedance_controller.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace dairlib {

DEFINE_string(
    impedance_controller_params,
    "examples/trifinger/parameters/trifinger_impedance_controller_params.yaml",
    "Config parameters for impedance controller");

DEFINE_string(lcm_channels,
              "examples/trifinger/parameters/lcm_channels_simulation.yaml",
              "Filepath containing lcm channels");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // load parameters from yaml files.
  drake::yaml::LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;

  auto controller_params =
      drake::yaml::LoadYamlFile<TrifingerImpedanceControllerParams>(
          FLAGS_impedance_controller_params);

  auto lcm_channel_params =
      drake::yaml::LoadYamlFile<TrifingerLcmChannels>(FLAGS_lcm_channels);

  // start building diagram.
  drake::systems::DiagramBuilder<double> builder;

  // create plant.
  drake::multibody::MultibodyPlant<double> plant(0.0);
  drake::multibody::Parser parser(&plant, nullptr);
  parser.AddModels(FindResourceOrThrow(controller_params.trifinger_model));
  drake::math::RigidTransform<double> X_WI =
      drake::math::RigidTransform<double>::Identity();
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link"),
                   X_WI);
  plant.Finalize();
  auto plant_context = plant.CreateDefaultContext();

  // create lcm systems.
  drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0");
  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);

  auto fingertips_target_kinematics_sub =
      builder.AddSystem(drake::systems::lcm::LcmSubscriberSystem::Make<
                        dairlib ::lcmt_fingertips_target_kinematics>(
          lcm_channel_params.fingertips_delta_position_channel, &lcm));

  auto fingertips_target_kinematics_receiver =
      builder.AddSystem<systems::FingertipTargetKinematicsReceiver>(
          plant, plant_context.get(),
          controller_params.fingertip_0_name,
          controller_params.fingertip_120_name,
          controller_params.fingertip_240_name,
          controller_params.delta_pos_update_frequency);

  auto trifinger_command_pub = builder.AddSystem(
      drake::systems::lcm::LcmPublisherSystem::Make<dairlib ::lcmt_robot_input>(
          lcm_channel_params.trifinger_input_channel, &lcm,
          drake::systems::TriggerTypeSet(
              {drake::systems::TriggerType::kForced})));
  auto trifinger_command_sender =
      builder.AddSystem<systems::RobotCommandSender>(plant);

  auto impedance_debug_output_pub =
      builder.AddSystem(drake::systems::lcm::LcmPublisherSystem::Make<
                        dairlib::lcmt_trifinger_impedance_debug>(
          lcm_channel_params.impedance_debug_channel, &lcm,
          drake::systems::TriggerTypeSet(
              {drake::systems::TriggerType::kForced})));
  auto impedance_controller =
      builder.AddSystem<systems::TrifingerImpedanceControl>(
          plant, plant_context.get(), controller_params.fingertip_0_name,
          controller_params.fingertip_120_name,
          controller_params.fingertip_240_name,
          controller_params.Kp_fingertip_0, controller_params.Kd_fingertip_0,
          controller_params.Kp_fingertip_120,
          controller_params.Kd_fingertip_120,
          controller_params.Kp_fingertip_240,
          controller_params.Kd_fingertip_240);

  builder.Connect(fingertips_target_kinematics_sub->get_output_port(),
                  fingertips_target_kinematics_receiver
                          ->get_input_port_fingertips_target_kinematics());
  builder.Connect(state_receiver->get_output_port(0),
                  fingertips_target_kinematics_receiver->get_input_port_state());
  builder.Connect(state_receiver->get_output_port(0),
                  impedance_controller->get_input_port_state());
  builder.Connect(
          fingertips_target_kinematics_receiver->get_output_(),
          impedance_controller->get_input_port_fingertips_delta_position());
  builder.Connect(impedance_controller->get_commanded_torque_port(),
                  trifinger_command_sender->get_input_port(0));
  builder.Connect(trifinger_command_sender->get_output_port(0),
                  trifinger_command_pub->get_input_port());
  builder.Connect(impedance_controller->get_impedance_debug_output_port(),
                  impedance_debug_output_pub->get_input_port());

  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("trifinger_impedance_controller"));

  // Run lcm-driven simulation.
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, std::move(owned_diagram), state_receiver,
      lcm_channel_params.trifinger_state_channel, true);
  DrawAndSaveDiagramGraph(*loop.get_diagram());
  loop.Simulate();
  return 0;
}
}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }