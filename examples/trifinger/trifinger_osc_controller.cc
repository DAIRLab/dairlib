#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "dairlib/lcmt_estimated_joint_friction_trifinger.hpp"
#include "dairlib/lcmt_fingertips_delta_position.hpp"
#include "examples/trifinger/systems/fingertips_delta_position_receiver.h"
#include "examples/trifinger/systems/fingertips_target_traj_demultiplexer.h"
#include "examples/trifinger/systems/trifinger_joint_friction_compensation.h"
#include "multibody/multibody_utils.h"
#include "parameters/trifinger_lcm_channels.h"
#include "parameters/trifinger_osc_controller_params.h"
#include "systems/controllers/osc/joint_space_tracking_data.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/controllers/osc/relative_translation_tracking_data.h"
#include "systems/controllers/osc/rot_space_tracking_data.h"
#include "systems/controllers/osc/trans_space_tracking_data.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace dairlib {

using drake::math::RigidTransform;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using multibody::MakeNameToPositionsMap;
using multibody::MakeNameToVelocitiesMap;

// using systems::controllers::ExternalForceTrackingData;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::RelativeTranslationTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;

DEFINE_string(osqp_settings,
              "examples/trifinger/parameters"
              "/trifinger_osc_qp_settings.yaml",
              "Filepath containing qp settings");
DEFINE_string(controller_parameters,
              "examples/trifinger/parameters/trifinger_osc_controller_params"
              ".yaml",
              "Controller settings such as channels. Attempting to minimize "
              "number of gflags");
DEFINE_string(lcm_channels,
              "examples/trifinger/parameters/lcm_channels_simulation.yaml",
              "Filepath containing lcm channels");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // load parameters from yaml files.
  drake::yaml::LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;

  auto controller_params = drake::yaml::LoadYamlFile<TrifingerControllerParams>(
      FLAGS_controller_parameters);

  auto lcm_channel_params =
      drake::yaml::LoadYamlFile<TrifingerLcmChannels>(FLAGS_lcm_channels);

  auto gains = drake::yaml::LoadYamlFile<OSCGains>(
      FindResourceOrThrow(FLAGS_controller_parameters), {}, {}, yaml_options);
  drake::solvers::SolverOptions solver_options =
      drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
          FindResourceOrThrow(FLAGS_osqp_settings))
          .GetAsSolverOptions(drake::solvers::OsqpSolver::id());

  // start building diagram.
  DiagramBuilder<double> builder;

  // create plant.
  drake::multibody::MultibodyPlant<double> plant(0.0);
  Parser parser(&plant, nullptr);
  parser.AddModels(FindResourceOrThrow(controller_params.trifinger_model));
  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link"),
                   X_WI);
  plant.Finalize();
  auto plant_context = plant.CreateDefaultContext();

  // create lcm systems.
  drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0");
  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);

  auto fingertips_delta_position_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_fingertips_delta_position>(
          lcm_channel_params.fingertips_delta_position_channel, &lcm));

  auto fingertips_delta_position_receiver =
      builder.AddSystem<systems::FingertipDeltaPositionReceiver>(
          plant, plant_context.get(),
          controller_params.min_fingertips_delta_position,
          controller_params.max_fingertips_delta_position,
          controller_params.fingertip_0_name,
          controller_params.fingertip_120_name,
          controller_params.fingertip_240_name,
          controller_params.delta_pos_update_frequency);

  auto trifinger_cur_fingertips_pos_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_fingertips_position>(
          lcm_channel_params.fingertips_position_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));

  builder.Connect(fingertips_delta_position_receiver
                      ->get_output_port_lcm_cur_fingertips_pos(),
                  trifinger_cur_fingertips_pos_pub->get_input_port());

  auto trifinger_command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          lcm_channel_params.trifinger_input_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto trifinger_command_sender =
      builder.AddSystem<systems::RobotCommandSender>(plant);

  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      plant, plant_context.get(), false);
  auto osc_debug_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
          lcm_channel_params.osc_debug_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto fingertip_0_position_tracking_data =
      std::make_unique<TransTaskSpaceTrackingData>(
          "fingertip_0_target", controller_params.Kp_fingertip_0,
          controller_params.Kd_fingertip_0, controller_params.W_fingertip_0,
          plant, plant);
  fingertip_0_position_tracking_data->AddPointToTrack(
      controller_params.fingertip_0_name);
  auto fingertip_120_position_tracking_data =
      std::make_unique<TransTaskSpaceTrackingData>(
          "fingertip_120_target", controller_params.Kp_fingertip_120,
          controller_params.Kd_fingertip_120, controller_params.W_fingertip_120,
          plant, plant);
  fingertip_120_position_tracking_data->AddPointToTrack(
      controller_params.fingertip_120_name);
  auto fingertip_240_position_tracking_data =
      std::make_unique<TransTaskSpaceTrackingData>(
          "fingertip_240_target", controller_params.Kp_fingertip_240,
          controller_params.Kd_fingertip_240, controller_params.W_fingertip_240,
          plant, plant);
  fingertip_240_position_tracking_data->AddPointToTrack(
      controller_params.fingertip_240_name);

  // Disable feed-forward acceleration
  std::set<int> disabled_indices;
  for (int i = 0; i < 3; ++i) {
    disabled_indices.emplace_hint(disabled_indices.end(), i);
  }
  fingertip_0_position_tracking_data->DisableFeedforwardAccel(disabled_indices);
  fingertip_120_position_tracking_data->DisableFeedforwardAccel(
      disabled_indices);
  fingertip_240_position_tracking_data->DisableFeedforwardAccel(
      disabled_indices);

  osc->AddTrackingData(std::move(fingertip_0_position_tracking_data));
  osc->AddTrackingData(std::move(fingertip_120_position_tracking_data));
  osc->AddTrackingData(std::move(fingertip_240_position_tracking_data));
  osc->SetInputCostWeights(gains.W_input_regularization);
  osc->SetOsqpSolverOptions(solver_options);
  osc->Build();

  auto target_traj_demultiplexer =
      builder.AddSystem<dairlib::systems::FingertipsTargetTrajDemultiplexer>(
          plant_context.get());
  builder.Connect(fingertips_delta_position_sub->get_output_port(),
                  fingertips_delta_position_receiver
                      ->get_input_port_fingertips_delta_position());
  builder.Connect(state_receiver->get_output_port(0),
                  fingertips_delta_position_receiver->get_input_port_state());

  builder.Connect(fingertips_delta_position_receiver
                      ->get_output_port_fingertips_target_traj(),
                  target_traj_demultiplexer->get_input_port_traj());
  builder.Connect(
      target_traj_demultiplexer->get_output_port_fingertip_0_target_traj(),
      osc->get_input_port_tracking_data("fingertip_0_target"));
  builder.Connect(
      target_traj_demultiplexer->get_output_port_fingertip_120_target_traj(),
      osc->get_input_port_tracking_data("fingertip_120_target"));
  builder.Connect(
      target_traj_demultiplexer->get_output_port_fingertip_240_target_traj(),
      osc->get_input_port_tracking_data("fingertip_240_target"));

  if (controller_params.require_friction_compensation) {
    auto joint_friction_compensator =
        builder.AddSystem<dairlib::systems::JointFrictionCompensator>(
            plant, plant_context.get());
    auto estimated_friction_torque_pub =
        builder.AddSystem(LcmPublisherSystem::Make<
                          dairlib ::lcmt_estimated_joint_friction_trifinger>(
            lcm_channel_params.estimated_friction_torque, &lcm,
            TriggerTypeSet({TriggerType::kForced})));
    builder.Connect(state_receiver->get_output_port(0),
                    joint_friction_compensator->get_input_port_state());
    builder.Connect(
        osc->get_output_port_osc_command(),
        joint_friction_compensator->get_uncompensated_torque_input_port());
    builder.Connect(
        joint_friction_compensator->get_compensated_torque_output_port(),
        trifinger_command_sender->get_input_port(0));
    builder.Connect(
        joint_friction_compensator->get_estimated_friction_torque_port(),
        estimated_friction_torque_pub->get_input_port());
  } else {
    builder.Connect(osc->get_output_port_osc_command(),
                    trifinger_command_sender->get_input_port(0));
  }

  builder.Connect(trifinger_command_sender->get_output_port(),
                  trifinger_command_pub->get_input_port());
  builder.Connect(osc->get_output_port_osc_debug(),
                  osc_debug_pub->get_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  osc->get_input_port_robot_output());

  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("trifinger_osc_controller"));

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