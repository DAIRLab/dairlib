#include <dairlib/lcmt_radio_out.hpp>
#include <gflags/gflags.h>

// Core includes for configuration, system building, and OSC
#include "common/eigen_utils.h"
#include "examples/plate-balancing/parameters/lcm_channel_config.h"
#include "examples/plate-balancing/parameters/osc_controller_config.h"
#include "examples/plate-balancing/parameters/plate_balancing_config.h"
#include "examples/plate-balancing/systems/end_effector_force.h"
#include "examples/plate-balancing/systems/end_effector_orientation.h"
#include "examples/plate-balancing/systems/end_effector_position.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
#include "systems/controllers/gravity_compensator.h"
#include "systems/controllers/osc/end_effector_force.h"
#include "systems/controllers/osc/end_effector_orientation.h"
#include "systems/controllers/osc/end_effector_position.h"
#include "systems/controllers/osc/external_force_tracking_data.h"
#include "systems/controllers/osc/joint_space_tracking_data.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/controllers/osc/relative_translation_tracking_data.h"
#include "systems/controllers/osc/rot_space_tracking_data.h"
#include "systems/controllers/osc/trans_space_tracking_data.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/lcmt_systems/robot_input_systems.h"
#include "systems/lcmt_systems/robot_output_systems.h"
#include "systems/primitives/radio_parser.h"
#include "systems/system_utils.h"
#include "systems/trajectory_optimization/lcm_trajectory_systems.h"

#include "drake/common/find_resource.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

using drake::math::RigidTransform;
using drake::multibody::Parser;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

// Command-line flags for configuration
DEFINE_string(plate_balancing_config,
              "examples/plate-balancing/config/plate_balancing_config.yaml",
              "YAML file specifying controller and channel settings.");
DEFINE_bool(simulation, true, "Run in simulation (true) or hardware (false)");

namespace dairlib {

using multibody::MakeNameToPositionsMap;
using multibody::MakeNameToVelocitiesMap;
using systems::GravityCompensationRemover;
using systems::LcmC3TrajectoryReceiver;
using systems::LcmDrivenLoop;
using systems::RadioToVector;
using systems::controllers::ExternalForceTrackingData;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::OperationalSpaceControl;
using systems::controllers::RelativeTranslationTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;
using systems::lcmt_systems::RobotInputGenerator;
using systems::lcmt_systems::RobotOutputConsumer;

namespace examples {
namespace plate_balancing {

// Entry point for the OSC controller application
int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Load all configuration parameters from YAML files
  drake::yaml::LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;
  PlateBalancingConfig main_config =
      drake::yaml::LoadYamlFile<PlateBalancingConfig>(
          FLAGS_plate_balancing_config);
  LcmChannelConfig lcm_channel_params =
      drake::yaml::LoadYamlFile<LcmChannelConfig>(
          FLAGS_simulation ? main_config.lcm_simulation_settings_file
                           : main_config.lcm_hardware_settings_file);
  OSCControllerConfig controller_config =
      drake::yaml::LoadYamlFile<OSCControllerConfig>(
          main_config.osc_contoller_config_file);
  OSCGains gains = drake::yaml::LoadYamlFile<OSCGains>(
      FindResourceOrThrow(main_config.osc_contoller_config_file), {}, {},
      yaml_options);
  drake::solvers::SolverOptions solver_options =
      drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
          FindResourceOrThrow(main_config.osc_osqp_setting_file))
          .GetAsSolverOptions(drake::solvers::OsqpSolver::id());

  // Build the system diagram for the OSC controller
  DiagramBuilder<double> builder;

  // Construct the MultibodyPlant for the robot and parse models
  drake::multibody::MultibodyPlant<double> plant(0.0);
  Parser parser(&plant, nullptr);
  parser.AddModelsFromUrl(controller_config.franka_model);

  // Weld the robot base to the world frame
  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"),
                   X_WI);

  // Optionally add and weld the end effector, if specified in config
  if (!controller_config.end_effector_name.empty()) {
    drake::multibody::ModelInstanceIndex end_effector_index = parser.AddModels(
        FindResourceOrThrow(controller_config.end_effector_model))[0];
    RigidTransform<double> T_EE_W =
        RigidTransform<double>(drake::math::RotationMatrix<double>(),
                               controller_config.tool_attachment_frame);
    plant.WeldFrames(plant.GetFrameByName("panda_link7"),
                     plant.GetFrameByName(controller_config.end_effector_name,
                                          end_effector_index),
                     T_EE_W);
  } else {
    std::cout << "OSC plant has been constructed with no end effector."
              << std::endl;
  }

  plant.Finalize();
  auto plant_context = plant.CreateDefaultContext();

  // Create LCM interface for communication
  drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0");

  // Add LCM systems for state, command, and trajectory communication
  auto state_receiver = builder.AddSystem<RobotOutputConsumer>(plant);
  auto end_effector_trajectory_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<c3::lcmt_c3_trajectory>(
          lcm_channel_params.c3_actor_channel, &lcm));
  auto end_effector_position_receiver =
      builder.AddSystem<LcmC3TrajectoryReceiver>(
          "end_effector_position_target");
  auto end_effector_force_receiver =
      builder.AddSystem<LcmC3TrajectoryReceiver>("end_effector_force_target");
  auto end_effector_orientation_receiver =
      builder.AddSystem<LcmC3TrajectoryReceiver>(
          "end_effector_orientation_target", true);
  auto franka_command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          lcm_channel_params.franka_input_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto osc_command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          lcm_channel_params.osc_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto franka_command_sender = builder.AddSystem<RobotInputGenerator>(plant);
  auto osc_command_sender = builder.AddSystem<RobotInputGenerator>(plant);

  // Trajectory generators for end effector position, orientation, and force
  auto end_effector_trajectory =
      builder.AddSystem<systems::EndEffectorTrajectoryGenerator>(
          controller_config.neutral_position,
          controller_config.ignore_messages_count.value_or(0));
  end_effector_trajectory->SetRemoteControlParameters(
      controller_config.neutral_position, controller_config.x_scale,
      controller_config.y_scale, controller_config.z_scale);
  auto end_effector_orientation_trajectory =
      builder.AddSystem<systems::EndEffectorOrientationTrajectoryGenerator>();
  end_effector_orientation_trajectory->SetTrackOrientation(
      controller_config.track_end_effector_orientation);
  auto end_effector_force_trajectory =
      builder.AddSystem<systems::EndEffectorForceTrajectoryGenerator>(
          controller_config.ignore_messages_count.value_or(0));

  // Radio input for remote control
  auto radio_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
          lcm_channel_params.radio_channel, &lcm));
  auto radio_to_vector = builder.AddSystem<RadioToVector>();

  // Add and configure the Operational Space Controller (OSC)
  auto osc = builder.AddSystem<OperationalSpaceControl>(
      plant, plant_context.get(), false);

  // Optionally add debug publisher for OSC internal state
  if (controller_config.publish_debug_info) {
    auto osc_debug_pub =
        builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
            lcm_channel_params.osc_debug_channel, &lcm,
            TriggerTypeSet({TriggerType::kForced})));
    builder.Connect(osc->get_output_port_osc_debug(),
                    osc_debug_pub->get_input_port());
  }

  // Configure tracking data for OSC: position, orientation, force, and joint
  auto end_effector_position_tracking_data =
      std::make_unique<TransTaskSpaceTrackingData>(
          "end_effector_target", controller_config.K_p_end_effector,
          controller_config.K_d_end_effector, controller_config.W_end_effector,
          plant, plant);
  end_effector_position_tracking_data->AddPointToTrack(
      controller_config.end_effector_name);
  const VectorXd& end_effector_acceleration_limits =
      controller_config.end_effector_acceleration * Vector3d::Ones();
  end_effector_position_tracking_data->SetCmdAccelerationBounds(
      -end_effector_acceleration_limits, end_effector_acceleration_limits);

  auto mid_link_position_tracking_data_for_rel =
      std::make_unique<JointSpaceTrackingData>(
          "panda_joint2_target", controller_config.K_p_mid_link,
          controller_config.K_d_mid_link, controller_config.W_mid_link, plant,
          plant);
  mid_link_position_tracking_data_for_rel->AddJointToTrack("panda_joint2",
                                                           "panda_joint2dot");

  auto end_effector_force_tracking_data =
      std::make_unique<ExternalForceTrackingData>(
          "end_effector_force", controller_config.W_ee_lambda, plant, plant,
          controller_config.end_effector_name, Vector3d::Zero());

  auto end_effector_orientation_tracking_data =
      std::make_unique<RotTaskSpaceTrackingData>(
          "end_effector_orientation_target",
          controller_config.K_p_end_effector_rot,
          controller_config.K_d_end_effector_rot,
          controller_config.W_end_effector_rot, plant, plant);
  end_effector_orientation_tracking_data->AddFrameToTrack(
      controller_config.end_effector_name);

  Eigen::VectorXd orientation_target = Eigen::VectorXd::Zero(4);
  orientation_target(0) = 1;

  // Register all tracking data with the OSC
  osc->AddTrackingData(std::move(end_effector_position_tracking_data));
  osc->AddConstTrackingData(std::move(mid_link_position_tracking_data_for_rel),
                            1.6 * VectorXd::Ones(1));
  osc->AddTrackingData(std::move(end_effector_orientation_tracking_data));
  osc->AddForceTrackingData(std::move(end_effector_force_tracking_data));
  osc->SetAccelerationCostWeights(gains.W_acceleration);
  osc->SetInputCostWeights(gains.W_input_regularization);
  osc->SetInputSmoothingCostWeights(gains.W_input_smoothing_regularization);

  if (controller_config.enforce_acceleration_constraints) {
    osc->EnableAccelerationConstraints();
  } else {
    osc->DisableAccelerationConstraints();
  }
  osc->SetContactFriction(controller_config.mu);
  osc->SetOsqpSolverOptions(solver_options);

  osc->Build();

  // Connect all systems in the diagram

  // If enabled, remove gravity compensation from OSC output before sending to
  // robot
  if (controller_config.cancel_gravity_compensation) {
    auto gravity_compensator =
        builder.AddSystem<GravityCompensationRemover>(plant, *plant_context);
    builder.Connect(osc->get_output_port_osc_command(),
                    gravity_compensator->get_input_port());
    builder.Connect(gravity_compensator->get_output_port(),
                    franka_command_sender->get_input_port());
  } else {
    if (!FLAGS_simulation) {
      std::cerr << "Using hardware lcm channels but not cancelling gravity "
                   "compensation. Please check the OSC settings"
                << std::endl;
      return -1;
    }
    builder.Connect(osc->get_output_port_osc_command(),
                    franka_command_sender->get_input_port(0));
  }

  // Connect radio input to trajectory generators
  builder.Connect(*radio_sub, *radio_to_vector);
  builder.Connect(radio_to_vector->get_output_port(),
                  end_effector_trajectory->get_input_port_radio());
  builder.Connect(radio_to_vector->get_output_port(),
                  end_effector_orientation_trajectory->get_input_port_radio());
  builder.Connect(radio_to_vector->get_output_port(),
                  end_effector_force_trajectory->get_input_port_radio());

  // Connect command senders to LCM publishers
  builder.Connect(franka_command_sender->get_output_port(),
                  franka_command_pub->get_input_port());
  builder.Connect(osc_command_sender->get_output_port(),
                  osc_command_pub->get_input_port());
  builder.Connect(osc->get_output_port_osc_command(),
                  osc_command_sender->get_input_port(0));

  // Connect state and trajectory receivers to OSC and trajectory generators
  builder.Connect(state_receiver->get_output_port(0),
                  osc->get_input_port_robot_output());
  builder.Connect(
      end_effector_trajectory_sub->get_output_port(),
      end_effector_position_receiver->get_input_port_lcm_trajectory());
  builder.Connect(end_effector_trajectory_sub->get_output_port(),
                  end_effector_force_receiver->get_input_port_lcm_trajectory());
  builder.Connect(
      end_effector_trajectory_sub->get_output_port(),
      end_effector_orientation_receiver->get_input_port_lcm_trajectory());
  builder.Connect(end_effector_position_receiver->get_output_port(0),
                  end_effector_trajectory->get_input_port_trajectory());
  builder.Connect(
      end_effector_orientation_receiver->get_output_port(0),
      end_effector_orientation_trajectory->get_input_port_trajectory());
  builder.Connect(end_effector_trajectory->get_output_port(0),
                  osc->get_input_port_tracking_data("end_effector_target"));
  builder.Connect(
      end_effector_orientation_trajectory->get_output_port(0),
      osc->get_input_port_tracking_data("end_effector_orientation_target"));
  builder.Connect(end_effector_force_receiver->get_output_port(0),
                  end_effector_force_trajectory->get_input_port_trajectory());
  builder.Connect(end_effector_force_trajectory->get_output_port(0),
                  osc->get_input_port_tracking_data("end_effector_force"));

  // Build the complete system diagram
  auto owned_diagram = builder.Build();
  std::shared_ptr<Diagram<double>> shared_diagram = std::move(owned_diagram);
  shared_diagram->set_name(("plate_balancing/osc_controller"));
  DrawAndSaveDiagramGraph(*shared_diagram);
  // Run the LCM-driven control loop (real-time or simulation)
  LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, shared_diagram, state_receiver,
      lcm_channel_params.franka_state_channel, true);

  loop.Simulate();
  return 0;
}
}  // namespace plate_balancing
}  // namespace examples
}  // namespace dairlib

// Main entry point
int main(int argc, char* argv[]) {
  return dairlib::examples::plate_balancing::DoMain(argc, argv);
}