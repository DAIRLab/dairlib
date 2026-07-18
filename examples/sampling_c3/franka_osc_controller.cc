
#include <dairlib/lcmt_radio_out.hpp>
#include <dairlib/lcmt_timestamped_saved_traj.hpp>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "examples/sampling_c3/parameter_headers/lcm_channels.h"
#include "examples/sampling_c3/parameter_headers/osc_params.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_controller_params.h"
#include "examples/sampling_c3/sampling_c3_utils.h"
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
#include "systems/robot_lcm_systems.h"
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

namespace dairlib {

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
using multibody::MakeNameToPositionsMap;
using multibody::MakeNameToVelocitiesMap;

using systems::controllers::ExternalForceTrackingData;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::RelativeTranslationTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;

DEFINE_bool(is_simulation, true, "True for simulation, false for hardware");
DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");
DEFINE_string(demo_name, "jacktoy",
              "Demo within sampling_c3; used to find controller params file");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::lcm::DrakeLcm lcm(FLAGS_lcm_url);

  // Load parameters.
  std::string controller_params_path =
      "examples/sampling_c3/" + FLAGS_demo_name +
      "/parameters/sampling_c3_controller_params.yaml";
  SamplingC3ControllerParams controller_params =
      drake::yaml::LoadYamlFile<SamplingC3ControllerParams>(
          controller_params_path);
  SamplingC3OSCParams osc_params =
      drake::yaml::LoadYamlFile<SamplingC3OSCParams>(
          controller_params.osc_params_file);
  std::string lcm_channels_file =
      FLAGS_is_simulation ? controller_params.lcm_channels_simulation_file
                          : controller_params.lcm_channels_hardware_file;
  SamplingC3LcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<SamplingC3LcmChannels>(lcm_channels_file);
  drake::solvers::SolverOptions solver_options =
      drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
          FindResourceOrThrow(controller_params.osc_qp_settings_file))
          .GetAsSolverOptions(drake::solvers::OsqpSolver::id());

  // Create a Franka-only plant.
  drake::multibody::MultibodyPlant<double> plant(0.0);
  AddFrankaToPlant(&plant);
  plant.Finalize();
  auto plant_context = plant.CreateDefaultContext();

  // Piece together the diagram.
  DiagramBuilder<double> builder;

  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);
  auto end_effector_trajectory_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.tracking_trajectory_actor_channel, &lcm));
  auto end_effector_position_receiver =
      builder.AddSystem<systems::LcmTrajectoryReceiver>(
          "end_effector_position_target");
  auto end_effector_force_receiver =
      builder.AddSystem<systems::LcmTrajectoryReceiver>(
          "end_effector_force_target");
  auto end_effector_orientation_receiver =
      builder.AddSystem<systems::LcmOrientationTrajectoryReceiver>(
          "end_effector_orientation_target");
  auto franka_command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          lcm_channel_params.franka_input_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto osc_command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          lcm_channel_params.osc_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto franka_command_sender =
      builder.AddSystem<systems::RobotCommandSender>(plant);
  auto osc_command_sender =
      builder.AddSystem<systems::RobotCommandSender>(plant);
  auto end_effector_trajectory =
      builder.AddSystem<EndEffectorPositionTrajectoryGenerator>(
          plant, plant_context.get(), osc_params.neutral_position,
          osc_params.teleop_neutral_position, kEndEffectorName);
  end_effector_trajectory->SetRemoteControlParameters(
      osc_params.neutral_position, osc_params.x_scale, osc_params.y_scale,
      osc_params.z_scale);
  auto end_effector_orientation_trajectory =
      builder.AddSystem<EndEffectorOrientationTrajectoryGenerator>();
  end_effector_orientation_trajectory->SetTrackOrientation(
      osc_params.track_end_effector_orientation);
  auto end_effector_force_trajectory =
      builder.AddSystem<EndEffectorForceTrajectoryGenerator>();
  auto radio_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
          lcm_channel_params.radio_channel, &lcm));
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      plant, plant_context.get(), false);
  if (osc_params.publish_debug_info) {
    auto osc_debug_pub =
        builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
            lcm_channel_params.osc_debug_channel, &lcm,
            TriggerTypeSet({TriggerType::kForced})));
    builder.Connect(osc->get_output_port_osc_debug(),
                    osc_debug_pub->get_input_port());
  }

  auto end_effector_position_tracking_data =
      std::make_unique<TransTaskSpaceTrackingData>(
          "end_effector_target", osc_params.K_p_end_effector,
          osc_params.K_d_end_effector, osc_params.W_end_effector, plant, plant);
  end_effector_position_tracking_data->AddPointToTrack(kEndEffectorName);
  const VectorXd& end_effector_acceleration_limits =
      osc_params.end_effector_acceleration * Vector3d::Ones();
  end_effector_position_tracking_data->SetCmdAccelerationBounds(
      -end_effector_acceleration_limits, end_effector_acceleration_limits);
  auto mid_link_position_tracking_data_for_rel =
      std::make_unique<JointSpaceTrackingData>(
          "panda_joint2_target", osc_params.K_p_mid_link,
          osc_params.K_d_mid_link, osc_params.W_mid_link, plant, plant);
  mid_link_position_tracking_data_for_rel->AddJointToTrack("panda_joint2",
                                                           "panda_joint2dot");

  auto end_effector_force_tracking_data =
      std::make_unique<ExternalForceTrackingData>(
          "end_effector_force", osc_params.W_ee_lambda, plant, plant,
          kEndEffectorName, Vector3d::Zero());

  auto end_effector_orientation_tracking_data =
      std::make_unique<RotTaskSpaceTrackingData>(
          "end_effector_orientation_target", osc_params.K_p_end_effector_rot,
          osc_params.K_d_end_effector_rot, osc_params.W_end_effector_rot, plant,
          plant);
  end_effector_orientation_tracking_data->AddFrameToTrack(kEndEffectorName);
  Eigen::VectorXd orientation_target = Eigen::VectorXd::Zero(4);
  orientation_target(0) = 1;
  osc->AddTrackingData(std::move(end_effector_position_tracking_data));
  // Since the Franka has 7 joints to control a 6 DOF EE command, add an
  // additional tracking objective for joint 2 at a good configuration for the
  // sampling C3 experiments.  1.1 joint target empirically works well.
  osc->AddConstTrackingData(std::move(mid_link_position_tracking_data_for_rel),
                            1.1 * VectorXd::Ones(1));
  osc->AddTrackingData(std::move(end_effector_orientation_tracking_data));
  osc->AddForceTrackingData(std::move(end_effector_force_tracking_data));
  osc->SetAccelerationCostWeights(osc_params.W_acceleration);
  osc->SetInputCostWeights(osc_params.W_input_regularization);
  osc->SetInputSmoothingCostWeights(
      osc_params.W_input_smoothing_regularization);
  if (osc_params.enforce_acceleration_constraints) {
    osc->EnableAccelerationConstraints();
  } else {
    osc->DisableAccelerationConstraints();
  }
  osc->SetContactFriction(osc_params.mu);
  osc->SetOsqpSolverOptions(solver_options);

  osc->Build();

  if (osc_params.cancel_gravity_compensation) {
    if (FLAGS_is_simulation) {
      std::cerr << "Sim OSC needs cancel_gravity_compensation: false"
                << std::endl;
      return -1;
      return -1;
    }
    auto gravity_compensator =
        builder.AddSystem<systems::GravityCompensationRemover>(plant,
                                                               *plant_context);
    builder.Connect(osc->get_output_port_osc_command(),
                    gravity_compensator->get_input_port());
    builder.Connect(gravity_compensator->get_output_port(),
                    franka_command_sender->get_input_port());
  } else {
    if (!FLAGS_is_simulation) {
      std::cerr << "HW OSC needs cancel_gravity_compensation: true"
                << std::endl;
      return -1;
    }
    builder.Connect(osc->get_output_port_osc_command(),
                    franka_command_sender->get_input_port(0));
  }

  builder.Connect(radio_sub->get_output_port(0),
                  end_effector_trajectory->get_input_port_radio());
  builder.Connect(radio_sub->get_output_port(0),
                  end_effector_orientation_trajectory->get_input_port_radio());
  builder.Connect(radio_sub->get_output_port(0),
                  end_effector_force_trajectory->get_input_port_radio());
  builder.Connect(franka_command_sender->get_output_port(),
                  franka_command_pub->get_input_port());
  builder.Connect(osc_command_sender->get_output_port(),
                  osc_command_pub->get_input_port());
  builder.Connect(osc->get_output_port_osc_command(),
                  osc_command_sender->get_input_port(0));

  builder.Connect(state_receiver->get_output_port(0),
                  osc->get_input_port_robot_output());
  builder.Connect(end_effector_trajectory_sub->get_output_port(),
                  end_effector_position_receiver->get_input_port_trajectory());
  builder.Connect(end_effector_trajectory_sub->get_output_port(),
                  end_effector_force_receiver->get_input_port_trajectory());
  builder.Connect(
      end_effector_trajectory_sub->get_output_port(),
      end_effector_orientation_receiver->get_input_port_trajectory());
  builder.Connect(end_effector_position_receiver->get_output_port(0),
                  end_effector_trajectory->get_input_port_trajectory());
  builder.Connect(state_receiver->get_output_port(0),
                  end_effector_trajectory->get_input_port_state());
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

  auto owned_diagram = builder.Build();
  std::shared_ptr<Diagram<double>> shared_diagram = std::move(owned_diagram);
  shared_diagram->set_name(("sampling_c3_franka_osc_controller"));
  DrawAndSaveDiagramGraph(*shared_diagram);
  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, shared_diagram, state_receiver,
      lcm_channel_params.franka_state_channel, true);
  loop.Simulate();
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
