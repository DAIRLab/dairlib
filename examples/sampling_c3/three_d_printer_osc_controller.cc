#include <iostream>
#include <limits>
#include <string>
#include <vector>

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
#include "systems/three_d_printer_kinematics.h"
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

namespace {

/// Reports whether @p end_effector_position is inside [@p lower, @p upper],
/// printing a per-axis diagnosis to stderr when it is not.
bool EndEffectorIsInWorkspace(const Vector3d& end_effector_position,
                              const Vector3d& lower, const Vector3d& upper) {
  const std::vector<std::string> axis_names = {"x", "y", "z"};
  std::string violations;
  for (int i = 0; i < 3; ++i) {
    const double p = end_effector_position(i);
    if (!(p >= lower(i))) {
      violations += "  " + axis_names[i] + " = " + std::to_string(p) +
                    " m is below the lower limit " + std::to_string(lower(i)) +
                    " m (by " + std::to_string(lower(i) - p) + " m)\n";
    } else if (p > upper(i)) {
      violations += "  " + axis_names[i] + " = " + std::to_string(p) +
                    " m is above the upper limit " + std::to_string(upper(i)) +
                    " m (by " + std::to_string(p - upper(i)) + " m)\n";
    }
  }
  if (violations.empty()) {
    return true;
  }
  std::cerr
      << "\n"
      << "ERROR: the printer's end effector is outside the configured "
         "workspace limits.\n"
      << "The controller refuses to start, since its first command would "
         "otherwise\n"
      << "yank the end effector back into the workspace along a straight "
         "line\n"
      << "through whatever it is currently touching.\n\n"
      << violations << "\n"
      << "Measured end effector tip (world frame): ["
      << end_effector_position.transpose() << "] m\n"
      << "Workspace limits (from the demo's sampling C3 options yaml):\n"
      << "  lower = [" << lower.transpose() << "] m\n"
      << "  upper = [" << upper.transpose() << "] m\n\n"
      << "To recover, jog the printer back inside these bounds (or re-run "
         "homing.py\n"
      << "on the printer driver host), then restart this controller.\n"
      << std::endl;
  return false;
}

}  // namespace

DEFINE_bool(is_simulation, true, "True for simulation, false for hardware");
DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");
DEFINE_string(demo_name, "cone",
              "Demo within sampling_c3/three_d_printer/; used to find "
              "controller params file");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_demo_name != "cone") {
    throw std::runtime_error("Unknown --demo_name value: " + FLAGS_demo_name);
  }

  drake::lcm::DrakeLcm lcm(FLAGS_lcm_url);

  // ------------------------------------------------------------------------
  // Load parameters
  // ------------------------------------------------------------------------
  std::string controller_params_path =
      "examples/sampling_c3/three_d_printer/" + FLAGS_demo_name +
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

  // ------------------------------------------------------------------------
  // Build plant
  // ------------------------------------------------------------------------
  drake::multibody::MultibodyPlant<double> plant(0.0);
  Add3DPrinterToPlant(&plant);
  plant.Finalize();

  auto plant_context = plant.CreateDefaultContext();

  // ------------------------------------------------------------------------
  // Diagram
  // ------------------------------------------------------------------------
  DiagramBuilder<double> builder;
  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);
  auto end_effector_trajectory_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.tracking_trajectory_actor_channel, &lcm));
  auto end_effector_position_receiver =
      builder.AddSystem<systems::LcmTrajectoryReceiver>(
          "end_effector_position_target");
  auto three_d_printer_command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          lcm_channel_params.robot_input_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto three_d_printer_command_sender =
      builder.AddSystem<systems::ThreeDPrinterCommandSender>(plant);
  auto end_effector_position_tracking_data =
      std::make_unique<TransTaskSpaceTrackingData>(
          "end_effector_target", osc_params.K_p_end_effector,
          osc_params.K_d_end_effector, osc_params.W_end_effector, plant, plant);
  end_effector_position_tracking_data->AddPointToTrack(k3dEndEffectorTipName);

  const VectorXd& end_effector_acceleration_limits =
      osc_params.end_effector_acceleration * Vector3d::Ones();
  end_effector_position_tracking_data->SetCmdAccelerationBounds(
      -end_effector_acceleration_limits, end_effector_acceleration_limits);

  auto radio_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
          lcm_channel_params.radio_channel, &lcm));
  auto end_effector_trajectory =
      builder.AddSystem<EndEffectorPositionTrajectoryGenerator>(
          plant, plant_context.get(), osc_params.neutral_position,
          osc_params.teleop_neutral_position, k3dEndEffectorTipName);
  end_effector_trajectory->SetRemoteControlParameters(
      osc_params.neutral_position, osc_params.x_scale, osc_params.y_scale,
      osc_params.z_scale);

  auto printer_inverse_kinematics =
      builder.AddSystem<systems::ThreeDPrinterInverseKinematics>(
          plant, plant_context.get(), k3dEndEffectorTipName);

  // ------------------------------------------------------------------------
  // Workspace limits
  // ------------------------------------------------------------------------
  // The printer driver clamps too, but in printer joint (carriage) coordinates
  // and with no knowledge of the end effector geometry, so it is not a
  // backstop for these limits.  Enforce them here, in two places:
  //   1. on the teleop target, so it can't wind up past the boundary; and
  //   2. on the published command, as a hard backstop on every path.
  // Both use the raw limits, with no workspace_margins applied.  That is
  // strictly looser than the margined box SamplingC3Controller already clamps
  // its published plan to, so a valid C3 trajectory passes through untouched
  // and only teleop or a malformed plan can reach these bounds.
  const auto [workspace_lower, workspace_upper] =
      GetWorkspaceBox(controller_params.sampling_c3_options.workspace_limits);
  end_effector_trajectory->SetWorkspaceLimits(workspace_lower, workspace_upper);
  const Vector3d& end_effector_offset =
      printer_inverse_kinematics->get_end_effector_offset();
  three_d_printer_command_sender->SetPositionLimits(
      workspace_lower - end_effector_offset,
      workspace_upper - end_effector_offset);

  // ------------------------------------------------------------------------
  // Connections
  // ------------------------------------------------------------------------
  builder.Connect(three_d_printer_command_sender->get_output_port(),
                  three_d_printer_command_pub->get_input_port());
  builder.Connect(end_effector_trajectory_sub->get_output_port(),
                  end_effector_position_receiver->get_input_port_trajectory());
  builder.Connect(end_effector_position_receiver->get_output_port(0),
                  end_effector_trajectory->get_input_port_trajectory());
  builder.Connect(state_receiver->get_output_port(0),
                  end_effector_trajectory->get_input_port_state());
  builder.Connect(radio_sub->get_output_port(0),
                  end_effector_trajectory->get_input_port_radio());
  builder.Connect(end_effector_trajectory->get_output_port(0),
                  printer_inverse_kinematics->get_input_port_trajectory());
  builder.Connect(printer_inverse_kinematics->get_output_port_trajectory(),
                  three_d_printer_command_sender->get_input_port(0));

  // ------------------------------------------------------------------------
  // Build
  // ------------------------------------------------------------------------
  auto owned_diagram = builder.Build();
  std::shared_ptr<Diagram<double>> shared_diagram = std::move(owned_diagram);
  shared_diagram->set_name("sampling_c3_three_d_printer_osc_controller");
  DrawAndSaveDiagramGraph(*shared_diagram);

  // ------------------------------------------------------------------------
  // LCM Loop
  // ------------------------------------------------------------------------
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, shared_diagram, state_receiver,
      lcm_channel_params.robot_state_channel, true);

  // ------------------------------------------------------------------------
  // Refuse to start from a state outside the workspace limits
  // ------------------------------------------------------------------------
  {
    systems::Subscriber<dairlib::lcmt_robot_output> state_sub(
        &lcm, lcm_channel_params.robot_state_channel);
    drake::log()->info(
        "Waiting for the first state message to check the "
        "workspace limits");
    drake::lcm::LcmHandleSubscriptionsUntil(
        &lcm, [&]() { return state_sub.count() > 0; });

    // Decode through the diagram's own state receiver rather than duplicating
    // its name-to-index mapping.  LcmDrivenLoop overwrites this fixed value on
    // every iteration, so fixing it here does not disturb the loop.
    auto& diagram_context = loop.get_diagram_mutable_context();
    auto& state_receiver_context = shared_diagram->GetMutableSubsystemContext(
        *state_receiver, &diagram_context);
    state_receiver->get_input_port(0).FixValue(&state_receiver_context,
                                               state_sub.message());
    const auto& state_output =
        state_receiver->get_output_port(0).Eval<systems::OutputVector<double>>(
            state_receiver_context);

    // Use a scratch context:  plant_context is shared with the trajectory
    // generator and the inverse kinematics system, which relies on it.
    auto check_context = plant.CreateDefaultContext();
    plant.SetPositions(check_context.get(), state_output.GetPositions());
    const Vector3d measured_end_effector_position =
        plant
            .EvalBodyPoseInWorld(*check_context,
                                 plant.GetBodyByName(k3dEndEffectorTipName))
            .translation();
    if (!EndEffectorIsInWorkspace(measured_end_effector_position,
                                  workspace_lower, workspace_upper)) {
      return 1;
    }
  }

  loop.Simulate();

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }