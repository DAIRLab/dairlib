#include <iostream>
#include <limits>

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
DEFINE_string(demo_name, "three_d_printer",
              "Demo within sampling_c3; used to find controller params file");
DEFINE_double(printer_target_offset_x, 0.0,
              "Printer target offset along x, in meters");
DEFINE_double(printer_target_offset_y, 0.0075,
              "Printer target offset along y, in meters");
DEFINE_double(printer_target_offset_z, 0.107,
              "Printer target offset along z, in meters");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  drake::lcm::DrakeLcm lcm(FLAGS_lcm_url);

  // ------------------------------------------------------------------------
  // Load parameters
  // ------------------------------------------------------------------------

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

  auto osc_command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          lcm_channel_params.osc_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));

  auto three_d_printer_command_sender =
      builder.AddSystem<systems::ThreeDPrinterCommandSender>(
          plant,
          Vector3d(FLAGS_printer_target_offset_x, FLAGS_printer_target_offset_y,
                   FLAGS_printer_target_offset_z));

  auto osc_command_sender =
      builder.AddSystem<systems::ThreeDPrinterCommandSender>(plant);

  auto end_effector_position_tracking_data =
      std::make_unique<TransTaskSpaceTrackingData>(
          "end_effector_target", osc_params.K_p_end_effector,
          osc_params.K_d_end_effector, osc_params.W_end_effector, plant, plant);

  end_effector_position_tracking_data->AddPointToTrack(k3dEndEffectorName);

  const VectorXd& end_effector_acceleration_limits =
      osc_params.end_effector_acceleration * Vector3d::Ones();

  end_effector_position_tracking_data->SetCmdAccelerationBounds(
      -end_effector_acceleration_limits, end_effector_acceleration_limits);

  // ------------------------------------------------------------------------
  // Connections
  // ------------------------------------------------------------------------

  builder.Connect(three_d_printer_command_sender->get_output_port(),
                  three_d_printer_command_pub->get_input_port());

  builder.Connect(osc_command_sender->get_output_port(),
                  osc_command_pub->get_input_port());

  builder.Connect(end_effector_trajectory_sub->get_output_port(),
                  end_effector_position_receiver->get_input_port_trajectory());

  builder.Connect(end_effector_position_receiver->get_output_port(0),
                  osc_command_sender->get_input_port(0));

  builder.Connect(end_effector_position_receiver->get_output_port(0),
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

  loop.Simulate();

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }