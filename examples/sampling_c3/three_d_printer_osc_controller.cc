#include <dairlib/lcmt_radio_out.hpp>
#include <dairlib/lcmt_timestamped_saved_traj.hpp>
#include <gflags/gflags.h>

#include <iostream>
#include <limits>

#include "common/eigen_utils.h"
#include "examples/sampling_c3/sampling_c3_utils.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_controller_params.h"
#include "examples/sampling_c3/parameter_headers/lcm_channels.h"
#include "examples/sampling_c3/parameter_headers/osc_params.h"
#include "systems/controllers/osc/end_effector_force.h"
#include "systems/controllers/osc/end_effector_orientation.h"
#include "systems/controllers/osc/end_effector_position.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
#include "systems/controllers/gravity_compensator.h"
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

DEFINE_bool(is_simulation, true,
            "True for simulation, false for hardware");
DEFINE_string(lcm_url,
              "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");
DEFINE_string(demo_name,
              "jacktoy",
              "Demo within sampling_c3; used to find controller params file");

int DoMain(int argc, char* argv[]) {
  std::cout << "\n========== STARTING OSC CONTROLLER ==========\n"
            << std::endl;

  gflags::ParseCommandLineFlags(&argc, &argv, true);

  std::cout << "[DEBUG] demo_name      = "
            << FLAGS_demo_name << std::endl;
  std::cout << "[DEBUG] is_simulation = "
            << FLAGS_is_simulation << std::endl;
  std::cout << "[DEBUG] lcm_url       = "
            << FLAGS_lcm_url << std::endl;

  drake::lcm::DrakeLcm lcm(FLAGS_lcm_url);

  std::cout << "[DEBUG] DrakeLcm created."
            << std::endl;

  // ------------------------------------------------------------------------
  // Load parameters
  // ------------------------------------------------------------------------

  std::string controller_params_path =
      "examples/sampling_c3/" +
      FLAGS_demo_name +
      "/parameters/sampling_c3_controller_params.yaml";

  std::cout << "\n========== LOADING PARAMETERS =========="
            << std::endl;

  std::cout << "[DEBUG] controller params file:\n  "
            << controller_params_path << std::endl;

  SamplingC3ControllerParams controller_params =
      drake::yaml::LoadYamlFile<SamplingC3ControllerParams>(
          controller_params_path);

  std::cout << "[DEBUG] Controller params loaded."
            << std::endl;

  std::cout << "[DEBUG] osc params file:\n  "
            << controller_params.osc_params_file
            << std::endl;

  SamplingC3OSCParams osc_params =
      drake::yaml::LoadYamlFile<SamplingC3OSCParams>(
          controller_params.osc_params_file);

  std::cout << "[DEBUG] OSC params loaded."
            << std::endl;

  std::string lcm_channels_file =
      FLAGS_is_simulation
          ? controller_params.lcm_channels_simulation_file
          : controller_params.lcm_channels_hardware_file;

  std::cout << "[DEBUG] LCM channel file:\n  "
            << lcm_channels_file << std::endl;

  SamplingC3LcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<SamplingC3LcmChannels>(
          lcm_channels_file);

  std::cout << "[DEBUG] LCM channel params loaded."
            << std::endl;

  std::cout << "[DEBUG] Loading solver settings:\n  "
            << controller_params.osc_qp_settings_file
            << std::endl;

  drake::solvers::SolverOptions solver_options =
      drake::yaml::LoadYamlFile<
          solvers::SolverOptionsFromYaml>(
          FindResourceOrThrow(
              controller_params.osc_qp_settings_file))
          .GetAsSolverOptions(
              drake::solvers::OsqpSolver::id());

  std::cout << "[DEBUG] Solver options loaded."
            << std::endl;

  // ------------------------------------------------------------------------
  // Build plant
  // ------------------------------------------------------------------------

  std::cout << "\n========== BUILDING PLANT =========="
            << std::endl;

  drake::multibody::MultibodyPlant<double> plant(0.0);

  Add3DPrinterToPlant(&plant);

  std::cout << "[DEBUG] 3D printer added."
            << std::endl;

  plant.Finalize();

  std::cout << "[DEBUG] Plant finalized."
            << std::endl;

  std::cout << "[DEBUG] nq = "
            << plant.num_positions()
            << std::endl;

  std::cout << "[DEBUG] nv = "
            << plant.num_velocities()
            << std::endl;

  std::cout << "[DEBUG] nu = "
            << plant.num_actuated_dofs()
            << std::endl;

  auto plant_context =
      plant.CreateDefaultContext();

  std::cout << "[DEBUG] Default context created."
            << std::endl;

  // ------------------------------------------------------------------------
  // Diagram
  // ------------------------------------------------------------------------

  std::cout << "\n========== BUILDING DIAGRAM =========="
            << std::endl;

  DiagramBuilder<double> builder;

  auto state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(
          plant);

  std::cout << "[DEBUG] Added RobotOutputReceiver."
            << std::endl;

  std::cout << "[DEBUG] Tracking trajectory channel: "
            << lcm_channel_params
                   .tracking_trajectory_actor_channel
            << std::endl;

  auto end_effector_trajectory_sub =
      builder.AddSystem(
          LcmSubscriberSystem::Make<
              dairlib::lcmt_timestamped_saved_traj>(
              lcm_channel_params
                  .tracking_trajectory_actor_channel,
              &lcm));

  auto end_effector_position_receiver =
      builder.AddSystem<
          systems::LcmTrajectoryReceiver>(
          "end_effector_position_target");

  std::cout << "[DEBUG] Added trajectory receiver."
            << std::endl;

  std::cout << "[DEBUG] Robot command channel: "
            << lcm_channel_params
                   .three_d_printer_input_channel
            << std::endl;

  auto three_d_printer_command_pub =
      builder.AddSystem(
          LcmPublisherSystem::Make<
              dairlib::lcmt_robot_output>(
              lcm_channel_params
                  .three_d_printer_input_channel,
              &lcm,
              TriggerTypeSet(
                  {TriggerType::kForced})));

  std::cout << "[DEBUG] OSC command channel: "
            << lcm_channel_params
                   .three_d_printer_osc_channel
            << std::endl;

  auto osc_command_pub =
      builder.AddSystem(
          LcmPublisherSystem::Make<
              dairlib::lcmt_robot_output>(
              lcm_channel_params
                  .three_d_printer_osc_channel,
              &lcm,
              TriggerTypeSet(
                  {TriggerType::kForced})));

  auto three_d_printer_command_sender =
      builder.AddSystem<
          systems::ThreeDPrinterCommandSender>(
          plant);

  auto osc_command_sender =
      builder.AddSystem<
          systems::ThreeDPrinterCommandSender>(
          plant);

  std::cout << "[DEBUG] Added command senders."
            << std::endl;

  auto end_effector_position_tracking_data =
      std::make_unique<TransTaskSpaceTrackingData>(
          "end_effector_target",
          osc_params.K_p_end_effector,
          osc_params.K_d_end_effector,
          osc_params.W_end_effector,
          plant,
          plant);

  std::cout << "[DEBUG] Created tracking data."
            << std::endl;

  std::cout << "[DEBUG] Kp = "
            << osc_params.K_p_end_effector
            << std::endl;

  std::cout << "[DEBUG] Kd = "
            << osc_params.K_d_end_effector
            << std::endl;

  std::cout << "[DEBUG] Weight = "
            << osc_params.W_end_effector
            << std::endl;
  std::cout << "Tracking body: " << k3dEndEffectorName << std::endl;
std::cout << "HasBodyNamed? "
          << plant.HasBodyNamed(k3dEndEffectorName)
          << std::endl;
  end_effector_position_tracking_data
      ->AddPointToTrack(k3dEndEffectorName);
      
  std::cout << "[DEBUG] added point to track: " << k3dEndEffectorName
            << std::endl;

  const VectorXd&
      end_effector_acceleration_limits =
          osc_params.end_effector_acceleration *
          Vector3d::Ones();

  std::cout << "[DEBUG] Acceleration limits = "
            << end_effector_acceleration_limits.transpose()
            << std::endl;

  end_effector_position_tracking_data
      ->SetCmdAccelerationBounds(
          -end_effector_acceleration_limits,
          end_effector_acceleration_limits);

  // ------------------------------------------------------------------------
  // Connections
  // ------------------------------------------------------------------------

  std::cout << "\n========== CONNECTING SYSTEMS =========="
            << std::endl;

  builder.Connect(
      three_d_printer_command_sender->get_output_port(),
      three_d_printer_command_pub->get_input_port());

  std::cout << "[DEBUG] Connected three_d_printer command sender."
            << std::endl;

  builder.Connect(
      osc_command_sender->get_output_port(),
      osc_command_pub->get_input_port());

  std::cout << "[DEBUG] Connected osc command sender."
            << std::endl;

  builder.Connect(
      end_effector_trajectory_sub->get_output_port(),
      end_effector_position_receiver
          ->get_input_port_trajectory());

  std::cout << "[DEBUG] Connected trajectory subscriber."
            << std::endl;




  std::cout << "[DEBUG] Connected trajectory generator inputs."
            << std::endl;

  builder.Connect(
      end_effector_position_receiver
          ->get_output_port(0),
      osc_command_sender->get_input_port(0));

  builder.Connect(
      end_effector_position_receiver
          ->get_output_port(0),
      three_d_printer_command_sender->get_input_port(0));

  std::cout << "[DEBUG] Connected trajectory outputs."
            << std::endl;

  // ------------------------------------------------------------------------
  // Build
  // ------------------------------------------------------------------------

  std::cout << "\n========== BUILDING DIAGRAM =========="
            << std::endl;

  auto owned_diagram = builder.Build();

  std::cout << "[DEBUG] Diagram built."
            << std::endl;

  std::shared_ptr<Diagram<double>> shared_diagram =
      std::move(owned_diagram);

  shared_diagram->set_name(
      "sampling_c3_three_d_printer_osc_controller");

  std::cout << "[DEBUG] Diagram name set."
            << std::endl;

  DrawAndSaveDiagramGraph(*shared_diagram);

  std::cout << "[DEBUG] Diagram graph saved."
            << std::endl;

  // ------------------------------------------------------------------------
  // LCM Loop
  // ------------------------------------------------------------------------

  std::cout << "\n========== STARTING LCM LOOP =========="
            << std::endl;

  std::cout << "[DEBUG] State channel = "
            << lcm_channel_params
                   .three_d_printer_state_channel
            << std::endl;

  systems::LcmDrivenLoop<dairlib::lcmt_robot_output>
      loop(
          &lcm,
          shared_diagram,
          state_receiver,
          lcm_channel_params
              .three_d_printer_state_channel,
          true);

  std::cout << "[DEBUG] Entering loop.Simulate()"
            << std::endl;

  loop.Simulate();

  std::cout << "[DEBUG] loop.Simulate() returned"
            << std::endl;

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::DoMain(argc, argv);
}