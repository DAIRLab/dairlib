
#include <dairlib/lcmt_radio_out.hpp>
#include <dairlib/lcmt_timestamped_saved_traj.hpp>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "examples/sampling_c3/parameter_headers/lcm_channels.h"
#include "examples/sampling_c3/parameter_headers/franka_osc_controller_params.h"
#include "systems/controllers/osc/end_effector_force.h"
#include "systems/controllers/osc/end_effector_orientation.h"
#include "systems/controllers/osc/end_effector_position.h"
#include "joint_trajectory_generator.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
#include "systems/controllers/gravity_compensator.h"
#include "systems/controllers/osc/joint_space_tracking_data.h"
#include "systems/controllers/osc/operational_space_control.h"
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
using std::string;

using systems::controllers::JointSpaceTrackingData;

// TODO @bibit parameter overhaul
DEFINE_string(
    osqp_settings,
    "examples/sampling_c3/shared_parameters/franka_osc_qp_settings.yaml",
    "Filepath containing qp settings");
DEFINE_string(
    controller_parameters,
    "examples/sampling_c3/jacktoy/parameters/franka_osc_controller_params.yaml",
    "Controller settings such as channels.");
DEFINE_string(
    lcm_channels,
    "examples/sampling_c3/shared_parameters/lcm_channels_simulation.yaml",
    "Filepath containing lcm channels");
DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");
DEFINE_string(demo_name, "jacktoy",
              "Name for the demo, used when building filepaths for output.");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::lcm::DrakeLcm lcm(FLAGS_lcm_url);
  std::string base_path = "examples/sampling_c3/" + FLAGS_demo_name + "/";

  // Load parameters.
  drake::yaml::LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;
  FrankaControllerParams controller_params =
      drake::yaml::LoadYamlFile<FrankaControllerParams>(
          base_path + "parameters/franka_osc_controller_params.yaml");
  SamplingC3LcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<SamplingC3LcmChannels>(FLAGS_lcm_channels);
  OSCGains gains = drake::yaml::LoadYamlFile<OSCGains>(
      FindResourceOrThrow(base_path +
                          "parameters/franka_osc_controller_params.yaml"),
      {}, {}, yaml_options);
  drake::solvers::SolverOptions solver_options =
      drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
          FindResourceOrThrow(FLAGS_osqp_settings))
          .GetAsSolverOptions(drake::solvers::OsqpSolver::id());

  // Create a Franka-only plant.
  drake::multibody::MultibodyPlant<double> plant(0.0);
  Parser parser(&plant, nullptr);
  parser.SetAutoRenaming(true);
  parser.AddModelsFromUrl(controller_params.franka_model)[0];
  parser.AddModels(
      FindResourceOrThrow(controller_params.end_effector_model))[0];

  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("panda_link0"), X_WI);

  RigidTransform<double> T_EE_W = RigidTransform<double>(
      drake::math::RotationMatrix<double>(
          drake::math::RollPitchYaw<double>(3.1415, 0, 0)),
      controller_params.tool_attachment_frame);
  plant.WeldFrames(plant.GetFrameByName("panda_link7"),
                   plant.GetFrameByName("end_effector_flange"), T_EE_W);

  plant.Finalize();
  auto plant_context = plant.CreateDefaultContext();

  // Piece together the diagram.
  DiagramBuilder<double> builder;

  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);
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
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      plant, plant, plant_context.get(), plant_context.get(), false);
  if (controller_params.publish_debug_info) {
    auto osc_debug_pub =
        builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
            lcm_channel_params.osc_debug_channel, &lcm,
            TriggerTypeSet({TriggerType::kForced})));
    builder.Connect(osc->get_output_port_osc_debug(),
                    osc_debug_pub->get_input_port());
  }
  // WARNING:  Hard-coded initial joint configurations for the robot in the
  // sampling c3 experiments.
  VectorXd target_position = VectorXd::Zero(7);
  target_position << 2.191, 1.1, -1.33, -2.22, 1.30, 2.02, 0.08;
  auto joint_traj_generator =
      builder.AddSystem<JointTrajectoryGenerator>(plant, target_position);
  std::vector<std::unique_ptr<JointSpaceTrackingData>> joint_tracking_data_vec;
  std::vector<std::string> joint_names = {
      "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
      "panda_joint5", "panda_joint6", "panda_joint7"};
  for (int joint_idx = 0; joint_idx < joint_names.size(); ++joint_idx) {
    string joint_name = joint_names[joint_idx];
    MatrixXd W = 1.0 * MatrixXd::Identity(1, 1);
    MatrixXd K_p = 100 * MatrixXd::Identity(1, 1);
    MatrixXd K_d = 5 * MatrixXd::Identity(1, 1);
    joint_tracking_data_vec.push_back(std::make_unique<JointSpaceTrackingData>(
        joint_name + "_traj", K_p, K_d, W, plant, plant));
    joint_tracking_data_vec[joint_idx]->AddJointToTrack(joint_name,
                                                        joint_name + "dot");

    osc->AddTrackingData(std::move(joint_tracking_data_vec[joint_idx]));

    builder.Connect(joint_traj_generator->get_output_port_joint(joint_idx),
                    osc->get_input_port_tracking_data(joint_name + "_traj"));
  }

  osc->SetAccelerationCostWeights(gains.W_acceleration);
  osc->SetInputCostWeights(gains.W_input_regularization);
  osc->SetInputSmoothingCostWeights(gains.W_input_smoothing_regularization);

  osc->SetContactFriction(controller_params.mu);
  osc->SetOsqpSolverOptions(solver_options);

  osc->Build();

  if (controller_params.cancel_gravity_compensation) {
    // TODO @bibit don't hardcode parameter filepaths
    if (FLAGS_lcm_channels ==
        base_path + "shared_parameters/lcm_channels_simulation.yaml") {
      std::cerr << "In simulation, OSC needs to have "
                   "cancel_gravity_compensation: false"
                << std::endl;
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
    if (FLAGS_lcm_channels ==
        base_path + "shared_parameters/lcm_channels_hardware.yaml") {
      std::cerr
          << "In hardware, OSC needs to have cancel_gravity_compensation: true"
          << std::endl;
      return -1;
    }
    builder.Connect(osc->get_output_port_osc_command(),
                    franka_command_sender->get_input_port(0));
  }

  builder.Connect(franka_command_sender->get_output_port(),
                  franka_command_pub->get_input_port());
  builder.Connect(osc_command_sender->get_output_port(),
                  osc_command_pub->get_input_port());
  builder.Connect(osc->get_output_port_osc_command(),
                  osc_command_sender->get_input_port(0));
  builder.Connect(state_receiver->get_output_port(0),
                  osc->get_input_port_robot_output());
  builder.Connect(state_receiver->get_output_port(0),
                  joint_traj_generator->get_input_port_robot_state());

  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("sampling_c3_franka_joint_osc_controller"));
  DrawAndSaveDiagramGraph(*owned_diagram);
  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, std::move(owned_diagram), state_receiver,
      lcm_channel_params.franka_state_channel, true);
  loop.Simulate();
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
