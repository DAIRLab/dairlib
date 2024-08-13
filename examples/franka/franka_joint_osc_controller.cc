
#include <dairlib/lcmt_radio_out.hpp>
#include <dairlib/lcmt_timestamped_saved_traj.hpp>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "examples/franka/parameters/franka_lcm_channels.h"
#include "examples/franka/parameters/franka_osc_controller_params.h"
#include "examples/franka/systems/end_effector_force.h"
#include "examples/franka/systems/end_effector_orientation.h"
#include "examples/franka/systems/end_effector_position.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
#include "systems/controllers/gravity_compensator.h"
#include "systems/controllers/osc/joint_space_tracking_data.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/primitives/radio_parser.h"
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
#include "examples/franka/systems/joint_trajectory_generator.h"

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

DEFINE_string(osqp_settings,
              "examples/franka/parameters/franka_osc_qp_settings.yaml",
              "Filepath containing qp settings");
DEFINE_string(
    controller_parameters,
    "examples/franka/parameters/franka_osc_controller_params.yaml",
    "Controller settings such as channels. Attempting to minimize "
    "number of gflags");
DEFINE_string(lcm_channels,
              "examples/franka/parameters/lcm_channels_simulation.yaml",
              "Filepath containing lcm channels");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // load parameters
  drake::yaml::LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;
  FrankaControllerParams controller_params =
      drake::yaml::LoadYamlFile<FrankaControllerParams>(
          FLAGS_controller_parameters);
  FrankaLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<FrankaLcmChannels>(FLAGS_lcm_channels);
  OSCGains gains = drake::yaml::LoadYamlFile<OSCGains>(
      FindResourceOrThrow(FLAGS_controller_parameters), {}, {}, yaml_options);
  drake::solvers::SolverOptions solver_options =
      drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
          FindResourceOrThrow(FLAGS_osqp_settings))
          .GetAsSolverOptions(drake::solvers::OsqpSolver::id());
  DiagramBuilder<double> builder;

  drake::multibody::MultibodyPlant<double> plant(0.0);
  Parser parser(&plant, nullptr);
  parser.AddModelsFromUrl(controller_params.franka_model);

  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"),
                   X_WI);

  if (!controller_params.end_effector_name.empty()) {
    drake::multibody::ModelInstanceIndex end_effector_index = parser.AddModels(
        FindResourceOrThrow(controller_params.end_effector_model))[0];
    RigidTransform<double> T_EE_W =
        RigidTransform<double>(drake::math::RotationMatrix<double>(),
                               controller_params.tool_attachment_frame);
    plant.WeldFrames(plant.GetFrameByName("panda_link7"),
                     plant.GetFrameByName(controller_params.end_effector_name,
                                          end_effector_index),
                     T_EE_W);
  } else {
    std::cout << "OSC plant has been constructed with no end effector."
              << std::endl;
  }

  plant.Finalize();
  auto plant_context = plant.CreateDefaultContext();

  drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0");

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
      plant, plant_context.get(), false);
  if (controller_params.publish_debug_info) {
    auto osc_debug_pub =
        builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
            lcm_channel_params.osc_debug_channel, &lcm,
            TriggerTypeSet({TriggerType::kForced})));
    builder.Connect(osc->get_output_port_osc_debug(),
                    osc_debug_pub->get_input_port());
  }
  VectorXd target_position = VectorXd::Zero(7);
//  target_position << -1.3, 1.6, 1.6, -2.1, 1.57, 1.62, -0.81;
  target_position << 0,  -0.785,  0,  -2.356,  0,  1.57,  0.785;
  auto joint_traj_generator =
      builder.AddSystem<JointTrajectoryGenerator>(plant, target_position);
  std::vector<std::unique_ptr<JointSpaceTrackingData>> joint_tracking_data_vec;
  std::vector<std::string> joint_names = {
      "panda_joint1", "panda_joint2", "panda_joint3",
      "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};
  for (int joint_idx = 0; joint_idx < joint_names.size();
       ++joint_idx) {
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
    auto gravity_compensator =
        builder.AddSystem<systems::GravityCompensationRemover>(plant,
                                                               *plant_context);
    builder.Connect(osc->get_output_port_osc_command(),
                    gravity_compensator->get_input_port());
    builder.Connect(gravity_compensator->get_output_port(),
                    franka_command_sender->get_input_port());
  } else {
    if (FLAGS_lcm_channels ==
        "examples/franka/parameters/lcm_channels_hardware.yaml") {
      std::cerr << "Using hardware lcm channels but not cancelling gravity "
                   "compensation. Please check the OSC settings"
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
  owned_diagram->set_name(("franka_osc_controller"));
  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, std::move(owned_diagram), state_receiver,
      lcm_channel_params.franka_state_channel, true);
  DrawAndSaveDiagramGraph(*loop.get_diagram());
  loop.Simulate();
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }