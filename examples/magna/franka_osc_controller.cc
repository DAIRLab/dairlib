#include <dairlib/lcmt_franka_hand_target_position.hpp>
#include <dairlib/lcmt_radio_out.hpp>
#include <dairlib/lcmt_timestamped_saved_traj.hpp>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "common/find_resource.h"
#include "examples/magna/parameter_headers/lcm_channel_params.h"
#include "examples/magna/parameter_headers/osc_params.h"
#include "solvers/solver_options_io.h"
#include "systems/controllers/gravity_compensator.h"
#include "systems/controllers/osc/end_effector_force.h"
#include "systems/controllers/osc/end_effector_orientation.h"
#include "systems/controllers/osc/end_effector_position.h"
#include "systems/controllers/osc/external_force_tracking_data.h"
#include "systems/controllers/osc/joint_space_tracking_data.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/controllers/osc/rot_space_tracking_data.h"
#include "systems/controllers/osc/trans_space_tracking_data.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/franka_hand_target_position_receiver.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"
#include "systems/trajectory_optimization/lcm_trajectory_systems.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace dairlib {
namespace magna {
static constexpr const char* kFrankaModel =
    "package://drake_models/franka_description/urdf/"
    "panda_arm_hand_with_long_fingers.urdf";
static constexpr const char* kEndEffectorName = "finger_tip";

using drake::math::RigidTransform;
using drake::multibody::ModelInstanceIndex;
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

using dairlib::EndEffectorForceTrajectoryGenerator;
using dairlib::EndEffectorOrientationTrajectoryGenerator;
using dairlib::EndEffectorPositionTrajectoryGenerator;
using dairlib::systems::controllers::ExternalForceTrackingData;
using dairlib::systems::controllers::JointSpaceTrackingData;
using dairlib::systems::controllers::OperationalSpaceControl;
using dairlib::systems::controllers::RotTaskSpaceTrackingData;
using dairlib::systems::controllers::TransTaskSpaceTrackingData;

DEFINE_bool(is_simulation, true, "True for simulation, false for hardware");
DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");
int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::lcm::DrakeLcm lcm(FLAGS_lcm_url);

  // Load parameters.
  MagnaOSCParams osc_params = drake::yaml::LoadYamlFile<MagnaOSCParams>(
      "examples/magna/parameters/osc_params.yaml");
  MagnaLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<MagnaLcmChannels>(
          "examples/magna/parameters/lcm_channels_simulation.yaml");
  drake::solvers::SolverOptions solver_options =
      drake::yaml::LoadYamlFile<dairlib::solvers::SolverOptionsFromYaml>(
          dairlib::FindResourceOrThrow(
              "examples/magna/parameters/osc_qp_settings.yaml"))
          .GetAsSolverOptions(drake::solvers::OsqpSolver::id());

  // Create a Franka-only plant (including gripper).
  drake::multibody::MultibodyPlant<double> plant(0.0);
  Parser parser(&plant);
  parser.SetAutoRenaming(true);
  ModelInstanceIndex franka_index = parser.AddModelsFromUrl(kFrankaModel)[0];
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"),
                   RigidTransform<double>::Identity());
  plant.Finalize();
  auto plant_context = plant.CreateDefaultContext();

  // Piece together the diagram.
  DiagramBuilder<double> builder;

  auto state_receiver =
      builder.AddSystem<dairlib::systems::RobotOutputReceiver>(plant);
  auto end_effector_trajectory_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.tracking_trajectory_actor_channel, &lcm));
  auto end_effector_position_receiver =
      builder.AddSystem<dairlib::systems::LcmTrajectoryReceiver>(
          "end_effector_position_target");
  auto end_effector_force_receiver =
      builder.AddSystem<dairlib::systems::LcmTrajectoryReceiver>(
          "end_effector_force_target");
  auto end_effector_orientation_receiver =
      builder.AddSystem<dairlib::systems::LcmOrientationTrajectoryReceiver>(
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
      builder.AddSystem<dairlib::systems::RobotCommandSender>(plant);
  auto osc_command_sender =
      builder.AddSystem<dairlib::systems::RobotCommandSender>(plant);
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
  auto osc = builder.AddSystem<OperationalSpaceControl>(
      plant, plant, plant_context.get(), plant_context.get(), false);
  if (osc_params.publish_debug_info) {
    auto osc_debug_pub =
        builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
            lcm_channel_params.osc_debug_channel, &lcm,
            TriggerTypeSet({TriggerType::kForced})));
    builder.Connect(osc->get_output_port_osc_debug(),
                    osc_debug_pub->get_input_port());
  }

  // Add regularization cost to maintain joint positions (except for the gripper
  // fingers) as much as possible
  VectorXd joint_position_target = VectorXd::Zero(9);
  joint_position_target << 1.2822, 0.29, -1.40629, -1.8419, 0.30038, 2.39, 0.57512, 0, 0;
  std::vector<std::unique_ptr<JointSpaceTrackingData>>
      joint_position_tracking_data_vec;
  std::vector<std::string> joint_position_names = {
      "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
      "panda_joint5", "panda_joint6", "panda_joint7"};
  for (int joint_idx = 0; joint_idx < joint_position_names.size();
       ++joint_idx) {
    joint_position_tracking_data_vec.push_back(
        std::make_unique<JointSpaceTrackingData>(
            joint_position_names[joint_idx] + "_traj", osc_params.K_p_mid_link,
            osc_params.K_d_mid_link, osc_params.W_mid_link, plant, plant));
    joint_position_tracking_data_vec[joint_idx]->AddJointToTrack(
        joint_position_names[joint_idx],
        joint_position_names[joint_idx] + "dot");
    osc->AddConstTrackingData(
        std::move(joint_position_tracking_data_vec[joint_idx]),
        joint_position_target[joint_idx] * VectorXd::Ones(1));
  }

  // In real hardware, command positions can be sent directly to the Franka
  // gripper. In simulation, we need a PD controller. Within the OSC framework,
  // we add costs to track the gripper finger positions.
  if (FLAGS_is_simulation) {
    auto franka_hand_target_position_receiver =
        builder.AddSystem<dairlib::systems::FrankaHandTargetPositionReceiver>();
    auto franka_hand_target_position_subscriber = builder.AddSystem(
        LcmSubscriberSystem::Make<dairlib::lcmt_franka_hand_target_position>(
            lcm_channel_params.franka_hand_target_position_channel, &lcm));
    builder.Connect(franka_hand_target_position_subscriber->get_output_port(),
                    franka_hand_target_position_receiver->get_input_port());
    std::vector<std::unique_ptr<JointSpaceTrackingData>>
        finger_position_tracking_data_vec;
    std::vector<std::string> finger_position_names = {"panda_finger_joint1",
                                                      "panda_finger_joint2"};
    for (int finger_idx = 0; finger_idx < finger_position_names.size();
         ++finger_idx) {
      finger_position_tracking_data_vec.push_back(
          std::make_unique<JointSpaceTrackingData>(
              finger_position_names[finger_idx] + "_traj",
              osc_params.K_p_gripper, osc_params.K_d_gripper,
              osc_params.W_gripper, plant, plant));
      finger_position_tracking_data_vec[finger_idx]->AddJointToTrack(
          finger_position_names[finger_idx],
          finger_position_names[finger_idx] + "dot");
      osc->AddTrackingData(
          std::move(finger_position_tracking_data_vec[finger_idx]));
      builder.Connect(franka_hand_target_position_receiver->get_output_port(0),
                      osc->get_input_port_tracking_data(
                          finger_position_names[finger_idx] + "_traj"));
    }
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
  osc->AddTrackingData(std::move(end_effector_position_tracking_data));
  osc->AddTrackingData(std::move(end_effector_orientation_tracking_data));
  osc->AddForceTrackingData(std::move(end_effector_force_tracking_data));
  osc->SetAccelerationCostWeights(osc_params.W_acceleration);
  osc->SetInputCostWeights(osc_params.W_input_regularization);
  osc->SetInputSmoothingCostWeights(
      osc_params.W_input_smoothing_regularization);
  osc->SetAccelerationConstraints(osc_params.enforce_acceleration_constraints);

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
        builder.AddSystem<dairlib::systems::GravityCompensationRemover>(
            plant, *plant_context);
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
  dairlib::DrawAndSaveDiagramGraph(*shared_diagram);
  // Run lcm-driven simulation
  dairlib::systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, shared_diagram, state_receiver,
      lcm_channel_params.franka_state_channel, true);
  loop.Simulate();
  return 0;
}

}  // namespace magna
}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::magna::DoMain(argc, argv); }
