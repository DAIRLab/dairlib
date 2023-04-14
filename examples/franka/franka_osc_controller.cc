
#include <dairlib/lcmt_radio_out.hpp>
#include <dairlib/lcmt_timestamped_saved_traj.hpp>
#include <gflags/gflags.h>

#include "examples/franka/franka_controller_params.h"
#include "examples/franka/systems/end_effector_orientation.h"
#include "examples/franka/systems/end_effector_trajectory.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
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

using systems::controllers::JointSpaceTrackingData;
using systems::controllers::RelativeTranslationTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;

DEFINE_string(osqp_settings,
              "examples/Cassie/osc_run/osc_running_qp_settings.yaml",
              "Filepath containing qp settings");
DEFINE_string(controller_settings, "",
              "Controller settings such as channels. Attempting to minimize "
              "number of gflags");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // load parameters
  drake::yaml::LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;
  FrankaControllerParams controller_params =
      drake::yaml::LoadYamlFile<FrankaControllerParams>(
          "examples/franka/franka_controller_params.yaml");
  OSCGains gains = drake::yaml::LoadYamlFile<OSCGains>(
      FindResourceOrThrow("examples/franka/franka_controller_params.yaml"), {},
      {}, yaml_options);
  drake::solvers::SolverOptions solver_options =
      drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
          FindResourceOrThrow(FLAGS_osqp_settings))
          .GetAsSolverOptions(drake::solvers::OsqpSolver::id());
  DiagramBuilder<double> builder;

  drake::multibody::MultibodyPlant<double> plant(0.0);
  Parser parser(&plant, nullptr);
  parser.AddModelFromFile("examples/franka/urdf/franka.urdf");
  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"),
                   X_WI);

  VectorXd rotor_inertias(plant.num_actuators());
  rotor_inertias << 61, 61, 61, 61, 61, 61, 61;
  rotor_inertias *= 1e-6;
  VectorXd gear_ratios(plant.num_actuators());
  gear_ratios << 25, 25, 25, 25, 25, 25, 25;
  std::vector<std::string> motor_joint_names = {
      "panda_motor1", "panda_motor2", "panda_motor3", "panda_motor4",
      "panda_motor5", "panda_motor6", "panda_motor7"};
  for (int i = 0; i < rotor_inertias.size(); ++i) {
    auto& joint_actuator = plant.get_mutable_joint_actuator(
        drake::multibody::JointActuatorIndex(i));
    joint_actuator.set_default_rotor_inertia(rotor_inertias(i));
    joint_actuator.set_default_gear_ratio(gear_ratios(i));
    DRAKE_DEMAND(motor_joint_names[i] == joint_actuator.name());
  }

  plant.Finalize();
  auto plant_context = plant.CreateDefaultContext();

  drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0");

//  auto lcm_traj =
//      LcmTrajectory("examples/franka/saved_trajectories/franka_defaults");
//  auto orientation_traj = LcmTrajectory::Trajectory();
//  orientation_traj.datapoints = MatrixXd::Zero(4, 2);
//  orientation_traj.time_vector = Eigen::Vector2d(0.0, 1.0);
//  orientation_traj.traj_name = "end_effector_orientation_target";
//  orientation_traj.datatypes = std::vector<std::string>(2, "orientation");
//  lcm_traj.AddTrajectory(orientation_traj.traj_name, orientation_traj);
//  lcm_traj.WriteToFile("examples/franka/saved_trajectories/franka_defaults");

  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);
  auto end_effector_trajectory_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          controller_params.c3_channel, &lcm));
  auto end_effector_receiver =
      builder.AddSystem<systems::LcmTrajectoryReceiver>("end_effector_traj");
  auto end_effector_orientation_receiver =
      builder.AddSystem<systems::LcmOrientationTrajectoryReceiver>(
          "end_effector_orientation_target");
  auto command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          controller_params.controller_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto command_sender = builder.AddSystem<systems::RobotCommandSender>(plant);
  auto end_effector_trajectory =
      builder.AddSystem<EndEffectorTrajectoryGenerator>(plant,
                                                        plant_context.get());
  auto end_effector_orientation_trajectory =
      builder.AddSystem<EndEffectorOrientationGenerator>(plant,
                                                         plant_context.get());
  end_effector_orientation_trajectory->SetTrackOrientation(controller_params.track_end_effector_orientation);
  auto radio_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
          controller_params.radio_channel, &lcm));
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      plant, plant, plant_context.get(), plant_context.get(), false);
  auto osc_debug_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
          controller_params.osc_debug_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));

  osc->SetAccelerationCostWeights(gains.W_acceleration);
  osc->SetInputCostWeights(gains.W_input_regularization);
  osc->SetInputSmoothingCostWeights(gains.W_input_smoothing_regularization);

  auto end_effector_position_tracking_data =
      std::make_unique<TransTaskSpaceTrackingData>(
          "end_effector_target", controller_params.K_p_end_effector,
          controller_params.K_d_end_effector, controller_params.W_end_effector,
          plant, plant);
  end_effector_position_tracking_data->AddPointToTrack("paddle");
  const VectorXd& bound =
      controller_params.end_effector_acceleration * Vector3d::Ones();
  end_effector_position_tracking_data->SetCmdAccelerationBounds(-bound, bound);
  auto end_effector_position_tracking_data_for_rel =
      std::make_unique<TransTaskSpaceTrackingData>(
          "end_effector_target", controller_params.K_p_end_effector,
          controller_params.K_d_end_effector, controller_params.W_end_effector,
          plant, plant);
  end_effector_position_tracking_data_for_rel->AddPointToTrack("paddle");
  auto mid_link_position_tracking_data_for_rel =
      std::make_unique<TransTaskSpaceTrackingData>(
          "mid_link", controller_params.K_p_mid_link,
          controller_params.K_d_mid_link, controller_params.W_mid_link, plant,
          plant);
  mid_link_position_tracking_data_for_rel->AddPointToTrack("panda_link4");
  auto wrist_relative_tracking_data =
      std::make_unique<RelativeTranslationTrackingData>(
          "wrist_down_target", controller_params.K_p_mid_link,
          controller_params.K_d_mid_link, controller_params.W_mid_link, plant,
          plant, mid_link_position_tracking_data_for_rel.get(),
          end_effector_position_tracking_data_for_rel.get());
  Eigen::Vector3d wrist_down_target = Eigen::Vector3d::Zero();
  wrist_down_target(1) = 0.0;
  wrist_down_target(2) = 0.0;

  auto end_effector_orientation_tracking_data =
      std::make_unique<RotTaskSpaceTrackingData>(
          "end_effector_orientation_target",
          controller_params.K_p_end_effector_rot,
          controller_params.K_d_end_effector_rot,
          controller_params.W_end_effector_rot, plant, plant);
  end_effector_orientation_tracking_data->AddFrameToTrack("paddle");
  Eigen::VectorXd orientation_target = Eigen::VectorXd::Zero(4);
  orientation_target(0) = 1;

  osc->AddTrackingData(std::move(end_effector_position_tracking_data));
  osc->AddConstTrackingData(std::move(wrist_relative_tracking_data),
                            wrist_down_target);
  osc->AddTrackingData(std::move(end_effector_orientation_tracking_data));

  osc->SetContactFriction(controller_params.mu);
  osc->SetOsqpSolverOptions(solver_options);

  osc->Build();

  builder.Connect(state_receiver->get_output_port(0),
                  end_effector_trajectory->get_input_port_state());
  builder.Connect(radio_sub->get_output_port(0),
                  end_effector_trajectory->get_input_port_radio());
  builder.Connect(state_receiver->get_output_port(0),
                  end_effector_orientation_trajectory->get_input_port_state());
  builder.Connect(radio_sub->get_output_port(0),
                  end_effector_orientation_trajectory->get_input_port_radio());
  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port());
  builder.Connect(osc->get_output_port_osc_command(),
                  command_sender->get_input_port(0));
  builder.Connect(osc->get_output_port_osc_debug(),
                  osc_debug_pub->get_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  osc->get_input_port_robot_output());
  builder.Connect(end_effector_trajectory_sub->get_output_port(),
                  end_effector_receiver->get_input_port_trajectory());
  builder.Connect(
      end_effector_trajectory_sub->get_output_port(),
      end_effector_orientation_receiver->get_input_port_trajectory());
  builder.Connect(end_effector_receiver->get_output_port(0),
                  end_effector_trajectory->get_input_port_trajectory());
  builder.Connect(end_effector_orientation_receiver->get_output_port(0),
                  end_effector_orientation_trajectory->get_input_port_trajectory());
  builder.Connect(end_effector_trajectory->get_output_port(0),
                  osc->get_input_port_tracking_data("end_effector_target"));
  builder.Connect(end_effector_orientation_trajectory->get_output_port(0),
                  osc->get_input_port_tracking_data("end_effector_orientation_target"));



  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("franka_osc_controller"));
  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, std::move(owned_diagram), state_receiver,
      controller_params.state_channel, true);
  DrawAndSaveDiagramGraph(*loop.get_diagram());
  loop.Simulate();
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }