
#include <dairlib/lcmt_radio_out.hpp>
#include <dairlib/lcmt_timestamped_saved_traj.hpp>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"

#include "examples/cube_flip/trifinger/parameter_headers/trifinger_osc_controller_params.h"
#include "examples/cube_flip/trifinger/parameter_headers/trifinger_lcm_channels.h"
#include "examples/cube_flip/trifinger/trifinger_position_splitter.h"
#include "multibody/kinematic/distance_evaluator.h"

#include "systems/controllers/osc/end_effector_force.h"
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
#include "drake/multibody/tree/linear_bushing_roll_pitch_yaw.h"

namespace dairlib {

using drake::systems::Diagram;
using drake::math::RigidTransform;
using drake::multibody::Parser;
using drake::multibody::ModelInstanceIndex;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::vector;
using multibody::MakeNameToPositionsMap;
using multibody::MakeNameToVelocitiesMap;

using systems::controllers::ExternalForceTrackingData;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::RelativeTranslationTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;

DEFINE_string(osqp_settings,
              "examples/cube_flip/trifinger/parameters/trifinger_osc_qp_settings.yaml",
              "Filepath containing qp settings");
DEFINE_string(controller_parameters,
              "examples/cube_flip/trifinger/parameters/trifinger_osc_controller_params.yaml",
              "Controller settings such as channels. Attempting to minimize "
              "number of gflags");
DEFINE_string(lcm_channels,
              "examples/cube_flip/trifinger/parameters/trifinger_lcm_channels_simulation.yaml",
              "Filepath containing lcm channels");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // load parameters
  drake::yaml::LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;
  TrifingerOSCControllerParams controller_params =
      drake::yaml::LoadYamlFile<TrifingerOSCControllerParams>(
          FLAGS_controller_parameters);

  TrifingerLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<TrifingerLcmChannels>(FLAGS_lcm_channels);

  OSCGains gains = drake::yaml::LoadYamlFile<OSCGains>(
      FindResourceOrThrow(FLAGS_controller_parameters), {}, {}, yaml_options);

  drake::solvers::SolverOptions solver_options =
      drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
          FindResourceOrThrow(FLAGS_osqp_settings))
          .GetAsSolverOptions(drake::solvers::OsqpSolver::id());

  DiagramBuilder<double> builder;

  drake::multibody::MultibodyPlant<double> plant(0.0);
  Parser parser(&plant, nullptr);
	parser.SetAutoRenaming(true);
  
	parser.package_map().Add(
    "robot_properties_fingers", 
    "examples/cube_flip/trifinger/robot_properties_fingers"
  );
  drake::multibody::ModelInstanceIndex trifinger_index =
      parser.AddModels(FindResourceOrThrow(controller_params.trifinger_model))[0];
  ModelInstanceIndex fingertips_index = 
    parser.AddModels(FindResourceOrThrow(controller_params.end_effector_model))[0];
  Eigen::Vector3d base_translation(-0 * Vector3d::UnitZ());
  RigidTransformd X_WI(drake::math::RotationMatrix<double>(), base_translation);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link"), X_WI);

   // HARDCODED
  RigidTransformd X_identity(drake::math::RotationMatrix<double>(), Eigen::Vector3d::Zero());
  vector<std::string> trifinger_tip_names = {
    "finger_tip_link_0", "finger_tip_link_120", "finger_tip_link_240"};

  for (int i = 0; i < 3; i++) {
    const auto& trifinger_tip_frame = plant.GetFrameByName(trifinger_tip_names[i], trifinger_index);
    const auto& fingertip_frame = plant.GetFrameByName(controller_params.end_effector_names[i], fingertips_index);
    plant.WeldFrames(trifinger_tip_frame, fingertip_frame, X_identity);
  }
  plant.Finalize();


  auto plant_context = plant.CreateDefaultContext();
  std::cout << "n_u_: " << plant.num_actuators() << std::endl;

  drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0");

  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);

  auto end_effector_trajectory_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.c3_actor_channel, &lcm));
  auto ee_position_receiver_splitter =
      builder.AddSystem<TrifingerPositionSplitter>(
          "end_effector_position_target");
  auto end_effector_force_receiver =
      builder.AddSystem<systems::LcmTrajectoryReceiver>(
          "end_effector_force_target"); 
  auto trifinger_command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          lcm_channel_params.trifinger_input_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto osc_command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          lcm_channel_params.osc_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto trifinger_command_sender =
      builder.AddSystem<systems::RobotCommandSender>(plant);
  auto osc_command_sender =
      builder.AddSystem<systems::RobotCommandSender>(plant);

  auto end_effector_trajectory_finger_0 =
    builder.AddSystem<EndEffectorPositionTrajectoryGenerator>(plant, 
        plant_context.get(), controller_params.neutral_position[0], false,
        controller_params.end_effector_names[0]);
  end_effector_trajectory_finger_0->SetRemoteControlParameters(
    controller_params.neutral_position[0], controller_params.x_scale,
    controller_params.y_scale, controller_params.z_scale);

  auto end_effector_trajectory_finger_120 =
    builder.AddSystem<EndEffectorPositionTrajectoryGenerator>(plant, 
        plant_context.get(), controller_params.neutral_position[1], false,
        controller_params.end_effector_names[1]);
  end_effector_trajectory_finger_120->SetRemoteControlParameters(
    controller_params.neutral_position[1], controller_params.x_scale,
    controller_params.y_scale, controller_params.z_scale);

  auto end_effector_trajectory_finger_240 =
    builder.AddSystem<EndEffectorPositionTrajectoryGenerator>(plant, 
        plant_context.get(), controller_params.neutral_position[2], false,
        controller_params.end_effector_names[2]);
  end_effector_trajectory_finger_240->SetRemoteControlParameters(
    controller_params.neutral_position[2], controller_params.x_scale,
    controller_params.y_scale, controller_params.z_scale);

  auto end_effector_force_trajectory =
      builder.AddSystem<EndEffectorForceTrajectoryGenerator>(9);
  auto radio_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
          lcm_channel_params.radio_channel, &lcm));
          
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

  const VectorXd& end_effector_acceleration_limits =
    controller_params.end_effector_acceleration * Vector3d::Ones();


  auto ee_position_tracking_data_0 =
      std::make_unique<TransTaskSpaceTrackingData>(
          "end_effector_target_finger_0", controller_params.K_p_end_effector,
          controller_params.K_d_end_effector, controller_params.W_end_effector, 
          plant, plant);
  ee_position_tracking_data_0->AddPointToTrack(
      controller_params.end_effector_names[0]);
  ee_position_tracking_data_0->SetCmdAccelerationBounds(
      -end_effector_acceleration_limits, end_effector_acceleration_limits);     

  auto ee_position_tracking_data_120 =
      std::make_unique<TransTaskSpaceTrackingData>(
          "end_effector_target_finger_120", controller_params.K_p_end_effector,
          controller_params.K_d_end_effector, controller_params.W_end_effector,
          plant, plant);
  ee_position_tracking_data_120->AddPointToTrack(
      controller_params.end_effector_names[1]);          
  ee_position_tracking_data_120->SetCmdAccelerationBounds(
      -end_effector_acceleration_limits, end_effector_acceleration_limits);   

  auto ee_position_tracking_data_240 =
      std::make_unique<TransTaskSpaceTrackingData>(
          "end_effector_target_finger_240", controller_params.K_p_end_effector,
          controller_params.K_d_end_effector, controller_params.W_end_effector,
          plant, plant);
  ee_position_tracking_data_240->AddPointToTrack(
      controller_params.end_effector_names[2]);          
  ee_position_tracking_data_240->SetCmdAccelerationBounds(
      -end_effector_acceleration_limits, end_effector_acceleration_limits);   


  vector<Vector3d> pts_to_track;
  for (int i = 0; i < 3; i++) {
    pts_to_track.push_back(Vector3d::Zero());
  }
  auto end_effector_force_tracking_data =
      std::make_unique<ExternalForceTrackingData>(
          "end_effector_force", controller_params.W_ee_lambda, plant, plant,
          controller_params.end_effector_names, pts_to_track);
  
  osc->AddTrackingData(std::move(ee_position_tracking_data_0));
  osc->AddTrackingData(std::move(ee_position_tracking_data_120));
  osc->AddTrackingData(std::move(ee_position_tracking_data_240));
  osc->AddForceTrackingData(std::move(end_effector_force_tracking_data));
  osc->SetAccelerationCostWeights(gains.W_acceleration);
  osc->SetInputCostWeights(gains.W_input_regularization);
  osc->SetInputSmoothingCostWeights(gains.W_input_smoothing_regularization);
  osc->SetAccelerationConstraints(
      controller_params.enforce_acceleration_constraints);
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
                    trifinger_command_sender->get_input_port());
  } else {
    if (FLAGS_lcm_channels ==
        "examples/franka/parameters/lcm_channels_hardware.yaml") {
      std::cerr << "Using hardware lcm channels but not cancelling gravity "
                   "compensation. Please check the OSC settings"
                << std::endl;
      return -1;
    }
    builder.Connect(osc->get_output_port_osc_command(),
                    trifinger_command_sender->get_input_port(0));
  }

  builder.Connect(radio_sub->get_output_port(),
                  end_effector_trajectory_finger_0->get_input_port_radio());
  builder.Connect(radio_sub->get_output_port(),
                  end_effector_trajectory_finger_120->get_input_port_radio());
  builder.Connect(radio_sub->get_output_port(),
                  end_effector_trajectory_finger_240->get_input_port_radio());
  builder.Connect(radio_sub->get_output_port(),
                  end_effector_force_trajectory->get_input_port_radio());
                  
  builder.Connect(trifinger_command_sender->get_output_port(),
                  trifinger_command_pub->get_input_port());
  builder.Connect(osc_command_sender->get_output_port(),
                  osc_command_pub->get_input_port());
  builder.Connect(osc->get_output_port_osc_command(),
                  osc_command_sender->get_input_port(0));

  builder.Connect(state_receiver->get_output_port(0),
                  osc->get_input_port_robot_output());
  builder.Connect(end_effector_trajectory_sub->get_output_port(),
                  ee_position_receiver_splitter->get_input_port_trajectory());
  builder.Connect(end_effector_trajectory_sub->get_output_port(),
                  end_effector_force_receiver->get_input_port_trajectory());

  builder.Connect(ee_position_receiver_splitter->get_output_port_finger_0(),
                  end_effector_trajectory_finger_0->get_input_port_trajectory());
  builder.Connect(ee_position_receiver_splitter->get_output_port_finger_120(),
                  end_effector_trajectory_finger_120->get_input_port_trajectory());
  builder.Connect(ee_position_receiver_splitter->get_output_port_finger_240(),
                  end_effector_trajectory_finger_240->get_input_port_trajectory());

  builder.Connect(end_effector_trajectory_finger_0->get_output_port(0),
                  osc->get_input_port_tracking_data("end_effector_target_finger_0"));
  builder.Connect(end_effector_trajectory_finger_120->get_output_port(0),
                  osc->get_input_port_tracking_data("end_effector_target_finger_120"));
  builder.Connect(end_effector_trajectory_finger_240->get_output_port(0),
                  osc->get_input_port_tracking_data("end_effector_target_finger_240"));

  builder.Connect(end_effector_force_receiver->get_output_port(0),
                  end_effector_force_trajectory->get_input_port_trajectory());
  builder.Connect(end_effector_force_trajectory->get_output_port(0),
                  osc->get_input_port_tracking_data("end_effector_force"));

  auto owned_diagram = builder.Build();
  std::shared_ptr<Diagram<double>> shared_diagram = std::move(owned_diagram);
  DrawAndSaveDiagramGraph(*shared_diagram, "/home/ericcui/diagrams/trifinger_osc_controller");
  // Run lcm-driven simulation
  std::cout << "Before lcm driven loop" << std::endl;
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, shared_diagram, state_receiver,
      lcm_channel_params.trifinger_state_channel, true);
  loop.Simulate();
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }