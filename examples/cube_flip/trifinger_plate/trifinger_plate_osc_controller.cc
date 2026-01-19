
#include <dairlib/lcmt_radio_out.hpp>
#include <dairlib/lcmt_timestamped_saved_traj.hpp>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"

#include "examples/cube_flip/trifinger_plate/parameter_headers/trifinger_plate_lcm_channels.h"
#include "examples/cube_flip/trifinger_plate/parameter_headers/trifinger_plate_osc_controller_params.h"
#include "multibody/kinematic/distance_evaluator.h"

#include "systems/controllers/osc/end_effector_force.h"
#include "systems/controllers/osc/end_effector_torque.h"
#include "systems/controllers/osc/end_effector_orientation.h"
#include "systems/controllers/osc/end_effector_position.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
#include "systems/controllers/gravity_compensator.h"
#include "systems/controllers/osc/external_force_tracking_data.h"
#include "systems/controllers/osc/external_torque_tracking_data.h"
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
using multibody::MakeNameToPositionsMap;
using multibody::MakeNameToVelocitiesMap;
using multibody::DistanceEvaluator;
using multibody::KinematicEvaluatorSet;

using systems::controllers::ExternalTorqueTrackingData;
using systems::controllers::ExternalForceTrackingData;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::RelativeTranslationTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;

DEFINE_string(osqp_settings,
              "examples/cube_flip/trifinger_plate/parameters/trifinger_plate_osc_qp_settings.yaml",
              "Filepath containing qp settings");
DEFINE_string(controller_parameters,
              "examples/cube_flip/trifinger_plate/parameters/trifinger_plate_osc_controller_params.yaml",
              "Controller settings such as channels. Attempting to minimize "
              "number of gflags");
DEFINE_string(lcm_channels,
              "examples/cube_flip/trifinger_plate/parameters/trifinger_lcm_channels_simulation.yaml",
              "Filepath containing lcm channels");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // load parameters
  drake::yaml::LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;
  TrifingerPlateControllerParams controller_params =
      drake::yaml::LoadYamlFile<TrifingerPlateControllerParams>(
          FLAGS_controller_parameters);

  TrifingerPlateLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<TrifingerPlateLcmChannels>(FLAGS_lcm_channels);

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
    "examples/cube_flip/trifinger_plate/robot_properties_fingers"
  );
  drake::multibody::ModelInstanceIndex trifinger_index =
      parser.AddModels(FindResourceOrThrow(controller_params.trifinger_model))[0];

	// Add end effector to plant
	drake::multibody::ModelInstanceIndex end_effector_index =
		parser.AddModels(FindResourceOrThrow(controller_params.end_effector_model))[0];

  Eigen::Vector3d base_translation(0.5 * Eigen::Vector3d::UnitZ());
  RigidTransformd X_WI(RotationMatrixd::MakeXRotation(M_PI), base_translation);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link"), X_WI);
  plant.Finalize();


  auto plant_context = plant.CreateDefaultContext();
  std::cout << "n_u_: " << plant.num_actuators() << std::endl;

  drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0");

  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);
  state_receiver->get_input_port(0);
  std::cout << "state reciever get input port 0" << std::endl;

  auto end_effector_trajectory_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.c3_actor_channel, &lcm));
  auto end_effector_position_receiver =
      builder.AddSystem<systems::LcmTrajectoryReceiver>(
          "end_effector_position_target");
  auto end_effector_force_receiver =
      builder.AddSystem<systems::LcmTrajectoryReceiver>(
          "end_effector_force_target"); 
  auto end_effector_torque_receiver =
      builder.AddSystem<systems::LcmTrajectoryReceiver>(
          "end_effector_torque_target");
  auto end_effector_orientation_receiver =
      builder.AddSystem<systems::LcmOrientationTrajectoryReceiver>(
          "end_effector_orientation_target");
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

  auto end_effector_trajectory =
      builder.AddSystem<EndEffectorPositionTrajectoryGenerator>(plant, 
          plant_context.get(), controller_params.neutral_position, false,
          controller_params.end_effector_name);
  end_effector_trajectory->SetRemoteControlParameters(
      controller_params.neutral_position, controller_params.x_scale,
      controller_params.y_scale, controller_params.z_scale);
  auto end_effector_orientation_trajectory =
      builder.AddSystem<EndEffectorOrientationTrajectoryGenerator>();
  end_effector_orientation_trajectory->SetTrackOrientation(
      controller_params.track_end_effector_orientation);
  auto end_effector_force_trajectory =
      builder.AddSystem<EndEffectorForceTrajectoryGenerator>();
  auto end_effector_torque_trajectory =
      builder.AddSystem<EndEffectorTorqueTrajectoryGenerator>();
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


  auto end_effector_position_tracking_data =
      std::make_unique<TransTaskSpaceTrackingData>(
          "end_effector_target", controller_params.K_p_end_effector,
          controller_params.K_d_end_effector, controller_params.W_end_effector,
          plant, plant);
  end_effector_position_tracking_data->AddPointToTrack(
      controller_params.end_effector_name);
  const VectorXd& end_effector_acceleration_limits =
      controller_params.end_effector_acceleration * Vector3d::Ones();
  end_effector_position_tracking_data->SetCmdAccelerationBounds(
      -end_effector_acceleration_limits, end_effector_acceleration_limits);

  auto end_effector_force_tracking_data =
      std::make_unique<ExternalForceTrackingData>(
          "end_effector_force", controller_params.W_ee_lambda, plant, plant,
          controller_params.end_effector_name, Vector3d::Zero());

  auto end_effector_torque_tracking_data =
      std::make_unique<ExternalTorqueTrackingData>(
          "end_effector_torque", controller_params.W_ee_lambda_tau, plant, plant,
          controller_params.end_effector_name, Vector3d::Zero());

  auto end_effector_orientation_tracking_data =
      std::make_unique<RotTaskSpaceTrackingData>(
          "end_effector_orientation_target",
          controller_params.K_p_end_effector_rot,
          controller_params.K_d_end_effector_rot,
          controller_params.W_end_effector_rot, plant, plant);
  end_effector_orientation_tracking_data->AddFrameToTrack(
      controller_params.end_effector_name);
  Eigen::VectorXd orientation_target = Eigen::VectorXd::Zero(4);
  orientation_target(0) = 1;


  // Add kinematic constraint between fingertips and plate
	std::vector<VectorXd> plate_socket_offsets = controller_params.finger_attachment_points;
	std::vector<std::string> tip_names = {
			"finger_tip_link_0", 
			"finger_tip_link_120", 
			"finger_tip_link_240"
	};
  multibody::KinematicEvaluatorSet<double> evaluators(plant);

  const auto& plate_frame = plant.GetFrameByName("plate", end_effector_index);

  const auto& tip_frame_0 = plant.GetFrameByName(tip_names[0], trifinger_index);
  auto finger_plate_constraint_0 =
    DistanceEvaluator(plant, Vector3d::Zero(), tip_frame_0, plate_socket_offsets[0], plate_frame, 0);
  const auto& tip_frame_120 = plant.GetFrameByName(tip_names[1], trifinger_index);
  auto finger_plate_constraint_120 =
    DistanceEvaluator(plant, Vector3d::Zero(), tip_frame_120, plate_socket_offsets[1], plate_frame, 0);
  const auto& tip_frame_240 = plant.GetFrameByName(tip_names[2], trifinger_index);
  auto finger_plate_constraint_240 =
    DistanceEvaluator(plant, Vector3d::Zero(), tip_frame_240, plate_socket_offsets[2], plate_frame, 0);

  evaluators.add_evaluator(&finger_plate_constraint_0);
	evaluators.add_evaluator(&finger_plate_constraint_120);
  evaluators.add_evaluator(&finger_plate_constraint_240);

  
  osc->AddTrackingData(std::move(end_effector_position_tracking_data));
  osc->AddTrackingData(std::move(end_effector_orientation_tracking_data));
  osc->AddForceTrackingData(std::move(end_effector_force_tracking_data));
  osc->AddTorqueTrackingData(std::move(end_effector_torque_tracking_data));
  osc->SetAccelerationCostWeights(gains.W_acceleration);
  osc->SetInputCostWeights(gains.W_input_regularization);
  osc->SetInputSmoothingCostWeights(gains.W_input_smoothing_regularization);
  osc->SetAccelerationConstraints(
      controller_params.enforce_acceleration_constraints);
  osc->AddKinematicConstraint(&evaluators);
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
                  end_effector_trajectory->get_input_port_radio());
  builder.Connect(radio_sub->get_output_port(),
                  end_effector_orientation_trajectory->get_input_port_radio());
  builder.Connect(radio_sub->get_output_port(),
                  end_effector_force_trajectory->get_input_port_radio());
  builder.Connect(radio_sub->get_output_port(),
                  end_effector_torque_trajectory->get_input_port_radio());
  builder.Connect(trifinger_command_sender->get_output_port(),
                  trifinger_command_pub->get_input_port());
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
  builder.Connect(end_effector_trajectory_sub->get_output_port(),
                  end_effector_torque_receiver->get_input_port_trajectory());
  builder.Connect(
      end_effector_trajectory_sub->get_output_port(),
      end_effector_orientation_receiver->get_input_port_trajectory());
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
  builder.Connect(end_effector_torque_receiver->get_output_port(0),
                  end_effector_torque_trajectory->get_input_port_trajectory());
  builder.Connect(end_effector_torque_trajectory->get_output_port(0),
                  osc->get_input_port_tracking_data("end_effector_torque"));
  auto owned_diagram = builder.Build();
  std::shared_ptr<Diagram<double>> shared_diagram = std::move(owned_diagram);
  shared_diagram->set_name(("trifinger_plate_osc_controller"));
  DrawAndSaveDiagramGraph(*shared_diagram);
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