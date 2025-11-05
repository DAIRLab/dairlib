#include <string>

#include <drake/lcm/drake_lcm.h>
#include <gflags/gflags.h>
#include <optional>

#include "common/find_resource.h"
#include "dairlib/lcmt_franka_cartesian_pose.hpp"
#include "dairlib/lcmt_osc_output.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "examples/magna/systems/cartesian_pose_trajectory_generator.h"
#include "examples/magna/systems/franka_common.h"
#include "examples/magna/systems/franka_target_cartesian_pose_receiver.h"
#include "franka_cartesian_osc_controller_params.h"
#include "systems/controllers/gravity_compensator.h"
#include "systems/controllers/osc/joint_space_tracking_data.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/controllers/osc/rot_space_tracking_data.h"
#include "systems/controllers/osc/trans_space_tracking_data.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"
#include "systems/trajectory_optimization/lcm_trajectory_systems.h"

#include "drake/common/text_logging.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

DEFINE_bool(
    is_spacemouse, false,
    "True for spacemouse (expecting single target cartesian pose), false "
    "(expecting target cartesian pose trajectory)");
DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");

using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

using Eigen::VectorXd;

namespace dairlib {

using systems::GravityCompensationRemover;
using systems::LcmDrivenLoop;
using systems::LcmOrientationTrajectoryReceiver;
using systems::LcmTrajectoryReceiver;
using systems::RobotCommandSender;
using systems::RobotOutputReceiver;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::OperationalSpaceControl;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;

namespace examples {
namespace magna {
namespace systems {
namespace controllers {

int RunFrankaCartesianOscController() {
  try {
    // setup lcm
    drake::lcm::DrakeLcm lcm(FLAGS_lcm_url);

    // Load controller parameters
    FrankaCartesianOSCControllerParams controller_params =
        drake::yaml::LoadYamlFile<FrankaCartesianOSCControllerParams>(
            FindResourceOrThrow("examples/magna/params/"
                                "franka_cartesian_osc_controller_params.yaml"));
    drake::solvers::SolverOptions solver_options =
        drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
            FindResourceOrThrow(controller_params.osc_qp_settings_file))
            .GetAsSolverOptions(drake::solvers::OsqpSolver::id());

    // Piece together the diagram.
    drake::systems::DiagramBuilder<double> builder;

    // Define channel inputs and outputs
    drake::multibody::MultibodyPlant<double> plant(0.0);
    AddFrankaToPlant(&plant, nullptr,
                     controller_params.end_effector_model_file);
    plant.Finalize();
    auto state_receiver = builder.AddSystem<RobotOutputReceiver>(plant);
    auto franka_command_sender = builder.AddSystem<RobotCommandSender>(plant);
    auto franka_command_pub =
        builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
            controller_params.franka_input_channel, &lcm,
            TriggerTypeSet({TriggerType::kForced})));
    auto osc_debug_pub =
        builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
            "OSC_DEBUG", &lcm, TriggerTypeSet({TriggerType::kForced})));

    // start [OSC controller]
    // Create a Franka-only plant for OSC
    drake::multibody::MultibodyPlant<double> osc_plant(0.0);
    AddFrankaToPlant(&osc_plant, nullptr,
                     controller_params.end_effector_model_file);
    osc_plant.Finalize();
    auto osc_context = osc_plant.CreateDefaultContext();

    // Create the OSC controller
    auto osc_controller = builder.AddSystem<OperationalSpaceControl>(
        osc_plant, osc_plant, osc_context.get(), osc_context.get(), false);

    // Add end-effector position tracking task
    auto end_effector_position_tracking_data =
        std::make_unique<TransTaskSpaceTrackingData>(
            "end_effector_translation_target",
            controller_params.Kp_ee_translation,
            controller_params.Kd_ee_translation,
            controller_params.W_ee_translation, osc_plant, osc_plant);
    end_effector_position_tracking_data->AddPointToTrack("panda_link7",
                                                         T_EE_L7.translation());
    const VectorXd& end_effector_acceleration_limits =
        controller_params.end_effector_acceleration * Eigen::Vector3d::Ones();
    end_effector_position_tracking_data->SetCmdAccelerationBounds(
        -end_effector_acceleration_limits, end_effector_acceleration_limits);
    osc_controller->AddTrackingData(
        std::move(end_effector_position_tracking_data));

    // Add end-effector orientation tracking task
    auto end_effector_orientation_tracking_data =
        std::make_unique<RotTaskSpaceTrackingData>(
            "end_effector_orientation_target", controller_params.Kp_ee_rotation,
            controller_params.Kd_ee_rotation, controller_params.W_ee_rotation,
            osc_plant, osc_plant);
    end_effector_orientation_tracking_data->AddFrameToTrack(
        "panda_link7", T_EE_L7.GetAsIsometry3());
    osc_controller->AddTrackingData(
        std::move(end_effector_orientation_tracking_data));

    // Track joint positions to maintain them as much as possible
    auto joint_position_tracking_data =
        std::make_unique<JointSpaceTrackingData>(
            "joint_position_target", controller_params.Kp_mid_link,
            controller_params.Kd_mid_link, controller_params.W_mid_link,
            osc_plant, osc_plant);
    // Maintain joint positions as much as possible
    std::vector<std::string> joint_position_names = {
        "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
        "panda_joint5", "panda_joint6", "panda_joint7"};
    std::vector<std::string> joint_velocity_names = {
        "panda_joint1dot", "panda_joint2dot", "panda_joint3dot",
        "panda_joint4dot", "panda_joint5dot", "panda_joint6dot",
        "panda_joint7dot"};
    joint_position_tracking_data->AddJointsToTrack(joint_position_names,
                                                   joint_velocity_names);
    osc_controller->AddTrackingData(std::move(joint_position_tracking_data));

    // Set other OSC parameters
    osc_controller->SetAccelerationCostWeights(
        controller_params.osc_gains.W_acceleration);
    osc_controller->SetInputCostWeights(
        controller_params.osc_gains.W_input_regularization);
    osc_controller->SetInputSmoothingCostWeights(
        controller_params.osc_gains.W_input_smoothing_regularization);
    osc_controller->SetAccelerationConstraints(
        controller_params.enforce_acceleration_constraints);
    osc_controller->SetContactFriction(controller_params.osc_gains.mu);
    osc_controller->SetOsqpSolverOptions(solver_options);

    // Build the OSC controller
    osc_controller->Build();
    drake::log()->info("Finished building OSC controller.");
    // end [OSC controller]

    // Perform gravity compensation and send output of OSC
    // controller to command sender
    auto gravity_compensator =
        builder.AddSystem<GravityCompensationRemover>(osc_plant, *osc_context);
    builder.Connect(osc_controller->get_output_port_osc_command(),
                    gravity_compensator->get_input_port());
    builder.Connect(gravity_compensator->get_output_port(),
                    franka_command_sender->get_input_port());
    builder.Connect(franka_command_sender->get_output_port(),
                    franka_command_pub->get_input_port());
    builder.Connect(osc_controller->get_output_port_osc_debug(),
                    osc_debug_pub->get_input_port());

    // Connect inputs to OSC controller
    builder.Connect(state_receiver->get_output_port(),
                    osc_controller->get_input_port_robot_output());
    auto context = plant.CreateDefaultContext();
    if (FLAGS_is_spacemouse) {
      // Build trajectory from single target pose
      auto target_cartesian_pose_subscriber = builder.AddSystem(
          LcmSubscriberSystem::Make<dairlib::lcmt_franka_cartesian_pose>(
              controller_params.target_cartesian_pose_channel, &lcm));
      auto target_cartesian_pose_receiver =
          builder.AddSystem<FrankaTargetCartesianPoseReceiver>();
      builder.Connect(target_cartesian_pose_subscriber->get_output_port(),
                      target_cartesian_pose_receiver->get_input_port());

      // Create a trajectory with a single pose
      auto cartesian_pose_trajectory_generator =
          builder.AddSystem<CartesianPoseTrajectoryGenerator>(
              plant, context.get(), controller_params.end_effector_name);
      builder.Connect(
          state_receiver->get_output_port(),
          cartesian_pose_trajectory_generator->get_input_port_robot_state());
      builder.Connect(target_cartesian_pose_receiver->get_output_port(),
                      cartesian_pose_trajectory_generator
                          ->get_input_port_target_cartesian_pose());

      // Send translation and rotation trajectories to OSC controller
      builder.Connect(cartesian_pose_trajectory_generator
                          ->get_output_port_translation_trajectory(),
                      osc_controller->get_input_port_tracking_data(
                          "end_effector_translation_target"));

      builder.Connect(cartesian_pose_trajectory_generator
                          ->get_output_port_rotation_trajectory(),
                      osc_controller->get_input_port_tracking_data(
                          "end_effector_orientation_target"));

      builder.Connect(cartesian_pose_trajectory_generator
                          ->get_output_port_joint_trajectory(),
                      osc_controller->get_input_port_tracking_data(
                          "joint_position_target"));
    } else {
      // Receive target trajectory directly
      auto end_effector_trajectory_subscriber = builder.AddSystem(
          LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
              controller_params.target_cartesian_pose_trajectory_channel,
              &lcm));

      auto cartesian_pose_trajectory_generator =
          builder.AddSystem<CartesianPoseTrajectoryGenerator>(
              plant, context.get(), controller_params.end_effector_name, true);
      builder.Connect(
          state_receiver->get_output_port(),
          cartesian_pose_trajectory_generator->get_input_port_robot_state());

      // Send translation trajectory to OSC controller
      auto end_effector_position_receiver = builder.AddSystem<
          LcmTrajectoryReceiver>("end_effector_translation_target");
      builder.Connect(
          end_effector_trajectory_subscriber->get_output_port(),
          end_effector_position_receiver->get_input_port_trajectory());
      builder.Connect(
          end_effector_position_receiver->get_output_port(),
          cartesian_pose_trajectory_generator
              ->get_input_port_target_cartesian_translation_trajectory());
      builder.Connect(cartesian_pose_trajectory_generator
                          ->get_output_port_translation_trajectory(),
                      osc_controller->get_input_port_tracking_data(
                          "end_effector_translation_target"));

      // Send orientation trajectory to OSC controller
      auto end_effector_orientation_receiver =
          builder.AddSystem<LcmOrientationTrajectoryReceiver>(
              "end_effector_orientation_target");
      builder.Connect(
          end_effector_trajectory_subscriber->get_output_port(),
          end_effector_orientation_receiver->get_input_port_trajectory());
      builder.Connect(
          end_effector_orientation_receiver->get_output_port(),
          cartesian_pose_trajectory_generator
              ->get_input_port_target_cartesian_rotation_trajectory());
      builder.Connect(cartesian_pose_trajectory_generator
                          ->get_output_port_rotation_trajectory(),
                      osc_controller->get_input_port_tracking_data(
                          "end_effector_orientation_target"));

      builder.Connect(cartesian_pose_trajectory_generator
                          ->get_output_port_joint_trajectory(),
                      osc_controller->get_input_port_tracking_data(
                          "joint_position_target"));
    }

    auto owned_diagram = builder.Build();
    std::shared_ptr<drake::systems::Diagram<double>> shared_diagram_ptr =
        std::move(owned_diagram);
    shared_diagram_ptr->set_name(("franka_cartesian_osc_controller"));
    DrawAndSaveDiagramGraph(*shared_diagram_ptr);
    // Run lcm-driven simulation
    LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
        &lcm, shared_diagram_ptr, state_receiver,
        controller_params.franka_state_channel, true);
    drake::log()->info("Starting LCM-driven loop simulation.");
    loop.Simulate();
    drake::log()->info("Finished LCM-driven loop simulation.");
  } catch (const std::exception& e) {
    drake::log()->error(
        "Exception caught in RunFrankaCartesianOscController: {}", e.what());
    return -1;
  }
  return 0;
}
}  // namespace controllers
}  // namespace systems
}  // namespace magna
}  // namespace examples
}  // namespace dairlib

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::set_log_level("trace");
  return dairlib::examples::magna::systems::controllers::
      RunFrankaCartesianOscController();
}