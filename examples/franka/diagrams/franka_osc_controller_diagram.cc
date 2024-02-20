//
// Created by yangwill on 2/19/24.
//

#include "examples/franka/diagrams/franka_osc_controller_diagram.h"

#include <dairlib/lcmt_radio_out.hpp>
#include <dairlib/lcmt_timestamped_saved_traj.hpp>
#include <drake/common/find_resource.h>
#include <drake/common/yaml/yaml_io.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake/systems/lcm/lcm_subscriber_system.h>
#include <drake/systems/primitives/pass_through.h>

#include "common/eigen_utils.h"
#include "examples/franka/parameters/franka_lcm_channels.h"
#include "examples/franka/parameters/franka_osc_controller_params.h"
#include "examples/franka/systems/end_effector_force_trajectory.h"
#include "examples/franka/systems/end_effector_orientation.h"
#include "examples/franka/systems/end_effector_trajectory.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
#include "systems/controllers/gravity_compensator.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"
#include "systems/primitives/radio_parser.h"
#include "systems/trajectory_optimization/lcm_trajectory_systems.h"

namespace dairlib {

namespace examples {
namespace controllers {

using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::math::RigidTransform;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
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

FrankaOSCControllerDiagram::FrankaOSCControllerDiagram(
    const string& controller_settings, const string& lcm_channels,
    drake::lcm::DrakeLcm* lcm) {
  DiagramBuilder<double> builder;

  // load parameters
  drake::yaml::LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;
  FrankaControllerParams controller_params =
      drake::yaml::LoadYamlFile<FrankaControllerParams>(controller_settings);
  FrankaLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<FrankaLcmChannels>(lcm_channels);
  OSCGains gains = drake::yaml::LoadYamlFile<OSCGains>(
      FindResourceOrThrow(controller_settings), {}, {}, yaml_options);
  drake::solvers::SolverOptions solver_options =
      drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
          FindResourceOrThrow(
              "examples/franka/parameters/franka_osc_qp_settings.yaml"))
          .GetAsSolverOptions(drake::solvers::OsqpSolver::id());

  plant_ = new drake::multibody::MultibodyPlant<double> (0.0);
  Parser parser(plant_, nullptr);
  parser.AddModels(drake::FindResourceOrThrow(controller_params.franka_model));

  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant_->WeldFrames(plant_->world_frame(), plant_->GetFrameByName("panda_link0"),
                   X_WI);

  if (!controller_params.end_effector_name.empty()) {
    drake::multibody::ModelInstanceIndex end_effector_index = parser.AddModels(
        FindResourceOrThrow(controller_params.end_effector_model))[0];
    RigidTransform<double> T_EE_W =
        RigidTransform<double>(drake::math::RotationMatrix<double>(),
                               controller_params.tool_attachment_frame);
    plant_->WeldFrames(plant_->GetFrameByName("panda_link7"),
                     plant_->GetFrameByName(controller_params.end_effector_name,
                                          end_effector_index),
                     T_EE_W);
  } else {
    std::cout << "OSC plant has been constructed with no end effector."
              << std::endl;
  }

  plant_->Finalize();
  plant_context_ = plant_->CreateDefaultContext();

  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(*plant_);
//  auto end_effector_trajectory_receiver =
//      builder.AddSystem(
//          drake::systems::PassThrough<lcmt_timestamped_saved_traj>(drake::Value<lcmt_timestamped_saved_traj>{}));
  auto end_effector_position_receiver =
          builder.AddSystem<systems::LcmTrajectoryReceiver>(
              "end_effector_position_target");
  auto end_effector_force_receiver =
      builder.AddSystem<systems::LcmTrajectoryReceiver>(
          "end_effector_force_target");
  auto end_effector_orientation_receiver =
      builder.AddSystem<systems::LcmOrientationTrajectoryReceiver>(
          "end_effector_orientation_target");
  auto franka_command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          lcm_channel_params.franka_input_channel, lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto osc_command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          lcm_channel_params.osc_channel, lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto franka_command_sender =
      builder.AddSystem<systems::RobotCommandSender>(*plant_);
  auto osc_command_sender =
      builder.AddSystem<systems::RobotCommandSender>(*plant_);
  auto end_effector_trajectory =
      builder.AddSystem<EndEffectorTrajectoryGenerator>();
  auto radio_to_vector = builder.AddSystem<systems::RadioToVector>();
  VectorXd neutral_position = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      controller_params.neutral_position.data(),
      controller_params.neutral_position.size());
  end_effector_trajectory->SetRemoteControlParameters(
      neutral_position, controller_params.x_scale, controller_params.y_scale,
      controller_params.z_scale);
  auto end_effector_orientation_trajectory =
      builder.AddSystem<EndEffectorOrientationGenerator>();
  end_effector_orientation_trajectory->SetTrackOrientation(
      controller_params.track_end_effector_orientation);
  auto end_effector_force_trajectory =
      builder.AddSystem<EndEffectorForceTrajectoryGenerator>();
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      *plant_, *plant_, plant_context_.get(), plant_context_.get(), false);
  if (controller_params.publish_debug_info) {
    auto osc_debug_pub =
        builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
            lcm_channel_params.osc_debug_channel, lcm,
            TriggerTypeSet({TriggerType::kForced})));
    builder.Connect(osc->get_output_port_osc_debug(),
                    osc_debug_pub->get_input_port());
  }

  auto end_effector_position_tracking_data =
      std::make_unique<TransTaskSpaceTrackingData>(
          "end_effector_target", controller_params.K_p_end_effector,
          controller_params.K_d_end_effector, controller_params.W_end_effector,
          *plant_, *plant_);
  end_effector_position_tracking_data->AddPointToTrack(
      controller_params.end_effector_name);
  const VectorXd& end_effector_acceleration_limits =
      controller_params.end_effector_acceleration * Vector3d::Ones();
  end_effector_position_tracking_data->SetCmdAccelerationBounds(
      -end_effector_acceleration_limits, end_effector_acceleration_limits);
  auto mid_link_position_tracking_data_for_rel =
      std::make_unique<JointSpaceTrackingData>(
          "panda_joint2_target", controller_params.K_p_mid_link,
          controller_params.K_d_mid_link, controller_params.W_mid_link, *plant_,
          *plant_);
  mid_link_position_tracking_data_for_rel->AddJointToTrack("panda_joint2",
                                                           "panda_joint2dot");

  auto end_effector_force_tracking_data =
      std::make_unique<ExternalForceTrackingData>(
          "end_effector_force", controller_params.W_ee_lambda, *plant_, *plant_,
          controller_params.end_effector_name, Vector3d::Zero());

  auto end_effector_orientation_tracking_data =
      std::make_unique<RotTaskSpaceTrackingData>(
          "end_effector_orientation_target",
          controller_params.K_p_end_effector_rot,
          controller_params.K_d_end_effector_rot,
          controller_params.W_end_effector_rot, *plant_, *plant_);
  end_effector_orientation_tracking_data->AddFrameToTrack(
      controller_params.end_effector_name);
  Eigen::VectorXd orientation_target = Eigen::VectorXd::Zero(4);
  orientation_target(0) = 1;
  osc->AddTrackingData(std::move(end_effector_position_tracking_data));
  osc->AddConstTrackingData(std::move(mid_link_position_tracking_data_for_rel),
                            1.6 * VectorXd::Ones(1));
  osc->AddTrackingData(std::move(end_effector_orientation_tracking_data));
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
        builder.AddSystem<systems::GravityCompensationRemover>(*plant_,
                                                               *plant_context_);
    builder.Connect(osc->get_output_port_osc_command(),
                    gravity_compensator->get_input_port());
    builder.Connect(gravity_compensator->get_output_port(),
                    franka_command_sender->get_input_port());
  } else {
    DRAKE_DEMAND(lcm_channels ==
                 "examples/franka/parameters/lcm_channels_simulation.yaml");
    builder.Connect(osc->get_output_port_osc_command(),
                    franka_command_sender->get_input_port(0));
  }

  builder.Connect(radio_to_vector->get_output_port(),
                  end_effector_trajectory->get_input_port_radio());
  builder.Connect(radio_to_vector->get_output_port(),
                  end_effector_orientation_trajectory->get_input_port_radio());
  builder.Connect(radio_to_vector->get_output_port(),
                  end_effector_force_trajectory->get_input_port_radio());
  builder.Connect(franka_command_sender->get_output_port(),
                  franka_command_pub->get_input_port());
  builder.Connect(osc_command_sender->get_output_port(),
                  osc_command_pub->get_input_port());
  builder.Connect(osc->get_output_port_osc_command(),
                  osc_command_sender->get_input_port(0));

  builder.Connect(state_receiver->get_output_port(0),
                  osc->get_input_port_robot_output());
//  builder.Connect(end_effector_trajectory_receiver->get_output_port(),
//                  end_effector_position_receiver->get_input_port_trajectory());
//  builder.Connect(end_effector_trajectory_receiver->get_output_port(),
//                  end_effector_force_receiver->get_input_port_trajectory());
//  builder.Connect(
//      end_effector_trajectory_receiver->get_output_port(),
//      end_effector_orientation_receiver->get_input_port_trajectory());
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

  // Publisher connections
  builder.ExportInput(state_receiver->get_input_port(), "lcmt_robot_output");
  builder.ExportInput(end_effector_position_receiver->get_input_port(),
                      "end_effector_position: lcmt_timestamped_trajectory");
  builder.ExportInput(end_effector_orientation_receiver->get_input_port(),
                      "end_effector_orientation: lcmt_timestamped_trajectory");
  builder.ExportInput(end_effector_force_receiver->get_input_port(),
                      "end_effector_force: lcmt_timestamped_trajectory");
  builder.ExportInput(radio_to_vector->get_input_port(), "raw_radio");
  builder.ExportOutput(osc->get_output_port_osc_command(), "robot_input");
  //  builder.ExportOutput(c3_trajectory_generator->get_output_port_object_trajectory(),
  //  "object_trajectory");
  //  builder.ExportOutput(c3_output_sender->get_output_port_c3_force(),
  //  "c3_forces");

  builder.BuildInto(this);
  this->set_name("FrankaOSCController");
  DrawAndSaveDiagramGraph(*this);
}

}  // namespace controllers
}  // namespace examples
}  // namespace dairlib
