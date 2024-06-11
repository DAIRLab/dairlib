//
// Created by yangwill on 2/19/24.
//

#include "examples/franka/diagrams/franka_c3_controller_diagram.h"

#include <dairlib/lcmt_radio_out.hpp>
#include <drake/common/find_resource.h>
#include <drake/common/yaml/yaml_io.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake/systems/lcm/lcm_subscriber_system.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/multiplexer.h>
#include <gflags/gflags.h>
#include <drake/systems/primitives/pass_through.h>
#include <drake/systems/primitives/discrete_time_delay.h>

#include "common/eigen_utils.h"
#include "examples/franka/parameters/franka_c3_controller_params.h"
#include "examples/franka/parameters/franka_c3_scene_params.h"
#include "examples/franka/parameters/franka_lcm_channels.h"
#include "examples/franka/systems/c3_state_sender.h"
#include "examples/franka/systems/c3_trajectory_generator.h"
#include "examples/franka/systems/franka_kinematics.h"
#include "examples/franka/systems/plate_balancing_target.h"
#include "multibody/multibody_utils.h"
#include "solvers/lcs_factory.h"
#include "systems/controllers/c3/lcs_factory_system.h"
#include "systems/controllers/c3/c3_controller.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/primitives/radio_parser.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"
#include "systems/trajectory_optimization/c3_output_systems.h"

namespace dairlib {

namespace examples {
namespace controllers {

using dairlib::solvers::LCSFactory;
using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::math::RigidTransform;
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

FrankaC3ControllerDiagram::FrankaC3ControllerDiagram(
    const string& controller_settings, const C3Options c3_options, const string& lcm_channels,
    drake::lcm::DrakeLcm* lcm, bool publish_c3_debug) {
  this->set_name("FrankaC3Controller");
  DiagramBuilder<double> builder;

  FrankaC3ControllerParams controller_params =
      drake::yaml::LoadYamlFile<FrankaC3ControllerParams>(controller_settings);
  FrankaLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<FrankaLcmChannels>(lcm_channels);
  FrankaC3SceneParams scene_params =
      drake::yaml::LoadYamlFile<FrankaC3SceneParams>(
          controller_params.c3_scene_file[controller_params.scene_index]);
//  C3Options c3_options = drake::yaml::LoadYamlFile<C3Options>(
//      controller_params.c3_options_file[controller_params.scene_index]);
  drake::solvers::SolverOptions solver_options =
      drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
          FindResourceOrThrow(controller_params.osqp_settings_file))
          .GetAsSolverOptions(drake::solvers::OsqpSolver::id());

  plant_franka_ = new drake::multibody::MultibodyPlant<double>(0.0);
  Parser parser_franka(plant_franka_, nullptr);
  parser_franka.AddModelsFromUrl(scene_params.franka_model);
  drake::multibody::ModelInstanceIndex end_effector_index =
      parser_franka.AddModels(
          FindResourceOrThrow(scene_params.end_effector_model))[0];

  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant_franka_->WeldFrames(plant_franka_->world_frame(),
                            plant_franka_->GetFrameByName("panda_link0"), X_WI);

  RigidTransform<double> T_EE_W =
      RigidTransform<double>(drake::math::RotationMatrix<double>(),
                             scene_params.tool_attachment_frame);
  plant_franka_->WeldFrames(
      plant_franka_->GetFrameByName("panda_link7"),
      plant_franka_->GetFrameByName("plate", end_effector_index), T_EE_W);

  plant_franka_->Finalize();
  plant_franka_context_ = plant_franka_->CreateDefaultContext();

  ///
  plant_tray_ = new drake::multibody::MultibodyPlant<double>(0.0);
  Parser parser_tray(plant_tray_, nullptr);
  parser_tray.AddModels(scene_params.object_models[0]);
  plant_tray_->Finalize();
  plant_tray_context_ = plant_tray_->CreateDefaultContext();

  drake::planning::RobotDiagramBuilder<double> lcs_diagram_builder;
  lcs_diagram_builder.parser().SetAutoRenaming(true);
  lcs_diagram_builder.parser().AddModels(scene_params.end_effector_lcs_model);

  std::vector<drake::multibody::ModelInstanceIndex> environment_model_indices;
  environment_model_indices.resize(scene_params.environment_models.size());
  for (int i = 0; i < scene_params.environment_models.size(); ++i) {
    environment_model_indices[i] = lcs_diagram_builder.parser().AddModels(
        FindResourceOrThrow(scene_params.environment_models[i]))[0];
    RigidTransform<double> T_E_W =
        RigidTransform<double>(drake::math::RollPitchYaw<double>(
                                   scene_params.environment_orientations[i]),
                               scene_params.environment_positions[i]);
    lcs_diagram_builder.plant().WeldFrames(
        lcs_diagram_builder.plant().world_frame(),
        lcs_diagram_builder.plant().GetFrameByName("base", environment_model_indices[i]),
        T_E_W);
  }
  for (int i = 0; i < scene_params.object_models.size(); ++i) {
    lcs_diagram_builder.parser().AddModels(scene_params.object_models[i]);
  }
  lcs_diagram_builder.plant().WeldFrames(
      lcs_diagram_builder.plant().world_frame(),
      lcs_diagram_builder.plant().GetFrameByName("base_link"), X_WI);
  lcs_diagram_builder.plant().Finalize();
  robot_diagram_for_lcs_ = lcs_diagram_builder.Build();
  plant_for_lcs_autodiff_ = drake::systems::System<double>::ToAutoDiffXd(
      robot_diagram_for_lcs_->plant());

  robot_diagram_root_context_ = robot_diagram_for_lcs_->CreateDefaultContext();
  plant_for_lcs_autodiff_context_ =
      plant_for_lcs_autodiff_->CreateDefaultContext();

  ///
  std::vector<drake::geometry::GeometryId> end_effector_contact_points =
      robot_diagram_for_lcs_->plant().GetCollisionGeometriesForBody(
          robot_diagram_for_lcs_->plant().GetBodyByName("plate"));
  for (int i = 0; i < environment_model_indices.size(); ++i) {
    std::vector<drake::geometry::GeometryId>
        environment_support_contact_points =
        robot_diagram_for_lcs_->plant().GetCollisionGeometriesForBody(
            robot_diagram_for_lcs_->plant().GetBodyByName("base",
                                        environment_model_indices[i]));
    end_effector_contact_points.insert(
        end_effector_contact_points.end(),
        environment_support_contact_points.begin(),
        environment_support_contact_points.end());
  }
  std::vector<drake::geometry::GeometryId> tray_geoms =
      robot_diagram_for_lcs_->plant().GetCollisionGeometriesForBody(
          robot_diagram_for_lcs_->plant().GetBodyByName("tray"));
  std::unordered_map<std::string, std::vector<drake::geometry::GeometryId>>
      contact_geoms;
  contact_geoms["PLATE"] = end_effector_contact_points;
  contact_geoms["TRAY"] = tray_geoms;

  std::vector<SortedPair<GeometryId>> contact_pairs;
  for (auto geom_id : contact_geoms["PLATE"]) {
    contact_pairs.emplace_back(geom_id, contact_geoms["TRAY"][0]);
  }

  auto franka_state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(*plant_franka_);
  auto tray_state_receiver =
      builder.AddSystem<systems::ObjectStateReceiver>(*plant_tray_);
  auto reduced_order_model_receiver =
      builder.AddSystem<systems::FrankaKinematics>(
          *plant_franka_, plant_franka_context_.get(), *plant_tray_,
          plant_tray_context_.get(), scene_params.end_effector_name,
          "tray", controller_params.include_end_effector_orientation);

  auto plate_balancing_target =
      builder.AddSystem<systems::PlateBalancingTargetGenerator>(
          *plant_tray_, scene_params.end_effector_thickness,
          controller_params.near_target_threshold);
  plate_balancing_target->SetRemoteControlParameters(
      controller_params.first_target[controller_params.scene_index],
      controller_params.second_target[controller_params.scene_index],
      controller_params.third_target[controller_params.scene_index],
      controller_params.x_scale, controller_params.y_scale,
      controller_params.z_scale);
  std::vector<int> input_sizes = {3, 7, 3, 6};
  auto target_state_mux =
      builder.AddSystem<drake::systems::Multiplexer>(input_sizes);
  auto end_effector_zero_velocity_source =
      builder.AddSystem<drake::systems::ConstantVectorSource>(
          VectorXd::Zero(3));
  auto tray_zero_velocity_source =
      builder.AddSystem<drake::systems::ConstantVectorSource>(
          VectorXd::Zero(6));
  builder.Connect(plate_balancing_target->get_output_port_end_effector_target(),
                  target_state_mux->get_input_port(0));
  builder.Connect(plate_balancing_target->get_output_port_tray_target(),
                  target_state_mux->get_input_port(1));
  builder.Connect(end_effector_zero_velocity_source->get_output_port(),
                  target_state_mux->get_input_port(2));
  builder.Connect(tray_zero_velocity_source->get_output_port(),
                  target_state_mux->get_input_port(3));
  auto lcs_factory = builder.AddSystem<systems::LCSFactorySystem>(
      robot_diagram_for_lcs_->plant(),
      robot_diagram_for_lcs_->mutable_plant_context(
          robot_diagram_root_context_.get()),
      *plant_for_lcs_autodiff_, *plant_for_lcs_autodiff_context_, contact_pairs,
      c3_options);
  auto controller = builder.AddSystem<systems::C3Controller>(
      robot_diagram_for_lcs_->plant(), c3_options);
  auto c3_trajectory_generator =
      builder.AddSystem<systems::C3TrajectoryGenerator>(
          robot_diagram_for_lcs_->plant(), c3_options);
//  auto placeholder_trajectory = lcmt_timestamped_saved_traj();
  auto placeholder_solution = C3Output::C3Solution();
  placeholder_solution.x_sol_ = Eigen::MatrixXf::Zero(c3_options.g_x.size(), c3_options.N);
  placeholder_solution.lambda_sol_ = Eigen::MatrixXf::Zero(c3_options.g_lambda.size(), c3_options.N);
  placeholder_solution.u_sol_ = Eigen::MatrixXf::Zero(c3_options.g_u.size(), c3_options.N);
  placeholder_solution.time_vector_ = Eigen::VectorXf::LinSpaced(c3_options.N, 0, 1);
  auto discrete_time_delay = builder.AddSystem<drake::systems::DiscreteTimeDelay>(1 / c3_options.publish_frequency,
                                                                                  2,
                                                                                  drake::Value(placeholder_solution));
  std::vector<std::string> state_names = {
      "end_effector_x",  "end_effector_y", "end_effector_z",  "tray_qw",
      "tray_qx",         "tray_qy",        "tray_qz",         "tray_x",
      "tray_y",          "tray_z",         "end_effector_vx", "end_effector_vy",
      "end_effector_vz", "tray_wx",        "tray_wy",         "tray_wz",
      "tray_vz",         "tray_vz",        "tray_vz",
  };
  auto c3_state_sender =
      builder.AddSystem<systems::C3StateSender>(3 + 7 + 3 + 6, state_names);
  c3_trajectory_generator->SetPublishEndEffectorOrientation(
      controller_params.include_end_effector_orientation);
  auto c3_output_sender = builder.AddSystem<systems::C3OutputSender>();
  auto passthrough = builder.AddSystem<drake::systems::PassThrough<double>>(18);

  controller->SetOsqpSolverOptions(solver_options);

  // publisher connections
  DRAKE_DEMAND(c3_options.publish_frequency > 0);
  builder.Connect(controller->get_output_port_c3_solution(),
                  discrete_time_delay->get_input_port());


  builder.Connect(franka_state_receiver->get_output_port(),
                  reduced_order_model_receiver->get_input_port_franka_state());
  builder.Connect(target_state_mux->get_output_port(),
                  controller->get_input_port_target());
  builder.Connect(lcs_factory->get_output_port_lcs(),
                  controller->get_input_port_lcs());
  builder.Connect(tray_state_receiver->get_output_port(),
                  reduced_order_model_receiver->get_input_port_object_state());
  builder.Connect(tray_state_receiver->get_output_port(),
                  plate_balancing_target->get_input_port_tray_state());
  builder.Connect(reduced_order_model_receiver->get_output_port(),
                  controller->get_input_port_lcs_state());
  builder.Connect(reduced_order_model_receiver->get_output_port(),
                  lcs_factory->get_input_port_lcs_state());
  builder.Connect(passthrough->get_output_port(),
                  plate_balancing_target->get_input_port_radio());
  builder.Connect(discrete_time_delay->get_output_port(),
                  c3_trajectory_generator->get_input_port_c3_solution());
  builder.Connect(lcs_factory->get_output_port_lcs_contact_jacobian(),
                  c3_output_sender->get_input_port_lcs_contact_info());

  builder.Connect(target_state_mux->get_output_port(),
                  c3_state_sender->get_input_port_target_state());
  builder.Connect(reduced_order_model_receiver->get_output_port_lcs_state(),
                  c3_state_sender->get_input_port_actual_state());

  builder.Connect(discrete_time_delay->get_output_port(),
                  c3_output_sender->get_input_port_c3_solution());
  builder.Connect(controller->get_output_port_c3_intermediates(),
                  c3_output_sender->get_input_port_c3_intermediates());


  if (publish_c3_debug){
    auto actor_trajectory_sender = builder.AddSystem(
        LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
            lcm_channel_params.c3_actor_channel, lcm,
            TriggerTypeSet({TriggerType::kForced})));
    auto object_trajectory_sender = builder.AddSystem(
        LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
            lcm_channel_params.c3_object_channel, lcm,
            TriggerTypeSet({TriggerType::kForced})));
    auto c3_output_publisher =
        builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_output>(
            lcm_channel_params.c3_debug_output_channel, lcm,
            TriggerTypeSet({TriggerType::kForced})));
    auto c3_target_state_publisher =
        builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_state>(
            lcm_channel_params.c3_target_state_channel, lcm,
            TriggerTypeSet({TriggerType::kForced})));
    auto c3_actual_state_publisher =
        builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_state>(
            lcm_channel_params.c3_actual_state_channel, lcm,
            TriggerTypeSet({TriggerType::kForced})));
    auto c3_forces_publisher =
        builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_forces>(
            lcm_channel_params.c3_force_channel, lcm,
            TriggerTypeSet({TriggerType::kForced})));
    builder.Connect(c3_trajectory_generator->get_output_port_actor_trajectory(),
                    actor_trajectory_sender->get_input_port());
    builder.Connect(c3_trajectory_generator->get_output_port_object_trajectory(),
                    object_trajectory_sender->get_input_port());
    builder.Connect(c3_state_sender->get_output_port_target_c3_state(),
                    c3_target_state_publisher->get_input_port());
    builder.Connect(c3_state_sender->get_output_port_actual_c3_state(),
                    c3_actual_state_publisher->get_input_port());
    builder.Connect(c3_output_sender->get_output_port_c3_debug(),
                    c3_output_publisher->get_input_port());
    builder.Connect(c3_output_sender->get_output_port_c3_force(),
                    c3_forces_publisher->get_input_port());
  }



  // Publisher connections
  franka_state_port_ = builder.ExportInput(franka_state_receiver->get_input_port(),
                      "franka_state: lcmt_robot_output");
  tray_state_port_ = builder.ExportInput(tray_state_receiver->get_input_port(),
                      "tray_state: lcmt_object_state");
  radio_port_ = builder.ExportInput(passthrough->get_input_port(), "raw_radio");
  mpc_plan_port_ = builder.ExportOutput(
      c3_trajectory_generator->get_output_port_actor_trajectory(),
      "actor_trajectory");

  builder.BuildInto(this);
  this->set_name("FrankaC3Controller");
}

}  // namespace controllers
}  // namespace examples
}  // namespace dairlib
