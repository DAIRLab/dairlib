
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

using dairlib::solvers::LCSFactory;
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

DEFINE_string(controller_settings,
              "examples/franka/parameters/franka_c3_controller_params.yaml",
              "Controller settings such as channels. Attempting to minimize "
              "number of gflags");
DEFINE_string(lcm_channels,
              "examples/franka/parameters/lcm_channels_simulation.yaml",
              "Filepath containing lcm channels");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0");

  // load parameters
  drake::yaml::LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;
  FrankaC3ControllerParams controller_params =
      drake::yaml::LoadYamlFile<FrankaC3ControllerParams>(
          FLAGS_controller_settings);
  FrankaLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<FrankaLcmChannels>(FLAGS_lcm_channels);
  C3Options c3_options = drake::yaml::LoadYamlFile<C3Options>(
      controller_params.c3_options_file[controller_params.scene_index]);
  FrankaC3SceneParams scene_params =
      drake::yaml::LoadYamlFile<FrankaC3SceneParams>(
          controller_params.c3_scene_file[controller_params.scene_index]);
  drake::solvers::SolverOptions solver_options =
      drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
          FindResourceOrThrow(controller_params.osqp_settings_file))
          .GetAsSolverOptions(drake::solvers::OsqpSolver::id());

  DiagramBuilder<double> plant_builder;

  MultibodyPlant<double> plant_franka(0.0);
  Parser parser_franka(&plant_franka, nullptr);
  parser_franka.AddModelsFromUrl(scene_params.franka_model);
  drake::multibody::ModelInstanceIndex end_effector_index =
      parser_franka.AddModels(
          FindResourceOrThrow(scene_params.end_effector_model))[0];

  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant_franka.WeldFrames(plant_franka.world_frame(),
                          plant_franka.GetFrameByName("panda_link0"), X_WI);

  RigidTransform<double> T_EE_W =
      RigidTransform<double>(drake::math::RotationMatrix<double>(),
                             scene_params.tool_attachment_frame);
  plant_franka.WeldFrames(
      plant_franka.GetFrameByName("panda_link7"),
      plant_franka.GetFrameByName("plate", end_effector_index), T_EE_W);

  plant_franka.Finalize();
  auto franka_context = plant_franka.CreateDefaultContext();

  ///
  MultibodyPlant<double> plant_tray(0.0);
  Parser parser_tray(&plant_tray, nullptr);

  auto tray_index = parser_tray.AddModels(scene_params.object_models[0])[0];
  auto object_index = parser_tray.AddModels(scene_params.object_models[1])[0];
  plant_tray.Finalize();
  auto tray_context = plant_tray.CreateDefaultContext();

  ///
  auto [plant_for_lcs, scene_graph] =
      AddMultibodyPlantSceneGraph(&plant_builder, 0.0);
  Parser lcs_parser(&plant_for_lcs);
  lcs_parser.SetAutoRenaming(true);
  lcs_parser.AddModels(scene_params.end_effector_lcs_model);

  std::vector<drake::multibody::ModelInstanceIndex> environment_model_indices;
  environment_model_indices.resize(scene_params.environment_models.size());
  for (int i = 0; i < scene_params.environment_models.size(); ++i) {
    environment_model_indices[i] = lcs_parser.AddModels(
        FindResourceOrThrow(scene_params.environment_models[i]))[0];
    RigidTransform<double> T_E_W =
        RigidTransform<double>(drake::math::RollPitchYaw<double>(
                                   scene_params.environment_orientations[i]),
                               scene_params.environment_positions[i]);
    plant_for_lcs.WeldFrames(
        plant_for_lcs.world_frame(),
        plant_for_lcs.GetFrameByName("base", environment_model_indices[i]),
        T_E_W);
  }
  for (int i = 0; i < scene_params.object_models.size(); ++i) {
    lcs_parser.AddModels(scene_params.object_models[i]);
  }

  plant_for_lcs.WeldFrames(plant_for_lcs.world_frame(),
                           plant_for_lcs.GetFrameByName("base_link"), X_WI);
  plant_for_lcs.Finalize();
  std::unique_ptr<MultibodyPlant<drake::AutoDiffXd>> plant_for_lcs_autodiff =
      drake::systems::System<double>::ToAutoDiffXd(plant_for_lcs);

  auto plant_diagram = plant_builder.Build();
  std::unique_ptr<drake::systems::Context<double>> diagram_context =
      plant_diagram->CreateDefaultContext();
  auto& plant_for_lcs_context = plant_diagram->GetMutableSubsystemContext(
      plant_for_lcs, diagram_context.get());
  auto plate_context_ad = plant_for_lcs_autodiff->CreateDefaultContext();

  ///
  std::vector<drake::geometry::GeometryId> end_effector_contact_points =
      plant_for_lcs.GetCollisionGeometriesForBody(
          plant_for_lcs.GetBodyByName("plate"));
  for (int i = 0; i < environment_model_indices.size(); ++i) {
    std::vector<drake::geometry::GeometryId>
        environment_support_contact_points =
            plant_for_lcs.GetCollisionGeometriesForBody(
                plant_for_lcs.GetBodyByName("base",
                                            environment_model_indices[i]));
    end_effector_contact_points.insert(
        end_effector_contact_points.end(),
        environment_support_contact_points.begin(),
        environment_support_contact_points.end());
  }
  std::vector<drake::geometry::GeometryId> tray_geoms =
      plant_for_lcs.GetCollisionGeometriesForBody(
          plant_for_lcs.GetBodyByName("tray"));
  std::vector<drake::geometry::GeometryId> object_geoms =
      plant_for_lcs.GetCollisionGeometriesForBody(
          plant_for_lcs.GetBodyByName("base"));

  std::vector<SortedPair<GeometryId>> contact_pairs;
  for (auto geom_id : end_effector_contact_points) {
    contact_pairs.emplace_back(geom_id, tray_geoms[0]);
  }
  for (auto geom_id : object_geoms) {
    contact_pairs.emplace_back(tray_geoms[0], geom_id);
  }

  DiagramBuilder<double> builder;

  auto tray_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.tray_state_channel, &lcm));
  auto object_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.object_state_channel, &lcm));
  auto franka_state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_franka);
  auto tray_state_receiver =
      builder.AddSystem<systems::ObjectStateReceiver>(plant_tray, tray_index);
  auto object_state_receiver =
      builder.AddSystem<systems::ObjectStateReceiver>(plant_tray, object_index);
  auto reduced_order_model_receiver =
      builder.AddSystem<systems::FrankaKinematics>(
          plant_franka, franka_context.get(), plant_tray, tray_context.get(),
          scene_params.end_effector_name, "tray",
          controller_params.include_end_effector_orientation);
  auto actor_trajectory_sender = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.c3_actor_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));

  auto object_trajectory_sender = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.c3_object_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));

  auto c3_output_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_output>(
          lcm_channel_params.c3_debug_output_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto c3_target_state_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_state>(
          lcm_channel_params.c3_target_state_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto c3_actual_state_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_state>(
          lcm_channel_params.c3_actual_state_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto c3_forces_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_forces>(
          lcm_channel_params.c3_force_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto radio_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
          lcm_channel_params.radio_channel, &lcm));
  auto radio_to_vector = builder.AddSystem<systems::RadioToVector>();

  auto plate_balancing_target =
      builder.AddSystem<systems::PlateBalancingTargetGenerator>(
          plant_tray, scene_params.end_effector_thickness,
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
      plant_for_lcs, plant_for_lcs_context, *plant_for_lcs_autodiff,
      *plate_context_ad, contact_pairs, c3_options);
  auto controller =
      builder.AddSystem<systems::C3Controller>(plant_for_lcs, c3_options);
  auto c3_trajectory_generator =
      builder.AddSystem<systems::C3TrajectoryGenerator>(plant_for_lcs,
                                                        c3_options);
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
  controller->SetOsqpSolverOptions(solver_options);

  builder.Connect(*radio_sub, *radio_to_vector);
  builder.Connect(franka_state_receiver->get_output_port(),
                  reduced_order_model_receiver->get_input_port_franka_state());
  builder.Connect(target_state_mux->get_output_port(),
                  controller->get_input_port_target());
  builder.Connect(lcs_factory->get_output_port_lcs(),
                  controller->get_input_port_lcs());
  builder.Connect(tray_state_sub->get_output_port(),
                  tray_state_receiver->get_input_port());
  builder.Connect(tray_state_receiver->get_output_port(),
                  reduced_order_model_receiver->get_input_port_object_state());
  builder.Connect(tray_state_receiver->get_output_port(),
                  plate_balancing_target->get_input_port_tray_state());
  builder.Connect(reduced_order_model_receiver->get_output_port(),
                  controller->get_input_port_lcs_state());
  builder.Connect(reduced_order_model_receiver->get_output_port(),
                  lcs_factory->get_input_port_lcs_state());
  builder.Connect(radio_to_vector->get_output_port(),
                  plate_balancing_target->get_input_port_radio());
  builder.Connect(controller->get_output_port_c3_solution(),
                  c3_trajectory_generator->get_input_port_c3_solution());
  builder.Connect(lcs_factory->get_output_port_lcs_contact_jacobian(),
                  c3_output_sender->get_input_port_lcs_contact_info());
  builder.Connect(c3_trajectory_generator->get_output_port_actor_trajectory(),
                  actor_trajectory_sender->get_input_port());
  builder.Connect(c3_trajectory_generator->get_output_port_object_trajectory(),
                  object_trajectory_sender->get_input_port());
  builder.Connect(target_state_mux->get_output_port(),
                  c3_state_sender->get_input_port_target_state());
  builder.Connect(reduced_order_model_receiver->get_output_port_lcs_state(),
                  c3_state_sender->get_input_port_actual_state());
  builder.Connect(c3_state_sender->get_output_port_target_c3_state(),
                  c3_target_state_publisher->get_input_port());
  builder.Connect(c3_state_sender->get_output_port_actual_c3_state(),
                  c3_actual_state_publisher->get_input_port());
  builder.Connect(controller->get_output_port_c3_solution(),
                  c3_output_sender->get_input_port_c3_solution());
  builder.Connect(controller->get_output_port_c3_intermediates(),
                  c3_output_sender->get_input_port_c3_intermediates());
  builder.Connect(c3_output_sender->get_output_port_c3_debug(),
                  c3_output_publisher->get_input_port());
  builder.Connect(c3_output_sender->get_output_port_c3_force(),
                  c3_forces_publisher->get_input_port());
  //  builder.Connect(c3_output_sender->get_output_port_next_c3_input(),
  //                  lcs_factory->get_input_port_lcs_input());

  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("franka_c3_controller"));
  plant_diagram->set_name(("franka_c3_plant"));
  //  DrawAndSaveDiagramGraph(*plant_diagram);

  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, std::move(owned_diagram), franka_state_receiver,
      lcm_channel_params.franka_state_channel, true);
  DrawAndSaveDiagramGraph(*loop.get_diagram());
  //  auto& controller_context = loop.get_diagram()->GetMutableSubsystemContext(
  //      *controller, &loop.get_diagram_mutable_context());
  //  controller->get_input_port_target().FixValue(&controller_context, x_des);
  LcmHandleSubscriptionsUntil(
      &lcm, [&]() { return tray_state_sub->GetInternalMessageCount() > 1; });
  loop.Simulate();
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }