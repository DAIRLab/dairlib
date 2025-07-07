#include <c3/core/c3.h>
#include <c3/core/solver_options_io.h>
#include <c3/systems/c3_controller.h>
#include <c3/systems/lcmt_generators/c3_output_generator.h>
#include <c3/systems/lcmt_generators/c3_trajectory_generator.h>
#include <c3/systems/lcmt_generators/contact_force_generator.h>
#include <c3/systems/lcs_factory_system.h>
#include <dairlib/lcmt_radio_out.hpp>
#include <drake/common/find_resource.h>
#include <drake/common/text_logging.h>
#include <drake/common/yaml/yaml_io.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake/systems/lcm/lcm_subscriber_system.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/multiplexer.h>
#include <gflags/gflags.h>

#include "examples/plate-balancing/parameters/c3_scene_config.h"
#include "examples/plate-balancing/parameters/lcm_channel_config.h"
#include "examples/plate-balancing/parameters/plate_balancing_c3_controller_options.h"
#include "examples/plate-balancing/parameters/plate_balancing_config.h"
#include "examples/plate-balancing/parameters/plate_balancing_target_config.h"
#include "examples/plate-balancing/systems/franka_kinematics.h"
#include "examples/plate-balancing/systems/plate_balancing_target.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/lcmt_generators/robot_state_generator.h"
#include "systems/primitives/radio_parser.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

DEFINE_string(plate_balancing_config,
              "examples/plate-balancing/config/plate_balancing_config.yaml",
              "Controller settings such as channels. Attempting to minimize "
              "number of gflags");
DEFINE_bool(simulation, true, "Running in simulation or hardware");

using c3::C3;
using c3::ConstraintVariable;
using c3::SolverOptionsFromYaml;
using c3::systems::C3Controller;
using c3::systems::LCSFactorySystem;
using c3::systems::lcmt_generators::C3OutputGenerator;
using c3::systems::lcmt_generators::C3TrajectoryGenerator;
using c3::systems::lcmt_generators::C3TrajectoryGeneratorConfig;
using c3::systems::lcmt_generators::ContactForceGenerator;

using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::math::RigidTransform;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::ConstantVectorSource;
using drake::systems::DiagramBuilder;
using drake::systems::Multiplexer;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace dairlib {

using multibody::MakeNameToPositionsMap;
using multibody::MakeNameToVelocitiesMap;
using systems::LcmDrivenLoop;
using systems::ObjectStateReceiver;
using systems::RadioToVector;
using systems::RobotOutputReceiver;
using systems::lcmt_generators::RobotStateGenerator;

namespace examples {
namespace plate_balancing {

// Function to add the C3 controller to the diagram builder
C3Controller* AddC3ControllerToBuilder(
    DiagramBuilder<double>& builder, MultibodyPlant<double>& plant_for_lcs,
    PlateBalancingC3ControllerOptions& controller_options,
    drake::solvers::SolverOptions& solver_options) {
  // Get state and input dimensions from the plant.
  int n_x = plant_for_lcs.num_positions() + plant_for_lcs.num_velocities();
  int n_u = plant_for_lcs.num_actuators();

  // Create cost matrices based on C3 options.
  auto cost = C3::CreateCostMatricesFromC3Options(
      controller_options.c3_options, controller_options.lcs_factory_options.N);

  // Add the C3 controller to the builder.
  auto controller =
      builder.AddSystem<C3Controller>(plant_for_lcs, cost, controller_options);

  // Add workspace limits as linear constraints.
  if (controller_options.workspace_limits.size() > 0) {
    // Number of constraints to be added
    int n_c = controller_options.workspace_limits.size();
    drake::log()->info("Adding {} Workspace constraints", n_c);
    Eigen::MatrixXd A = MatrixXd::Zero(n_c, n_x);
    Eigen::VectorXd lb = VectorXd::Zero(n_c);
    Eigen::VectorXd ub = VectorXd::Zero(n_c);
    for (int i = 0; i < n_c; ++i) {
      A.block(i, 0, 1, 3) =
          controller_options.workspace_limits[i].segment(0, 3).transpose();
      lb[i] = controller_options.workspace_limits[i][3];
      ub[i] = controller_options.workspace_limits[i][4];
    }
    controller->AddLinearConstraint(A, lb, ub, c3::ConstraintVariable::STATE);
  }
  // Add horizontal and vertical input limits as linear constraints
  if (controller_options.u_horizontal_limits.size() == 2 &&
      controller_options.u_vertical_limits.size() == 2) {
    drake::log()->info(
        "Adding horizontal and vertical input limits to C3 controller. "
        "Horizontal limits (x, y): [{}, {}], Vertical limits (z): [{}, {}]",
        controller_options.u_horizontal_limits[0],
        controller_options.u_horizontal_limits[1],
        controller_options.u_vertical_limits[0],
        controller_options.u_vertical_limits[1]);
    Eigen::MatrixXd A = MatrixXd::Identity(3, n_u);
    Eigen::Vector3d lb = {controller_options.u_horizontal_limits[0],
                          controller_options.u_horizontal_limits[0],
                          controller_options.u_vertical_limits[0]};
    Eigen::Vector3d ub = {controller_options.u_horizontal_limits[1],
                          controller_options.u_horizontal_limits[1],
                          controller_options.u_vertical_limits[1]};
    controller->AddLinearConstraint(A, lb, ub, c3::ConstraintVariable::INPUT);
  }
  controller->SetSolverOptions(solver_options);
  return controller;
}

// Main function for the plate balancing example
int DoMain(std::string plate_balancing_config, bool is_simulation) {
  drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0");

  // Load parameters from YAML files
  drake::yaml::LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;
  PlateBalancingConfig main_config =
      drake::yaml::LoadYamlFile<PlateBalancingConfig>(plate_balancing_config);
  PlateBalancingTargetConfig target_config =
      drake::yaml::LoadYamlFile<PlateBalancingTargetConfig>(
          main_config.plate_balancing_target_config_file);
  LcmChannelConfig lcm_channel_params =
      drake::yaml::LoadYamlFile<LcmChannelConfig>(
          is_simulation ? main_config.lcm_simulation_settings_file
                        : main_config.lcm_hardware_settings_file);
  PlateBalancingC3ControllerOptions controller_options =
      drake::yaml::LoadYamlFile<PlateBalancingC3ControllerOptions>(
          main_config.get_c3_controller_option_file());
  C3SceneConfig scene_params = drake::yaml::LoadYamlFile<C3SceneConfig>(
      main_config.get_c3_scene_config_file());
  drake::solvers::SolverOptions solver_options =
      drake::yaml::LoadYamlFile<SolverOptionsFromYaml>(
          FindResourceOrThrow(main_config.c3_osqp_settings_file))
          .GetAsSolverOptions(drake::solvers::OsqpSolver::id());

  // Create MultibodyPlant for Franka robot
  MultibodyPlant<double> plant_franka(0.0);
  Parser parser_franka(&plant_franka, nullptr);
  parser_franka.AddModelsFromUrl(scene_params.franka_model);
  drake::multibody::ModelInstanceIndex end_effector_index =
      parser_franka.AddModels(
          FindResourceOrThrow(scene_params.end_effector_model))[0];

  // Weld Franka to the world frame
  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant_franka.WeldFrames(plant_franka.world_frame(),
                          plant_franka.GetFrameByName("panda_link0"), X_WI);

  // Weld end effector to Franka
  RigidTransform<double> T_EE_W =
      RigidTransform<double>(drake::math::RotationMatrix<double>(),
                             scene_params.tool_attachment_frame);
  plant_franka.WeldFrames(
      plant_franka.GetFrameByName("panda_link7"),
      plant_franka.GetFrameByName("plate", end_effector_index), T_EE_W);

  plant_franka.Finalize();
  auto franka_context = plant_franka.CreateDefaultContext();

  /// Create MultibodyPlant for the tray
  MultibodyPlant<double> plant_tray(0.0);
  Parser parser_tray(&plant_tray, nullptr);
  parser_tray.AddModels(scene_params.object_models[0]);
  plant_tray.Finalize();
  auto tray_context = plant_tray.CreateDefaultContext();

  /// Create DiagramBuilder for the plant
  DiagramBuilder<double> plant_builder;
  auto [plant_for_lcs, scene_graph] =
      AddMultibodyPlantSceneGraph(&plant_builder, 0.0);
  Parser lcs_parser(&plant_for_lcs);
  lcs_parser.SetAutoRenaming(true);
  lcs_parser.AddModels(scene_params.end_effector_lcs_model);

  // Add environment models to the plant
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
  // Add object models to the plant
  for (int i = 0; i < scene_params.object_models.size(); ++i) {
    lcs_parser.AddModels(scene_params.object_models[i]);
  }

  // Weld the plant to the world frame
  plant_for_lcs.WeldFrames(plant_for_lcs.world_frame(),
                           plant_for_lcs.GetFrameByName("base_link"), X_WI);
  plant_for_lcs.Finalize();

  // Convert the plant to AutoDiffXd for contact force calculations
  std::unique_ptr<MultibodyPlant<drake::AutoDiffXd>> plant_for_lcs_autodiff =
      drake::systems::System<double>::ToAutoDiffXd(plant_for_lcs);

  auto plant_diagram = plant_builder.Build();
  std::unique_ptr<drake::systems::Context<double>> diagram_context =
      plant_diagram->CreateDefaultContext();
  auto& plant_for_lcs_context = plant_diagram->GetMutableSubsystemContext(
      plant_for_lcs, diagram_context.get());
  auto plate_context_ad = plant_for_lcs_autodiff->CreateDefaultContext();

  /// Define contact geometries
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
  std::unordered_map<std::string, std::vector<drake::geometry::GeometryId>>
      contact_geoms;
  contact_geoms["PLATE"] = end_effector_contact_points;
  contact_geoms["TRAY"] = tray_geoms;

  // Define contact pairs for the LCSFactorySystem
  std::vector<SortedPair<GeometryId>> contact_pairs;
  for (auto geom_id : contact_geoms["PLATE"]) {
    contact_pairs.emplace_back(geom_id, contact_geoms["TRAY"][0]);
  }

  // Build the main diagram
  DiagramBuilder<double> builder;

  // Add LCM subscribers for tray and radio state
  auto tray_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.tray_state_channel, &lcm));
  auto franka_state_receiver =
      builder.AddSystem<RobotOutputReceiver>(plant_franka);
  auto tray_state_receiver = builder.AddSystem<ObjectStateReceiver>(plant_tray);
  auto reduced_order_model_receiver =
      builder.AddSystem<systems::FrankaKinematics>(
          plant_franka, franka_context.get(), plant_tray, tray_context.get(),
          scene_params.end_effector_name, "tray",
          main_config.include_end_effector_orientation);

  auto radio_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
          lcm_channel_params.radio_channel, &lcm));
  auto radio_to_vector = builder.AddSystem<RadioToVector>();

  // Add plate balancing target generator
  auto plate_balancing_target =
      builder.AddSystem<systems::PlateBalancingTargetGenerator>(
          plant_tray, scene_params.end_effector_thickness,
          target_config.near_target_threshold);
  plate_balancing_target->SetRemoteControlParameters(
      target_config.first_target[main_config.scene_index],
      target_config.second_target[main_config.scene_index],
      target_config.third_target[main_config.scene_index],
      target_config.x_scale, target_config.y_scale, target_config.z_scale);
  std::vector<int> input_sizes = {3, 7, 3, 6};
  auto target_state_mux =
      builder.AddSystem<drake::systems::Multiplexer>(input_sizes);
  auto end_effector_zero_velocity_source =
      builder.AddSystem<drake::systems::ConstantVectorSource>(
          VectorXd::Zero(3));
  builder.Connect(plate_balancing_target->get_output_port_end_effector_target(),
                  target_state_mux->get_input_port(0));
  builder.Connect(plate_balancing_target->get_output_port_tray_target(),
                  target_state_mux->get_input_port(1));
  builder.Connect(end_effector_zero_velocity_source->get_output_port(),
                  target_state_mux->get_input_port(2));
  builder.Connect(
      plate_balancing_target->get_output_port_tray_velocity_target(),
      target_state_mux->get_input_port(3));
  // Add LCS factory and C3 controller
  auto lcs_factory = builder.AddSystem<LCSFactorySystem>(
      plant_for_lcs, plant_for_lcs_context, *plant_for_lcs_autodiff,
      *plate_context_ad, contact_pairs, controller_options.lcs_factory_options);

  auto controller = AddC3ControllerToBuilder(
      builder, plant_for_lcs, controller_options, solver_options);

  // Connect systems
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
  builder.Connect(reduced_order_model_receiver->get_output_port_lcs_state(),
                  controller->get_input_port_lcs_state());
  builder.Connect(reduced_order_model_receiver->get_output_port_lcs_state(),
                  lcs_factory->get_input_port_lcs_state());
  builder.Connect(reduced_order_model_receiver->get_output_port_lcs_input(),
                  lcs_factory->get_input_port_lcs_input());
  builder.Connect(radio_to_vector->get_output_port(),
                  plate_balancing_target->get_input_port_radio());

  // Add C3 output and contact force publishers
  C3OutputGenerator::AddLcmPublisherToBuilder(
      builder, controller->get_output_port_c3_solution(),
      controller->get_output_port_c3_intermediates(),
      lcm_channel_params.c3_debug_output_channel, &lcm,
      TriggerTypeSet({TriggerType::kForced}));
  ContactForceGenerator::AddLcmPublisherToBuilder(
      builder, controller->get_output_port_c3_solution(),
      lcs_factory->get_output_port_lcs_contact_description(),
      lcm_channel_params.c3_force_channel, &lcm,
      TriggerTypeSet({TriggerType::kForced}));
  C3TrajectoryGeneratorConfig actor_config =
      drake::yaml::LoadYamlFile<C3TrajectoryGeneratorConfig>(
          main_config.c3_actor_trajectory_generator_config);
  C3TrajectoryGenerator::AddLcmPublisherToBuilder(
      builder, actor_config, controller->get_output_port_c3_solution(),
      lcm_channel_params.c3_actor_channel, &lcm,
      TriggerTypeSet({TriggerType::kForced}));
  C3TrajectoryGeneratorConfig object_config =
      drake::yaml::LoadYamlFile<C3TrajectoryGeneratorConfig>(
          main_config.c3_object_trajectory_generator_config);
  C3TrajectoryGenerator::AddLcmPublisherToBuilder(
      builder, object_config, controller->get_output_port_c3_solution(),
      lcm_channel_params.c3_object_channel, &lcm,
      TriggerTypeSet({TriggerType::kForced}));
  RobotStateGenerator::AddLcmPublisherToBuilder(
      builder, plant_for_lcs.GetStateNames(), false,
      target_state_mux->get_output_port(),
      lcm_channel_params.c3_target_state_channel, &lcm,
      TriggerTypeSet({TriggerType::kForced}));
  RobotStateGenerator::AddLcmPublisherToBuilder(
      builder, plant_for_lcs.GetStateNames(), true,
      reduced_order_model_receiver->get_output_port_lcs_state(),
      lcm_channel_params.c3_actual_state_channel, &lcm,
      TriggerTypeSet({TriggerType::kForced}));

  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("run_c3_controller"));
  plant_diagram->set_name(("franka_c3_plant"));

  // Run lcm-driven simulation
  LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, std::move(owned_diagram), franka_state_receiver,
      lcm_channel_params.franka_state_channel, true);
  DrawAndSaveDiagramGraph(*loop.get_diagram());

  LcmHandleSubscriptionsUntil(
      &lcm, [&]() { return tray_state_sub->GetInternalMessageCount() > 1; });
  loop.Simulate();
  return 0;
}

}  // namespace plate_balancing
}  // namespace examples
}  // namespace dairlib

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return dairlib::examples::plate_balancing::DoMain(
      FLAGS_plate_balancing_config, FLAGS_simulation);
}