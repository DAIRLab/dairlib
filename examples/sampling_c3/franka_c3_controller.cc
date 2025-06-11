
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
#include "control_target_generator.h"
#include "examples/sampling_c3/parameter_headers/franka_c3_controller_params.h"
#include "examples/sampling_c3/parameter_headers/lcm_channels.h"
#include "examples/sampling_c3/parameter_headers/franka_sim_params.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_options.h"
#include "examples/sampling_c3/parameter_headers/trajectory_params.h"
#include "examples/sampling_c3/parameter_headers/sampling_params.h"
#include "multibody/multibody_utils.h"
#include "solvers/lcs_factory.h"
#include "systems/controllers/sampling_based_c3_controller.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/franka_kinematics.h"
#include "systems/robot_lcm_systems.h"
#include "systems/senders/c3_state_sender.h"
#include "systems/senders/sample_buffer_sender.h"
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
using std::vector;

// TODO: @bibit parameter overhaul, don't hardcode yaml filepaths
DEFINE_string(
    lcm_channels,
    "examples/sampling_c3/shared_parameters/lcm_channels_simulation.yaml",
    "Filepath containing lcm channels");
DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");
DEFINE_string(demo_name, "jacktoy",
              "Name for the demo, used when building filepaths for output.");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::lcm::DrakeLcm lcm(FLAGS_lcm_url);
  std::string base_path = "examples/sampling_c3/" + FLAGS_demo_name + "/";

  // Load parameters.
  drake::yaml::LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;
  FrankaC3ControllerParams controller_params =
      drake::yaml::LoadYamlFile<FrankaC3ControllerParams>(
          base_path + "parameters/franka_c3_controller_params.yaml");
  SamplingC3TrajectoryParams trajectory_params =
      drake::yaml::LoadYamlFile<SamplingC3TrajectoryParams>(
          base_path + "parameters/trajectory_params.yaml");
  // Sim params are only used to keep the offsets between different models
  // in the scene consistent across all systems.  TODO @bibit redo
  FrankaSimParams sim_params = drake::yaml::LoadYamlFile<FrankaSimParams>(
      base_path + "parameters/franka_sim_params.yaml");
  SamplingC3LcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<SamplingC3LcmChannels>(FLAGS_lcm_channels);
  SamplingC3Options sampling_c3_options =
      drake::yaml::LoadYamlFile<SamplingC3Options>(
          base_path + "parameters/sampling_c3_options.yaml");
  SamplingParams sampling_params = drake::yaml::LoadYamlFile<SamplingParams>(
      controller_params.sampling_params_file);

  // Create a Franka-only plant.
  MultibodyPlant<double> plant_franka(0.0);
  Parser parser_franka(&plant_franka, nullptr);
  parser_franka.AddModelsFromUrl(controller_params.franka_model)[0];
  parser_franka.AddModels(
    FindResourceOrThrow(controller_params.end_effector_model))[0];

  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant_franka.WeldFrames(plant_franka.world_frame(),
                          plant_franka.GetFrameByName("panda_link0"), X_WI);

  RigidTransform<double> T_EE_W = RigidTransform<double>(
      drake::math::RotationMatrix<double>(
          drake::math::RollPitchYaw<double>(3.1415, 0, 0)),
      sim_params.tool_attachment_frame);
  plant_franka.WeldFrames(plant_franka.GetFrameByName("panda_link7"),
                          plant_franka.GetFrameByName("end_effector_flange"),
                          T_EE_W);

  plant_franka.Finalize();
  auto franka_context = plant_franka.CreateDefaultContext();

  // Create an object-only plant.
  MultibodyPlant<double> plant_object(0.0);
  Parser parser_object(&plant_object, nullptr);
  parser_object.AddModels(controller_params.object_model);
  plant_object.Finalize();
  auto object_context = plant_object.CreateDefaultContext();

  // Create the LCS plant containing a floating EE, object, and ground.
  DiagramBuilder<double> plant_lcs_builder;
  auto [plant_lcs, scene_graph] =
      AddMultibodyPlantSceneGraph(&plant_lcs_builder, 0.0);
  Parser parser_lcs(&plant_lcs);
  parser_lcs.SetAutoRenaming(true);
  parser_lcs.AddModels(controller_params.end_effector_simple_model);
  parser_lcs.AddModels(controller_params.object_model);
  parser_lcs.AddModels(controller_params.ground_model);

  Eigen::Vector3d p_world_to_ground =
      sim_params.p_world_to_franka + sim_params.p_franka_to_ground;
  RigidTransform<double> X_W_G = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), p_world_to_ground);
  plant_lcs.WeldFrames(plant_lcs.world_frame(),
                       plant_lcs.GetFrameByName("base_link"), X_WI);
  plant_lcs.WeldFrames(plant_lcs.world_frame(),
                       plant_lcs.GetFrameByName("ground"), X_W_G);
  plant_lcs.Finalize();
  std::unique_ptr<MultibodyPlant<drake::AutoDiffXd>> plant_lcs_autodiff =
      drake::systems::System<double>::ToAutoDiffXd(plant_lcs);

  auto plant_lcs_diagram = plant_lcs_builder.Build();
  std::unique_ptr<drake::systems::Context<double>> diagram_context =
      plant_lcs_diagram->CreateDefaultContext();
  auto& plant_lcs_context = plant_lcs_diagram->GetMutableSubsystemContext(
      plant_lcs, diagram_context.get());
  auto plant_lcs_context_ad =
      plant_lcs_autodiff->CreateDefaultContext();

  // Build the contact pairs based on the demo.
  std::vector<std::vector<SortedPair<GeometryId>>> contact_pairs;
  std::unordered_map<std::string, drake::geometry::GeometryId> contact_geoms;

  if (FLAGS_demo_name == "jacktoy") {
    drake::geometry::GeometryId ee_contact_points =
        plant_lcs.GetCollisionGeometriesForBody(
            plant_lcs.GetBodyByName("end_effector_simple"))[0];
    drake::geometry::GeometryId capsule1_geoms =
        plant_lcs.GetCollisionGeometriesForBody(
            plant_lcs.GetBodyByName("capsule_1"))[0];
    drake::geometry::GeometryId capsule2_geoms =
        plant_lcs.GetCollisionGeometriesForBody(
            plant_lcs.GetBodyByName("capsule_2"))[0];
    drake::geometry::GeometryId capsule3_geoms =
        plant_lcs.GetCollisionGeometriesForBody(
            plant_lcs.GetBodyByName("capsule_3"))[0];

    drake::geometry::GeometryId capsule1_sphere1_geoms =
        plant_lcs.GetCollisionGeometriesForBody(
            plant_lcs.GetBodyByName("capsule_1"))[1];
    drake::geometry::GeometryId capsule1_sphere2_geoms =
        plant_lcs.GetCollisionGeometriesForBody(
            plant_lcs.GetBodyByName("capsule_1"))[2];
    drake::geometry::GeometryId capsule2_sphere1_geoms =
        plant_lcs.GetCollisionGeometriesForBody(
            plant_lcs.GetBodyByName("capsule_2"))[1];
    drake::geometry::GeometryId capsule2_sphere2_geoms =
        plant_lcs.GetCollisionGeometriesForBody(
            plant_lcs.GetBodyByName("capsule_2"))[2];
    drake::geometry::GeometryId capsule3_sphere1_geoms =
        plant_lcs.GetCollisionGeometriesForBody(
            plant_lcs.GetBodyByName("capsule_3"))[1];
    drake::geometry::GeometryId capsule3_sphere2_geoms =
        plant_lcs.GetCollisionGeometriesForBody(
            plant_lcs.GetBodyByName("capsule_3"))[2];

    drake::geometry::GeometryId ground_geoms =
        plant_lcs.GetCollisionGeometriesForBody(
            plant_lcs.GetBodyByName("ground"))[0];

    contact_geoms["EE"] = ee_contact_points;
    contact_geoms["CAPSULE_1"] = capsule1_geoms;
    contact_geoms["CAPSULE_2"] = capsule2_geoms;
    contact_geoms["CAPSULE_3"] = capsule3_geoms;
    contact_geoms["CAPSULE_1_SPHERE_1"] = capsule1_sphere1_geoms;
    contact_geoms["CAPSULE_1_SPHERE_2"] = capsule1_sphere2_geoms;
    contact_geoms["CAPSULE_2_SPHERE_1"] = capsule2_sphere1_geoms;
    contact_geoms["CAPSULE_2_SPHERE_2"] = capsule2_sphere2_geoms;
    contact_geoms["CAPSULE_3_SPHERE_1"] = capsule3_sphere1_geoms;
    contact_geoms["CAPSULE_3_SPHERE_2"] = capsule3_sphere2_geoms;
    contact_geoms["GROUND"] = ground_geoms;

    // EE-object contact pairs first.
    std::vector<SortedPair<GeometryId>> ee_contact_pairs;
    ee_contact_pairs.push_back(
        SortedPair(contact_geoms["EE"], contact_geoms["CAPSULE_1"]));
    ee_contact_pairs.push_back(
        SortedPair(contact_geoms["EE"], contact_geoms["CAPSULE_2"]));
    ee_contact_pairs.push_back(
        SortedPair(contact_geoms["EE"], contact_geoms["CAPSULE_3"]));

    contact_pairs.push_back(ee_contact_pairs);

    // If desired, add an EE-ground contact pair.
    if (sampling_c3_options.num_contacts_index == 2 ||
        sampling_c3_options.num_contacts_index == 3) {
      std::vector<SortedPair<GeometryId>> ee_ground_contact{
          SortedPair(contact_geoms["EE"], contact_geoms["GROUND"])};
      contact_pairs.push_back(ee_ground_contact);
    }

    // Object-ground contact pairs last.
    std::vector<SortedPair<GeometryId>> ground_object_contact_pairs;
    ground_object_contact_pairs.push_back(SortedPair(
        contact_geoms["CAPSULE_1_SPHERE_1"], contact_geoms["GROUND"]));
    ground_object_contact_pairs.push_back(SortedPair(
        contact_geoms["CAPSULE_1_SPHERE_2"], contact_geoms["GROUND"]));
    ground_object_contact_pairs.push_back(SortedPair(
        contact_geoms["CAPSULE_2_SPHERE_1"], contact_geoms["GROUND"]));
    ground_object_contact_pairs.push_back(SortedPair(
        contact_geoms["CAPSULE_2_SPHERE_2"], contact_geoms["GROUND"]));
    ground_object_contact_pairs.push_back(SortedPair(
        contact_geoms["CAPSULE_3_SPHERE_1"], contact_geoms["GROUND"]));
    ground_object_contact_pairs.push_back(SortedPair(
        contact_geoms["CAPSULE_3_SPHERE_2"], contact_geoms["GROUND"]));

    contact_pairs.push_back(ground_object_contact_pairs);
  }
  else {
    throw std::runtime_error("Unknown --demo_name value: " + FLAGS_demo_name);
  }

  // Piece together the diagram.
  DiagramBuilder<double> builder;

  auto object_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.object_state_channel, &lcm));
  auto franka_state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_franka);
  auto object_state_receiver =
      builder.AddSystem<systems::ObjectStateReceiver>(plant_object);
  auto radio_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
          lcm_channel_params.radio_channel, &lcm));
  auto reduced_order_model_receiver =
      builder.AddSystem<systems::FrankaKinematics>(
          plant_franka, franka_context.get(), plant_object,
          object_context.get(), controller_params.end_effector_name,
          controller_params.object_body_name,
          controller_params.include_end_effector_orientation);

  // Select the target generator based on the demo.
  std::unique_ptr<systems::TargetGenerator> target_generator;
  if (FLAGS_demo_name == "jacktoy") {
    target_generator =
        std::make_unique<systems::TargetGeneratorJacktoy>(plant_object);
  } else {
    throw std::runtime_error("Unknown --demo_name value: " + FLAGS_demo_name);
  }
  auto* control_target = builder.AddSystem(std::move(target_generator));
  // TODO @bibit this list just needs to exclude ball_rolling, as demos are
  // added.
  const std::vector<std::string> demos_with_target_params = {"jacktoy"};
  if (std::find(demos_with_target_params.begin(),
                demos_with_target_params.end(),
                FLAGS_demo_name) != demos_with_target_params.end()) {
    control_target->SetRemoteControlParameters(
        trajectory_params.goal_mode, trajectory_params.fixed_target_position,
        trajectory_params.fixed_target_orientation,
        trajectory_params.lookahead_step_size,
        trajectory_params.lookahead_angle, trajectory_params.angle_hysteresis,
        trajectory_params.angle_err_to_vel_factor,
        trajectory_params.ee_target_z_offset_above_object,
        trajectory_params.position_success_threshold,
        trajectory_params.orientation_success_threshold,
        trajectory_params.random_goal_x_limits,
        trajectory_params.random_goal_y_limits,
        trajectory_params.random_goal_radius_limits,
        trajectory_params.resting_object_height);
  }

  // Input sizes are EE position (3), object pose (7), EE velocity (3), object
  // velocities (6).
  std::vector<int> input_sizes = {3, 7, 3, 6};
  auto target_state_mux =
      builder.AddSystem<drake::systems::Multiplexer>(input_sizes);
  auto final_target_state_mux =
      builder.AddSystem<drake::systems::Multiplexer>(input_sizes);
  auto end_effector_zero_velocity_source =
      builder.AddSystem<drake::systems::ConstantVectorSource>(
          VectorXd::Zero(3));
  auto object_zero_velocity_source =
      builder.AddSystem<drake::systems::ConstantVectorSource>(
          VectorXd::Zero(6));
  auto target_gen_info_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.target_generator_info_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));

  builder.Connect(control_target->get_output_port_end_effector_target(),
                  target_state_mux->get_input_port(0));
  builder.Connect(control_target->get_output_port_object_target(),
                  target_state_mux->get_input_port(1));
  builder.Connect(end_effector_zero_velocity_source->get_output_port(),
                  target_state_mux->get_input_port(2));
  builder.Connect(control_target->get_output_port_object_velocity_target(),
                  target_state_mux->get_input_port(3));
  builder.Connect(control_target->get_output_port_target_gen_info(),
                  target_gen_info_publisher->get_input_port());
  builder.Connect(control_target->get_output_port_end_effector_target(),
                  final_target_state_mux->get_input_port(0));
  builder.Connect(control_target->get_output_port_object_final_target(),
                  final_target_state_mux->get_input_port(1));
  builder.Connect(end_effector_zero_velocity_source->get_output_port(),
                  final_target_state_mux->get_input_port(2));
  builder.Connect(object_zero_velocity_source->get_output_port(),
                  final_target_state_mux->get_input_port(3));

  // Sampling C3 controller.
  auto controller = builder.AddSystem<systems::SamplingC3Controller>(
      plant_lcs, &plant_lcs_context, *plant_lcs_autodiff,
      plant_lcs_context_ad.get(), contact_pairs, sampling_c3_options,
      sampling_params);
  drake::solvers::SolverOptions solver_options =
      drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
          FindResourceOrThrow(controller_params.osqp_settings_file))
          .GetAsSolverOptions(drake::solvers::OsqpSolver::id());
  controller->SetOsqpSolverOptions(solver_options);

  // Systems for publishing the current and best planned trajectories.
  auto actor_trajectory_sender_curr_plan = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.c3_actor_curr_plan_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto object_trajectory_sender_curr_plan = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.c3_object_curr_plan_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto actor_trajectory_sender_best_plan = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.c3_actor_best_plan_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto object_trajectory_sender_best_plan = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.c3_object_best_plan_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));

  // C3 senders.
  auto c3_output_sender_curr_plan =
      builder.AddNamedSystem("c3_output_sender_curr_plan",
                             std::make_unique<systems::C3OutputSender>());
  auto c3_output_publisher_curr_plan =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_output>(
          lcm_channel_params.c3_debug_output_curr_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto c3_forces_publisher_curr_plan =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_forces>(
          lcm_channel_params.c3_force_curr_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));

  auto c3_output_sender_best_plan =
      builder.AddNamedSystem("c3_output_sender_best_plan",
                             std::make_unique<systems::C3OutputSender>());
  auto c3_output_publisher_best_plan =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_output>(
          lcm_channel_params.c3_debug_output_best_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto c3_forces_publisher_best_plan =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_forces>(
          lcm_channel_params.c3_force_best_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));

  // Systems for publishing the tracking output.
  auto actor_c3_execution_trajectory_sender = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.c3_trajectory_exec_actor_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto actor_repos_execution_trajectory_sender = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.repos_trajectory_exec_actor_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto actor_tracking_trajectory_sender = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.tracking_trajectory_actor_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));

  // Sample-related senders/publishers.
  auto sample_buffer_sender = builder.AddSystem<systems::SampleBufferSender>(
      sampling_params.N_sample_buffer, plant_lcs.num_positions());
  auto sample_buffer_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_sample_buffer>(
          lcm_channel_params.sample_buffer_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto sample_locations_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.sample_locations_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto sample_costs_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.sample_costs_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));

  // Debugging publishers.
  auto controller_debug_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_sampling_c3_debug>(
          lcm_channel_params.sampling_c3_debug_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto is_c3_mode_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.is_c3_mode_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));

  // Dynamically feasible plan publishers.
  auto dynamically_feasible_curr_plan_actor_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.dynamically_feasible_curr_actor_plan_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto dynamically_feasible_curr_plan_object_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.dynamically_feasible_curr_plan_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto dynamically_feasible_best_plan_actor_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.dynamically_feasible_best_actor_plan_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto dynamically_feasible_best_plan_object_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.dynamically_feasible_best_plan_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));

  std::vector<std::string> state_names = {
      "end_effector_x",  "end_effector_y",  "end_effector_z", 
      "object_qw",       "object_qx",       "object_qy",       "object_qz",
      "object_x",        "object_y",        "object_z",
      "end_effector_vx", "end_effector_vy", "end_effector_vz",
      "object_wx" ,      "object_wy",       "object_wz",
      "object_vz",       "object_vz",       "object_vz",
  };
  // C3 state senders:  actual, target, and final target.
  auto c3_state_sender = builder.AddSystem<systems::C3StateSender>(
      plant_lcs.num_positions() + plant_lcs.num_velocities(),
      state_names);
  auto c3_target_state_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_state>(
          lcm_channel_params.c3_target_state_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto c3_actual_state_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_state>(
          lcm_channel_params.c3_actual_state_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto c3_final_target_state_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_state>(
          lcm_channel_params.c3_final_target_state_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));

  builder.Connect(franka_state_receiver->get_output_port(),
                  reduced_order_model_receiver->get_input_port_franka_state());
  builder.Connect(object_state_sub->get_output_port(),
                  object_state_receiver->get_input_port());
  builder.Connect(object_state_receiver->get_output_port(),
                  reduced_order_model_receiver->get_input_port_object_state());
  builder.Connect(reduced_order_model_receiver->get_output_port(),
                  controller->get_input_port_lcs_state());
  builder.Connect(object_state_receiver->get_output_port(),
                  control_target->get_input_port_object_state());
  builder.Connect(target_state_mux->get_output_port(),
                  controller->get_input_port_target());
  builder.Connect(final_target_state_mux->get_output_port(),
                  controller->get_input_port_final_target());
  builder.Connect(radio_sub->get_output_port(),
                  controller->get_input_port_radio());
  builder.Connect(radio_sub->get_output_port(),
                  control_target->get_input_port_radio());
  builder.Connect(controller->get_output_port_c3_solution_curr_plan_actor(),
                  actor_trajectory_sender_curr_plan->get_input_port());
  builder.Connect(controller->get_output_port_c3_solution_curr_plan_object(),
                  object_trajectory_sender_curr_plan->get_input_port());
  builder.Connect(controller->get_output_port_c3_solution_best_plan_actor(),
                  actor_trajectory_sender_best_plan->get_input_port());
  builder.Connect(controller->get_output_port_c3_solution_best_plan_object(),
                  object_trajectory_sender_best_plan->get_input_port());
  builder.Connect(controller->get_output_port_c3_solution_curr_plan(),
                  c3_output_sender_curr_plan->get_input_port_c3_solution());
  builder.Connect(controller->get_output_port_c3_intermediates_curr_plan(),
                  c3_output_sender_curr_plan->get_input_port_c3_intermediates());
  builder.Connect(controller->get_output_port_lcs_contact_jacobian_curr_plan(),
                  c3_output_sender_curr_plan->get_input_port_lcs_contact_info());
  builder.Connect(c3_output_sender_curr_plan->get_output_port_c3_debug(),
                  c3_output_publisher_curr_plan->get_input_port());
  builder.Connect(c3_output_sender_curr_plan->get_output_port_c3_force(),
                  c3_forces_publisher_curr_plan->get_input_port());
  builder.Connect(controller->get_output_port_c3_solution_best_plan(),
                  c3_output_sender_best_plan->get_input_port_c3_solution());
  builder.Connect(controller->get_output_port_c3_intermediates_best_plan(),
                  c3_output_sender_best_plan->get_input_port_c3_intermediates());
  builder.Connect(controller->get_output_port_lcs_contact_jacobian_best_plan(),
                  c3_output_sender_best_plan->get_input_port_lcs_contact_info());
  builder.Connect(c3_output_sender_best_plan->get_output_port_c3_debug(),
                  c3_output_publisher_best_plan->get_input_port());
  builder.Connect(c3_output_sender_best_plan->get_output_port_c3_force(),
                  c3_forces_publisher_best_plan->get_input_port());
  builder.Connect(controller->get_output_port_dynamically_feasible_curr_plan_actor(),
                  dynamically_feasible_curr_plan_actor_publisher->get_input_port());
  builder.Connect(controller->get_output_port_dynamically_feasible_best_plan_actor(),
                  dynamically_feasible_best_plan_actor_publisher->get_input_port());
  builder.Connect(controller->get_output_port_dynamically_feasible_curr_plan_object(),
                  dynamically_feasible_curr_plan_object_publisher->get_input_port());
  builder.Connect(controller->get_output_port_dynamically_feasible_curr_plan_object(),
                  dynamically_feasible_best_plan_object_publisher->get_input_port());
  builder.Connect(target_state_mux->get_output_port(),
                  c3_state_sender->get_input_port_target_state());
  builder.Connect(final_target_state_mux->get_output_port(),
                  c3_state_sender->get_input_port_final_target_state());
  builder.Connect(reduced_order_model_receiver->get_output_port_lcs_state(),
                  c3_state_sender->get_input_port_actual_state());
  builder.Connect(c3_state_sender->get_output_port_target_c3_state(),
                  c3_target_state_publisher->get_input_port());
  builder.Connect(c3_state_sender->get_output_port_final_target_c3_state(),
                  c3_final_target_state_publisher->get_input_port());
  builder.Connect(c3_state_sender->get_output_port_actual_c3_state(),
                  c3_actual_state_publisher->get_input_port());
  builder.Connect(controller->get_output_port_c3_traj_execute_actor(),
                  actor_c3_execution_trajectory_sender->get_input_port());
  builder.Connect(controller->get_output_port_repos_traj_execute_actor(),
                  actor_repos_execution_trajectory_sender->get_input_port());
  builder.Connect(controller->get_output_port_traj_execute_actor(),
                  actor_tracking_trajectory_sender->get_input_port());
  builder.Connect(controller->get_output_port_all_sample_locations(),
                  sample_locations_publisher->get_input_port());
  builder.Connect(controller->get_output_port_all_sample_costs(),
                  sample_costs_publisher->get_input_port());
  builder.Connect(controller->get_output_port_is_c3_mode(),
                  is_c3_mode_publisher->get_input_port());
  builder.Connect(controller->get_output_port_debug(),
                  controller_debug_publisher->get_input_port());
  builder.Connect(sample_buffer_sender->get_output_port_sample_buffer(),
                  sample_buffer_publisher->get_input_port());
  builder.Connect(controller->get_output_port_sample_buffer_configurations(),
                  sample_buffer_sender->get_input_port_samples());
  builder.Connect(controller->get_output_port_sample_buffer_costs(),
                  sample_buffer_sender->get_input_port_sample_costs());

  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("sampling_c3_controller_" + FLAGS_demo_name));
  plant_lcs_diagram->set_name(("sampling_c3_lcs_plant" + FLAGS_demo_name));
  DrawAndSaveDiagramGraph(*owned_diagram);
  DrawAndSaveDiagramGraph(*plant_lcs_diagram);

  // Run lcm-driven simulation.  The buffer size argument is needed to ensure
  // the latest messages are used in the control loop.  See
  // https://github.com/DAIRLab/dairlib/pull/366 for more details.
  int lcm_buffer_size = 200;
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, std::move(owned_diagram), franka_state_receiver,
      lcm_channel_params.franka_state_channel, true, lcm_buffer_size);
  LcmHandleSubscriptionsUntil(
      &lcm, [&]() { return object_state_sub->GetInternalMessageCount() > 1; });
  loop.Simulate();
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
