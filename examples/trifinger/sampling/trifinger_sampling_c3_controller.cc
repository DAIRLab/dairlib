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
#include "examples/sampling_c3/parameter_headers/lcm_channels.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_controller_params.h"
#include "examples/trifinger/systems/trifinger_kinematics.h"
#include "examples/trifinger/trifinger_goal_generator.h"
#include "examples/trifinger/trifinger_utils.h"
#include "multibody/multibody_utils.h"
#include "solvers/lcs_factory.h"
#include "systems/controllers/sampling_based_c3_controller.h"
#include "systems/framework/lcm_driven_loop.h"
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
using drake::systems::Diagram;
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

DEFINE_bool(is_simulation, true, "True for simulation, false for hardware");
DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::lcm::DrakeLcm lcm(FLAGS_lcm_url);

  // Load parameters.
  std::string controller_params_path =
      "examples/trifinger/sampling/parameters/"
      "sampling_c3_controller_params.yaml";
  SamplingC3ControllerParams controller_params =
      drake::yaml::LoadYamlFile<SamplingC3ControllerParams>(
          controller_params_path);
  std::string lcm_channels_file =
      FLAGS_is_simulation ? controller_params.lcm_channels_simulation_file
                          : controller_params.lcm_channels_hardware_file;
  SamplingC3LcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<SamplingC3LcmChannels>(lcm_channels_file);

  // Create a Trifinger-only plant.
  MultibodyPlant<double> trifinger(0.0);
  AddTrifingerToPlant(&trifinger);
  trifinger.Finalize();
  auto trifinger_context = trifinger.CreateDefaultContext();

  // Create an object-only plant.
  MultibodyPlant<double> plant_object(0.0);
  AddObjectToPlant(&plant_object, nullptr, controller_params.object_model);
  plant_object.Finalize();
  auto object_context = plant_object.CreateDefaultContext();

  // Create the LCS plant containing a floating EE, object, and ground.
  DiagramBuilder<double> plant_lcs_builder;
  auto [plant_lcs, scene_graph] =
      AddMultibodyPlantSceneGraph(&plant_lcs_builder, 0.0);
  auto [trifinger_lcs_index, object_lcs_index] = AddLCSModelsToPlant(
      &plant_lcs, &scene_graph, controller_params.object_model);
  plant_lcs.Finalize();

  std::unique_ptr<MultibodyPlant<drake::AutoDiffXd>> plant_lcs_autodiff =
      drake::systems::System<double>::ToAutoDiffXd(plant_lcs);

  auto plant_lcs_diagram = plant_lcs_builder.Build();
  std::unique_ptr<drake::systems::Context<double>> diagram_context =
      plant_lcs_diagram->CreateDefaultContext();
  auto& plant_lcs_context = plant_lcs_diagram->GetMutableSubsystemContext(
      plant_lcs, diagram_context.get());
  auto plant_lcs_context_ad = plant_lcs_autodiff->CreateDefaultContext();

  // Build the contact pairs based on the demo.
  std::vector<std::vector<SortedPair<GeometryId>>> contact_pairs;
  std::vector<SortedPair<GeometryId>> ee_ground_contact_pairs;
  std::vector<SortedPair<GeometryId>> ee_contact_pairs;
  std::vector<SortedPair<GeometryId>> ground_object_contact_pairs;
  std::unordered_map<std::string, drake::geometry::GeometryId> contact_geoms;

  drake::geometry::GeometryId ground_geoms =
      plant_lcs.GetCollisionGeometriesForBody(
          plant_lcs.GetBodyByName("ground"))[0];
  drake::geometry::GeometryId fingertip_0_geoms =
      plant_lcs.GetCollisionGeometriesForBody(
          plant_lcs.GetBodyByName("finger_tip_link_0"))[0];
  drake::geometry::GeometryId fingertip_120_geoms =
      plant_lcs.GetCollisionGeometriesForBody(
          plant_lcs.GetBodyByName("finger_tip_link_120"))[0];
  drake::geometry::GeometryId fingertip_240_geoms =
      plant_lcs.GetCollisionGeometriesForBody(
          plant_lcs.GetBodyByName("finger_tip_link_240"))[0];
  drake::geometry::GeometryId cube_geoms =
      plant_lcs.GetCollisionGeometriesForBody(
          plant_lcs.GetBodyByName("cube"))[0];
  drake::geometry::GeometryId cube_sphere_1_geoms =
      plant_lcs.GetCollisionGeometriesForBody(
          plant_lcs.GetBodyByName("ground_contact_point_1"))[0];
  drake::geometry::GeometryId cube_sphere_3_geoms =
      plant_lcs.GetCollisionGeometriesForBody(
          plant_lcs.GetBodyByName("ground_contact_point_3"))[0];
  drake::geometry::GeometryId cube_sphere_5_geoms =
      plant_lcs.GetCollisionGeometriesForBody(
          plant_lcs.GetBodyByName("ground_contact_point_5"))[0];
  drake::geometry::GeometryId cube_sphere_7_geoms =
      plant_lcs.GetCollisionGeometriesForBody(
          plant_lcs.GetBodyByName("ground_contact_point_7"))[0];

  contact_geoms["GROUND"] = ground_geoms;
  contact_geoms["FINGERTIP_0"] = fingertip_0_geoms;
  contact_geoms["FINGERTIP_120"] = fingertip_120_geoms;
  contact_geoms["FINGERTIP_240"] = fingertip_240_geoms;
  contact_geoms["CUBE"] = cube_geoms;
  contact_geoms["CUBE_SPHERE_1"] = cube_sphere_1_geoms;
  contact_geoms["CUBE_SPHERE_3"] = cube_sphere_3_geoms;
  contact_geoms["CUBE_SPHERE_5"] = cube_sphere_5_geoms;
  contact_geoms["CUBE_SPHERE_7"] = cube_sphere_7_geoms;

  ee_ground_contact_pairs = {};

  ee_contact_pairs.push_back(
      SortedPair(contact_geoms["FINGERTIP_0"], contact_geoms["CUBE"]));
  ee_contact_pairs.push_back(
      SortedPair(contact_geoms["FINGERTIP_120"], contact_geoms["CUBE"]));
  ee_contact_pairs.push_back(
      SortedPair(contact_geoms["FINGERTIP_240"], contact_geoms["CUBE"]));

  ground_object_contact_pairs.push_back(
      SortedPair(contact_geoms["CUBE_SPHERE_1"], contact_geoms["GROUND"]));
  ground_object_contact_pairs.push_back(
      SortedPair(contact_geoms["CUBE_SPHERE_3"], contact_geoms["GROUND"]));
  ground_object_contact_pairs.push_back(
      SortedPair(contact_geoms["CUBE_SPHERE_5"], contact_geoms["GROUND"]));
  ground_object_contact_pairs.push_back(
      SortedPair(contact_geoms["CUBE_SPHERE_7"], contact_geoms["GROUND"]));

  // Order: EE-object, object-ground.
  contact_pairs.push_back(ee_ground_contact_pairs);
  contact_pairs.push_back(ee_contact_pairs);
  contact_pairs.push_back(ground_object_contact_pairs);

  // Piece together the diagram.
  DiagramBuilder<double> builder;

  auto object_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.object_state_channel, &lcm));
  auto trifinger_state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(trifinger);
  auto object_state_receiver =
      builder.AddSystem<systems::ObjectStateReceiver>(plant_object);
  auto radio_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
          lcm_channel_params.radio_channel, &lcm));
  auto reduced_order_model_receiver =
      builder.AddSystem<systems::TrifingerKinematics>(
          trifinger, trifinger_context.get(), plant_object,
          object_context.get(), kFingertip0Name, kFingertip120Name,
          kFingertip240Name, controller_params.object_body_name);

  std::unique_ptr<systems::SamplingC3GoalGeneratorTrifinger> target_generator;
  target_generator =
      std::make_unique<systems::SamplingC3GoalGeneratorTrifinger>(
          plant_lcs.num_positions(trifinger_lcs_index), plant_object,
          controller_params.goal_params, kNominalOrientationsPlanar);
  auto* control_target = builder.AddSystem(std::move(target_generator));

  std::vector<int> input_sizes = {plant_lcs.num_positions(trifinger_lcs_index),
                                  plant_lcs.num_positions(object_lcs_index),
                                  plant_lcs.num_velocities(trifinger_lcs_index),
                                  plant_lcs.num_velocities(object_lcs_index)};
  auto target_state_mux =
      builder.AddSystem<drake::systems::Multiplexer>(input_sizes);
  auto final_target_state_mux =
      builder.AddSystem<drake::systems::Multiplexer>(input_sizes);
  auto end_effector_zero_velocity_source =
      builder.AddSystem<drake::systems::ConstantVectorSource>(
          VectorXd::Zero(plant_lcs.num_velocities(trifinger_lcs_index)));
  auto object_zero_velocity_source =
      builder.AddSystem<drake::systems::ConstantVectorSource>(
          VectorXd::Zero(plant_lcs.num_velocities(object_lcs_index)));
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
      plant_lcs_context_ad.get(), contact_pairs, controller_params);

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
      controller_params.sampling_params.N_sample_buffer,
      plant_lcs.num_positions());
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
      "finger0_x",    "finger0_y",    "finger0_z",    "finger120_x",
      "finger120_y",  "finger120_z",  "finger240_x",  "finger240_y",
      "finger240_z",  "object_qw",    "object_qx",    "object_qy",
      "object_qz",    "object_x",     "object_y",     "object_z",
      "finger0_vx",   "finger0_vy",   "finger0_vz",   "finger120_vx",
      "finger120_vy", "finger120_vz", "finger240_vx", "finger240_vy",
      "finger240_vz", "object_wx",    "object_wy",    "object_wz",
      "object_vx",    "object_vy",    "object_vz",
  };
  // C3 state senders:  actual, target, and final target.
  auto c3_state_sender = builder.AddSystem<systems::C3StateSender>(
      plant_lcs.num_positions() + plant_lcs.num_velocities(), state_names);
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

  builder.Connect(
      trifinger_state_receiver->get_output_port(),
      reduced_order_model_receiver->get_input_port_trifinger_state());
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
  builder.Connect(
      controller->get_output_port_c3_intermediates_curr_plan(),
      c3_output_sender_curr_plan->get_input_port_c3_intermediates());
  builder.Connect(
      controller->get_output_port_lcs_contact_jacobian_curr_plan(),
      c3_output_sender_curr_plan->get_input_port_lcs_contact_info());
  builder.Connect(c3_output_sender_curr_plan->get_output_port_c3_debug(),
                  c3_output_publisher_curr_plan->get_input_port());
  builder.Connect(c3_output_sender_curr_plan->get_output_port_c3_force(),
                  c3_forces_publisher_curr_plan->get_input_port());
  builder.Connect(controller->get_output_port_c3_solution_best_plan(),
                  c3_output_sender_best_plan->get_input_port_c3_solution());
  builder.Connect(
      controller->get_output_port_c3_intermediates_best_plan(),
      c3_output_sender_best_plan->get_input_port_c3_intermediates());
  builder.Connect(
      controller->get_output_port_lcs_contact_jacobian_best_plan(),
      c3_output_sender_best_plan->get_input_port_lcs_contact_info());
  builder.Connect(c3_output_sender_best_plan->get_output_port_c3_debug(),
                  c3_output_publisher_best_plan->get_input_port());
  builder.Connect(c3_output_sender_best_plan->get_output_port_c3_force(),
                  c3_forces_publisher_best_plan->get_input_port());
  builder.Connect(
      controller->get_output_port_dynamically_feasible_curr_plan_actor(),
      dynamically_feasible_curr_plan_actor_publisher->get_input_port());
  builder.Connect(
      controller->get_output_port_dynamically_feasible_best_plan_actor(),
      dynamically_feasible_best_plan_actor_publisher->get_input_port());
  builder.Connect(
      controller->get_output_port_dynamically_feasible_curr_plan_object(),
      dynamically_feasible_curr_plan_object_publisher->get_input_port());
  builder.Connect(
      controller->get_output_port_dynamically_feasible_curr_plan_object(),
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
  owned_diagram->set_name(("sampling_c3_controller_trifinger"));
  plant_lcs_diagram->set_name(("sampling_c3_lcs_plant_trifinger"));
  DrawAndSaveDiagramGraph(*owned_diagram);
  DrawAndSaveDiagramGraph(*plant_lcs_diagram);

  // Run lcm-driven simulation.  The buffer size argument is needed to ensure
  // the latest messages are used in the control loop.  See
  // https://github.com/DAIRLab/dairlib/pull/366 for more details.
  int lcm_buffer_size = 200;
  std::shared_ptr<Diagram<double>> shared_diagram = std::move(owned_diagram);
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, shared_diagram, trifinger_state_receiver,
      lcm_channel_params.franka_state_channel, true, lcm_buffer_size);
  LcmHandleSubscriptionsUntil(
      &lcm, [&]() { return object_state_sub->GetInternalMessageCount() > 1; });
  loop.Simulate();
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
    drake::log()->set_level(spdlog::level::info);
    return dairlib::DoMain(argc, argv);
}
