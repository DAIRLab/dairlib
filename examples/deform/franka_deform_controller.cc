
#include <dairlib/lcmt_elastoplastic_network.hpp>
#include <dairlib/lcmt_material_points.hpp>
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

#include "c3/multibody/lcs_factory.h"
#include "c3/systems/lcmt_generators/c3_output_generator.h"
#include "c3/systems/lcmt_generators/contact_force_generator.h"
#include "common/eigen_utils.h"
#include "examples/deform/deform_utils.h"
#include "examples/deform/mpm_model_reducer.h"
#include "examples/deform/parameter_headers/deform_controller_params.h"
#include "examples/deform/parameter_headers/elastoplastic_sc3_options.h"
#include "examples/deform/parameter_headers/lcm_channels.h"
#include "examples/deform/parameter_headers/reduced_model_params.h"
#include "multibody/multibody_utils.h"
#include "systems/controllers/elastoplastic_sampling_c3_controller.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/franka_kinematics.h"
#include "systems/robot_lcm_systems.h"
#include "systems/senders/c3_state_sender.h"
#include "systems/senders/sample_buffer_sender.h"
#include "systems/system_utils.h"

namespace dairlib {

using c3::multibody::LCSFactory;
using c3::systems::lcmt_generators::C3OutputGenerator;
using c3::systems::lcmt_generators::ContactForceGenerator;
using drake::AutoDiffXd;
using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::math::RigidTransform;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Context;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using drake::systems::InputPort;
using drake::systems::InputPortIndex;
using drake::systems::OutputPort;
using drake::systems::OutputPortIndex;
using drake::systems::System;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmInterfaceSystem;
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
  std::cout << "Loading deform controller params from YAML..." << std::endl;
  DeformControllerParams deform_controller_params =
      drake::yaml::LoadYamlFile<DeformControllerParams>(
          "examples/deform/parameters/deform_controller_params.yaml");
  std::cout << "Finished loading deform controller params." << std::endl;
  std::cout << "-> deform_controller_params.sampling_c3_controller_params."
               "sampling_c3_options.lcs_dt_resolution: "
            << deform_controller_params.sampling_c3_controller_params
                   .sampling_c3_options.lcs_dt_resolution
            << std::endl;
  ReducedModelParams reduced_model_params =
      drake::yaml::LoadYamlFile<ReducedModelParams>(
          deform_controller_params.reduced_model_params_file);
  std::string lcm_channels_file =
      FLAGS_is_simulation
          ? deform_controller_params.lcm_channels_simulation_file
          : deform_controller_params.lcm_channels_hardware_file;
  DeformLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<DeformLcmChannels>(lcm_channels_file);
  ElastoPlasticSC3Options elastoplastic_sc3_options =
      drake::yaml::LoadYamlFile<ElastoPlasticSC3Options>(
          deform_controller_params.elastoplastic_sc3_options_file);
  ElastoPlasticGoalParams elastoplastic_goal_params =
      drake::yaml::LoadYamlFile<ElastoPlasticGoalParams>(
          deform_controller_params.goal_params_file);

  // Piece together the diagram.
  DiagramBuilder<double> builder;

  // 1) Franka state receiver.
  MultibodyPlant<double> plant_franka(0.0);
  ModelInstanceIndex robot_index =
      AddFrankaToPlant(&plant_franka, nullptr, true, true, true);
  plant_franka.Finalize();
  auto franka_context = plant_franka.CreateDefaultContext();
  auto franka_state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_franka);

  // 2) MPM object state and radio receivers.
  auto mpm_points_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_material_points>(
          lcm_channel_params.mpm_channel, &lcm));
  auto radio_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
          lcm_channel_params.radio_channel, &lcm));

  // 3) Convert MPM points to a reduced elastoplastic network model.  This
  // involves first converting the MPM points to tetrahedra, then converting to
  // an elastoplastic network.
  int n_internal_contact_geometries;
  if (reduced_model_params.reduction_type ==
      ReducedModelTypes::kSupportDirections) {
    n_internal_contact_geometries =
        reduced_model_params.support_directions_vec.size();
  } else {
    throw std::runtime_error("Other model reduction types not implemented.");
  }
  auto mpm_to_tetrahedra =
      builder.AddSystem<dairlib::systems::MpmPointsToTetrahedra>(
          reduced_model_params);
  auto tetrahedra_to_elastoplastic_network =
      builder.AddSystem<dairlib::systems::TetrahedraToElastoPlasticNetwork>(
          reduced_model_params.youngs_modulus,
          reduced_model_params.yield_stress,
          reduced_model_params.spring_constant_method);
  auto tetrahedra_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_tetrahedra>(
          lcm_channel_params.tetrahedra_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto elastoplastic_model_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_elastoplastic_network>(
          lcm_channel_params.reduced_model_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  builder.Connect(mpm_points_sub->get_output_port(),
                  mpm_to_tetrahedra->get_input_port_lcmt_material_points());
  builder.Connect(
      mpm_to_tetrahedra->get_output_port_lcmt_tetrahedra(),
      tetrahedra_to_elastoplastic_network->get_input_port_lcmt_tetrahedra());
  builder.Connect(mpm_to_tetrahedra->get_output_port_lcmt_tetrahedra(),
                  tetrahedra_publisher->get_input_port());
  builder.Connect(tetrahedra_to_elastoplastic_network
                      ->get_output_port_lcmt_elastoplastic_network(),
                  elastoplastic_model_publisher->get_input_port());

  // 4) LCS state.  Requires a plant with the deformable LCS model.
  MultibodyPlant<double> plant_deform_network(0.0);
  vector<ModelInstanceIndex> node_indices = AddDeformableLCSModelToPlant(
      &plant_deform_network, nullptr, n_internal_contact_geometries,
      reduced_model_params.mass);
  plant_deform_network.Finalize();
  auto deform_context = plant_deform_network.CreateDefaultContext();

  auto franka_kinematics = builder.AddSystem<systems::FrankaKinematics>(
      plant_franka, franka_context.get(), &plant_deform_network,
      deform_context.get(), kEndEffectorName, n_internal_contact_geometries,
      false);
  builder.Connect(franka_state_receiver->get_output_port(),
                  franka_kinematics->get_input_port_franka_state());
  builder.Connect(tetrahedra_to_elastoplastic_network
                      ->get_output_port_lcmt_elastoplastic_network(),
                  franka_kinematics->get_input_port_elastoplastic_network());

  // 5) Sampling C3 controller:  This requires several new inputs.
  /////
  // Create the LCS plant containing a floating EE, deformable nodes, ground,
  // and box.
  DiagramBuilder<double> plant_lcs_builder;
  auto [plant_lcs, scene_graph] =
      AddMultibodyPlantSceneGraph(&plant_lcs_builder, 0.0);
  vector<ModelInstanceIndex> object_indices_lcs =
      AddLCSModelsForDeformableToPlant(&plant_lcs, &scene_graph,
                                       n_internal_contact_geometries,
                                       reduced_model_params.mass, true);
  plant_lcs.Finalize();

  // Get the autodiff plant and contexts.
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_lcs_autodiff =
      System<double>::ToAutoDiffXd(plant_lcs);
  auto plant_lcs_diagram = plant_lcs_builder.Build();
  std::unique_ptr<Context<double>> diagram_context =
      plant_lcs_diagram->CreateDefaultContext();
  auto& plant_lcs_context = plant_lcs_diagram->GetMutableSubsystemContext(
      plant_lcs, diagram_context.get());
  auto plant_lcs_context_ad = plant_lcs_autodiff->CreateDefaultContext();

  // Collect the collision geometries.
  std::unordered_map<std::string, GeometryId> contact_geoms;
  GeometryId ee_contact_geom = plant_lcs.GetCollisionGeometriesForBody(
      plant_lcs.GetBodyByName("end_effector_simple"))[0];
  contact_geoms["EE"] = ee_contact_geom;
  GeometryId box_contact_geom = plant_lcs.GetCollisionGeometriesForBody(
      plant_lcs.GetBodyByName("box"))[0];
  contact_geoms["box"] = box_contact_geom;
  vector<GeometryId> internal_contact_geometries;
  for (int i = 0; i < n_internal_contact_geometries; ++i) {
    internal_contact_geometries.push_back(
        plant_lcs.GetCollisionGeometriesForBody(
            plant_lcs.GetBodyByName("pt", object_indices_lcs[i]))[0]);
    contact_geoms["deformable_node_" + std::to_string(i)] =
        internal_contact_geometries[i];
  }

  // Build the external contact pairs.
  vector<vector<SortedPair<GeometryId>>> external_contact_pair_lists;
  vector<SortedPair<GeometryId>> ee_ground_contact_pairs;
  vector<SortedPair<GeometryId>> ee_object_contact_pairs;
  vector<SortedPair<GeometryId>> ground_object_contact_pairs;

  ee_ground_contact_pairs.push_back(
      SortedPair(contact_geoms["EE"], contact_geoms["box"]));
  for (int i = 0; i < n_internal_contact_geometries; ++i) {
    ee_object_contact_pairs.push_back(
        SortedPair(contact_geoms["EE"],
                   contact_geoms["deformable_node_" + std::to_string(i)]));
    ground_object_contact_pairs.push_back(
        SortedPair(contact_geoms["box"],
                   contact_geoms["deformable_node_" + std::to_string(i)]));
  }
  // Order:  EE-ground, EE-object, object-ground.
  external_contact_pair_lists.push_back(ee_ground_contact_pairs);
  external_contact_pair_lists.push_back(ee_object_contact_pairs);
  external_contact_pair_lists.push_back(ground_object_contact_pairs);

  // Create the controller.
  std::cout << "deform_controller_params.sampling_c3_controller_params."
               "sampling_c3_options.lcs_dt_resolution: "
            << deform_controller_params.sampling_c3_controller_params
                   .sampling_c3_options.lcs_dt_resolution
            << std::endl;
  auto controller = builder.AddSystem<systems::ElastoPlasticSC3Controller>(
      plant_lcs, &plant_lcs_context, *plant_lcs_autodiff,
      plant_lcs_context_ad.get(), external_contact_pair_lists,
      internal_contact_geometries, deform_controller_params);

  // Wire the controller to existing systems.
  builder.Connect(franka_kinematics->get_output_port_lcs_state(),
                  controller->get_input_port_lcs_state());
  builder.Connect(tetrahedra_to_elastoplastic_network
                      ->get_output_port_lcmt_elastoplastic_network(),
                  controller->get_input_port_elastoplastic());
  builder.Connect(radio_sub->get_output_port(),
                  controller->get_input_port_radio());
  /////

  // 6) Make a goal target for the controller.  For now, this is constant.
  DRAKE_DEMAND(elastoplastic_goal_params.goal_mode == GoalMode::kFixedGoal);
  VectorXd goal_target = VectorXd::Zero(6 + 6 * n_internal_contact_geometries);
  Vector3d avg_node_target = Vector3d::Zero();
  for (int i = 0; i < n_internal_contact_geometries; ++i) {
    goal_target.segment<3>(3 + 3 * i) =
        elastoplastic_goal_params.fixed_node_targets_vector[i];
    avg_node_target += elastoplastic_goal_params.fixed_node_targets_vector[i];
  }
  goal_target.segment<3>(0) = avg_node_target / n_internal_contact_geometries;
  goal_target[2] += elastoplastic_goal_params.ee_target_z_offset_above_object;
  auto desired_target_source =
      builder.AddSystem<drake::systems::ConstantVectorSource>(goal_target);
  auto desired_final_target_source =
      builder.AddSystem<drake::systems::ConstantVectorSource>(goal_target);
  builder.Connect(desired_target_source->get_output_port(),
                  controller->get_input_port_target());
  builder.Connect(desired_final_target_source->get_output_port(),
                  controller->get_input_port_final_target());

  // 7) C3 state senders:  actual, target, and final target.
  std::vector<std::string> state_names = {"end_effector_x", "end_effector_y",
                                          "end_effector_z"};
  std::vector<std::string> node_pos_names = {"node_x", "node_y", "node_z"};
  std::vector<std::string> node_vel_names = {"node_vx", "node_vy", "node_vz"};
  for (int i = 0; i < n_internal_contact_geometries; i++) {
    for (int j = 0; j < node_pos_names.size(); j++) {
      std::string item = node_pos_names.at(j) + "_" + std::to_string(i);
      state_names.push_back(item);
    }
  }
  state_names.push_back("end_effector_vx");
  state_names.push_back("end_effector_vy");
  state_names.push_back("end_effector_vz");
  for (int i = 0; i < n_internal_contact_geometries; i++) {
    for (int j = 0; j < node_vel_names.size(); j++) {
      std::string item = node_vel_names.at(j) + "_" + std::to_string(i);
      state_names.push_back(item);
    }
  }
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
  builder.Connect(franka_kinematics->get_output_port_lcs_state(),
                  c3_state_sender->get_input_port_actual_state());
  builder.Connect(desired_target_source->get_output_port(),
                  c3_state_sender->get_input_port_target_state());
  builder.Connect(desired_final_target_source->get_output_port(),
                  c3_state_sender->get_input_port_final_target_state());
  builder.Connect(c3_state_sender->get_output_port_target_c3_state(),
                  c3_target_state_publisher->get_input_port());
  builder.Connect(c3_state_sender->get_output_port_final_target_c3_state(),
                  c3_final_target_state_publisher->get_input_port());
  builder.Connect(c3_state_sender->get_output_port_actual_c3_state(),
                  c3_actual_state_publisher->get_input_port());

  // 8) Publish the controller outputs.
  /////
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
      deform_controller_params.sampling_params.N_sample_buffer,
      plant_lcs.num_positions(), "sample_buffer_sender");
  auto sample_buffer_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_sample_buffer>(
          lcm_channel_params.sample_buffer_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto unsuccessful_sample_buffer_sender =
      builder.AddSystem<systems::SampleBufferSender>(
          deform_controller_params.sampling_params.N_unsuccessful_sample_buffer,
          plant_lcs.num_positions(), "unsuccessful_sample_buffer_sender");
  auto unsuccessful_sample_buffer_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_sample_buffer>(
          lcm_channel_params.unsuccessful_sample_buffer_channel, &lcm,
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
          lcm_channel_params.elastoplastic_sc3_debug_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto is_c3_mode_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.is_c3_mode_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));

  C3OutputGenerator::AddLcmPublisherToBuilder(
      builder, controller->get_output_port_c3_solution_curr_plan(),
      controller->get_output_port_c3_intermediates_curr_plan(),
      lcm_channel_params.c3_debug_output_curr_channel, &lcm,
      TriggerTypeSet({TriggerType::kForced}));
  ContactForceGenerator::AddLcmPublisherToBuilder(
      builder, controller->get_output_port_c3_solution_curr_plan(),
      controller->get_output_port_lcs_contact_jacobian_curr_plan(),
      lcm_channel_params.c3_force_curr_channel, &lcm,
      TriggerTypeSet({TriggerType::kForced}));
  C3OutputGenerator::AddLcmPublisherToBuilder(
      builder, controller->get_output_port_c3_solution_best_plan(),
      controller->get_output_port_c3_intermediates_best_plan(),
      lcm_channel_params.c3_debug_output_best_channel, &lcm,
      TriggerTypeSet({TriggerType::kForced}));
  ContactForceGenerator::AddLcmPublisherToBuilder(
      builder, controller->get_output_port_c3_solution_best_plan(),
      controller->get_output_port_lcs_contact_jacobian_best_plan(),
      lcm_channel_params.c3_force_best_channel, &lcm,
      TriggerTypeSet({TriggerType::kForced}));
  C3OutputGenerator::AddLcmPublisherToBuilder(
      builder,
      controller->get_output_port_dynamically_feasible_debug_curr_plan(),
      controller->get_output_port_dynamically_feasible_debug_intermediates(),
      lcm_channel_params.dynamically_feasible_debug_curr_channel, &lcm,
      TriggerTypeSet({TriggerType::kForced}));
  C3OutputGenerator::AddLcmPublisherToBuilder(
      builder,
      controller->get_output_port_dynamically_feasible_debug_best_plan(),
      controller->get_output_port_dynamically_feasible_debug_intermediates(),
      lcm_channel_params.dynamically_feasible_debug_best_channel, &lcm,
      TriggerTypeSet({TriggerType::kForced}));

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

  builder.Connect(controller->get_output_port_sample_buffer_configurations(),
                  sample_buffer_sender->get_input_port_samples());
  builder.Connect(controller->get_output_port_sample_buffer_costs(),
                  sample_buffer_sender->get_input_port_sample_costs());

  builder.Connect(
      controller->get_output_port_unsuccessful_sample_buffer_configurations(),
      unsuccessful_sample_buffer_sender->get_input_port_samples());
  builder.Connect(
      controller->get_output_port_unsuccessful_sample_buffer_costs(),
      unsuccessful_sample_buffer_sender->get_input_port_sample_costs());

  builder.Connect(sample_buffer_sender->get_output_port_sample_buffer(),
                  sample_buffer_publisher->get_input_port());
  builder.Connect(
      unsuccessful_sample_buffer_sender->get_output_port_sample_buffer(),
      unsuccessful_sample_buffer_publisher->get_input_port());
  /////

  //////////////////////////////////////////////////////////////////////////////

  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("deform_controller"));
  DrawAndSaveDiagramGraph(*owned_diagram);

  // Run lcm-driven simulation.  The buffer size argument is needed to ensure
  // the latest messages are used in the control loop.  See
  // https://github.com/DAIRLab/dairlib/pull/366 for more details.
  int lcm_buffer_size = 200;
  std::shared_ptr<Diagram<double>> shared_diagram = std::move(owned_diagram);
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, shared_diagram, franka_state_receiver,
      lcm_channel_params.robot_state_channel, true, lcm_buffer_size);

  LcmHandleSubscriptionsUntil(
      &lcm, [&]() { return mpm_points_sub->GetInternalMessageCount() > 1; });
  loop.Simulate();
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
