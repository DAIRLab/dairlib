
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
#include "examples/sampling_c3/parameter_headers/lcm_channels.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_controller_params.h"
#include "examples/sampling_c3/sampling_c3_utils.h"
#include "goal_generator.h"
#include "multibody/multibody_utils.h"
#include "systems/controllers/sampling_based_c3_controller.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "systems/senders/c3_state_sender.h"
#include "systems/senders/sample_buffer_sender.h"
#include "systems/system_utils.h"
#include "systems/three_d_printer_kinematics.h"

namespace dairlib {

using c3::multibody::LCSFactory;
using c3::systems::lcmt_generators::C3OutputGenerator;
using c3::systems::lcmt_generators::ContactForceGenerator;
using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::math::RigidTransform;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using drake::systems::InputPort;
using drake::systems::InputPortIndex;
using drake::systems::OutputPort;
using drake::systems::OutputPortIndex;
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
DEFINE_string(demo_name, "three_d_printer",
              "Demo within sampling_c3; used to find controller params file");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::lcm::DrakeLcm lcm(FLAGS_lcm_url);

  // Load parameters.
  std::string controller_params_path =
      "examples/sampling_c3/" + FLAGS_demo_name +
      "/parameters/sampling_c3_controller_params.yaml";
  SamplingC3ControllerParams controller_params =
      drake::yaml::LoadYamlFile<SamplingC3ControllerParams>(
          controller_params_path);
  std::string lcm_channels_file =
      FLAGS_is_simulation ? controller_params.lcm_channels_simulation_file
                          : controller_params.lcm_channels_hardware_file;
  SamplingC3LcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<SamplingC3LcmChannels>(lcm_channels_file);
  SamplingC3Options sampling_c3_options =
      drake::yaml::LoadYamlFile<SamplingC3Options>(
          controller_params.sampling_c3_options_file);

  // Create a Franka-only plant (no need to add walls to this).
  MultibodyPlant<double> plant_three_d_printer(0.0);
  Add3DPrinterToPlant(&plant_three_d_printer, nullptr, true, true, false);
  plant_three_d_printer.Finalize();
  auto three_d_printer_context = plant_three_d_printer.CreateDefaultContext();

  // Create an object-only plant.
  MultibodyPlant<double> plant_object(0.0);
  std::vector<ModelInstanceIndex> object_indices = AddObjectsToPlant(
      &plant_object, nullptr, controller_params.object_models);

  plant_object.Finalize();
  auto object_context = plant_object.CreateDefaultContext();

  // Create the LCS plant containing a floating EE, object, and ground.
  DiagramBuilder<double> plant_lcs_builder;
  auto [plant_lcs, scene_graph] =
      AddMultibodyPlantSceneGraph(&plant_lcs_builder, 0.0);
  std::vector<ModelInstanceIndex> object_indices_lcs =
      AddLCSModelsTo3DPrinterPlant(&plant_lcs, &scene_graph,
                                   controller_params.object_models);
  plant_lcs.Finalize();

  std::unique_ptr<MultibodyPlant<drake::AutoDiffXd>> plant_lcs_autodiff =
      drake::systems::System<double>::ToAutoDiffXd(plant_lcs);

  auto plant_lcs_diagram = plant_lcs_builder.Build();
  std::unique_ptr<drake::systems::Context<double>> diagram_context =
      plant_lcs_diagram->CreateDefaultContext();
  auto& plant_lcs_context = plant_lcs_diagram->GetMutableSubsystemContext(
      plant_lcs, diagram_context.get());
  auto plant_lcs_context_ad = plant_lcs_autodiff->CreateDefaultContext();

  auto context = plant_lcs.CreateDefaultContext();
  Eigen::VectorXd x_pos = plant_lcs.GetPositionsAndVelocities(*context);

  // Build the contact pairs based on the demo.
  std::vector<std::vector<SortedPair<GeometryId>>> contact_pairs;
  std::vector<SortedPair<GeometryId>> ee_contact_pairs;
  std::vector<SortedPair<GeometryId>> ground_object_contact_pairs;
  std::unordered_map<std::string, drake::geometry::GeometryId> contact_geoms;

  // All demos include the end effector and ground.
  drake::geometry::GeometryId ee_contact_points =
      plant_lcs.GetCollisionGeometriesForBody(
          plant_lcs.GetBodyByName("end_effector_simple"))[0];
  drake::geometry::GeometryId ground_geoms =
      plant_lcs.GetCollisionGeometriesForBody(
          plant_lcs.GetBodyByName("ground"))[0];

  contact_geoms["EE"] = ee_contact_points;
  contact_geoms["GROUND"] = ground_geoms;

  std::vector<SortedPair<GeometryId>> ee_ground_contact{
      SortedPair(contact_geoms["EE"], contact_geoms["GROUND"])};

  // For each pair of object-object or wall-object, we store the contact pairs
  // between their convex pieces
  std::vector<std::vector<SortedPair<GeometryId>>> object_object_contact_pairs;
  std::vector<std::vector<SortedPair<GeometryId>>> wall_object_contact_pairs;

  // Build the 3D-printer-specific contact pairs: EE-object, object-ground,
  // ramp-object, EE-ramp, and object-object.
  std::vector<std::vector<GeometryId>> all_object_geoms;
  for (int i = 0; i < controller_params.base_names.size(); i++) {
    std::string body_name = controller_params.base_names.at(i);
    const std::vector<drake::geometry::GeometryId>& object_geoms =
        plant_lcs.GetCollisionGeometriesForBody(
            plant_lcs.GetBodyByName(body_name));
    const std::vector<drake::geometry::GeometryId>& ramp_geoms =
        plant_lcs.GetCollisionGeometriesForBody(
            plant_lcs.GetBodyByName("ramp_link"));

    drake::geometry::GeometryId corner_1_sphere_geoms = object_geoms[1];
    drake::geometry::GeometryId corner_2_sphere_geoms = object_geoms[2];
    drake::geometry::GeometryId corner_3_sphere_geoms = object_geoms[3];
    drake::geometry::GeometryId corner_4_sphere_geoms = object_geoms[4];
    drake::geometry::GeometryId corner_5_sphere_geoms = object_geoms[5];
    drake::geometry::GeometryId corner_6_sphere_geoms = object_geoms[6];
    drake::geometry::GeometryId corner_7_sphere_geoms = object_geoms[7];

    contact_geoms["CORNER_1_SPHERE"] = corner_1_sphere_geoms;
    contact_geoms["CORNER_2_SPHERE"] = corner_2_sphere_geoms;
    contact_geoms["CORNER_3_SPHERE"] = corner_3_sphere_geoms;
    contact_geoms["CORNER_4_SPHERE"] = corner_4_sphere_geoms;
    contact_geoms["CORNER_5_SPHERE"] = corner_5_sphere_geoms;
    contact_geoms["CORNER_6_SPHERE"] = corner_6_sphere_geoms;
    contact_geoms["CORNER_7_SPHERE"] = corner_7_sphere_geoms;

    ground_object_contact_pairs.push_back(
        SortedPair(contact_geoms["CORNER_1_SPHERE"], contact_geoms["GROUND"]));
    ground_object_contact_pairs.push_back(
        SortedPair(contact_geoms["CORNER_2_SPHERE"], contact_geoms["GROUND"]));
    ground_object_contact_pairs.push_back(
        SortedPair(contact_geoms["CORNER_3_SPHERE"], contact_geoms["GROUND"]));
    ground_object_contact_pairs.push_back(
        SortedPair(contact_geoms["CORNER_4_SPHERE"], contact_geoms["GROUND"]));
    ground_object_contact_pairs.push_back(
        SortedPair(contact_geoms["CORNER_5_SPHERE"], contact_geoms["GROUND"]));
    ground_object_contact_pairs.push_back(
        SortedPair(contact_geoms["CORNER_6_SPHERE"], contact_geoms["GROUND"]));
    ground_object_contact_pairs.push_back(
        SortedPair(contact_geoms["CORNER_7_SPHERE"], contact_geoms["GROUND"]));

    for (int j = 0; j < ramp_geoms.size(); j++) {
      for (int k = 1; k < object_geoms.size(); k++) {
        ground_object_contact_pairs.push_back(
            SortedPair(ramp_geoms[j], object_geoms[k]));
      }
      ee_contact_pairs.push_back(
          SortedPair(contact_geoms["EE"], ramp_geoms[j]));
    }

    const std::vector<drake::geometry::GeometryId>
        object_geoms_without_spheres = std::vector<drake::geometry::GeometryId>(
            object_geoms.begin(), object_geoms.end() - 7);

    ee_contact_pairs.push_back(
        SortedPair(contact_geoms["EE"], object_geoms[0]));
    all_object_geoms.push_back(object_geoms_without_spheres);
  }

  // Object-object contact pairs (excluding end effector), each pair of
  // convex pieces for each pair of objects
  for (int i = 0; i + 1 < controller_params.num_objects; i++) {
    for (int j = i + 1; j < controller_params.num_objects; j++) {
      std::vector<SortedPair<GeometryId>> convex_piece_pairs;
      const std::vector<GeometryId>& object_1_geoms = all_object_geoms.at(i);
      const std::vector<GeometryId>& object_2_geoms = all_object_geoms.at(j);

      for (const auto& g1 : object_1_geoms) {
        for (const auto& g2 : object_2_geoms) {
          convex_piece_pairs.emplace_back(g1, g2);
        }
      }
      object_object_contact_pairs.push_back(std::move(convex_piece_pairs));
    }
  }
  // Order:  EE-ground, EE-object, object-ground, object-object, object-wall
  contact_pairs.push_back(ee_ground_contact);
  contact_pairs.push_back(ee_contact_pairs);
  contact_pairs.push_back(ground_object_contact_pairs);
  for (const auto& obj_obj_pair : object_object_contact_pairs) {
    contact_pairs.push_back(obj_obj_pair);
  }
  for (const auto& wall_obj_pair : wall_object_contact_pairs) {
    contact_pairs.push_back(wall_obj_pair);
  }

  // Piece together the diagram.
  DiagramBuilder<double> builder;

  assert(lcm_channel_params.object_state_channels.size() ==
         controller_params.num_objects);
  std::vector<LcmSubscriberSystem*> object_state_subs;
  for (int i = 0; i < controller_params.num_objects; ++i) {
    object_state_subs.push_back(
        builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_object_state>(
            lcm_channel_params.object_state_channels.at(i), &lcm)));
  }
  auto franka_state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_three_d_printer);

  std::vector<systems::ObjectStateReceiver*> object_state_receivers;
  for (int i = 0; i < object_indices.size(); ++i) {
    object_state_receivers.push_back(
        builder.AddSystem<systems::ObjectStateReceiver>(
            plant_lcs, object_indices_lcs.at(i)));
  }

  auto radio_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
          lcm_channel_params.radio_channel, &lcm));

  auto reduced_order_model_receiver =
      builder.AddSystem<systems::ThreeDPrinterKinematics>(
          plant_three_d_printer, three_d_printer_context.get(), plant_object,
          object_context.get(), k3dEndEffectorTipName,
          controller_params.base_names);

  std::unique_ptr<systems::SamplingC3GoalGenerator> target_generator =
      std::make_unique<systems::SamplingC3GoalGeneratorPlanar>(
          plant_object, controller_params.goal_params, object_indices);
  auto* control_target = builder.AddSystem(std::move(target_generator));

  // Input sizes are EE position (3), object pose (7), EE velocity (3), object
  // velocities (6).
  std::vector<int> input_sizes = {3};                        // ee position
  for (int i = 0; i < controller_params.num_objects; i++) {  // object pose
    input_sizes.push_back(7);
  }
  input_sizes.push_back(3);  // ee velocity
  for (int i = 0; i < controller_params.num_objects;
       i++) {  // object velocities
    input_sizes.push_back(6);
  }

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

  // Port 0 ee target
  builder.Connect(control_target->get_output_port_end_effector_target(),
                  target_state_mux->get_input_port(0));

  // Ports 1 to n object targets
  std::vector<const OutputPort<double>*> output_ports_object_target =
      control_target->get_output_ports_object_target();
  for (int i = 0; i < controller_params.num_objects; i++) {
    builder.Connect(*(output_ports_object_target.at(i)),
                    target_state_mux->get_input_port(i + 1));
  }

  builder.Connect(
      end_effector_zero_velocity_source
          ->get_output_port(),  // Port n+1 ee velo target
      target_state_mux->get_input_port(controller_params.num_objects + 1));

  // Ports (n+2) to (2n+1) object velo targets
  std::vector<const OutputPort<double>*> output_ports_object_velocity_target =
      control_target->get_output_ports_object_velocity_target();
  for (int i = 0; i < controller_params.num_objects; i++) {
    builder.Connect(*(output_ports_object_velocity_target.at(i)),
                    target_state_mux->get_input_port(
                        i + controller_params.num_objects + 2));
  }
  builder.Connect(control_target->get_output_port_target_gen_info(),
                  target_gen_info_publisher->get_input_port());

  // Port 0 ee target
  builder.Connect(control_target->get_output_port_end_effector_target(),
                  final_target_state_mux->get_input_port(0));

  // Ports 1 to n object targets
  std::vector<const OutputPort<double>*> output_ports_object_final_target =
      control_target->get_output_ports_object_final_target();
  for (int i = 0; i < controller_params.num_objects; i++) {
    builder.Connect(*(output_ports_object_final_target.at(i)),
                    final_target_state_mux->get_input_port(i + 1));
  }

  // Port n+1 ee velo target
  builder.Connect(end_effector_zero_velocity_source->get_output_port(),
                  final_target_state_mux->get_input_port(
                      controller_params.num_objects + 1));

  // Ports (n+2) to (2n+1) constant vector
  for (int i = 0; i < controller_params.num_objects; i++) {
    builder.Connect(object_zero_velocity_source->get_output_port(),
                    final_target_state_mux->get_input_port(
                        i + controller_params.num_objects + 2));
  }

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
      plant_lcs.num_positions(), "sample_buffer_sender");
  auto sample_buffer_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_sample_buffer>(
          lcm_channel_params.sample_buffer_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto unsuccessful_sample_buffer_sender =
      builder.AddSystem<systems::SampleBufferSender>(
          controller_params.sampling_params.N_unsuccessful_sample_buffer,
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

  std::vector<std::string> state_names = {"end_effector_x", "end_effector_y",
                                          "end_effector_z"};

  std::vector<std::string> object_pose_names = {
      "object_qw", "object_qx", "object_qy", "object_qz",
      "object_x",  "object_y",  "object_z"};
  std::vector<std::string> object_velo_names = {"object_wx", "object_wy",
                                                "object_wz", "object_vx",
                                                "object_vy", "object_vz"};
  for (int i = 0; i < controller_params.num_objects; i++) {
    for (int j = 0; j < object_pose_names.size(); j++) {
      std::string item = object_pose_names.at(j) + "_" + std::to_string(i);
      state_names.push_back(item);
    }
  }
  state_names.push_back("end_effector_vx");
  state_names.push_back("end_effector_vy");
  state_names.push_back("end_effector_vz");

  for (int i = 0; i < controller_params.num_objects; i++) {
    for (int j = 0; j < object_velo_names.size(); j++) {
      std::string item = object_velo_names.at(j) + "_" + std::to_string(i);
      state_names.push_back(item);
    }
  }

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

  builder.Connect(franka_state_receiver->get_output_port(),
                  reduced_order_model_receiver->get_input_port_printer_state());
  for (int i = 0; i < controller_params.num_objects; i++) {
    builder.Connect(object_state_subs.at(i)->get_output_port(),
                    object_state_receivers.at(i)->get_input_port());
  }

  std::vector<const drake::systems::InputPort<double>*>
      reduced_order_model_receivers =
          reduced_order_model_receiver->get_input_ports_object_state();
  for (int i = 0; i < controller_params.num_objects; i++) {
    builder.Connect(object_state_receivers.at(i)->get_output_port(),
                    *(reduced_order_model_receivers.at(i)));
  }

  builder.Connect(reduced_order_model_receiver->get_output_port_lcs_state(),
                  controller->get_input_port_lcs_state());

  std::vector<const drake::systems::InputPort<double>*>
      input_ports_object_state = control_target->get_input_ports_object_state();
  for (int i = 0; i < controller_params.num_objects; i++) {
    builder.Connect(object_state_receivers.at(i)->get_output_port(),
                    *(input_ports_object_state.at(i)));
  }

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
  builder.Connect(
      unsuccessful_sample_buffer_sender->get_output_port_sample_buffer(),
      unsuccessful_sample_buffer_publisher->get_input_port());
  builder.Connect(
      controller->get_output_port_unsuccessful_sample_buffer_configurations(),
      unsuccessful_sample_buffer_sender->get_input_port_samples());
  builder.Connect(
      controller->get_output_port_unsuccessful_sample_buffer_costs(),
      unsuccessful_sample_buffer_sender->get_input_port_sample_costs());

  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("sampling_c3_controller_" + FLAGS_demo_name));
  plant_lcs_diagram->set_name(("sampling_c3_lcs_plant" + FLAGS_demo_name));
  DrawAndSaveDiagramGraph(*owned_diagram);
  DrawAndSaveDiagramGraph(*plant_lcs_diagram);

  // Run lcm-driven simulation.  The buffer size argument is needed to ensure
  // the latest messages are used in the control loop.  See
  // https://github.com/DAIRLab/dairlib/pull/366 for more details.
  int lcm_buffer_size = 200;
  std::shared_ptr<Diagram<double>> shared_diagram = std::move(owned_diagram);
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, shared_diagram, franka_state_receiver,
      lcm_channel_params.robot_state_channel, true, lcm_buffer_size);

  LcmHandleSubscriptionsUntil(&lcm, [&]() {
    int total_count = 0;
    for (const auto& sub : object_state_subs) {
      if (sub->GetInternalMessageCount() <= 1) {
        return false;
      }
    }
    return true;
  });
  loop.Simulate();
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
