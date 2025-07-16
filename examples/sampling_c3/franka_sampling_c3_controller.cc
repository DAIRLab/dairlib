
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
#include "goal_generator.h"
#include "examples/sampling_c3/sampling_c3_utils.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_controller_params.h"
#include "examples/sampling_c3/parameter_headers/lcm_channels.h"
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
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::InputPort;
using drake::systems::OutputPort;
using drake::systems::InputPortIndex;
using drake::systems::OutputPortIndex;
using Eigen::MatrixXd;
using drake::multibody::ModelInstanceIndex;

using Eigen::Vector3d;
using Eigen::VectorXd;
using multibody::MakeNameToPositionsMap;
using multibody::MakeNameToVelocitiesMap;
using std::vector;


DEFINE_bool(is_simulation, true, "True for simulation, false for hardware");
DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");
DEFINE_string(demo_name, "jacktoy",
              "Demo within sampling_c3; used to find controller params file");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::lcm::DrakeLcm lcm(FLAGS_lcm_url);

  // Load parameters.
  std::string controller_params_path = "examples/sampling_c3/" +
    FLAGS_demo_name + "/parameters/sampling_c3_controller_params.yaml";
  SamplingC3ControllerParams controller_params =
      drake::yaml::LoadYamlFile<SamplingC3ControllerParams>(
          controller_params_path);
  std::string lcm_channels_file = FLAGS_is_simulation ?
      controller_params.lcm_channels_simulation_file :
      controller_params.lcm_channels_hardware_file;
  SamplingC3LcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<SamplingC3LcmChannels>(lcm_channels_file);

  // Create a Franka-only plant.
  MultibodyPlant<double> plant_franka(0.0);
  AddFrankaToPlant(&plant_franka);
  plant_franka.Finalize();
  auto franka_context = plant_franka.CreateDefaultContext();

  // Create an object-only plant.
  MultibodyPlant<double> plant_object(0.0);
  std::vector<ModelInstanceIndex> object_indices = AddObjectsToPlant(
    &plant_object, nullptr, controller_params.object_models);

	
	// exclude ee and ground
	//std::vector<ModelInstanceIndex> object_indices(full_object_indices.begin()+2, full_object_indices.end()); 
  plant_object.Finalize();
  auto object_context = plant_object.CreateDefaultContext();


  // Create the LCS plant containing a floating EE, object, and ground.
  DiagramBuilder<double> plant_lcs_builder;
  auto [plant_lcs, scene_graph] =
    AddMultibodyPlantSceneGraph(&plant_lcs_builder, 0.0);
  std::vector<ModelInstanceIndex> object_indices_lcs = 
	AddLCSModelsToPlant(&plant_lcs, &scene_graph, controller_params.object_models,
                  controller_params.include_end_effector_orientation);


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

  std::vector<SortedPair<GeometryId>> object_object_contact_pairs;

  if (FLAGS_demo_name == "jacktoy") {
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

    contact_geoms["CAPSULE_1"] = capsule1_geoms;
    contact_geoms["CAPSULE_2"] = capsule2_geoms;
    contact_geoms["CAPSULE_3"] = capsule3_geoms;
    contact_geoms["CAPSULE_1_SPHERE_1"] = capsule1_sphere1_geoms;
    contact_geoms["CAPSULE_1_SPHERE_2"] = capsule1_sphere2_geoms;
    contact_geoms["CAPSULE_2_SPHERE_1"] = capsule2_sphere1_geoms;
    contact_geoms["CAPSULE_2_SPHERE_2"] = capsule2_sphere2_geoms;
    contact_geoms["CAPSULE_3_SPHERE_1"] = capsule3_sphere1_geoms;
    contact_geoms["CAPSULE_3_SPHERE_2"] = capsule3_sphere2_geoms;

    ee_contact_pairs.push_back(
        SortedPair(contact_geoms["EE"], contact_geoms["CAPSULE_1"]));
    ee_contact_pairs.push_back(
        SortedPair(contact_geoms["EE"], contact_geoms["CAPSULE_2"]));
    ee_contact_pairs.push_back(
        SortedPair(contact_geoms["EE"], contact_geoms["CAPSULE_3"]));

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
  }
  else if (FLAGS_demo_name == "push_t") {
    drake::geometry::GeometryId vertical_geoms =
        plant_lcs.GetCollisionGeometriesForBody(
            plant_lcs.GetBodyByName("vertical_link"))[0];
    drake::geometry::GeometryId horizontal_geoms =
        plant_lcs.GetCollisionGeometriesForBody(
            plant_lcs.GetBodyByName("horizontal_link"))[0];

    drake::geometry::GeometryId top_left_sphere_geoms =
        plant_lcs.GetCollisionGeometriesForBody(
            plant_lcs.GetBodyByName("vertical_link"))[1];
    drake::geometry::GeometryId top_right_sphere_geoms =
        plant_lcs.GetCollisionGeometriesForBody(
            plant_lcs.GetBodyByName("vertical_link"))[2];
    drake::geometry::GeometryId bottom_sphere_geoms =
        plant_lcs.GetCollisionGeometriesForBody(
            plant_lcs.GetBodyByName("vertical_link"))[3];

    contact_geoms["VERTICAL_LINK"] = vertical_geoms;
    contact_geoms["HORIZONTAL_LINK"] = horizontal_geoms;
    contact_geoms["TOP_LEFT_SPHERE"] = top_left_sphere_geoms;
    contact_geoms["TOP_RIGHT_SPHERE"] = top_right_sphere_geoms;
    contact_geoms["BOTTOM_SPHERE"] = bottom_sphere_geoms;

    ee_contact_pairs.push_back(
        SortedPair(contact_geoms["EE"], contact_geoms["HORIZONTAL_LINK"]));
    ee_contact_pairs.push_back(
        SortedPair(contact_geoms["EE"], contact_geoms["VERTICAL_LINK"]));

    ground_object_contact_pairs.push_back(SortedPair(
        contact_geoms["TOP_LEFT_SPHERE"], contact_geoms["GROUND"]));
    ground_object_contact_pairs.push_back(SortedPair(
        contact_geoms["TOP_RIGHT_SPHERE"], contact_geoms["GROUND"]));
    ground_object_contact_pairs.push_back(SortedPair(
        contact_geoms["BOTTOM_SPHERE"], contact_geoms["GROUND"]));
  }
  else if (FLAGS_demo_name == "anything") {
		for (int i = 0; i < object_indices_lcs.size(); i++) { // exclude ee/ground
			ModelInstanceIndex object_index = object_indices_lcs.at(i);
			drake::geometry::GeometryId mesh_geoms =
					plant_lcs.GetCollisionGeometriesForBody(
							plant_lcs.GetBodyByName("body", object_index))[0];

			drake::geometry::GeometryId top_left_sphere_geoms =
					plant_lcs.GetCollisionGeometriesForBody(
							plant_lcs.GetBodyByName("body", object_index))[1];
			drake::geometry::GeometryId top_right_sphere_geoms =
					plant_lcs.GetCollisionGeometriesForBody(
							plant_lcs.GetBodyByName("body", object_index))[2];
			drake::geometry::GeometryId bottom_sphere_geoms =
					plant_lcs.GetCollisionGeometriesForBody(
							plant_lcs.GetBodyByName("body", object_index))[3];

			contact_geoms["OBJECT_MESH_" + std::to_string(i)] = mesh_geoms;
			contact_geoms["TOP_LEFT_SPHERE_" + std::to_string(i)] = top_left_sphere_geoms;
			contact_geoms["TOP_RIGHT_SPHERE_" + std::to_string(i)] = top_right_sphere_geoms;
			contact_geoms["BOTTOM_SPHERE_" + std::to_string(i)] = bottom_sphere_geoms;

			ee_contact_pairs.push_back(
					SortedPair(contact_geoms["EE"], contact_geoms["OBJECT_MESH_" + std::to_string(i)]));

			ground_object_contact_pairs.push_back(SortedPair(
					contact_geoms["TOP_LEFT_SPHERE_" + std::to_string(i)], contact_geoms["GROUND"]));
			ground_object_contact_pairs.push_back(SortedPair(
					contact_geoms["TOP_RIGHT_SPHERE_" + std::to_string(i)], contact_geoms["GROUND"]));
			ground_object_contact_pairs.push_back(SortedPair(
					contact_geoms["BOTTOM_SPHERE_" + std::to_string(i)], contact_geoms["GROUND"]));
		} 
		// Object-object contact pairs (excluding end effector)
		for (int i = 0; i < controller_params.num_objects; i++) {
			for (int j = 0; j < controller_params.num_objects; j++) {
					if (j <= i) continue;

					std::string key1 = "OBJECT_MESH_" + std::to_string(i);
					std::string key2 = "OBJECT_MESH_" + std::to_string(j);

					object_object_contact_pairs.push_back(SortedPair(contact_geoms[key1], contact_geoms[key2]));
			}
		}
    
  }
  else {
    throw std::runtime_error("Unknown --demo_name value: " + FLAGS_demo_name);
  }
  // Order:  EE-ground, EE-object, object-ground.
  contact_pairs.push_back(ee_ground_contact);
  contact_pairs.push_back(ee_contact_pairs);
  contact_pairs.push_back(ground_object_contact_pairs);
  contact_pairs.push_back(object_object_contact_pairs);


  // Piece together the diagram.
  DiagramBuilder<double> builder;
	
	std::vector<LcmSubscriberSystem*> object_state_subs;

	for (int i = 0; i < controller_params.num_objects; ++i) {
			object_state_subs.push_back(builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_object_state>(
							lcm_channel_params.object_state_channels.at(i), &lcm)));
	}
			
  auto franka_state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_franka);

  std::vector<systems::ObjectStateReceiver*> object_state_receivers;
  for (int i = 0; i < object_indices.size(); ++i) {
    object_state_receivers.push_back(builder.AddSystem<systems::ObjectStateReceiver>(plant_object, object_indices.at(i)));
  }

  auto radio_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
          lcm_channel_params.radio_channel, &lcm));

  auto reduced_order_model_receiver =
      builder.AddSystem<systems::FrankaKinematics>(
          plant_franka, franka_context.get(), plant_object,
          object_context.get(), kEndEffectorName,
          controller_params.object_body_name,
          controller_params.include_end_effector_orientation,
					object_indices
				);

  // Select the target generator based on the demo.
  std::unique_ptr<systems::SamplingC3GoalGenerator> target_generator;
  if (FLAGS_demo_name == "jacktoy") {
    target_generator = 
			std::make_unique<systems::SamplingC3GoalGeneratorJacktoy>(
					plant_object, controller_params.goal_params, object_indices);
  } else if (FLAGS_demo_name == "push_t") {
    target_generator =
			std::make_unique<systems::SamplingC3GoalGeneratorPlanar>(
					plant_object, controller_params.goal_params, object_indices);
  } else if (FLAGS_demo_name == "anything") {
		target_generator =
			std::make_unique<systems::SamplingC3GoalGeneratorPlanar>(
					plant_object, controller_params.goal_params, object_indices);
			
	} else {
    throw std::runtime_error("Unknown --demo_name value: " + FLAGS_demo_name);
  }
	auto* control_target = builder.AddSystem(std::move(target_generator));

  // Input sizes are EE position (3), object pose (7), EE velocity (3), object
  // velocities (6).
  std::vector<int> input_sizes = {3}; 											// ee position
	for (int i = 0; i < controller_params.num_objects; i++) { // object pose
		input_sizes.push_back(7);
	} 
	input_sizes.push_back(3); 																// ee velocity
	for (int i = 0; i < controller_params.num_objects; i++) { // object velocities
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

  builder.Connect(end_effector_zero_velocity_source->get_output_port(),	  // Port n+1 ee velo target
                  target_state_mux->get_input_port(controller_params.num_objects + 1));
						
	// Ports (n+2) to (2n+1) object velo targets
	std::vector<const OutputPort<double>*> output_ports_object_velocity_target = 			
		control_target->get_output_ports_object_velocity_target();
	for (int i = 0; i < controller_params.num_objects; i++) {
  	builder.Connect(*(output_ports_object_velocity_target.at(i)),
                  target_state_mux->get_input_port(i + controller_params.num_objects + 2));
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
                  final_target_state_mux->get_input_port(controller_params.num_objects + 1));

	// Ports (n+2) to (2n+1) constant vector
	for (int i = 0; i < controller_params.num_objects; i++) {								
			builder.Connect(object_zero_velocity_source->get_output_port(),
																			final_target_state_mux->get_input_port(i + controller_params.num_objects + 2));
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
      "end_effector_x",  "end_effector_y",  "end_effector_z"};
	
	std::vector<std::string> object_pose_names = {"object_qw", "object_qx", "object_qy", "object_qz", "object_x", "object_y", "object_z"};
	std::vector<std::string> object_velo_names = {"object_wx", "object_wy", "object_wz", "object_vx", "object_vy", "object_vz"};
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
	for (int i = 0; i < controller_params.num_objects; i++) {
			builder.Connect(object_state_subs.at(i)->get_output_port(),
																			object_state_receivers.at(i)->get_input_port());
	}

	std::vector<const drake::systems::InputPort<double>*> reduced_order_model_receivers = 
			reduced_order_model_receiver->get_input_ports_object_state();
	for (int i = 0; i < controller_params.num_objects; i++) {
			builder.Connect(object_state_receivers.at(i)->get_output_port(),
																			*(reduced_order_model_receivers.at(i)));
	} 

  builder.Connect(reduced_order_model_receiver->get_output_port(),
                  controller->get_input_port_lcs_state());

	std::vector<const drake::systems::InputPort<double>*> input_ports_object_state = 
		control_target->get_input_ports_object_state();
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

  std::cout << "Before drawandsave" << std::endl;
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
      lcm_channel_params.franka_state_channel, true, lcm_buffer_size);
	std::cout << "constructed loop" << std::endl;

	LcmHandleSubscriptionsUntil(
			&lcm,
			[&]() {
				int total_count = 0;
				for (const auto& sub : object_state_subs) {
					if (sub->GetInternalMessageCount() < 1) {
						return false;
					}
				}
				return true;
				});
    std::cout << "LcmHandleSubscriptionsUntil finished" << std::endl;
	//std::this_thread::sleep_for(std::chrono::seconds(30));
  loop.Simulate();
  return 0;
}

}  // namespace dairlib



int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
