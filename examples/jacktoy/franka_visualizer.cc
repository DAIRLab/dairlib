#include <iostream>

#include <dairlib/lcmt_c3_forces.hpp>
#include <dairlib/lcmt_c3_state.hpp>
#include <dairlib/lcmt_timestamped_saved_traj.hpp>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/primitives/multiplexer.h>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "common/find_resource.h"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/jacktoy/parameters/franka_lcm_channels.h"
#include "examples/jacktoy/parameters/franka_sim_params.h"
#include "examples/jacktoy/parameters/franka_c3_controller_params.h"
#include "systems/controllers/sampling_params.h"
#include "multibody/com_pose_system.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"
#include "systems/trajectory_optimization/lcm_trajectory_systems.h"
#include "systems/visualization/lcm_visualization_systems.h"

#include "drake/common/find_resource.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "examples/jacktoy/systems/franka_kinematics.h"
#include "examples/jacktoy/systems/c3_mode_visualizer.h"
#include "drake/geometry/meshcat_visualizer_params.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

namespace dairlib {

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::systems::ObjectStateReceiver;
using dairlib::systems::RobotOutputReceiver;
using dairlib::systems::SubvectorPassThrough;
using drake::geometry::DrakeVisualizer;
using drake::geometry::SceneGraph;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::multibody::MultibodyPlant;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::multibody::UnitInertia;
using drake::systems::Simulator;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::rendering::MultibodyPositionToGeometryPose;

using drake::math::RigidTransform;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;

DEFINE_string(lcm_channels,
              "examples/jacktoy/parameters/lcm_channels_simulation.yaml",
              "Filepath containing lcm channels");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  FrankaSimParams sim_params = drake::yaml::LoadYamlFile<FrankaSimParams>(
      "examples/jacktoy/parameters/franka_sim_params.yaml");
  FrankaC3ControllerParams controller_params =
      drake::yaml::LoadYamlFile<FrankaC3ControllerParams>(
          "examples/jacktoy/parameters/franka_c3_controller_params.yaml");
     
  C3Options c3_options;
  SamplingC3SamplingParams sampling_params;
  int safety_mode_index = controller_params.run_in_safe_mode ? 0 : 1;
  std::string safety_mode_name = controller_params.run_in_safe_mode ? "safe" : "normal";
  std::cout << "Running in " << safety_mode_name << " mode" << std::endl;
  c3_options = drake::yaml::LoadYamlFile<C3Options>(
                controller_params.c3_options_file[safety_mode_index]);
  sampling_params = drake::yaml::LoadYamlFile<SamplingC3SamplingParams>(
                controller_params.sampling_params_file[safety_mode_index]);
                
  FrankaLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<FrankaLcmChannels>(FLAGS_lcm_channels);

  drake::systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

	// This plant is for visualizing the full franka on the meshcat visualizer.
	// This is not used for the simulation or for FK calculations for c3/repos 
	// mode switching visualization.
  MultibodyPlant<double> plant(0.0);

  Parser parser(&plant, &scene_graph);
  parser.SetAutoRenaming(true);
  drake::multibody::ModelInstanceIndex franka_index =
      parser.AddModels(drake::FindResourceOrThrow(sim_params.franka_model))[0];
  drake::multibody::ModelInstanceIndex ground_index =
      parser.AddModels(FindResourceOrThrow(sim_params.ground_model))[0];
  drake::multibody::ModelInstanceIndex platform_index =
      parser.AddModels(FindResourceOrThrow(sim_params.platform_model))[0];
  drake::multibody::ModelInstanceIndex end_effector_index =
      parser.AddModels(FindResourceOrThrow(sim_params.end_effector_model))[0];
  drake::multibody::ModelInstanceIndex jack_index =
      parser.AddModels(FindResourceOrThrow(sim_params.jack_model))[0];

  // All the urdfs have their origins at the world frame origin. We define all 
  // the offsets by welding the frames such that changing the offsets in 
  // the param file moves them to where we want in the world frame.
  // TODO: Do this in all the files.
  RigidTransform<double> T_EE_W = RigidTransform<double>(
      drake::math::RotationMatrix<double>(
        drake::math::RollPitchYaw<double>(3.1415, 0, 0)),
        sim_params.tool_attachment_frame);
  RigidTransform<double> X_F_P =
      RigidTransform<double>(drake::math::RotationMatrix<double>(),
                             sim_params.platform_franka_frame);
  RigidTransform<double> X_F_G_franka =
      RigidTransform<double>(drake::math::RotationMatrix<double>(),
                             sim_params.ground_franka_frame);

  // Create a rigid transform from the world frame to the panda_link0 frame.
  // Franka base is 2.45cm above the ground.
  RigidTransform<double> X_F_W = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), sim_params.franka_origin);

  plant.WeldFrames(plant.world_frame(), 
                   plant.GetFrameByName("panda_link0"), X_F_W);
  plant.WeldFrames(plant.GetFrameByName("panda_link7"), 
                   plant.GetFrameByName("end_effector_base"), T_EE_W);
  plant.WeldFrames(plant.GetFrameByName("panda_link0"),
                   plant.GetFrameByName("ground"), X_F_G_franka);
  plant.WeldFrames(plant.GetFrameByName("panda_link0"),
                   plant.GetFrameByName("platform"), X_F_P);

  plant.Finalize();


  // Loading the full franka model that will go into franka kinematics system
  // This needs to load the full franka and full end effector model.
	// Some of the frame definitions have been reused from the above plant so as 
	// to avoid having to redefine them.
	MultibodyPlant<double> plant_franka(0.0);
  Parser parser_franka(&plant_franka, nullptr);	
	parser_franka.AddModels(
		drake::FindResourceOrThrow(sim_params.franka_model))[0];
	parser_franka.AddModels(
		FindResourceOrThrow(sim_params.ground_model))[0];
	parser_franka.AddModels(
		FindResourceOrThrow(sim_params.platform_model))[0];
	parser_franka.AddModels(
		FindResourceOrThrow(sim_params.end_effector_model))[0];
			
  // All the urdfs have their origins at the world frame origin. We define all 
  // the offsets by welding the frames such that changing the offsets in 
  // the param file moves them to where we want in the world frame.
  // TODO: Do this in all the files.

  // Create a rigid transform from the world frame to the panda_link0 frame.
  // Franka base is 2.45cm above the ground.
  plant_franka.WeldFrames(plant_franka.world_frame(), 
                   plant_franka.GetFrameByName("panda_link0"), X_F_W);
  plant_franka.WeldFrames(plant_franka.GetFrameByName("panda_link7"), 
                   plant_franka.GetFrameByName("end_effector_base"), T_EE_W);
  plant_franka.WeldFrames(plant_franka.GetFrameByName("panda_link0"),
                   plant_franka.GetFrameByName("ground"), X_F_G_franka);
  plant_franka.WeldFrames(plant_franka.GetFrameByName("panda_link0"),
                   plant_franka.GetFrameByName("platform"), X_F_P);

  plant_franka.Finalize();
  auto franka_context = plant_franka.CreateDefaultContext();

  /// adding the jack model (TODO: Change to object instead of jack)
  MultibodyPlant<double> plant_jack(0.0);
  Parser parser_jack(&plant_jack, nullptr);
  parser_jack.AddModels(sim_params.jack_model);
  plant_jack.Finalize();
  auto jack_context = plant_jack.CreateDefaultContext();

  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();

  // Create state receiver.
  auto franka_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(
          lcm_channel_params.franka_state_channel, lcm));
  auto tray_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.object_state_channel, lcm));
//   auto box_state_sub =
//       builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_object_state>(
//           lcm_channel_params.box_state_channel, lcm));
  auto franka_state_receiver =
      builder.AddSystem<RobotOutputReceiver>(plant, franka_index);
  auto tray_state_receiver =
      builder.AddSystem<ObjectStateReceiver>(plant, jack_index);

  auto franka_passthrough = builder.AddSystem<SubvectorPassThrough>(
      franka_state_receiver->get_output_port(0).size(), 0,
      plant.num_positions(franka_index));
  auto robot_time_passthrough = builder.AddSystem<SubvectorPassThrough>(
      franka_state_receiver->get_output_port(0).size(),
      franka_state_receiver->get_output_port(0).size() - 1, 1);
  auto tray_passthrough = builder.AddSystem<SubvectorPassThrough>(
      tray_state_receiver->get_output_port(0).size(), 0,
      plant.num_positions(jack_index));

  std::vector<int> input_sizes = {plant.num_positions(franka_index),
                                  plant.num_positions(jack_index)};
  auto mux =
      builder.AddSystem<drake::systems::Multiplexer<double>>(input_sizes);

  // System that takes in the state of the franka and jack and outputs a
  // reduced order lcs state vector.
  auto reduced_order_model_receiver =
      builder.AddSystem<systems::FrankaKinematics>(
          plant_franka, franka_context.get(), plant_jack, jack_context.get(),
          sim_params.end_effector_name, 
          sim_params.object_body_name,
					false);
	builder.Connect(franka_state_receiver->get_output_port(),
		reduced_order_model_receiver->get_input_port_franka_state());
	builder.Connect(tray_state_receiver->get_output_port(),
		reduced_order_model_receiver->get_input_port_object_state());

	// This system subscribes to the lcmt_timestamped_saved_traj message containing
	auto is_c3_mode_sub = builder.AddSystem(
					LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
									lcm_channel_params.is_c3_mode_channel, lcm));

	// These systems subscribe to the c3 and repos trajectory execution channels
	// to visualize posible execution trajectories for the actor for either mode.
  auto c3_execution_trajectory_sub_actor = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.c3_trajectory_exec_actor_channel, lcm));
  auto repos_execution_trajectory_sub_actor = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.repos_trajectory_exec_actor_channel, lcm));

	// These systems subscribe to the c3 curr and best plan channels to visualize
	// the center of mass trajectories for the actor and object according to each
	// plan.
  auto trajectory_sub_actor_curr = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.c3_actor_curr_plan_channel, lcm));
  auto trajectory_sub_object_curr = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.c3_object_curr_plan_channel, lcm));
  auto trajectory_sub_force_curr =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_c3_forces>(
          lcm_channel_params.c3_force_curr_channel, lcm));

  auto trajectory_sub_actor_best = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.c3_actor_best_plan_channel, lcm));
  auto trajectory_sub_object_best = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.c3_object_best_plan_channel, lcm));
  auto trajectory_sub_force_best =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_c3_forces>(
          lcm_channel_params.c3_force_best_channel, lcm));

	// This system subscribes to the lcmt_saved_traj message containing sample 
	// locations. 
  auto sample_location_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.sample_locations_channel, lcm));

  auto c3_state_actual_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_c3_state>(
          lcm_channel_params.c3_actual_state_channel, lcm));
  auto c3_state_target_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_c3_state>(
          lcm_channel_params.c3_target_state_channel, lcm));
  auto to_pose =
      builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant);

  drake::geometry::MeshcatVisualizerParams params;
  params.publish_period = 1.0 / sim_params.visualizer_publish_rate;
  auto meshcat = std::make_shared<drake::geometry::Meshcat>();
  auto visualizer = &drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
      &builder, scene_graph, meshcat, std::move(params));
  meshcat->SetCameraPose(sim_params.camera_pose, sim_params.camera_target);

  if (sim_params.visualize_c3_workspace){
    double width = c3_options.world_x_limits[1] - c3_options.world_x_limits[0];
    double depth = c3_options.world_y_limits[1] - c3_options.world_y_limits[0];
    double height = c3_options.world_z_limits[1] - c3_options.world_z_limits[0];
    Vector3d workspace_center = {0.5 * (c3_options.world_x_limits[1] + c3_options.world_x_limits[0]),
                                 0.5 * (c3_options.world_y_limits[1] + c3_options.world_y_limits[0]),
                                 0.5 * (c3_options.world_z_limits[1] + c3_options.world_z_limits[0])};
    meshcat->SetObject("c3_state/c3_workspace", drake::geometry::Box(width, depth, height),
                       {0, 1, 0, 0.2});
    meshcat->SetTransform("c3_state/c3_workspace", RigidTransformd(workspace_center));
  }

  if (sim_params.visualize_execution_plan){
    auto c3_exec_trajectory_drawer_actor =
        builder.AddSystem<systems::LcmTrajectoryDrawer>(
            meshcat,"c3_exec_", "end_effector_position_target");
    auto repos_trajectory_drawer_actor =
        builder.AddSystem<systems::LcmTrajectoryDrawer>(meshcat,"repos_exec_",
                                                        "end_effector_position_target");
    c3_exec_trajectory_drawer_actor->SetLineColor(drake::geometry::Rgba({1, 0.75, 0.79, 1}));
    c3_exec_trajectory_drawer_actor->SetLineWidth(10000000);
    repos_trajectory_drawer_actor->SetLineColor(drake::geometry::Rgba({0, 0, 1, 1}));
    repos_trajectory_drawer_actor->SetLineWidth(10000000);
    c3_exec_trajectory_drawer_actor->SetNumSamples(5);
    repos_trajectory_drawer_actor->SetNumSamples(5);
    builder.Connect(c3_execution_trajectory_sub_actor->get_output_port(),
                    c3_exec_trajectory_drawer_actor->get_input_port_trajectory());
    builder.Connect(repos_execution_trajectory_sub_actor->get_output_port(),
                    repos_trajectory_drawer_actor->get_input_port_trajectory());
  }

  if (sim_params.visualize_center_of_mass_plan_curr){
    auto trajectory_drawer_actor_curr =
        builder.AddSystem<systems::LcmTrajectoryDrawer>(
            meshcat,"curr_", "end_effector_position_target");
    auto trajectory_drawer_object_curr =
        builder.AddSystem<systems::LcmTrajectoryDrawer>(meshcat,"curr_",
                                                        "object_position_target");
    trajectory_drawer_actor_curr->SetLineColor(drake::geometry::Rgba({1, 0, 0, 1}));
    trajectory_drawer_object_curr->SetLineColor(drake::geometry::Rgba({1, 0, 0, 1}));
    trajectory_drawer_actor_curr->SetNumSamples(5);
    trajectory_drawer_object_curr->SetNumSamples(5);
    builder.Connect(trajectory_sub_actor_curr->get_output_port(),
                    trajectory_drawer_actor_curr->get_input_port_trajectory());
    builder.Connect(trajectory_sub_object_curr->get_output_port(),
                    trajectory_drawer_object_curr->get_input_port_trajectory());
  }

  if (sim_params.visualize_center_of_mass_plan_best){
    auto trajectory_drawer_actor_best =
        builder.AddSystem<systems::LcmTrajectoryDrawer>(
            meshcat,"best_", "end_effector_position_target");
    auto trajectory_drawer_object_best =
        builder.AddSystem<systems::LcmTrajectoryDrawer>(meshcat,"best_",
                                                        "object_position_target");
    trajectory_drawer_actor_best->SetLineColor(drake::geometry::Rgba({0, 1, 0, 1}));
    trajectory_drawer_object_best->SetLineColor(drake::geometry::Rgba({0, 1, 0, 1}));
    trajectory_drawer_actor_best->SetNumSamples(5);
    trajectory_drawer_object_best->SetNumSamples(5);
    builder.Connect(trajectory_sub_actor_best->get_output_port(),
                    trajectory_drawer_actor_best->get_input_port_trajectory());
    builder.Connect(trajectory_sub_object_best->get_output_port(),
                    trajectory_drawer_object_best->get_input_port_trajectory());
  }

  if (sim_params.visualize_pose_trace_curr){
    auto object_pose_drawer_curr = builder.AddSystem<systems::LcmPoseDrawer>(
        meshcat, "curr_planned", FindResourceOrThrow(sim_params.visualizer_curr_sample_traj_jack_model),
        "object_position_target", "object_orientation_target", 5, false);
    // TODO: We might want this to be end_effector_simple_model
    auto end_effector_pose_drawer_curr = builder.AddSystem<systems::LcmPoseDrawer>(
        meshcat, "curr_planned", FindResourceOrThrow(sim_params.visualizer_curr_sample_end_effector_model),
        "end_effector_position_target", "end_effector_orientation_target", 5, false);

    builder.Connect(trajectory_sub_object_curr->get_output_port(),
                    object_pose_drawer_curr->get_input_port_trajectory());
    builder.Connect(trajectory_sub_actor_curr->get_output_port(),
                    end_effector_pose_drawer_curr->get_input_port_trajectory());
  }

  if (sim_params.visualize_pose_trace_best){
    auto object_pose_drawer_best = builder.AddSystem<systems::LcmPoseDrawer>(
        meshcat, "best_planned", FindResourceOrThrow(sim_params.visualizer_best_sample_traj_jack_model),
        "object_position_target", "object_orientation_target");
    // TODO: We might want this to be end_effector_simple_model
    auto end_effector_pose_drawer_best = builder.AddSystem<systems::LcmPoseDrawer>(
        meshcat, "best_planned", FindResourceOrThrow(sim_params.visualizer_best_sample_end_effector_model),
        "end_effector_position_target", "end_effector_orientation_target", 5, false);

    builder.Connect(trajectory_sub_object_best->get_output_port(),
                    object_pose_drawer_best->get_input_port_trajectory());
    builder.Connect(trajectory_sub_actor_best->get_output_port(),
                    end_effector_pose_drawer_best->get_input_port_trajectory());
  }

  if (sim_params.visualize_sample_locations){
		// This drawer object is used to visualize the sample locations.
		// This isn't designed to be used for visualizing sample locations but we 
		// use it for that purpose since the sample_location_sender sends out an 
		// lcmt_timestamped_traj with a trajectory by the name sample_locations.
		// The last argument "end_effector_orientation_target" is a dummy argument 
		// here that is not used.
    auto sample_locations_drawer = builder.AddSystem<systems::LcmPoseDrawer>(
        meshcat, "samples_", FindResourceOrThrow(sim_params.visualizer_sample_locations_model),
        "sample_locations", "end_effector_orientation_target", 
        std::max(sampling_params.num_additional_samples_c3, 
            sampling_params.num_additional_samples_repos) + 1, false);

    builder.Connect(sample_location_sub->get_output_port(),
                    sample_locations_drawer->get_input_port_trajectory());
  }

  if (sim_params.visualize_c3_state){
    auto c3_target_drawer =
        builder.AddSystem<systems::LcmC3TargetDrawer>(meshcat, true, true);
    builder.Connect(c3_state_actual_sub->get_output_port(),
                    c3_target_drawer->get_input_port_c3_state_actual());
    builder.Connect(c3_state_target_sub->get_output_port(),
                    c3_target_drawer->get_input_port_c3_state_target());
  }

  if (sim_params.visualize_c3_forces_curr){
    auto end_effector_force_drawer_curr = builder.AddSystem<systems::LcmForceDrawer>(
        meshcat, "curr_", "end_effector_position_target", "end_effector_force_target",
        "lcs_force_trajectory_curr");
    builder.Connect(trajectory_sub_actor_curr->get_output_port(),
                    end_effector_force_drawer_curr->get_input_port_actor_trajectory());
    builder.Connect(trajectory_sub_force_curr->get_output_port(),
                    end_effector_force_drawer_curr->get_input_port_force_trajectory());
    builder.Connect(robot_time_passthrough->get_output_port(),
                    end_effector_force_drawer_curr->get_input_port_robot_time());
  }

  if (sim_params.visualize_c3_forces_best){
    auto end_effector_force_drawer_best = builder.AddSystem<systems::LcmForceDrawer>(
        meshcat, "best_", "end_effector_position_target", "end_effector_force_target",
        "lcs_force_trajectory_best");
    builder.Connect(trajectory_sub_actor_best->get_output_port(),
                    end_effector_force_drawer_best->get_input_port_actor_trajectory());
    builder.Connect(trajectory_sub_force_best->get_output_port(),
                    end_effector_force_drawer_best->get_input_port_force_trajectory());
    builder.Connect(robot_time_passthrough->get_output_port(),
                    end_effector_force_drawer_best->get_input_port_robot_time());
  }

	if(sim_params.visualize_is_c3_mode){
		// TODO: Create a system that will read either the lcmt_timestamped_saved_traj
		// message containing the boolean value of whether the robot is in c3 mode
		// (if that doesn't work, then use a subscriber system to read the boolean).
		// This system will also take in the x_lcs output from the FK system and 
		// output an LcmTrajectory which will contain one knot point - either at the 
		// current end_effector location (if c3 mode) or at the base of the robot 
		// (if repos mode).
		auto c3_mode_visualizer = builder.AddSystem<systems::C3ModeVisualizer>();
		builder.Connect(is_c3_mode_sub->get_output_port(),
										c3_mode_visualizer->get_input_port_is_c3_mode());
		builder.Connect(reduced_order_model_receiver->get_output_port(),
										c3_mode_visualizer->get_input_port_curr_lcs_state());
    auto is_c3_mode_drawer = builder.AddSystem<systems::LcmPoseDrawer>(
        meshcat, "c3_mode", FindResourceOrThrow(sim_params.visualizer_c3_mode_model),
        "c3_mode_visualization", "end_effector_orientation_target", 1, false);
		builder.Connect(c3_mode_visualizer->get_output_port_c3_mode_visualization_traj(),
										is_c3_mode_drawer->get_input_port_trajectory());
	}

  builder.Connect(franka_passthrough->get_output_port(),
                  mux->get_input_port(0));
  builder.Connect(tray_passthrough->get_output_port(), mux->get_input_port(1));
  builder.Connect(*mux, *to_pose);
  builder.Connect(
      to_pose->get_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(*franka_state_receiver, *franka_passthrough);
  builder.Connect(*franka_state_receiver, *robot_time_passthrough);
  builder.Connect(*tray_state_receiver, *tray_passthrough);
  builder.Connect(*franka_state_sub, *franka_state_receiver);
  builder.Connect(*tray_state_sub, *tray_state_receiver);

  auto diagram = builder.Build();
  DrawAndSaveDiagramGraph(*diagram, "examples/jacktoy/visualizer_diagram");
  auto context = diagram->CreateDefaultContext();

  auto& franka_state_sub_context =
      diagram->GetMutableSubsystemContext(*franka_state_sub, context.get());
  auto& tray_state_sub_context =
      diagram->GetMutableSubsystemContext(*tray_state_sub, context.get());
//   auto& box_state_sub_context =
//       diagram->GetMutableSubsystemContext(*box_state_sub, context.get());
  franka_state_receiver->InitializeSubscriberPositions(
      plant, franka_state_sub_context);
  tray_state_receiver->InitializeSubscriberPositions(plant,
                                                     tray_state_sub_context);

  /// Use the simulator to drive at a fixed rate
  /// If set_publish_every_time_step is true, this publishes twice
  /// Set realtime rate. Otherwise, runs as fast as possible
  auto stepper =
      std::make_unique<Simulator<double>>(*diagram, std::move(context));
  stepper->set_publish_every_time_step(false);
  stepper->set_publish_at_initialization(false);
  stepper->set_target_realtime_rate(
      1.0);  // may need to change this to param.real_time_rate?
  stepper->Initialize();

  drake::log()->info("visualizer started");

  stepper->AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }
