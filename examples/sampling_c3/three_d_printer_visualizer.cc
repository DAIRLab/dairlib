#include <string>

#include <c3/lcmt_contact_forces.hpp>
#include <dairlib/lcmt_c3_state.hpp>
#include <dairlib/lcmt_sample_buffer.hpp>
#include <dairlib/lcmt_timestamped_saved_traj.hpp>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "common/find_resource.h"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/sampling_c3/c3_mode_visualizer.h"
#include "examples/sampling_c3/parameter_headers/lcm_channels.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_controller_params.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_options.h"
#include "examples/sampling_c3/parameter_headers/sampling_params.h"
#include "examples/sampling_c3/parameter_headers/visualizer_params.h"
#include "examples/sampling_c3/sampling_c3_utils.h"
#include "multibody/com_pose_system.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"
#include "systems/senders/sample_buffer_to_point_cloud.h"
#include "systems/system_utils.h"
#include "systems/three_d_printer_kinematics.h"
#include "systems/trajectory_optimization/lcm_trajectory_systems.h"
#include "systems/visualization/lcm_visualization_systems.h"
#include "systems/visualization/static_visualization_systems.h"

#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/meshcat_point_cloud_visualizer.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/meshcat_visualizer_params.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

namespace dairlib {

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::systems::ObjectStateReceiver;
using dairlib::systems::RobotOutputReceiver;
using dairlib::systems::SubvectorPassThrough;
using drake::geometry::DrakeVisualizer;
using drake::geometry::MeshcatPointCloudVisualizer;
using drake::geometry::SceneGraph;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::multibody::ModelInstanceIndex;
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

DEFINE_bool(is_simulation, true, "True for simulation, false for hardware");
DEFINE_string(demo_name, "cone",
              "Name for the sampling_c3/three_d_printer demo, used when "
              "building filepaths for output.");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_demo_name != "cone") {
    throw std::runtime_error("Unknown --demo_name value: " + FLAGS_demo_name);
  }

  // Controller params (root config).
  std::string controller_params_path =
      "examples/sampling_c3/three_d_printer/" + FLAGS_demo_name +
      "/parameters/sampling_c3_controller_params.yaml";
  auto controller_params =
      drake::yaml::LoadYamlFile<SamplingC3ControllerParams>(
          controller_params_path);

  auto vis_params = drake::yaml::LoadYamlFile<SamplingC3VisualizerParams>(
      controller_params.vis_params_file);

  SamplingC3Options sampling_c3_options = controller_params.sampling_c3_options;

  SamplingParams sampling_params = controller_params.sampling_params;

  std::string lcm_channels_file =
      FLAGS_is_simulation ? controller_params.lcm_channels_simulation_file
                          : controller_params.lcm_channels_hardware_file;

  auto lcm_channel_params =
      drake::yaml::LoadYamlFile<SamplingC3LcmChannels>(lcm_channels_file);

  drake::systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  // Build the visualizer plant.
  MultibodyPlant<double> plant(0.0);
  ModelInstanceIndex printer_index =
      Add3DPrinterToPlant(&plant, &scene_graph, true);

  // Getting vector of object indices for all objects
  std::vector<ModelInstanceIndex> object_indices_plant =
      AddObjectsToPlant(&plant, &scene_graph, vis_params.object_vis_models);
  plant.Finalize();

  // Create a printer-only plant.
  MultibodyPlant<double> plant_printer(0.0);
  Add3DPrinterToPlant(&plant_printer, nullptr, true);
  plant_printer.Finalize();
  auto printer_context = plant_printer.CreateDefaultContext();

  // Create an object-only plant.
  MultibodyPlant<double> plant_object(0.0);
  std::vector<ModelInstanceIndex> object_indices =
      AddObjectsToPlant(&plant_object, nullptr, vis_params.object_vis_models);
  plant_object.Finalize();
  auto object_context = plant_object.CreateDefaultContext();

  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();
  auto three_d_printer_state_receiver =
      builder.AddSystem<RobotOutputReceiver>(plant, printer_index);

  // Duplicating object state reciever for each object
  std::vector<ObjectStateReceiver*> object_state_receivers;
  for (ModelInstanceIndex obj_index : object_indices_plant) {
    object_state_receivers.push_back(
        builder.AddSystem<ObjectStateReceiver>(plant, obj_index));
  }

  auto three_d_printer_passthrough = builder.AddSystem<SubvectorPassThrough>(
      three_d_printer_state_receiver->get_output_port(0).size(), 0,
      plant.num_positions(printer_index));
  auto robot_time_passthrough = builder.AddSystem<SubvectorPassThrough>(
      three_d_printer_state_receiver->get_output_port(0).size(),
      three_d_printer_state_receiver->get_output_port(0).size() - 1, 1);

  // Duplicating passthrough for each object
  std::vector<SubvectorPassThrough<double>*> tray_passthroughs;
  for (int i = 0; i < object_state_receivers.size(); i++) {
    tray_passthroughs.push_back(builder.AddSystem<SubvectorPassThrough>(
        object_state_receivers.at(i)->get_output_port(0).size(), 0,
        plant.num_positions(object_indices_plant.at(i))));
  }

  std::vector<int> input_sizes = {plant.num_positions(printer_index)};
  for (ModelInstanceIndex obj_index : object_indices_plant) {
    input_sizes.push_back(plant.num_positions(obj_index));
  }

  auto mux =
      builder.AddSystem<drake::systems::Multiplexer<double>>(input_sizes);
  auto reduced_order_model_receiver =
      builder.AddSystem<systems::ThreeDPrinterKinematics>(
          plant_printer, printer_context.get(), plant_object,
          object_context.get(), k3dEndEffectorTipName,
          controller_params.base_names);

  builder.Connect(three_d_printer_state_receiver->get_output_port(),
                  reduced_order_model_receiver->get_input_port_printer_state());

  std::vector<const drake::systems::InputPort<double>*> ro_model_object_inputs =
      reduced_order_model_receiver->get_input_ports_object_state();
  for (int i = 0; i < object_state_receivers.size(); i++) {
    builder.Connect(object_state_receivers.at(i)->get_output_port(),
                    *(ro_model_object_inputs.at(i)));
  }

  // LCM subscribers.
  auto three_d_printer_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(
          lcm_channel_params.robot_state_channel, lcm));

  std::vector<LcmSubscriberSystem*> object_state_subs;
  for (int i = 0; i < object_state_receivers.size(); i++) {
    object_state_subs.push_back(
        builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_object_state>(
            lcm_channel_params.object_state_channels.at(i), lcm)));
  }

  auto is_c3_mode_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.is_c3_mode_channel, lcm));

  auto c3_execution_trajectory_sub_actor = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.c3_trajectory_exec_actor_channel, lcm));
  auto repos_execution_trajectory_sub_actor = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.repos_trajectory_exec_actor_channel, lcm));

  auto trajectory_sub_actor_curr = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.c3_actor_curr_plan_channel, lcm));
  auto trajectory_sub_object_curr = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.c3_object_curr_plan_channel, lcm));
  auto trajectory_sub_force_curr =
      builder.AddSystem(LcmSubscriberSystem::Make<c3::lcmt_contact_forces>(
          lcm_channel_params.c3_force_curr_channel, lcm));
  auto dynamically_feasible_trajectory_sub_object_curr = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.dynamically_feasible_curr_plan_channel, lcm));
  auto dynamically_feasible_trajectory_sub_actor_curr = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.dynamically_feasible_curr_actor_plan_channel,
          lcm));

  auto trajectory_sub_actor_best = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.c3_actor_best_plan_channel, lcm));
  auto trajectory_sub_object_best = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.c3_object_best_plan_channel, lcm));
  auto trajectory_sub_force_best =
      builder.AddSystem(LcmSubscriberSystem::Make<c3::lcmt_contact_forces>(
          lcm_channel_params.c3_force_best_channel, lcm));
  auto dynamically_feasible_trajectory_sub_object_best = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.dynamically_feasible_best_plan_channel, lcm));
  auto dynamically_feasible_trajectory_sub_actor_best = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.dynamically_feasible_best_actor_plan_channel,
          lcm));

  auto sample_location_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.sample_locations_channel, lcm));
  auto sample_buffer_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_sample_buffer>(
          lcm_channel_params.sample_buffer_channel, lcm));
  auto sample_costs_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.sample_costs_channel, lcm));

  auto c3_state_actual_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_c3_state>(
          lcm_channel_params.c3_actual_state_channel, lcm));
  auto c3_state_target_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_c3_state>(
          lcm_channel_params.c3_target_state_channel, lcm));
  auto c3_final_state_target_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_c3_state>(
          lcm_channel_params.c3_final_target_state_channel, lcm));

  auto to_pose =
      builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant);

  drake::geometry::MeshcatVisualizerParams params;
  params.publish_period = 1.0 / vis_params.visualizer_publish_rate;
  auto meshcat = std::make_shared<drake::geometry::Meshcat>();
  meshcat->SetCameraPose(vis_params.camera_pose, vis_params.camera_target);

  // Render the per-goal keep-out regions, nested under keep_out/goal_{i} so
  // they can be toggled per goal in meshcat).
  systems::StaticModelDrawer keep_out_drawer(meshcat);
  if (vis_params.visualize_keep_out_regions &&
      controller_params.keep_out_model_sequence.has_value()) {
    const auto& keep_out_models = *controller_params.keep_out_model_sequence;
    for (int i = 0; i < static_cast<int>(keep_out_models.size()); ++i) {
      if (keep_out_models.at(i).empty()) {
        continue;  // No keep-out regions for this goal step.
      }
      keep_out_drawer.AddModel(
          FindResourceOrThrow(keep_out_models.at(i)), "keep_out_base",
          "keep_out/goal_" + std::to_string(i), vis_params.keep_out_color);
    }
  }

  if (vis_params.visualize_c3_workspace) {
    // Note:  There are also robot radius limits which are not visualized.
    double width = sampling_c3_options.workspace_limits[0][4] -
                   sampling_c3_options.workspace_limits[0][3];  // x
    double depth = sampling_c3_options.workspace_limits[1][4] -
                   sampling_c3_options.workspace_limits[1][3];  // y
    double height = sampling_c3_options.workspace_limits[2][4] -
                    sampling_c3_options.workspace_limits[2][3];  // z
    Vector3d workspace_center(
        0.5 * (sampling_c3_options.workspace_limits[0][4] +
               sampling_c3_options.workspace_limits[0][3]),
        0.5 * (sampling_c3_options.workspace_limits[1][4] +
               sampling_c3_options.workspace_limits[1][3]),
        0.5 * (sampling_c3_options.workspace_limits[2][4] +
               sampling_c3_options.workspace_limits[2][3]));
    meshcat->SetObject("c3_workspace",
                       drake::geometry::Box(width, depth, height),
                       {0, 1, 0, 0.2});
    meshcat->SetTransform("c3_workspace", RigidTransformd(workspace_center));
  }

  if (vis_params.visualize_execution_plan) {
    auto c3_exec_trajectory_drawer_actor =
        builder.AddSystem<systems::LcmTrajectoryDrawer>(
            meshcat, "end_effector_position_target", "c3_exec_");
    auto repos_trajectory_drawer_actor =
        builder.AddSystem<systems::LcmTrajectoryDrawer>(
            meshcat, "end_effector_position_target", "repos_exec_");
    c3_exec_trajectory_drawer_actor->SetLineColor(
        drake::geometry::Rgba({1, 0.75, 0.79, 1}));
    c3_exec_trajectory_drawer_actor->SetLineWidth(10000000);
    repos_trajectory_drawer_actor->SetLineColor(
        drake::geometry::Rgba({0, 0, 1, 1}));
    repos_trajectory_drawer_actor->SetLineWidth(10000000);
    c3_exec_trajectory_drawer_actor->SetNumSamples(sampling_c3_options.N);
    repos_trajectory_drawer_actor->SetNumSamples(sampling_c3_options.N);
    builder.Connect(
        c3_execution_trajectory_sub_actor->get_output_port(),
        c3_exec_trajectory_drawer_actor->get_input_port_trajectory());
    builder.Connect(repos_execution_trajectory_sub_actor->get_output_port(),
                    repos_trajectory_drawer_actor->get_input_port_trajectory());
  }

  if (vis_params.visualize_center_of_mass_plan_curr) {
    auto trajectory_drawer_actor_curr =
        builder.AddSystem<systems::LcmTrajectoryDrawer>(
            meshcat, "end_effector_position_target", "curr_");
    auto trajectory_drawer_object_curr =
        builder.AddSystem<systems::LcmTrajectoryDrawer>(
            meshcat, "object_position_target", "curr_");
    trajectory_drawer_actor_curr->SetLineColor(
        drake::geometry::Rgba({1, 0, 0, 1}));
    trajectory_drawer_object_curr->SetLineColor(
        drake::geometry::Rgba({1, 0, 0, 1}));
    trajectory_drawer_actor_curr->SetNumSamples(sampling_c3_options.N);
    trajectory_drawer_object_curr->SetNumSamples(sampling_c3_options.N);
    builder.Connect(trajectory_sub_actor_curr->get_output_port(),
                    trajectory_drawer_actor_curr->get_input_port_trajectory());
    builder.Connect(trajectory_sub_object_curr->get_output_port(),
                    trajectory_drawer_object_curr->get_input_port_trajectory());
  }

  if (vis_params.visualize_center_of_mass_plan_best) {
    auto trajectory_drawer_actor_best =
        builder.AddSystem<systems::LcmTrajectoryDrawer>(
            meshcat, "end_effector_position_target", "best_");
    auto trajectory_drawer_object_best =
        builder.AddSystem<systems::LcmTrajectoryDrawer>(
            meshcat, "object_position_target", "best_");
    trajectory_drawer_actor_best->SetLineColor(
        drake::geometry::Rgba({0, 1, 0, 1}));
    trajectory_drawer_object_best->SetLineColor(
        drake::geometry::Rgba({0, 1, 0, 1}));
    trajectory_drawer_actor_best->SetNumSamples(sampling_c3_options.N);
    trajectory_drawer_object_best->SetNumSamples(sampling_c3_options.N);
    builder.Connect(trajectory_sub_actor_best->get_output_port(),
                    trajectory_drawer_actor_best->get_input_port_trajectory());
    builder.Connect(trajectory_sub_object_best->get_output_port(),
                    trajectory_drawer_object_best->get_input_port_trajectory());
  }
  std::vector<std::string> position_names;
  std::vector<std::string> orientation_names;

  for (int i = 0; i < object_indices.size(); i++) {
    position_names.push_back("object_position_target_" + std::to_string(i));
    orientation_names.push_back("object_orientation_target_" +
                                std::to_string(i));
  }

  if (vis_params.visualize_c3_plan_curr) {
    auto object_pose_drawer_curr = builder.AddSystem<systems::LcmPoseDrawer>(
        meshcat, vis_params.object_vis_models, position_names,
        orientation_names, "plans/curr_planned", sampling_c3_options.N, true,
        vis_params.c3_curr_object_color);

    builder.Connect(trajectory_sub_object_curr->get_output_port(),
                    object_pose_drawer_curr->get_input_port_trajectory());

    auto end_effector_pose_drawer_curr =
        builder.AddSystem<systems::LcmPoseDrawer>(
            meshcat, FindResourceOrThrow(vis_params.ee_vis_model),
            "end_effector_position_target", "end_effector_orientation_target",
            "plans/curr_planned", sampling_c3_options.N, false,
            vis_params.c3_curr_ee_color);
    builder.Connect(trajectory_sub_actor_curr->get_output_port(),
                    end_effector_pose_drawer_curr->get_input_port_trajectory());

    auto dynamically_feasible_object_pose_drawer_curr =
        builder.AddSystem<systems::LcmPoseDrawer>(
            meshcat, vis_params.object_vis_models, position_names,
            orientation_names, "plans/dynamically_feasible_curr_plan",
            sampling_c3_options.N + 1, true, vis_params.df_curr_object_color);
    builder.Connect(
        dynamically_feasible_trajectory_sub_object_curr->get_output_port(),
        dynamically_feasible_object_pose_drawer_curr
            ->get_input_port_trajectory());

    auto dynamically_feasible_actor_pose_drawer_curr_actor =
        builder.AddSystem<systems::LcmPoseDrawer>(
            meshcat, FindResourceOrThrow(vis_params.ee_vis_model),
            "ee_position_target", "end_effector_orientation_target",
            "plans/dynamically_feasible_curr_plan", sampling_c3_options.N + 1,
            false, vis_params.df_curr_ee_color);
    builder.Connect(
        dynamically_feasible_trajectory_sub_actor_curr->get_output_port(),
        dynamically_feasible_actor_pose_drawer_curr_actor
            ->get_input_port_trajectory());
  }

  if (vis_params.visualize_c3_plan_best) {
    auto object_pose_drawer_best = builder.AddSystem<systems::LcmPoseDrawer>(
        meshcat, vis_params.object_vis_models, position_names,
        orientation_names, "plans/best_planned", sampling_c3_options.N, true,
        vis_params.c3_best_object_color);
    builder.Connect(trajectory_sub_object_best->get_output_port(),
                    object_pose_drawer_best->get_input_port_trajectory());

    auto end_effector_pose_drawer_best =
        builder.AddSystem<systems::LcmPoseDrawer>(
            meshcat, FindResourceOrThrow(vis_params.ee_vis_model),
            "end_effector_position_target", "end_effector_orientation_target",
            "plans/best_planned", sampling_c3_options.N, false,
            vis_params.c3_best_ee_color);
    builder.Connect(trajectory_sub_actor_best->get_output_port(),
                    end_effector_pose_drawer_best->get_input_port_trajectory());

    auto dynamically_feasible_object_pose_drawer_best =
        builder.AddSystem<systems::LcmPoseDrawer>(
            meshcat, vis_params.object_vis_models, position_names,
            orientation_names, "plans/dynamically_feasible_best_plan",
            sampling_c3_options.N + 1, true, vis_params.df_best_object_color);
    builder.Connect(
        dynamically_feasible_trajectory_sub_object_best->get_output_port(),
        dynamically_feasible_object_pose_drawer_best
            ->get_input_port_trajectory());

    auto dynamically_feasible_actor_pose_drawer_best_actor =
        builder.AddSystem<systems::LcmPoseDrawer>(
            meshcat, FindResourceOrThrow(vis_params.ee_vis_model),
            "ee_position_target", "end_effector_orientation_target",
            "plans/dynamically_feasible_best_plan", sampling_c3_options.N + 1,
            false, vis_params.df_best_ee_color);
    builder.Connect(
        dynamically_feasible_trajectory_sub_actor_best->get_output_port(),
        dynamically_feasible_actor_pose_drawer_best_actor
            ->get_input_port_trajectory());
  }

  if (vis_params.visualize_sample_locations) {
    int from_buffer = 0;
    if (sampling_params.consider_best_buffer_sample_when_leaving_c3) {
      from_buffer = 1;
    }
    auto sample_locations_drawer = builder.AddSystem<systems::LcmPoseDrawer>(
        meshcat, FindResourceOrThrow(vis_params.ee_vis_model),
        "sample_locations", "unused_orientation_name", "samples",
        std::max(sampling_params.num_additional_samples_c3 + from_buffer,
                 sampling_params.num_additional_samples_repos + 1) +
            1,
        false, vis_params.sample_color);

    builder.Connect(sample_location_sub->get_output_port(),
                    sample_locations_drawer->get_input_port_trajectory());
  }

  if (vis_params.visualize_sample_buffer) {
    auto sample_buffer_to_point_cloud_converter =
        builder.AddSystem<systems::PointCloudFromSampleBuffer>();
    auto sample_buffer_point_cloud_visualizer =
        builder.AddSystem<MeshcatPointCloudVisualizer>(meshcat,
                                                       "sample_buffer");
    sample_buffer_point_cloud_visualizer->set_point_size(
        vis_params.sample_buffer_point_size);

    builder.Connect(sample_buffer_sub->get_output_port(),
                    sample_buffer_to_point_cloud_converter
                        ->get_input_port_lcmt_sample_buffer());
    builder.Connect(sample_buffer_to_point_cloud_converter
                        ->get_output_port_sample_buffer_point_cloud(),
                    sample_buffer_point_cloud_visualizer->cloud_input_port());
    builder.Connect(sample_costs_sub->get_output_port(),
                    sample_buffer_to_point_cloud_converter
                        ->get_input_port_new_sample_costs());
  }

  if (vis_params.visualize_unsuccessful_sample_buffer) {
    auto unsuccessful_sample_buffer_sub = builder.AddSystem(
        LcmSubscriberSystem::Make<dairlib::lcmt_sample_buffer>(
            lcm_channel_params.unsuccessful_sample_buffer_channel, lcm));
    auto unsuccessful_sample_buffer_drawer =
        builder.AddSystem<systems::LcmSampleBufferSphereDrawer>(
            meshcat, "unsuccessful_sample_buffer",
            sampling_params.N_unsuccessful_sample_buffer,
            sampling_params.unsuccessful_radius,
            vis_params.unsuccessful_sample_buffer_color);
    builder.Connect(
        unsuccessful_sample_buffer_sub->get_output_port(),
        unsuccessful_sample_buffer_drawer->get_input_port_lcmt_sample_buffer());
  }

  if (vis_params.visualize_c3_state) {
    if (vis_params.object_vis_models.size() == 1) {
      auto c3_target_drawer =
          builder.AddSystem<systems::LcmC3TargetDrawer>(meshcat, true, true);
      builder.Connect(c3_state_actual_sub->get_output_port(),
                      c3_target_drawer->get_input_port_c3_state_actual());
      builder.Connect(c3_state_target_sub->get_output_port(),
                      c3_target_drawer->get_input_port_c3_state_target());
      builder.Connect(c3_final_state_target_sub->get_output_port(),
                      c3_target_drawer->get_input_port_c3_state_final_target());
    } else {
      auto c3_target_drawer = builder.AddSystem<systems::LcmC3TargetDrawer>(
          meshcat, vis_params.object_vis_models.size(), true, true);
      builder.Connect(c3_state_actual_sub->get_output_port(),
                      c3_target_drawer->get_input_port_c3_state_actual());
      builder.Connect(c3_state_target_sub->get_output_port(),
                      c3_target_drawer->get_input_port_c3_state_target());
      builder.Connect(c3_final_state_target_sub->get_output_port(),
                      c3_target_drawer->get_input_port_c3_state_final_target());
    }
  }

  if (vis_params.visualize_c3_forces_curr) {
    auto end_effector_force_drawer_curr =
        builder.AddSystem<systems::LcmForceDrawer>(
            meshcat, "end_effector_position_target",
            "end_effector_force_target", "lcs_force_trajectory_curr", "curr_");
    builder.Connect(
        trajectory_sub_actor_curr->get_output_port(),
        end_effector_force_drawer_curr->get_input_port_actor_trajectory());
    builder.Connect(
        trajectory_sub_force_curr->get_output_port(),
        end_effector_force_drawer_curr->get_input_port_force_trajectory());
    builder.Connect(
        robot_time_passthrough->get_output_port(),
        end_effector_force_drawer_curr->get_input_port_robot_time());
  }

  if (vis_params.visualize_c3_forces_best) {
    auto end_effector_force_drawer_best =
        builder.AddSystem<systems::LcmForceDrawer>(
            meshcat, "end_effector_position_target",
            "end_effector_force_target", "lcs_force_trajectory_best", "best_");
    builder.Connect(
        trajectory_sub_actor_best->get_output_port(),
        end_effector_force_drawer_best->get_input_port_actor_trajectory());
    builder.Connect(
        trajectory_sub_force_best->get_output_port(),
        end_effector_force_drawer_best->get_input_port_force_trajectory());
    builder.Connect(
        robot_time_passthrough->get_output_port(),
        end_effector_force_drawer_best->get_input_port_robot_time());
  }

  if (vis_params.visualize_is_c3_mode) {
    auto c3_mode_visualizer =
        builder.AddSystem<systems::C3ModeVisualizer>(plant_object);
    builder.Connect(is_c3_mode_sub->get_output_port(),
                    c3_mode_visualizer->get_input_port_is_c3_mode());
    builder.Connect(reduced_order_model_receiver->get_output_port(),
                    c3_mode_visualizer->get_input_port_curr_lcs_state());
    auto is_c3_mode_drawer = builder.AddSystem<systems::LcmPoseDrawer>(
        meshcat, FindResourceOrThrow(vis_params.ee_vis_model),
        "c3_mode_visualization", "end_effector_orientation_target", "c3_mode",
        1, false, vis_params.is_c3_mode_color);
    builder.Connect(
        c3_mode_visualizer->get_output_port_c3_mode_visualization_traj(),
        is_c3_mode_drawer->get_input_port_trajectory());
  }

  builder.Connect(three_d_printer_passthrough->get_output_port(),
                  mux->get_input_port(0));
  for (int i = 1; i <= tray_passthroughs.size(); i++) {
    builder.Connect(tray_passthroughs.at(i - 1)->get_output_port(),
                    mux->get_input_port(i));
  }

  builder.Connect(*mux, *to_pose);
  builder.Connect(
      to_pose->get_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(*three_d_printer_state_receiver,
                  *three_d_printer_passthrough);
  // builder.Connect(*three_d_printer_state_receiver, *ee_passthrough);
  builder.Connect(*three_d_printer_state_receiver, *robot_time_passthrough);

  for (int i = 0; i < object_state_receivers.size(); i++) {
    builder.Connect(*(object_state_receivers.at(i)),
                    *(tray_passthroughs.at(i)));
  }
  builder.Connect(*three_d_printer_state_sub, *three_d_printer_state_receiver);

  for (int i = 0; i < object_state_subs.size(); i++) {
    builder.Connect(*(object_state_subs.at(i)),
                    *(object_state_receivers.at(i)));
  }

  auto visualizer = &drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
      &builder, scene_graph, meshcat, std::move(params));
  auto diagram = builder.Build();
  diagram->set_name(("sampling_c3_visualizer_" + FLAGS_demo_name));
  DrawAndSaveDiagramGraph(*diagram);
  auto context = diagram->CreateDefaultContext();
  auto& three_d_printer_state_sub_context = diagram->GetMutableSubsystemContext(
      *three_d_printer_state_sub, context.get());
  std::vector<drake::systems::Context<double>*> object_state_sub_contexts;
  for (int i = 0; i < object_state_receivers.size(); i++) {
    object_state_sub_contexts.push_back(&diagram->GetMutableSubsystemContext(
        *(object_state_subs.at(i)), context.get()));
  }
  three_d_printer_state_receiver->InitializeSubscriberPositions(
      plant, three_d_printer_state_sub_context);
  for (int i = 0; i < object_state_receivers.size(); i++) {
    object_state_receivers.at(i)->InitializeSubscriberPositions(
        plant, *object_state_sub_contexts.at(i));
  }
  /// Use the simulator to drive at a fixed rate
  /// If set_publish_every_time_step is true, this publishes twice
  auto simulator =
      std::make_unique<Simulator<double>>(*diagram, std::move(context));
  simulator->set_publish_every_time_step(false);
  simulator->set_publish_at_initialization(false);
  simulator->set_target_realtime_rate(1.0);
  simulator->Initialize();

  drake::log()->info("visualizer started");

  simulator->AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }
