#include <iostream>

#include <c3/lcmt_output.hpp>
#include <dairlib/lcmt_c3_forces.hpp>
#include <dairlib/lcmt_c3_state.hpp>
#include <dairlib/lcmt_elastoplastic_network.hpp>
#include <dairlib/lcmt_material_points.hpp>
#include <dairlib/lcmt_robot_output.hpp>
#include <dairlib/lcmt_sample_buffer.hpp>
#include <drake/systems/primitives/multiplexer.h>
#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "examples/deform/deform_utils.h"
#include "examples/deform/elastoplastic_model_interpreter.h"
#include "examples/deform/parameter_headers/deform_controller_params.h"
#include "examples/deform/parameter_headers/elastoplastic_sc3_options.h"
#include "examples/deform/parameter_headers/lcm_channels.h"
#include "examples/deform/parameter_headers/reduced_model_params.h"
#include "examples/deform/parameter_headers/visualizer_params.h"
#include "examples/sampling_c3/c3_mode_visualizer.h"
#include "systems/franka_kinematics.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"
#include "systems/senders/mpm_points_to_point_cloud.h"
#include "systems/senders/sample_buffer_to_point_cloud.h"
#include "systems/system_utils.h"
#include "systems/trajectory_optimization/lcm_trajectory_systems.h"
#include "systems/visualization/lcm_visualization_systems.h"

#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/meshcat_point_cloud_visualizer.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/meshcat_visualizer_params.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

namespace dairlib {

using dairlib::systems::ObjectStateReceiver;
using dairlib::systems::RobotOutputReceiver;
using dairlib::systems::SubvectorPassThrough;
using drake::geometry::Meshcat;
using drake::geometry::MeshcatPointCloudVisualizer;
using drake::geometry::MeshcatVisualizer;
using drake::geometry::MeshcatVisualizerParams;
using drake::geometry::SceneGraph;
using drake::math::RigidTransformd;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::systems::DiagramBuilder;
using drake::systems::Multiplexer;
using drake::systems::Simulator;
using drake::systems::lcm::LcmInterfaceSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::rendering::MultibodyPositionToGeometryPose;
using Eigen::Vector3d;

DEFINE_bool(is_simulation, true, "True for simulation, false for hardware");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Load parameters.
  DeformControllerParams deform_controller_params =
      drake::yaml::LoadYamlFile<DeformControllerParams>(
          "examples/deform/parameters/deform_controller_params.yaml");
  DeformVisualizerParams vis_params =
      drake::yaml::LoadYamlFile<DeformVisualizerParams>(
          deform_controller_params.vis_params_file);
  std::string lcm_channels_file =
      FLAGS_is_simulation
          ? deform_controller_params.lcm_channels_simulation_file
          : deform_controller_params.lcm_channels_hardware_file;
  DeformLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<DeformLcmChannels>(lcm_channels_file);
  ElastoPlasticSC3Options elastoplastic_sc3_options =
      drake::yaml::LoadYamlFile<ElastoPlasticSC3Options>(
          deform_controller_params.elastoplastic_sc3_options_file);
  SamplingParams sampling_params = deform_controller_params.sampling_params;
  ReducedModelParams reduced_model_params =
      drake::yaml::LoadYamlFile<ReducedModelParams>(
          deform_controller_params.reduced_model_params_file);

  // Get some helper variables.
  int n_nodes = reduced_model_params.support_directions.cols();

  // Start constructing the diagram.
  DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  // Build the franka-only plant.
  MultibodyPlant<double> plant_franka(0.0);
  ModelInstanceIndex robot_index =
      AddFrankaToPlant(&plant_franka, &scene_graph, true, true, true);
  plant_franka.Finalize();
  auto franka_context = plant_franka.CreateDefaultContext();

  // Build the deformable network plant.
  MultibodyPlant<double> plant_deform_network(0.0);
  vector<ModelInstanceIndex> node_indices =
      AddDeformableLCSModelToPlant(&plant_deform_network, nullptr, n_nodes);
  plant_deform_network.Finalize();
  auto deform_context = plant_deform_network.CreateDefaultContext();

  // Add meshcat visualizer.
  MeshcatVisualizerParams params;
  params.publish_period = 1.0 / vis_params.visualizer_publish_rate;
  auto meshcat = std::make_shared<Meshcat>();
  meshcat->SetCameraPose(vis_params.camera_pose, vis_params.camera_target);
  auto visualizer = &MeshcatVisualizer<double>::AddToBuilder(
      &builder, scene_graph, meshcat, std::move(params));

  // Note:  For some reason this plant puts the robot at the end of the position
  // vector, so connect robot passthrough to the mux's index 1 input and the
  // object passthrough to the mux's index 0 input.
  std::vector<int> input_sizes = {// plant_franka.num_positions(object_index),
                                  plant_franka.num_positions(robot_index)};
  auto mux = builder.AddSystem<Multiplexer<double>>(input_sizes);
  auto to_pose =
      builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant_franka);

  // Visualize the robot.
  auto lcm = builder.AddSystem<LcmInterfaceSystem>();
  auto robot_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(
          lcm_channel_params.robot_state_channel, lcm));
  auto robot_state_receiver =
      builder.AddSystem<RobotOutputReceiver>(plant_franka, robot_index);
  auto robot_q_passthrough = builder.AddSystem<SubvectorPassThrough>(
      robot_state_receiver->get_output_port(0).size(), 0,
      plant_franka.num_positions(robot_index));
  // Subscriber -> receiver -> passthrough -> mux -> to_pose -> scene_graph
  builder.Connect(*robot_state_sub, *robot_state_receiver);
  builder.Connect(*robot_state_receiver, *robot_q_passthrough);
  builder.Connect(robot_q_passthrough->get_output_port(),
                  mux->get_input_port(0));
  builder.Connect(*mux, *to_pose);
  builder.Connect(
      to_pose->get_output_port(),
      scene_graph.get_source_pose_port(plant_franka.get_source_id().value()));

  // Some subscribers that are possibly used by multiple visualizations.
  auto reduced_model_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_elastoplastic_network>(
          lcm_channel_params.reduced_model_channel, lcm));

  // Visualize the C3 workspace.
  if (vis_params.visualize_c3_workspace) {
    // Note:  There are also robot radius limits which are not visualized.
    double width = elastoplastic_sc3_options.workspace_limits[0][4] -
                   elastoplastic_sc3_options.workspace_limits[0][3];  // x
    double depth = elastoplastic_sc3_options.workspace_limits[1][4] -
                   elastoplastic_sc3_options.workspace_limits[1][3];  // y
    double height = elastoplastic_sc3_options.workspace_limits[2][4] -
                    elastoplastic_sc3_options.workspace_limits[2][3];  // z
    Vector3d workspace_center = {
        0.5 * (elastoplastic_sc3_options.workspace_limits[0][4] +
               elastoplastic_sc3_options.workspace_limits[0][3]),
        0.5 * (elastoplastic_sc3_options.workspace_limits[1][4] +
               elastoplastic_sc3_options.workspace_limits[1][3]),
        0.5 * (elastoplastic_sc3_options.workspace_limits[2][4] +
               elastoplastic_sc3_options.workspace_limits[2][3])};
    meshcat->SetObject("c3_workspace",
                       drake::geometry::Box(width, depth, height),
                       {0, 1, 0, 0.2});
    meshcat->SetTransform("c3_workspace", RigidTransformd(workspace_center));
  }

  // Visualize the C3 states (actual, target, final target).
  if (vis_params.visualize_c3_state || vis_params.visualize_c3_target_state ||
      vis_params.visualize_c3_final_target_state) {
    std::cout << "Adding C3 target state visualizer..." << std::endl;
    auto c3_state_actual_sub =
        builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_c3_state>(
            lcm_channel_params.c3_actual_state_channel, lcm));
    auto c3_state_target_sub =
        builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_c3_state>(
            lcm_channel_params.c3_target_state_channel, lcm));
    auto c3_final_state_target_sub =
        builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_c3_state>(
            lcm_channel_params.c3_final_target_state_channel, lcm));
    auto c3_target_drawer = builder.AddSystem<systems::LcmC3TargetDrawer>(
        meshcat, n_nodes, vis_params.model_reduction_point_model,
        vis_params.ee_vis_model, "base_link", RigidTransformd(),
        RigidTransformd(), vis_params.c3_state_actual_color,
        vis_params.c3_state_target_color,
        vis_params.c3_state_final_target_color, vis_params.visualize_c3_state,
        vis_params.visualize_c3_target_state,
        vis_params.visualize_c3_final_target_state);
    builder.Connect(c3_state_actual_sub->get_output_port(),
                    c3_target_drawer->get_input_port_c3_state_actual());
    builder.Connect(c3_state_target_sub->get_output_port(),
                    c3_target_drawer->get_input_port_c3_state_target());
    builder.Connect(c3_final_state_target_sub->get_output_port(),
                    c3_target_drawer->get_input_port_c3_state_final_target());
    builder.Connect(
        reduced_model_sub->get_output_port(),
        c3_target_drawer->get_input_port_lcmt_elastoplastic_network());
    std::cout << "C3 target state visualizer added." << std::endl;
  }

  // Visualize the sample locations.
  if (vis_params.visualize_sample_locations) {
    auto sample_location_sub = builder.AddSystem(
        LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
            lcm_channel_params.sample_locations_channel, lcm));
    int from_buffer =
        sampling_params.consider_best_buffer_sample_when_leaving_c3 ? 1 : 0;
    auto sample_locations_drawer = builder.AddSystem<systems::LcmPoseDrawer>(
        meshcat, FindResourceOrThrow(vis_params.ee_vis_model),
        "sample_locations", "samples",
        std::max(sampling_params.num_additional_samples_c3 + from_buffer,
                 sampling_params.num_additional_samples_repos + 1) +
            1,
        false, vis_params.sample_color, "base_link");
    builder.Connect(sample_location_sub->get_output_port(),
                    sample_locations_drawer->get_input_port_trajectory());
  }

  // Visualize the sample buffer.
  if (vis_params.visualize_sample_buffer) {
    auto sample_buffer_sub = builder.AddSystem(
        LcmSubscriberSystem::Make<dairlib::lcmt_sample_buffer>(
            lcm_channel_params.sample_buffer_channel, lcm));
    auto sample_costs_sub = builder.AddSystem(
        LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
            lcm_channel_params.sample_costs_channel, lcm));
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

  // Visualize the C3/repositioning mode indicator.
  if (vis_params.visualize_is_c3_mode) {
    auto franka_kinematics = builder.AddSystem<systems::FrankaKinematics>(
        plant_franka, franka_context.get(), &plant_deform_network,
        deform_context.get(), kEndEffectorName, n_nodes, false);
    auto is_c3_mode_sub = builder.AddSystem(
        LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
            lcm_channel_params.is_c3_mode_channel, lcm));
    auto c3_mode_visualizer =
        builder.AddSystem<systems::C3ModeVisualizer>(6 + 6 * n_nodes);
    builder.Connect(robot_state_receiver->get_output_port(),
                    franka_kinematics->get_input_port_franka_state());
    builder.Connect(reduced_model_sub->get_output_port(),
                    franka_kinematics->get_input_port_elastoplastic_network());
    builder.Connect(is_c3_mode_sub->get_output_port(),
                    c3_mode_visualizer->get_input_port_is_c3_mode());
    builder.Connect(franka_kinematics->get_output_port(),
                    c3_mode_visualizer->get_input_port_curr_lcs_state());
    auto is_c3_mode_drawer = builder.AddSystem<systems::LcmPoseDrawer>(
        meshcat, FindResourceOrThrow(vis_params.ee_vis_model),
        "c3_mode_visualization", "c3_mode", 1, false,
        vis_params.is_c3_mode_color, "base_link");
    builder.Connect(
        c3_mode_visualizer->get_output_port_c3_mode_visualization_traj(),
        is_c3_mode_drawer->get_input_port_trajectory());
  }

  // Visualize the MPM object.
  if (vis_params.visualize_mpm_points) {
    auto mpm_points_sub = builder.AddSystem(
        LcmSubscriberSystem::Make<dairlib::lcmt_material_points>(
            lcm_channel_params.mpm_channel, lcm));
    auto mpm_points_to_point_cloud_converter =
        builder.AddSystem<systems::PointCloudFromMpmPoints>();
    auto mpm_points_point_cloud_visualizer =
        builder.AddSystem<MeshcatPointCloudVisualizer>(meshcat, "mpm_points");
    mpm_points_point_cloud_visualizer->set_point_size(
        vis_params.mpm_point_size);

    builder.Connect(mpm_points_sub->get_output_port(),
                    mpm_points_to_point_cloud_converter
                        ->get_input_port_lcmt_material_points());
    builder.Connect(
        mpm_points_to_point_cloud_converter->get_output_port_mpm_point_cloud(),
        mpm_points_point_cloud_visualizer->cloud_input_port());
  }

  // Visualize the model reduction.
  if (vis_params.visualize_model_reduction) {
    auto reduced_model_interpreter =
        builder.AddSystem<dairlib::systems::ElastoPlasticModelInterpreter>(
            meshcat, n_nodes,
            FindResourceOrThrow(vis_params.model_reduction_point_model),
            vis_params.reduced_model_color);
    builder.Connect(
        reduced_model_sub->get_output_port(),
        reduced_model_interpreter->get_input_port_lcmt_elastoplastic_network());
  }

  // Visualize the C3 plan (deformable nodes, connections, and EE) for the
  // current C3 solve.
  if (vis_params.visualize_c3_plan_object ||
      vis_params.visualize_c3_plan_robot) {
    auto c3_plan_sub = builder.AddSystem(
        LcmSubscriberSystem::Make<c3::lcmt_output>(
            lcm_channel_params.c3_debug_output_curr_channel, lcm));
    auto c3_plan_drawer = builder.AddSystem<systems::LcmC3PlanDrawer>(
        meshcat, elastoplastic_sc3_options.N, n_nodes,
        vis_params.model_reduction_point_model, vis_params.ee_vis_model,
        "c3_plans/curr", "base_link", RigidTransformd(), RigidTransformd(),
        vis_params.c3_object_color, vis_params.c3_ee_color,
        vis_params.visualize_c3_plan_object, vis_params.visualize_c3_plan_robot);
    builder.Connect(c3_plan_sub->get_output_port(),
                    c3_plan_drawer->get_input_port_c3_plan());
    builder.Connect(
        reduced_model_sub->get_output_port(),
        c3_plan_drawer->get_input_port_lcmt_elastoplastic_network());
  }

  // Visualize the cost-driven ("dynamically feasible") plan (deformable
  // nodes, connections, and EE) for the current location.
  if (vis_params.visualize_cost_plan_object ||
      vis_params.visualize_cost_plan_robot) {
    auto cost_plan_sub = builder.AddSystem(
        LcmSubscriberSystem::Make<c3::lcmt_output>(
            lcm_channel_params.dynamically_feasible_debug_curr_channel, lcm));
    auto cost_plan_drawer = builder.AddSystem<systems::LcmC3PlanDrawer>(
        meshcat, elastoplastic_sc3_options.N, n_nodes,
        vis_params.model_reduction_point_model, vis_params.ee_vis_model,
        "cost_plans/curr", "base_link", RigidTransformd(), RigidTransformd(),
        vis_params.df_object_color, vis_params.df_ee_color,
        vis_params.visualize_cost_plan_object,
        vis_params.visualize_cost_plan_robot);
    builder.Connect(cost_plan_sub->get_output_port(),
                    cost_plan_drawer->get_input_port_c3_plan());
    builder.Connect(
        reduced_model_sub->get_output_port(),
        cost_plan_drawer->get_input_port_lcmt_elastoplastic_network());
  }

  // Build the diagram.
  auto diagram = builder.Build();
  diagram->set_name(("deform_franka_visualizer"));
  DrawAndSaveDiagramGraph(*diagram);
  auto context = diagram->CreateDefaultContext();

  // Set the initial configuration of the robot.
  auto& robot_state_sub_context =
      diagram->GetMutableSubsystemContext(*robot_state_sub, context.get());
  robot_state_receiver->InitializeSubscriberPositions(plant_franka,
                                                      robot_state_sub_context);

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
