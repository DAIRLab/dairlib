#include <iostream>

#include <dairlib/lcmt_c3_forces.hpp>
#include <dairlib/lcmt_c3_output.hpp>
#include <dairlib/lcmt_c3_state.hpp>
#include <dairlib/lcmt_material_points.hpp>
#include <dairlib/lcmt_robot_output.hpp>
#include <drake/systems/primitives/multiplexer.h>
#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "examples/deform/deform_utils.h"
#include "examples/deform/mpm_model_reducer.h"
#include "examples/deform/parameter_headers/deform_settings.h"
#include "examples/deform/parameter_headers/elastoplastic_c3_options.h"
#include "examples/deform/parameter_headers/lcm_channels.h"
#include "examples/deform/parameter_headers/reduced_model_params.h"
#include "examples/deform/parameter_headers/visualizer_params.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"
#include "systems/senders/mpm_points_to_point_cloud.h"
#include "systems/system_utils.h"
#include "systems/trajectory_optimization/lcm_trajectory_systems.h"
#include "systems/visualization/lcm_visualization_systems.h"

#include "drake/common/find_resource.h"
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

DEFINE_bool(is_simulation, true, "True for simulation, false for hardware");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Load parameters.
  DeformSettings deform_settings = drake::yaml::LoadYamlFile<DeformSettings>(
      "examples/deform/parameters/deform_settings.yaml");
  DeformVisualizerParams vis_params =
      drake::yaml::LoadYamlFile<DeformVisualizerParams>(
          deform_settings.vis_params_file);
  std::string lcm_channels_file =
      FLAGS_is_simulation ? deform_settings.lcm_channels_simulation_file
                          : deform_settings.lcm_channels_hardware_file;
  DeformLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<DeformLcmChannels>(lcm_channels_file);
  ReducedModelParams reduced_model_params =
      drake::yaml::LoadYamlFile<ReducedModelParams>(
          deform_settings.reduced_model_params_file);

  DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  // Build the visualizer plant.
  MultibodyPlant<double> plant(0.0);
  ModelInstanceIndex robot_index =
      AddFrankaToPlant(&plant, &scene_graph, true, true, true);
  plant.Finalize();

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
  std::vector<int> input_sizes = {// plant.num_positions(object_index),
                                  plant.num_positions(robot_index)};
  auto mux = builder.AddSystem<Multiplexer<double>>(input_sizes);
  auto to_pose =
      builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant);

  // Visualize the robot.
  auto lcm = builder.AddSystem<LcmInterfaceSystem>();
  auto robot_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(
          lcm_channel_params.robot_state_channel, lcm));
  auto robot_state_receiver =
      builder.AddSystem<RobotOutputReceiver>(plant, robot_index);
  auto robot_q_passthrough = builder.AddSystem<SubvectorPassThrough>(
      robot_state_receiver->get_output_port(0).size(), 0,
      plant.num_positions(robot_index));
  // Subscriber -> receiver -> passthrough -> mux -> to_pose -> scene_graph
  builder.Connect(*robot_state_sub, *robot_state_receiver);
  builder.Connect(*robot_state_receiver, *robot_q_passthrough);
  builder.Connect(robot_q_passthrough->get_output_port(),
                  mux->get_input_port(0));
  builder.Connect(*mux, *to_pose);
  builder.Connect(
      to_pose->get_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));

  // Visualize the MPM object.
  auto mpm_points_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_material_points>(
          lcm_channel_params.mpm_channel, lcm));
  if (vis_params.visualize_mpm_points) {
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
    DRAKE_ASSERT(reduced_model_params.reduction_type ==
                 ReducedModelTypes::kSupportDirections);
    auto mpm_reducer =
        builder.AddSystem<dairlib::systems::MpmPointsToReducedModelPoints>(
            reduced_model_params.support_directions);
    auto reduced_model_points_drawer =
        builder.AddSystem<systems::LcmPoseDrawer>(
            meshcat,
            FindResourceOrThrow(vis_params.model_reduction_point_model),
            "reduced_model_points", "unused_orientation_name", "reduced",
            reduced_model_params.support_directions.cols(), false,
            vis_params.model_reduction_point_color);

    builder.Connect(mpm_points_sub->get_output_port(),
                    mpm_reducer->get_input_port_lcmt_material_points());
    builder.Connect(mpm_reducer->get_output_port_lcmt_timestamped_saved_traj(),
                    reduced_model_points_drawer->get_input_port_trajectory());
  }

  // Build the diagram.
  auto diagram = builder.Build();
  diagram->set_name(("deform_franka_visualizer"));
  DrawAndSaveDiagramGraph(*diagram);
  auto context = diagram->CreateDefaultContext();

  // Set the initial configuration of the robot.
  auto& robot_state_sub_context =
      diagram->GetMutableSubsystemContext(*robot_state_sub, context.get());
  robot_state_receiver->InitializeSubscriberPositions(plant,
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
