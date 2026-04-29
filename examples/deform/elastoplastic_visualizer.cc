#include <iostream>

#include <dairlib/lcmt_c3_forces.hpp>
#include <dairlib/lcmt_c3_output.hpp>
#include <dairlib/lcmt_c3_state.hpp>
#include <dairlib/lcmt_robot_output.hpp>
#include <drake/systems/primitives/multiplexer.h>
#include <gflags/gflags.h>

#include "examples/deform/deform_utils.h"
#include "examples/deform/parameter_headers/elastoplastic_sc3_options.h"
#include "examples/deform/parameter_headers/lcm_channels.h"
#include "examples/deform/parameter_headers/visualizer_params.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"
#include "systems/trajectory_optimization/lcm_trajectory_systems.h"
#include "systems/visualization/lcm_visualization_systems.h"

#include "drake/common/text_logging.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/drake_visualizer.h"
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
using drake::geometry::SceneGraph;
using drake::math::RigidTransformd;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::systems::DiagramBuilder;
using drake::systems::Multiplexer;
using drake::systems::Simulator;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::rendering::MultibodyPositionToGeometryPose;

DEFINE_bool(is_simulation, true, "True for simulation, false for hardware");
DEFINE_string(demo, "1d_rigid", "Demo type");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Load parameters.
  DeformVisualizerParams vis_params =
      drake::yaml::LoadYamlFile<DeformVisualizerParams>(
          "examples/deform/parameters/demo_" + FLAGS_demo + "/vis_params.yaml");
  DeformLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<DeformLcmChannels>(
          "examples/deform/parameters/lcm_channels_sim.yaml");
  ElastoPlasticSC3Options elastoplastic_c3_options =
      drake::yaml::LoadYamlFile<ElastoPlasticSC3Options>(
          "examples/deform/parameters/demo_" + FLAGS_demo +
          "/elastoplastic_c3_options.yaml");

  DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  // Build the visualizer plant.
  MultibodyPlant<double> plant(0.0);
  std::vector<ModelInstanceIndex> model_indices =
      AddLCSModelsToPlant(&plant, &scene_graph, FLAGS_demo);
  ModelInstanceIndex robot_index = model_indices[0];
  ModelInstanceIndex object_index = model_indices[1];
  plant.Finalize();

  // Note:  For some reason this plant puts the robot at the end of the position
  // vector, so connect robot passthrough to the mux's index 1 input and the
  // object passthrough to the mux's index 0 input.
  std::vector<int> input_sizes = {plant.num_positions(object_index),
                                  plant.num_positions(robot_index)};
  auto mux = builder.AddSystem<Multiplexer<double>>(input_sizes);
  auto to_pose =
      builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant);

  // Instantiate and wire the visualizer systems for the robot.
  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();
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
                  mux->get_input_port(1));
  builder.Connect(*mux, *to_pose);
  builder.Connect(
      to_pose->get_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  std::cout << "Visualizer plant position names: " << std::endl;
  for (int i = 0; i < plant.num_positions(); i++) {
    std::cout << "  " << plant.GetPositionNames()[i] << std::endl;
  }

  // Instantiate and wire the visualizer systems for the object.
  auto object_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.object_state_channel, lcm));
  auto object_state_receiver =
      builder.AddSystem<ObjectStateReceiver>(plant, object_index);
  auto object_q_passthrough = builder.AddSystem<SubvectorPassThrough>(
      object_state_receiver->get_output_port(0).size(), 0,
      plant.num_positions(object_index));
  // Subscriber -> receiver -> passthrough -> mux
  builder.Connect(*object_state_sub, *object_state_receiver);
  builder.Connect(*object_state_receiver, *object_q_passthrough);
  builder.Connect(object_q_passthrough->get_output_port(),
                  mux->get_input_port(0));

  // Add meshcat visualizer.
  drake::geometry::MeshcatVisualizerParams params;
  params.publish_period = 1.0 / vis_params.visualizer_publish_rate;
  auto meshcat = std::make_shared<drake::geometry::Meshcat>();
  meshcat->SetCameraPose(vis_params.camera_pose, vis_params.camera_target);

  auto visualizer = &drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
      &builder, scene_graph, meshcat, std::move(params));

  // Add other annotations as desired.
  // Visualize C3 states.
  if (vis_params.visualize_c3_target_state) {
    auto c3_state_actual_sub =
        builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_c3_state>(
            lcm_channel_params.c3_actual_state_channel, lcm));
    auto c3_state_target_sub =
        builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_c3_state>(
            lcm_channel_params.c3_target_state_channel, lcm));

    std::string object_model;
    if (FLAGS_demo == "1d") {
      object_model = kElastoPlastic1DModel;
    } else if (FLAGS_demo == "1d_rigid") {
      object_model = kRigid1DModel;
    } else {
      throw std::runtime_error("Demo " + FLAGS_demo + " not handled yet.");
    }
    auto c3_target_drawer = builder.AddSystem<systems::LcmC3TargetDrawer>(
        meshcat, object_model, kEndEffector1DModel, "base_link",
        RigidTransformd(), RigidTransformd(k1DRobotPosOffset),
        vis_params.c3_state_object_color, vis_params.c3_state_ee_color,
        vis_params.visualize_c3_state, false);
    builder.Connect(c3_state_actual_sub->get_output_port(),
                    c3_target_drawer->get_input_port_c3_state_actual());
    builder.Connect(c3_state_target_sub->get_output_port(),
                    c3_target_drawer->get_input_port_c3_state_target());
  }

  // Visualize the C3 plan.
  if (vis_params.visualize_c3_plan_object ||
      vis_params.visualize_c3_plan_robot) {
    auto c3_plan_sub =
        builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_c3_output>(
            lcm_channel_params.c3_debug_output_channel, lcm));

    std::string object_model;
    if (FLAGS_demo == "1d") {
      object_model = kElastoPlastic1DModel;
    } else if (FLAGS_demo == "1d_rigid") {
      object_model = kRigid1DModel;
    } else {
      throw std::runtime_error("Demo " + FLAGS_demo + " not handled yet.");
    }
    auto c3_plan_drawer = builder.AddSystem<systems::LcmC3PlanDrawer>(
        meshcat, elastoplastic_c3_options.N, object_model, kEndEffector1DModel,
        "base_link", RigidTransformd(), RigidTransformd(k1DRobotPosOffset),
        vis_params.c3_object_color, vis_params.c3_ee_color,
        vis_params.visualize_c3_plan_object,
        vis_params.visualize_c3_plan_robot);
    builder.Connect(*c3_plan_sub, *c3_plan_drawer);
  }

  // Visualize C3 forces.
  if (vis_params.visualize_c3_forces) {
    auto c3_forces_sub =
        builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_c3_forces>(
            lcm_channel_params.c3_force_channel, lcm));
    auto c3_forces_drawer = builder.AddSystem<dairlib::systems::LcmForceDrawer>(
        meshcat, "end_effector_position_target", "end_effector_force_target",
        "lcs_force_trajectory");
    builder.Connect(c3_forces_sub->get_output_port(),
                    c3_forces_drawer->get_input_port_force_trajectory());
    // TODO @bibit:  So far drawing u_lcs vector is not implemented.
    // auto robot_time_passthrough = builder.AddSystem<SubvectorPassThrough>(
    //     robot_state_receiver->get_output_port(0).size(),
    //     robot_state_receiver->get_output_port(0).size() - 1, 1);
    // builder.Connect(robot_plan_sub->get_output_port(),
    //                 c3_forces_drawer->get_input_port_actor_trajectory());
    // builder.Connect(robot_time_passthrough->get_output_port(),
    //                 c3_forces_drawer->get_input_port_robot_time());
  }

  // Build the diagram.
  auto diagram = builder.Build();
  diagram->set_name(("elastoplastic_visualizer_" + FLAGS_demo));
  DrawAndSaveDiagramGraph(*diagram);
  auto context = diagram->CreateDefaultContext();

  // Set the initial configuration of the robot.
  auto& robot_state_sub_context =
      diagram->GetMutableSubsystemContext(*robot_state_sub, context.get());
  robot_state_receiver->InitializeSubscriberPositions(plant,
                                                      robot_state_sub_context);
  auto& object_state_sub_context =
      diagram->GetMutableSubsystemContext(*object_state_sub, context.get());
  object_state_receiver->InitializeSubscriberPositions(
      plant, object_state_sub_context);

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
