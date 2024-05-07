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
#include "examples/franka_ball_rolling/parameters/simulate_franka_params.h"
#include "examples/franka_ball_rolling/parameters/lcm_channels_params.h"

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
              "examples/franka_ball_rolling/parameters/lcm_channels_sim_params.yaml",
              "Filepath containing lcm channels");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  SimulateFrankaParams sim_param = drake::yaml::LoadYamlFile<SimulateFrankaParams>(
      "examples/franka_ball_rolling/parameters/simulate_franka_params.yaml");
  BallRollingLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<BallRollingLcmChannels>(FLAGS_lcm_channels);
//  FrankaSimSceneParams scene_params =
//      drake::yaml::LoadYamlFile<FrankaSimSceneParams>(
//          sim_params.sim_scene_file[sim_params.scene_index]);

  drake::systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  MultibodyPlant<double> plant(0.0);

  // load urdf models
  Parser parser(&plant, &scene_graph);
  parser.SetAutoRenaming(true);
  drake::multibody::ModelInstanceIndex franka_index = parser.AddModels(sim_param.franka_model)[0];
  drake::multibody::ModelInstanceIndex ground_index = parser.AddModels(sim_param.ground_model)[0];
  drake::multibody::ModelInstanceIndex end_effector_index = parser.AddModels(sim_param.end_effector_model)[0];
  drake::multibody::ModelInstanceIndex ball_index = parser.AddModels(sim_param.ball_model)[0];

  // visually give a offset stage model, not important
  parser.AddModels(sim_param.offset_model);

  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  RigidTransform<double> X_F_EE = RigidTransform<double>(sim_param.tool_attachment_frame);
  RigidTransform<double> X_F_G = RigidTransform<double>(sim_param.ground_offset_frame);

  plant.WeldFrames(plant.world_frame(),
                     plant.GetFrameByName("panda_link0"), X_WI);
  plant.WeldFrames(plant.GetFrameByName("panda_link7"),
                     plant.GetFrameByName("end_effector_base", end_effector_index), X_F_EE);
  plant.WeldFrames(plant.GetFrameByName("panda_link0"),
                     plant.GetFrameByName("visual_table_offset"), X_WI);
  plant.WeldFrames(plant.GetFrameByName("panda_link0"),
                     plant.GetFrameByName("ground", ground_index), X_F_G);

  plant.Finalize();

  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();
  // Create state receiver.
  auto franka_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(
          lcm_channel_params.franka_state_channel, lcm));
  auto ball_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.true_ball_state_channel, lcm));
  auto franka_state_receiver =
      builder.AddSystem<RobotOutputReceiver>(plant, franka_index);
  auto ball_state_receiver =
      builder.AddSystem<ObjectStateReceiver>(plant, ball_index);

  auto franka_passthrough = builder.AddSystem<SubvectorPassThrough>(
      franka_state_receiver->get_output_port(0).size(), 0,
      plant.num_positions(franka_index));
  auto robot_time_passthrough = builder.AddSystem<SubvectorPassThrough>(
      franka_state_receiver->get_output_port(0).size(),
      franka_state_receiver->get_output_port(0).size() - 1, 1);
  auto ball_passthrough = builder.AddSystem<SubvectorPassThrough>(
          ball_state_receiver->get_output_port(0).size(), 0,
      plant.num_positions(ball_index));

  std::vector<int> input_sizes = {plant.num_positions(franka_index),
                                  plant.num_positions(ball_index)};
  auto mux =
      builder.AddSystem<drake::systems::Multiplexer<double>>(input_sizes);

  auto trajectory_sub_ball = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.c3_object_trajectory_channel, lcm));
  auto trajectory_sub_force =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_c3_forces>(
          lcm_channel_params.c3_force_channel, lcm));

  auto c3_state_actual_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_c3_state>(
          lcm_channel_params.c3_actual_state_channel, lcm));
  auto c3_state_target_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_c3_state>(
          lcm_channel_params.c3_target_state_channel, lcm));
  auto to_pose =
      builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant);

  drake::geometry::MeshcatVisualizerParams params;
  params.publish_period = 1.0 / sim_param.visualizer_publish_rate;
  auto meshcat = std::make_shared<drake::geometry::Meshcat>();
  auto visualizer = &drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
      &builder, scene_graph, meshcat, std::move(params));

  meshcat->SetCameraPose(sim_param.camera_pose,
                         sim_param.camera_target);

  if (sim_param.visualize_plan) {
    auto trajectory_drawer_object =
        builder.AddSystem<systems::LcmTrajectoryDrawer>(
            meshcat, "object_position_target");
    trajectory_drawer_object->SetLineColor(drake::geometry::Rgba({0, 0, 1, 1}));
    trajectory_drawer_object->SetNumSamples(40);
    builder.Connect(trajectory_sub_ball->get_output_port(),
                    trajectory_drawer_object->get_input_port_trajectory());
  }

  if (sim_param.visualize_pose_trace) {
    auto ball_pose_drawer = builder.AddSystem<systems::LcmPoseDrawer>(
        meshcat,
        FindResourceOrThrow(sim_param.ball_model),
        "object_position_target", "object_orientation_target");
    builder.Connect(trajectory_sub_ball->get_output_port(),
                    ball_pose_drawer->get_input_port_trajectory());
  }


  if (sim_param.visualize_c3_object_state || sim_param.visualize_c3_end_effector_state) {
    auto c3_target_drawer =
        builder.AddSystem<systems::LcmC3TargetDrawer>(meshcat, sim_param.visualize_c3_object_state, sim_param.visualize_c3_end_effector_state);
    builder.Connect(c3_state_actual_sub->get_output_port(),
                    c3_target_drawer->get_input_port_c3_state_actual());
    builder.Connect(c3_state_target_sub->get_output_port(),
                    c3_target_drawer->get_input_port_c3_state_target());
  }

//  if (sim_param.visualize_c3_forces) {
//    auto end_effector_force_drawer = builder.AddSystem<systems::LcmForceDrawer>(
//        meshcat, "end_effector_position_target", "end_effector_force_target",
//        "lcs_force_trajectory");
//    builder.Connect(
//        trajectory_sub_actor->get_output_port(),
//        end_effector_force_drawer->get_input_port_actor_trajectory());
//    builder.Connect(
//        trajectory_sub_force->get_output_port(),
//        end_effector_force_drawer->get_input_port_force_trajectory());
//    builder.Connect(robot_time_passthrough->get_output_port(),
//                    end_effector_force_drawer->get_input_port_robot_time());
//  }

  builder.Connect(franka_passthrough->get_output_port(),
                  mux->get_input_port(0));
  builder.Connect(ball_passthrough->get_output_port(), mux->get_input_port(1));
  builder.Connect(*mux, *to_pose);
  builder.Connect(
      to_pose->get_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(*franka_state_receiver, *franka_passthrough);
  builder.Connect(*franka_state_receiver, *robot_time_passthrough);
  builder.Connect(*ball_state_receiver, *ball_passthrough);
  builder.Connect(*franka_state_sub, *franka_state_receiver);
  builder.Connect(*ball_state_sub, *ball_state_receiver);

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  auto& franka_state_sub_context =
      diagram->GetMutableSubsystemContext(*franka_state_sub, context.get());
  auto& ball_state_sub_context =
      diagram->GetMutableSubsystemContext(*ball_state_sub, context.get());
  franka_state_receiver->InitializeSubscriberPositions(
      plant, franka_state_sub_context);
  ball_state_receiver->InitializeSubscriberPositions(plant,
                                                     ball_state_sub_context);

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
