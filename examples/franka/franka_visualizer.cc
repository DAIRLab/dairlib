#include <iostream>

#include <dairlib/lcmt_timestamped_saved_traj.hpp>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/primitives/multiplexer.h>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "common/find_resource.h"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/franka/parameters/franka_lcm_channels.h"
#include "examples/franka/parameters/franka_sim_params.h"
#include "multibody/com_pose_system.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"
#include "systems/trajectory_optimization/lcm_trajectory_systems.h"

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

using dairlib::systems::RobotOutputReceiver;
using dairlib::systems::ObjectStateReceiver;
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
              "examples/franka/parameters/lcm_channels_simulation.yaml",
              "Filepath containing lcm channels");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  FrankaSimParams sim_params = drake::yaml::LoadYamlFile<FrankaSimParams>(
      "examples/franka/parameters/franka_sim_params.yaml");
  FrankaLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<FrankaLcmChannels>(FLAGS_lcm_channels);

  drake::systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  MultibodyPlant<double> plant(0.0);

  Parser parser(&plant, &scene_graph);
  parser.SetAutoRenaming(true);
  drake::multibody::ModelInstanceIndex franka_index =
      parser.AddModels(drake::FindResourceOrThrow(sim_params.franka_model))[0];

  drake::multibody::ModelInstanceIndex end_effector_index =
      parser.AddModels(FindResourceOrThrow(sim_params.end_effector_model))[0];
  drake::multibody::ModelInstanceIndex tray_index =
      parser.AddModels(FindResourceOrThrow(sim_params.tray_model))[0];
  drake::multibody::ModelInstanceIndex box_index =
      parser.AddModels(FindResourceOrThrow(sim_params.box_model))[0];
  multibody::AddFlatTerrain(&plant, &scene_graph, 1.0, 1.0);

  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  Vector3d franka_origin = Eigen::VectorXd::Zero(3);

  RigidTransform<double> R_X_W = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), franka_origin);
  RigidTransform<double> T_EE_W = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), sim_params.tool_attachment_frame);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"),
                   R_X_W);
  plant.WeldFrames(plant.GetFrameByName("panda_link7"),
                   plant.GetFrameByName("plate", end_effector_index), T_EE_W);

  if (sim_params.scene_index == 1){
    drake::multibody::ModelInstanceIndex left_support_index =
        parser.AddModels(FindResourceOrThrow(sim_params.left_support_model))[0];
    drake::multibody::ModelInstanceIndex right_support_index =
        parser.AddModels(FindResourceOrThrow(sim_params.right_support_model))[0];
    RigidTransform<double> T_S1_W = RigidTransform<double>(
        drake::math::RotationMatrix<double>(), sim_params.left_support_position);
    RigidTransform<double> T_S2_W = RigidTransform<double>(
        drake::math::RotationMatrix<double>(), sim_params.right_support_position);
    plant.WeldFrames(plant.world_frame(),
                     plant.GetFrameByName("support", left_support_index),
                     T_S1_W);
    plant.WeldFrames(plant.world_frame(),
                     plant.GetFrameByName("support", right_support_index),
                     T_S2_W);
  }

  plant.Finalize();

  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();

  // Create state receiver.
  auto franka_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(
          lcm_channel_params.franka_state_channel, lcm));
  auto tray_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.tray_state_channel, lcm));
  auto box_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.box_state_channel, lcm));
  auto franka_state_receiver =
      builder.AddSystem<RobotOutputReceiver>(plant, franka_index);
  auto tray_state_receiver =
      builder.AddSystem<ObjectStateReceiver>(plant, tray_index);
  auto box_state_receiver =
      builder.AddSystem<ObjectStateReceiver>(plant, box_index);

  auto franka_passthrough = builder.AddSystem<SubvectorPassThrough>(
      franka_state_receiver->get_output_port(0).size(), 0,
      plant.num_positions(franka_index));
  auto tray_passthrough = builder.AddSystem<SubvectorPassThrough>(
      tray_state_receiver->get_output_port(0).size(), 0,
      plant.num_positions(tray_index));
  auto box_passthrough = builder.AddSystem<SubvectorPassThrough>(
      tray_state_receiver->get_output_port(0).size(), 0,
      plant.num_positions(box_index));

  std::vector<int> input_sizes = {plant.num_positions(franka_index),
                                  plant.num_positions(tray_index),
                                  plant.num_positions(box_index)};
  auto mux =
      builder.AddSystem<drake::systems::Multiplexer<double>>(input_sizes);

  auto trajectory_sub_actor = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.c3_actor_channel, lcm));
  auto trajectory_sub_object = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.c3_object_channel, lcm));
  auto to_pose =
      builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant);

  drake::geometry::MeshcatVisualizerParams params;
  params.publish_period = 1.0 / 60.0;
  params.enable_alpha_slider = true;
  auto meshcat = std::make_shared<drake::geometry::Meshcat>();
  auto visualizer = &drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
      &builder, scene_graph, meshcat, std::move(params));
  auto trajectory_drawer_actor =
      builder.AddSystem<systems::LcmTrajectoryDrawer>(meshcat,
                                                      "end_effector_position_target");
  auto trajectory_drawer_object =
      builder.AddSystem<systems::LcmTrajectoryDrawer>(meshcat, "object_position_target");
  auto object_pose_drawer =
      builder.AddSystem<systems::LcmPoseDrawer>(
          meshcat, FindResourceOrThrow(sim_params.tray_model), "object_position_target",
          "object_orientation_target");
  auto end_effector_pose_drawer =
      builder.AddSystem<systems::LcmPoseDrawer>(
          meshcat, FindResourceOrThrow(sim_params.end_effector_model), "end_effector_position_target",
          "end_effector_orientation_target");
  auto end_effector_force_drawer =
      builder.AddSystem<systems::LcmForceDrawer>(
          meshcat, "end_effector_position_target",
          "end_effector_force_target");
  trajectory_drawer_actor->SetLineColor(drake::geometry::Rgba({1, 0, 0, 1}));
  trajectory_drawer_object->SetLineColor(drake::geometry::Rgba({0, 0, 1, 1}));
  trajectory_drawer_actor->SetNumSamples(5);
  trajectory_drawer_object->SetNumSamples(5);

  builder.Connect(franka_passthrough->get_output_port(),
                  mux->get_input_port(0));
  builder.Connect(tray_passthrough->get_output_port(), mux->get_input_port(1));
  builder.Connect(box_passthrough->get_output_port(), mux->get_input_port(2));
  builder.Connect(trajectory_sub_actor->get_output_port(),
                  trajectory_drawer_actor->get_input_port_trajectory());
  builder.Connect(trajectory_sub_object->get_output_port(),
                  trajectory_drawer_object->get_input_port_trajectory());
  builder.Connect(trajectory_sub_object->get_output_port(),
                  object_pose_drawer->get_input_port_trajectory());
  builder.Connect(trajectory_sub_actor->get_output_port(),
                  end_effector_pose_drawer->get_input_port_trajectory());
  builder.Connect(trajectory_sub_actor->get_output_port(),
                  end_effector_force_drawer->get_input_port_trajectory());
  builder.Connect(*mux, *to_pose);
  builder.Connect(
      to_pose->get_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(*franka_state_receiver, *franka_passthrough);
  builder.Connect(*tray_state_receiver, *tray_passthrough);
  builder.Connect(*box_state_receiver, *box_passthrough);

  builder.Connect(*franka_state_sub, *franka_state_receiver);
  builder.Connect(*tray_state_sub, *tray_state_receiver);
  builder.Connect(*box_state_sub, *box_state_receiver);

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  auto& franka_state_sub_context =
      diagram->GetMutableSubsystemContext(*franka_state_sub, context.get());
  auto& tray_state_sub_context =
      diagram->GetMutableSubsystemContext(*tray_state_sub, context.get());
  auto& box_state_sub_context =
      diagram->GetMutableSubsystemContext(*box_state_sub, context.get());
  franka_state_receiver->InitializeSubscriberPositions(
      plant, franka_state_sub_context);
  tray_state_receiver->InitializeSubscriberPositions(plant,
                                                     tray_state_sub_context);
  box_state_receiver->InitializeSubscriberPositions(plant,
                                                    box_state_sub_context);

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
