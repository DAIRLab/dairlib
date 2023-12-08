#include <iostream>

#include <dairlib/lcmt_timestamped_saved_traj.hpp>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/primitives/multiplexer.h>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "common/find_resource.h"
#include "dairlib/lcmt_robot_output.hpp"
#include "multibody/com_pose_system.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"
#include "systems/trajectory_optimization/lcm_trajectory_systems.h"

#include "examples/cube_franka/c3_parameters.h"

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
using dairlib::systems::SubvectorPassThrough;
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

DEFINE_string(channel, "FRANKA_OUTPUT",
              "LCM channel for receiving state. "
              "Use FRANKA_OUTPUT to get state from simulator, and "
              "use FRANKA_ROS_OUTPUT to get state from state estimator");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  C3Parameters param = drake::yaml::LoadYamlFile<C3Parameters>(
    "examples/cube_franka/parameters.yaml");

  drake::systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  MultibodyPlant<double> plant(0.0);

  Parser parser(&plant, &scene_graph);
  parser.AddModels("examples/cube_franka/robot_properties_fingers/urdf/franka_box.urdf");
  parser.AddModels("examples/cube_franka/robot_properties_fingers/urdf/jack.urdf");
  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"), X_WI);

  plant.Finalize();

  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();

  // Create state receiver.
  auto state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(
          FLAGS_channel, lcm));
  auto state_receiver = builder.AddSystem<RobotOutputReceiver>(plant);
  builder.Connect(*state_sub, *state_receiver);

  auto passthrough = builder.AddSystem<SubvectorPassThrough>(
      state_receiver->get_output_port(0).size(), 0, plant.num_positions());
  builder.Connect(*state_receiver, *passthrough);

  auto to_pose =
      builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant);
  builder.Connect(*passthrough, *to_pose);
  builder.Connect(
      to_pose->get_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));

  // state_receiver->set_publish_period(1.0/30.0);  // framerate


  drake::geometry::MeshcatVisualizerParams params;
  params.publish_period = 1.0 / 60.0;
  params.enable_alpha_slider = true;
  auto meshcat = std::make_shared<drake::geometry::Meshcat>();
  auto visualizer = &drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
      &builder, scene_graph, meshcat, std::move(params));
  auto trajectory_drawer_actor =
      builder.AddSystem<systems::LcmTrajectoryDrawer>(meshcat,
                                                      "c3_trajectory_generator_end_effector_position_target");
  auto trajectory_drawer_object =
      builder.AddSystem<systems::LcmTrajectoryDrawer>(meshcat, "c3_trajectory_generator_object_position_target");
  auto object_pose_drawer =
      builder.AddSystem<systems::LcmPoseDrawer>(
          meshcat, FindResourceOrThrow("examples/cube_franka/robot_properties_fingers/urdf/jack_current_sample.urdf"), "c3_trajectory_generator_object_position_target",
          "object_orientation_target", param.horizon_length, "current_location");
  auto end_effector_pose_drawer =
      builder.AddSystem<systems::LcmPoseDrawer>(
          meshcat, FindResourceOrThrow("examples/cube_franka/robot_properties_fingers/urdf/end_effector_current.urdf"), "c3_trajectory_generator_end_effector_position_target",
          "end_effector_orientation_target", param.horizon_length, "current_location");
  trajectory_drawer_actor->SetLineColor(drake::geometry::Rgba({1, 0, 0, 1}));
  trajectory_drawer_object->SetLineColor(drake::geometry::Rgba({0, 0, 1, 1}));
  trajectory_drawer_actor->SetNumSamples(5);
  trajectory_drawer_object->SetNumSamples(5);

  // Subscribe to the current position's C3 plan.
  auto trajectory_sub_actor = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          "ACTOR_CHANNEL", lcm));
  auto trajectory_sub_object = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          "OBJECT_CHANNEL", lcm));

  builder.Connect(trajectory_sub_actor->get_output_port(),
                  trajectory_drawer_actor->get_input_port_trajectory());
  builder.Connect(trajectory_sub_object->get_output_port(),
                  trajectory_drawer_object->get_input_port_trajectory());
  builder.Connect(trajectory_sub_object->get_output_port(),
                  object_pose_drawer->get_input_port_trajectory());
  builder.Connect(trajectory_sub_actor->get_output_port(),
                  end_effector_pose_drawer->get_input_port_trajectory());

  // Subscribe to the best sample's C3 plan.
  auto sample_trajectory_drawer_actor =
      builder.AddSystem<systems::LcmTrajectoryDrawer>(meshcat,
                                                      "c3_sample_trajectory_generator_end_effector_position_target");
  auto sample_trajectory_drawer_object =
      builder.AddSystem<systems::LcmTrajectoryDrawer>(meshcat, "c3_sample_trajectory_generator_object_position_target");
  auto sample_object_pose_drawer =
      builder.AddSystem<systems::LcmPoseDrawer>(
          meshcat, FindResourceOrThrow("examples/cube_franka/robot_properties_fingers/urdf/jack_best_sample.urdf"), "c3_sample_trajectory_generator_object_position_target",
          "c3_sample_trajectory_generator_object_orientation_target", 5, "sample_location"); 
  auto sample_end_effector_pose_drawer =
      builder.AddSystem<systems::LcmPoseDrawer>(
          meshcat, FindResourceOrThrow("examples/cube_franka/robot_properties_fingers/urdf/end_effector_best_sample.urdf"), "c3_sample_trajectory_generator_end_effector_position_target",
          "c3_sample_trajectory_generator_end_effector_orientation_target", 5, "sample_location");
  sample_trajectory_drawer_actor->SetLineColor(drake::geometry::Rgba({1, 0, 0, 1}));
  sample_trajectory_drawer_object->SetLineColor(drake::geometry::Rgba({0, 0, 1, 1}));
  sample_trajectory_drawer_actor->SetNumSamples(5);
  sample_trajectory_drawer_object->SetNumSamples(5);
  auto sample_trajectory_sub_actor = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          "ACTOR_SAMPLE_CHANNEL", lcm));
  auto sample_trajectory_sub_object = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          "OBJECT_SAMPLE_CHANNEL", lcm));

  builder.Connect(sample_trajectory_sub_actor->get_output_port(),
                  sample_trajectory_drawer_actor->get_input_port_trajectory());
  builder.Connect(sample_trajectory_sub_object->get_output_port(),
                  sample_trajectory_drawer_object->get_input_port_trajectory());
  builder.Connect(sample_trajectory_sub_object->get_output_port(),
                  sample_object_pose_drawer->get_input_port_trajectory());
  builder.Connect(sample_trajectory_sub_actor->get_output_port(),
                  sample_end_effector_pose_drawer->get_input_port_trajectory());


  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  auto& state_sub_context = diagram->GetMutableSubsystemContext(
      *state_sub, context.get());
  state_receiver->InitializeSubscriberPositions(plant, state_sub_context);

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
