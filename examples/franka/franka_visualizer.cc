#include <iostream>

#include <drake/multibody/parsing/parser.h>
#include <drake/systems/primitives/multiplexer.h>
#include <gflags/gflags.h>

#include "dairlib/lcmt_robot_output.hpp"
#include "multibody/com_pose_system.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

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
#include "common/find_resource.h"

DEFINE_string(channel, "FRANKA_OUTPUT",
              "LCM channel for receiving state. "
              "Use FRANKA_OUTPUT to get state from simulator, and "
              "use FRANKA_ROS_OUTPUT to get state from state estimator");

namespace dairlib {

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

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

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  drake::systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  MultibodyPlant<double> plant(0.0);

  Parser parser(&plant, &scene_graph);
  drake::multibody::ModelInstanceIndex franka_index =
      parser.AddModelFromFile("examples/franka/urdf/franka.urdf");
  drake::multibody::ModelInstanceIndex table_index = parser.AddModelFromFile(
      drake::FindResourceOrThrow(
          "drake/examples/kuka_iiwa_arm/models/table/"
          "extra_heavy_duty_table_surface_only_collision.sdf"),
      "table0");
  drake::multibody::ModelInstanceIndex second_table_index =
      parser.AddModelFromFile(
          dairlib::FindResourceOrThrow("examples/franka/urdf/table.sdf"),
          "table1");
  drake::multibody::ModelInstanceIndex tray_index =
      parser.AddModelFromFile(FindResourceOrThrow(
          "examples/franka/urdf/tray.sdf"));

  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  Vector3d franka_origin = Eigen::VectorXd::Zero(3);
  Vector3d second_table_origin = Eigen::VectorXd::Zero(3);
  franka_origin(2) = 0.7645;
  second_table_origin(0) = 0.75;
  RigidTransform<double> R_X_W = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), franka_origin);
  RigidTransform<double> T_X_W = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), second_table_origin);
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("link", table_index), X_WI);
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("link", second_table_index), T_X_W);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"),
                   R_X_W);

  plant.Finalize();

  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();

  // Create state receiver.
  auto franka_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(
          FLAGS_channel, lcm));
  auto tray_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(
          "TRAY_OUTPUT", lcm));
  auto franka_state_receiver =
      builder.AddSystem<RobotOutputReceiver>(plant, franka_index);
  auto tray_state_receiver =
      builder.AddSystem<RobotOutputReceiver>(plant, tray_index);

  builder.Connect(*franka_state_sub, *franka_state_receiver);
  builder.Connect(*tray_state_sub, *tray_state_receiver);

  auto franka_passthrough = builder.AddSystem<SubvectorPassThrough>(
      franka_state_receiver->get_output_port(0).size(), 0,
      plant.num_positions(franka_index));
  auto tray_passthrough = builder.AddSystem<SubvectorPassThrough>(
      tray_state_receiver->get_output_port(0).size(), 0,
      plant.num_positions(tray_index));
  builder.Connect(*franka_state_receiver, *franka_passthrough);
  builder.Connect(*tray_state_receiver, *tray_passthrough);

  std::vector<int> input_sizes = {plant.num_positions(franka_index),
                                  plant.num_positions(tray_index)};
  auto mux =
      builder.AddSystem<drake::systems::Multiplexer<double>>(input_sizes);
  builder.Connect(franka_passthrough->get_output_port(),
                  mux->get_input_port(0));
  builder.Connect(tray_passthrough->get_output_port(), mux->get_input_port(1));

  auto to_pose =
      builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant);
  builder.Connect(*mux, *to_pose);
  builder.Connect(
      to_pose->get_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));

  DrakeVisualizer<double>::AddToBuilder(&builder, scene_graph);
  drake::geometry::MeshcatVisualizerParams params;
  params.publish_period = 1.0 / 60.0;
  auto meshcat = std::make_shared<drake::geometry::Meshcat>();
  auto visualizer = &drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
      &builder, scene_graph, meshcat, std::move(params));

  // state_receiver->set_publish_period(1.0/30.0);  // framerate

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  auto& franka_state_sub_context =
      diagram->GetMutableSubsystemContext(*franka_state_sub, context.get());
  auto& tray_state_sub_context =
      diagram->GetMutableSubsystemContext(*tray_state_sub, context.get());
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
