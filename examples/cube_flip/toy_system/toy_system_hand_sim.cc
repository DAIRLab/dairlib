#include <cmath>
#include <chrono>
#include <iostream>
#include <thread>

#include <dairlib/lcmt_radio_out.hpp>
#include <drake/common/find_resource.h>
#include <drake/common/yaml/yaml_io.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/geometry/meshcat_visualizer_params.h>
#include <drake/multibody/meshcat/contact_visualizer.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/externally_applied_spatial_force.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/lcm/lcm_interface_system.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake/systems/lcm/lcm_subscriber_system.h>
#include <drake/systems/primitives/multiplexer.h>
#include <drake/visualization/visualization_config_functions.h>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "common/find_resource.h"

#include "multibody/multibody_utils.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "examples/cube_flip/toy_system/toy_system_params.h"


DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");

using drake::geometry::SceneGraph;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::ModelInstanceIndex;
using drake::systems::DiagramBuilder;
using drake::systems::lcm::LcmSubscriberSystem;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using dairlib::systems::AddActuationRecieverAndStateSenderLcm;
using drake::math::RigidTransform;
using drake::math::RotationMatrix;

namespace dairlib {

// Takes ic3 trajectory and executes it
int RunToySystem(drake::lcm::DrakeLcm& lcm) {

  ToySystemParams toy_params =
      drake::yaml::LoadYamlFile<ToySystemParams>(
          "examples/cube_flip/toy_system/toy_system_hand_params.yaml");

  // Build the main diagram.
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0001);
  Parser parser(&plant, &scene_graph);

  const std::string hand_file = "examples/cube_flip/urdf/fingertips_lcs.sdf";
	const std::string cube_file = "examples/cube_flip/urdf/actual_cube.sdf";
	const std::string ground_file = "examples/cube_flip/urdf/ground.urdf";

  ModelInstanceIndex ee_idx = parser.AddModels(hand_file)[0];
  ModelInstanceIndex object_idx = parser.AddModels(cube_file)[0];
  parser.AddModels(ground_file);

  RigidTransform<double> X_G = RigidTransform<double>(
    drake::math::RotationMatrix<double>(), {0, 0, 0.0});

  // RigidTransform<double> X_1 = RigidTransform<double>(
  //   drake::math::RotationMatrix<double>(), {0.08, 0, 0.05});
  // RigidTransform<double> X_2 = RigidTransform<double>(
  //   drake::math::RotationMatrix<double>(), {-0.08, 0, 0.05});
  // RigidTransform<double> X_3 = RigidTransform<double>(
  //   drake::math::RotationMatrix<double>(), {0, 0.08, 0.05});
  // RigidTransform<double> X_4 = RigidTransform<double>(
  //   drake::math::RotationMatrix<double>(), {0, -0.08, 0.05});

  RigidTransform<double> X_1 = RigidTransform<double>(
    drake::math::RotationMatrix<double>(), {0.08, 0, 0.05});
  RigidTransform<double> X_2 = RigidTransform<double>(
    drake::math::RotationMatrix<double>(), {-0.08, -0.04, 0.05});
  RigidTransform<double> X_3 = RigidTransform<double>(
    drake::math::RotationMatrix<double>(), {-0.08, 0.04, 0.05});

  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("base_link_1"), X_1);
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("base_link_2"), X_2);
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("base_link_3"), X_3);
  // plant.WeldFrames(plant.world_frame(),
  //                  plant.GetFrameByName("base_link_4"), X_4);                                                  
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("ground"), X_G);

  plant.Finalize();

  std::cout << "After finalize plant" << std::endl;

  // Just here for synchronized starting
  auto input_sub =
    builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_input>(
        "TOY_HAND_INPUT", &lcm));
  input_sub->set_name("start_switch");

  auto lcm_system =
      builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(&lcm);
  AddActuationRecieverAndStateSenderLcm(
      &builder, plant, lcm_system, "TOY_HAND_INPUT",
      "TOY_HAND_STATE", 1000,
      ee_idx, true, 0.0);

  auto object_state_sender =
      builder.AddSystem<systems::ObjectStateSender>(plant, true, object_idx);
  auto object_state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_object_state>(
          "TOY_OBJECT_STATE", lcm_system,
          1.0 / 1000));

  builder.Connect(plant.get_state_output_port(object_idx),
                  object_state_sender->get_input_port_state());
  builder.Connect(object_state_sender->get_output_port(),
                  object_state_pub->get_input_port());

  // Set up Meshcat visualizer.
  auto meshcat = std::make_shared<drake::geometry::Meshcat>(7001);
  drake::geometry::MeshcatVisualizerParams vis_params;

  drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
      &builder, scene_graph, meshcat, std::move(vis_params));

  drake::multibody::meshcat::ContactVisualizer<double>::AddToBuilder(
      &builder, plant, meshcat,
      drake::multibody::meshcat::ContactVisualizerParams());

  std::cout << "Before sim" << std::endl;

  auto diagram = builder.Build();
  drake::systems::Simulator<double> simulator(*diagram);

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(toy_params.sim_rate);

  // Set the initial state of the system.
  Eigen::VectorXd x0 = toy_params.x_init;
  std::cout << x0.transpose() << std::endl;
  auto& plant_context = diagram->GetMutableSubsystemContext(
      plant, &simulator.get_mutable_context());    
  plant.SetPositionsAndVelocities(&plant_context, x0);

  simulator.Initialize();

  LcmHandleSubscriptionsUntil(
    &lcm, [&]() { return input_sub->GetInternalMessageCount() > 0; });

  std::cout << "sim started" << std::endl;
  simulator.AdvanceTo(std::numeric_limits<double>::infinity());  

  return 0;
}

} // namespace dairlib

int main(int argc, char* argv[]) {
  // run with "bazel run //examples/cube_flip/toy_system:toy_system"


  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::lcm::DrakeLcm lcm(FLAGS_lcm_url);

  dairlib::RunToySystem(lcm);
  return -1;
}
