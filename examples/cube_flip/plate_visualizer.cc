#include <dairlib/lcmt_c3_forces.hpp>
#include <dairlib/lcmt_c3_state.hpp>
#include <dairlib/lcmt_sample_buffer.hpp>
#include <dairlib/lcmt_timestamped_saved_traj.hpp>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "common/find_resource.h"
#include "dairlib/lcmt_robot_output.hpp"

#include "examples/cube_flip/parameter_headers/visualizer_params.h"
#include "examples/cube_flip/trajectory_lcm_parser.h"


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

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Load parameters.
  CubeFlipVisualizerParams vis_params =
      drake::yaml::LoadYamlFile<CubeFlipVisualizerParams>("examples/cube_flip/vis_params.yaml");


  drake::systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  // Build the visualizer plant.
  MultibodyPlant<double> plant(0.0);

  Parser parser(&plant, &scene_graph);
  parser.SetAutoRenaming(true);
  
  ModelInstanceIndex plate_index = 
    parser.AddModels(FindResourceOrThrow(vis_params.plate_file))[0];
  ModelInstanceIndex cube_index = 
    parser.AddModels(FindResourceOrThrow(vis_params.cube_file))[0];

  plant.Finalize();


  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();


  auto trajectory_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          vis_params.trajectory_lcm_channel, lcm));

  auto trajectory_splitter =
      builder.AddSystem<TrajectoryLcmParser>(vis_params.ic3_num_iters, vis_params.trajectory_length);

  drake::geometry::MeshcatVisualizerParams params;
  params.publish_period = 1.0 / vis_params.visualizer_publish_rate;
  auto meshcat = std::make_shared<drake::geometry::Meshcat>();

  // Draw trajectory
  std::vector<systems::LcmPoseDrawer*> trajectory_drawers;
  for (int i = 0; i < vis_params.ic3_num_iters; i++) {
    trajectory_drawers.push_back(
      builder.AddSystem<systems::LcmPoseDrawer>(
            meshcat,
            FindResourceOrThrow(vis_params.cube_file),
            "position_" + std::to_string(i), 
            "orientation" + std::to_string(i),
            "trajectory_" + std::to_string(i), 
            vis_params.trajectory_length, true,
            vis_params.trace_color)
    );
  }

  builder.Connect(trajectory_sub->get_output_port(), trajectory_splitter->get_input_port_trajectory());

  for (int i = 0; i < vis_params.ic3_num_iters; i++) {
    builder.Connect(trajectory_splitter->get_output_port(i),
                    trajectory_drawers.at(i)->get_input_port_trajectory());
  }

  auto visualizer = &drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
    &builder, scene_graph, meshcat, std::move(params));

  auto diagram = builder.Build();
  diagram->set_name(("iC3_trajectory_visualizer"));
  DrawAndSaveDiagramGraph(*diagram);
  auto context = diagram->CreateDefaultContext();


  /// Use the simulator to drive at a fixed rate
  /// If set_publish_every_time_step is true, this publishes twice
  auto simulator =
      std::make_unique<Simulator<double>>(*diagram, std::move(context));
  simulator->set_publish_every_time_step(false);
  simulator->set_publish_at_initialization(false);
  simulator->set_target_realtime_rate(1.0);
  simulator->Initialize();

  drake::log()->info("visualizer started");
  std::cout << "Before simulator" << std::endl;

  simulator->AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::do_main(argc, argv);
}
