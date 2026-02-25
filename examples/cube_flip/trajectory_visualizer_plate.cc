#include <iostream>

#include <dairlib/lcmt_timestamped_saved_traj.hpp>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "common/find_resource.h"

#include "examples/cube_flip/parameter_headers/trajectory_visualizer_params.h"
#include "examples/cube_flip/trajectory_lcm_parser_plate.h"


#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "systems/system_utils.h"
#include "systems/trajectory_optimization/lcm_trajectory_systems.h"
#include "systems/visualization/lcm_visualization_systems.h"

#include "drake/common/find_resource.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/meshcat_visualizer_params.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace dairlib {

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::geometry::DrakeVisualizer;
using drake::geometry::SceneGraph;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::systems::Simulator;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;


DEFINE_bool(is_simulation, true, "True for simulation, false for hardware");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Load parameters.
  CubeFlipVisualizerParams vis_params =
      drake::yaml::LoadYamlFile<CubeFlipVisualizerParams>("examples/cube_flip/parameters/trajectory_vis_params_plate.yaml");

  drake::systems::DiagramBuilder<double> builder;

  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.001);
  scene_graph.set_name("scene_graph");

  // Build the visualizer plant.
  Parser parser(&plant, &scene_graph);
  parser.SetAutoRenaming(true);

  parser.AddModels(vis_params.ee_file);
  parser.AddModels(vis_params.cube_file);

  plant.Finalize();

  std::cout << "plant built" << std::endl;

  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();

  auto trajectory_sub_x = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          vis_params.trajectory_lcm_channel_x, lcm));

  auto trajectory_sub_c3 = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          vis_params.trajectory_lcm_channel_c3, lcm));

  auto trajectory_sub_x_real = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          vis_params.trajectory_lcm_channel_x_real, lcm));


  auto trajectory_splitter =
      builder.AddSystem<TrajectoryLcmParserPlate>(vis_params, 0, "cube_trajectory_splitter");

  auto plate_trajectory_splitter = 
      builder.AddSystem<TrajectoryLcmParserPlate>(vis_params, 1, "plate_trajectory_splitter");

  auto c3_plate_trajectory_splitter = 
      builder.AddSystem<TrajectoryLcmParserPlate>(vis_params, 1, "c3_plate_trajectory_splitter");

  auto c3_trajectory_splitter =
      builder.AddSystem<TrajectoryLcmParserPlate>(vis_params, 0, "c3_trajectory_splitter");

  auto real_trajectory_splitter =
      builder.AddSystem<TrajectoryLcmParserPlate>(vis_params, 0, "real_trajectory_splitter");

  drake::geometry::MeshcatVisualizerParams params;
  params.publish_period = 1.0 / vis_params.visualizer_publish_rate;
  auto meshcat = std::make_shared<drake::geometry::Meshcat>();

  std::cout << "before pose drawers" << std::endl;

  // Draw trajectory
  int skip_factor = vis_params.iter_downsampling_factor;
  std::vector<systems::LcmPoseDrawer*> trajectory_drawers;
  std::vector<systems::LcmPoseDrawer*> plate_trajectory_drawers;
  std::vector<systems::LcmPoseDrawer*> c3_plate_trajectory_drawers;
  std::vector<systems::LcmPoseDrawer*> c3_trajectory_drawers;
  std::vector<systems::LcmPoseDrawer*> real_trajectory_drawers;

  for (int i = skip_factor - 1; i < vis_params.ic3_num_iters; i += skip_factor) {
    std::cout << "pose drawer " << i << std::endl;
   
    auto* drawer = builder.AddSystem<systems::LcmPoseDrawer>(
            meshcat,
            FindResourceOrThrow(vis_params.cube_file),
            "positions_" + std::to_string(i), 
            "orientations_" + std::to_string(i),
            "iteration_" + std::to_string(i), 
            vis_params.trajectory_length / vis_params.downsampling_factor, true,
            vis_params.trace_color);

   auto* c3_drawer = builder.AddSystem<systems::LcmPoseDrawer>(
            meshcat,
            FindResourceOrThrow(vis_params.cube_file),
            "positions_" + std::to_string(i), 
            "orientations_" + std::to_string(i),
            "c3_iteration_" + std::to_string(i), 
            vis_params.trajectory_length / vis_params.downsampling_factor, true,
            vis_params.trace_color);


    auto* plate_drawer = builder.AddSystem<systems::LcmPoseDrawer>(
            meshcat,
            FindResourceOrThrow(vis_params.ee_file),
            "positions_" + std::to_string(i), 
            "orientations_" + std::to_string(i),
            "plate_iteration_" + std::to_string(i), 
            vis_params.trajectory_length / vis_params.downsampling_factor, true,
            vis_params.ee_trace_color);

    auto* c3_plate_drawer = builder.AddSystem<systems::LcmPoseDrawer>(
        meshcat,
        FindResourceOrThrow(vis_params.ee_file),
        "positions_" + std::to_string(i), 
        "orientations_" + std::to_string(i),
        "c3_plate_iteration_" + std::to_string(i), 
        vis_params.trajectory_length / vis_params.downsampling_factor, true,
        vis_params.ee_trace_color);

    auto* real_drawer = builder.AddSystem<systems::LcmPoseDrawer>(
        meshcat,
        FindResourceOrThrow(vis_params.cube_file),
        "positions_" + std::to_string(i), 
        "orientations_" + std::to_string(i),
        "real_iteration_" + std::to_string(i), 
        vis_params.trajectory_length / vis_params.downsampling_factor, true,
        vis_params.trace_color);


    trajectory_drawers.push_back(drawer);
    plate_trajectory_drawers.push_back(plate_drawer);
    c3_plate_trajectory_drawers.push_back(c3_plate_drawer);
    c3_trajectory_drawers.push_back(c3_drawer);
    real_trajectory_drawers.push_back(real_drawer);

  }

  builder.Connect(trajectory_sub_x->get_output_port(), trajectory_splitter->get_input_port_trajectory());
  builder.Connect(trajectory_sub_x_real->get_output_port(), plate_trajectory_splitter->get_input_port_trajectory());
  builder.Connect(trajectory_sub_c3->get_output_port(), c3_plate_trajectory_splitter->get_input_port_trajectory());
  builder.Connect(trajectory_sub_c3->get_output_port(), c3_trajectory_splitter->get_input_port_trajectory());
  builder.Connect(trajectory_sub_x_real->get_output_port(), real_trajectory_splitter->get_input_port_trajectory());

  for (int i = 0; i < vis_params.ic3_num_iters / skip_factor; i++) {
    builder.Connect(trajectory_splitter->get_output_port(i), 
        trajectory_drawers.at(i)->get_input_port_trajectory());
    builder.Connect(plate_trajectory_splitter->get_output_port(i), 
        plate_trajectory_drawers.at(i)->get_input_port_trajectory());
    builder.Connect(c3_plate_trajectory_splitter->get_output_port(i), 
        c3_plate_trajectory_drawers.at(i)->get_input_port_trajectory());
    builder.Connect(c3_trajectory_splitter->get_output_port(i), 
        c3_trajectory_drawers.at(i)->get_input_port_trajectory());
    builder.Connect(real_trajectory_splitter->get_output_port(i), 
        real_trajectory_drawers.at(i)->get_input_port_trajectory());
  }

  auto visualizer = &drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
    &builder, scene_graph, meshcat, std::move(params));

  std::cout << "before draw target" << std::endl;

  // Draw target
  std::vector<double> x_des = vis_params.x_des;
  VectorXd xd = Eigen::Map<Eigen::VectorXd>(x_des.data(), x_des.size()); 

  Eigen::Vector4d q_vec = xd.segment(5, 4);
	Eigen::Quaterniond q(q_vec(0), q_vec(1), q_vec(2), q_vec(3));
	q.normalize();
  RotationMatrixd R_target(q);
	RigidTransformd X_WF(R_target, xd.segment(10, 3));

	const double axis_len = 0.2;
	const double radius = 0.01;

	meshcat->SetObject("target_pose/x_axis", drake::geometry::Cylinder(radius, axis_len), drake::geometry::Rgba(1, 0, 0, 1));
	RigidTransformd X_FX(
		RotationMatrixd::MakeYRotation(-M_PI / 2.0),
		Eigen::Vector3d(axis_len / 2.0, 0, 0));
	meshcat->SetTransform("target_pose/x_axis", X_WF * X_FX);

	meshcat->SetObject("target_pose/y_axis", drake::geometry::Cylinder(radius, axis_len), drake::geometry::Rgba(0, 1, 0, 1));
	RigidTransformd X_FY(
		RotationMatrixd::MakeXRotation(M_PI / 2.0),
		Eigen::Vector3d(0, axis_len / 2.0, 0));
	meshcat->SetTransform("target_pose/y_axis", X_WF * X_FY);

	meshcat->SetObject("target_pose/z_axis", drake::geometry::Cylinder(radius, axis_len), drake::geometry::Rgba(0, 0, 1, 1));
	RigidTransformd X_FZ(
		RotationMatrixd::Identity(),
		Eigen::Vector3d(0, 0, axis_len / 2.0));
	meshcat->SetTransform("target_pose/z_axis", X_WF * X_FZ);


  auto diagram = builder.Build();
  diagram->set_name(("iC3_trajectory_visualizer"));
  DrawAndSaveDiagramGraph(*diagram);
  auto context = diagram->CreateDefaultContext();

  std::vector<double> x_init = vis_params.x_init;
  VectorXd x0 = Eigen::Map<Eigen::VectorXd>(x_init.data(), x_init.size());

  std::cout << x0.transpose() << std::endl;
  std::cout << plant.num_positions() << ", " << plant.num_velocities() << std::endl;

	auto& plant_context =
      diagram->GetMutableSubsystemContext(plant, context.get());
  plant.SetPositionsAndVelocities(&plant_context, x0);

  /// Use the simulator to drive at a fixed rate
  /// If set_publish_every_time_step is true, this publishes twice
  auto simulator =
      std::make_unique<Simulator<double>>(*diagram, std::move(context));
  // simulator->set_publish_every_time_step(false);
  // simulator->set_publish_at_initialization(false);
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
