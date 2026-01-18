#include <cmath>
#include <chrono>
#include <iostream>

#include <drake/geometry/drake_visualizer.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/geometry/meshcat_visualizer_params.h>
#include <drake/multibody/meshcat/contact_visualizer.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/lcm/lcm_subscriber_system.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/systems/primitives/vector_log_sink.h>
#include <gflags/gflags.h>
#include <drake/systems/framework/leaf_system.h>

#include "examples/cube_flip/toy_system/fixed_input.h"


DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");

using drake::SortedPair;
using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;
using drake::geometry::GeometryId;
using drake::geometry::SceneGraph;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;
using drake::systems::lcm::LcmSubscriberSystem;


// Takes ic3 trajectory and executes it
int RunToySystem(drake::lcm::DrakeLcm& lcm) {
  // Build the plant and scene graph for the pivoting system.
  DiagramBuilder<double> plant_builder;
  auto [plant_for_lcs, scene_graph_for_lcs] =
      AddMultibodyPlantSceneGraph(&plant_builder, 0);
  Parser parser_for_lcs(&plant_for_lcs, &scene_graph_for_lcs);

  const std::string plate_file_lcs = "examples/cube_flip/urdf/plate.sdf";
	const std::string cube_file_lcs = "examples/cube_flip/urdf/cube.sdf";

  parser_for_lcs.AddModels(plate_file_lcs);
  parser_for_lcs.AddModels(cube_file_lcs);

  plant_for_lcs.Finalize();

  // Build the plant diagram.
  auto plant_diagram = plant_builder.Build();

  // Build the main diagram.
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0001);
  Parser parser(&plant, &scene_graph);
  const std::string plate_file = "examples/cube_flip/urdf/plate.sdf";
	const std::string cube_file = "examples/cube_flip/urdf/cube.sdf";

  parser.AddModels(plate_file);
  parser.AddModels(cube_file);

  plant.Finalize();

  // Create contexts for the plant and LCS factory system.
  std::unique_ptr<drake::systems::Context<double>> plant_diagram_context =
      plant_diagram->CreateDefaultContext();
  auto plant_autodiff =
      drake::systems::System<double>::ToAutoDiffXd(plant_for_lcs);
  auto& plant_for_lcs_context = plant_diagram->GetMutableSubsystemContext(
      plant_for_lcs, plant_diagram_context.get());
  auto plant_context_autodiff = plant_autodiff->CreateDefaultContext();



	for (const auto& pname : plant.GetPositionNames()) {
		std::cout << pname << std::endl;
	}
	for (const auto& vname : plant.GetVelocityNames()) {
		std::cout << vname << std::endl;
	}
	
	
  Eigen::VectorXd xd(23);
  //xd << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
	xd << 0, 0, 0,
        0, 0,
        0, 0, 1, 0,
        -0.2, 0, 0, 
        0, 0, 0,
        0, 0,
        0, 0, 0, 
        0, 0, 0;


	std::cout << "xd: " << xd.transpose() << std::endl;
  auto ic3_x_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          "iC3_TRAJECTORY_X", &lcm));

  auto ic3_u_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          "iC3_TRAJECTORY_U", &lcm));
          
  auto fixed_input = builder.AddSystem<dairlib::FixedInput>(plant, 100, 0.01);
	builder.Connect(ic3_u_sub->get_output_port(),
									fixed_input->get_input_port_trajectory());
	builder.Connect(fixed_input->get_output_port_u(),
									plant.get_actuation_input_port());

	Eigen::Vector4d q_vec = xd.segment(5, 4);
	Eigen::Quaterniond q(q_vec(0), q_vec(1), q_vec(2), q_vec(3));
	q.normalize();
  RotationMatrixd R_target(q);
	RigidTransformd X_WF(R_target, xd.segment(10, 3));

  // Set up Meshcat visualizer.
  auto meshcat = std::make_shared<drake::geometry::Meshcat>(7001);
  drake::geometry::MeshcatVisualizerParams params;

  drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
      &builder, scene_graph, meshcat, std::move(params));

  drake::multibody::meshcat::ContactVisualizer<double>::AddToBuilder(
      &builder, plant, meshcat,
      drake::multibody::meshcat::ContactVisualizerParams());

	const double axis_len = 0.2;
	const double radius = 0.01;

	// meshcat->SetObject("target_pose/x_axis", drake::geometry::Cylinder(radius, axis_len), drake::geometry::Rgba(1, 0, 0, 1));
	// RigidTransformd X_FX(
	// 	RotationMatrixd::MakeYRotation(-M_PI / 2.0),
	// 	Eigen::Vector3d(axis_len / 2.0, 0, 0));
	// meshcat->SetTransform("target_pose/x_axis", X_WF * X_FX);

	// meshcat->SetObject("target_pose/y_axis", drake::geometry::Cylinder(radius, axis_len), drake::geometry::Rgba(0, 1, 0, 1));
	// RigidTransformd X_FY(
	// 	RotationMatrixd::MakeXRotation(M_PI / 2.0),
	// 	Eigen::Vector3d(0, axis_len / 2.0, 0));
	// meshcat->SetTransform("target_pose/y_axis", X_WF * X_FY);

	// meshcat->SetObject("target_pose/z_axis", drake::geometry::Cylinder(radius, axis_len), drake::geometry::Rgba(0, 0, 1, 1));
	// RigidTransformd X_FZ(
	// 	RotationMatrixd::Identity(),
	// 	Eigen::Vector3d(0, 0, axis_len / 2.0));
	// meshcat->SetTransform("target_pose/z_axis", X_WF * X_FZ);


  // Build the diagram.
  auto diagram = builder.Build();

  // Create a default context for the diagram.
  auto diagram_context = diagram->CreateDefaultContext();

  // Set the initial state of the system.
  Eigen::VectorXd x0(23);
	x0 << 0, 0, -0.02,
        0, 0,   
        1, 0, 0, 0, 
        -0.13, 0, 0,
        0, 0, 0,
        0, 0, 
        0, 0, 0, 
        0, 0, 0;


	auto& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());
  plant.SetPositionsAndVelocities(&plant_context, x0);


  LcmHandleSubscriptionsUntil(
    &lcm, [&]() { return ic3_u_sub->GetInternalMessageCount() > 1; });

  // Create and configure the simulator.
  drake::systems::Simulator<double> simulator(*diagram,
                                              std::move(diagram_context));
  simulator.set_target_realtime_rate(0.5);  // Run simulation at 0.5 speed.
  simulator.Initialize();
  simulator.AdvanceTo(10.0 + 100 * 0.01);  

  return 0;
}

int main(int argc, char* argv[]) {
  // run with "bazel run //examples/cube_flip/toy_system:toy_system"


  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::lcm::DrakeLcm lcm(FLAGS_lcm_url);

  RunToySystem(lcm);
  return -1;
}
