#include <iostream>

#include <dairlib/lcmt_timestamped_saved_traj.hpp>
#include <gflags/gflags.h>
#include <chrono>
#include <thread>

#include "common/eigen_utils.h"
#include "common/find_resource.h"

#include "examples/iC3/visualization/parameter_headers/trajectory_video_params.h"
#include "examples/iC3/visualization/lcm_pose_getter.h"


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
using drake::math::RigidTransform;
using drake::math::RotationMatrixd;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;


DEFINE_bool(is_simulation, true, "True for simulation, false for hardware");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Load parameters.
  TrajectoryVideoParams video_params =
      drake::yaml::LoadYamlFile<TrajectoryVideoParams>("examples/iC3/visualization/parameters/trajectory_video_params_plate.yaml");

  drake::systems::DiagramBuilder<double> builder;

  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.001);
  scene_graph.set_name("scene_graph");

  Parser parser(&plant, &scene_graph);
  parser.SetAutoRenaming(true);

  parser.AddModels(video_params.ee_file);
  parser.AddModels(video_params.cube_file);

  plant.Finalize();
  std::cout << "plant built" << std::endl;

  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();

  auto trajectory_sub_x = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          video_params.trajectory_lcm_channel_x, lcm));

  auto pose_getter = 
      builder.AddSystem<LcmPoseGetter>(video_params, "pose_getter", 0);

  builder.Connect(trajectory_sub_x->get_output_port(), pose_getter->get_input_port_trajectory());

  drake::geometry::MeshcatVisualizerParams params;
  params.publish_period = 1.0 / video_params.visualizer_publish_rate;
  auto meshcat = std::make_shared<drake::geometry::Meshcat>();
  auto visualizer = &drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
    &builder, scene_graph, meshcat, std::move(params));

  drake::geometry::Rgba cube_color(video_params.trace_color[0], 
      video_params.trace_color[1], video_params.trace_color[2], 1.0);  
  drake::geometry::Rgba hand_color(video_params.ee_trace_color[0], 
      video_params.ee_trace_color[1], video_params.ee_trace_color[2], 1.0);

  // Draw target
  std::vector<double> x_des = video_params.x_des;
  VectorXd xd = Eigen::Map<Eigen::VectorXd>(x_des.data(), x_des.size()); 

  Eigen::Vector4d q_vec = xd.segment(5, 4);
	Eigen::Quaterniond q(q_vec(0), q_vec(1), q_vec(2), q_vec(3));
	q.normalize();
  RotationMatrixd R_target(q);
	RigidTransformd X_WF(R_target, xd.segment(9, 3));

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

  // Start lcm systems
  auto simulator =
      std::make_unique<Simulator<double>>(*diagram, std::move(context));
  simulator->set_target_realtime_rate(0.001);
  simulator->set_publish_every_time_step(false);
  auto& root_context = simulator->get_mutable_context();
  auto& pose_getter_context = pose_getter->GetMyMutableContextFromRoot(&root_context);

  double current_sim_time = 0.0;
  const double sim_step = 0.01; 
  bool message_received = false;
  while (!message_received) {
    current_sim_time += sim_step;
    simulator->AdvanceTo(current_sim_time);
    if (pose_getter->get_input_port_trajectory().HasValue(pose_getter_context)) {
        message_received = true;
    }
  }
  std::cout << "current sim time " << current_sim_time << std::endl;


  visualizer->StartRecording(true);

  auto& plant_context = plant.GetMyMutableContextFromRoot(&root_context);

  for (int k = 0; k < video_params.trajectory_length; k++) {

    drake::systems::BasicVector<double> timestep(1);
    timestep.get_mutable_value()(0) = static_cast<double>(k);
    pose_getter->get_input_port_timestep().FixValue(&pose_getter_context, timestep);
    
    Eigen::VectorXd x = pose_getter->get_output_port(0)
                            .Eval<drake::systems::BasicVector<double>>(pose_getter_context).get_value();

    plant.SetPositionsAndVelocities(&plant_context, x);
    root_context.SetTime(k * video_params.dt);

    diagram->ForcedPublish(root_context);
  }

  // 4. Send the completely recorded animation sequence to the browser
  visualizer->PublishRecording();

  // Keep server alive for viewing
  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::do_main(argc, argv);
}
