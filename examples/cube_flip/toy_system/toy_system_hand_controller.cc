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
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/zero_order_hold.h>
#include <drake/systems/framework/diagram_builder.h>
#include "drake/systems/primitives/pass_through.h"

#include <gflags/gflags.h>
#include <drake/systems/framework/leaf_system.h>
#include "drake/common/yaml/yaml_io.h"

#include "examples/cube_flip/toy_system/lqr_input.h"
#include "examples/cube_flip/toy_system/fixed_input.h"
#include "dairlib/lcmt_lqr_output.hpp"
#include "systems/controllers/c3/ic3_tracking_controller.h"
#include "systems/controllers/c3/lcs_factory_system.h"
#include "systems/system_utils.h"
#include "systems/robot_lcm_systems.h"
#include "systems/framework/lcm_driven_loop.h"

#include "examples/cube_flip/parameter_headers/iC3_options.h"
#include "solvers/c3_options.h"
#include "examples/cube_flip/toy_system/toy_system_params.h"
#include "examples/cube_flip/toy_system/toy_utils.h"


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
using drake::multibody::ModelInstanceIndex;
using drake::systems::DiagramBuilder;
using drake::systems::lcm::LcmSubscriberSystem;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::math::RigidTransform;
using drake::math::RotationMatrix;

namespace dairlib {

// Takes ic3 trajectory and executes it
int RunToySystem(drake::lcm::DrakeLcm& lcm) {

  iC3Options ic3_options =
      drake::yaml::LoadYamlFile<iC3Options>(
          "examples/cube_flip/toy_system/toy_ic3_hand_options.yaml");

  C3Options c3_options =
      drake::yaml::LoadYamlFile<C3Options>(
          "examples/cube_flip/toy_system/toy_c3_hand_options.yaml");

  ToySystemParams toy_params =
      drake::yaml::LoadYamlFile<ToySystemParams>(
          "examples/cube_flip/toy_system/toy_system_hand_params.yaml");

  DiagramBuilder<double> plant_builder;
  auto [plant_for_lcs, scene_graph_for_lcs] =
      AddMultibodyPlantSceneGraph(&plant_builder, 0);
  Parser parser_for_lcs(&plant_for_lcs, &scene_graph_for_lcs);

  const std::string ee_file_lcs = "examples/cube_flip/urdf/fingertips_lcs.sdf";
	const std::string cube_file_lcs = "examples/cube_flip/urdf/cube_for_lcs.sdf";
	const std::string ground_file_lcs = "examples/cube_flip/urdf/ground.urdf";

  parser_for_lcs.AddModels(ee_file_lcs);
  parser_for_lcs.AddModels(cube_file_lcs);
  parser_for_lcs.AddModels(ground_file_lcs);

  RigidTransform<double> X_G_lcs = RigidTransform<double>(
    drake::math::RotationMatrix<double>(), {0, 0, 0.0});
  // RigidTransform<double> X_1_lcs = RigidTransform<double>(
  //   drake::math::RotationMatrix<double>(), {0.08, 0, 0.05});
  // RigidTransform<double> X_2_lcs = RigidTransform<double>(
  //   drake::math::RotationMatrix<double>(), {-0.08, 0, 0.05});
  // RigidTransform<double> X_3_lcs = RigidTransform<double>(
  //   drake::math::RotationMatrix<double>(), {0, 0.08, 0.05});
  // RigidTransform<double> X_4_lcs = RigidTransform<double>(
  //   drake::math::RotationMatrix<double>(), {0, -0.08, 0.05});

  RigidTransform<double> X_1_lcs = RigidTransform<double>(
    drake::math::RotationMatrix<double>(), {0.08, 0, 0.05});
  RigidTransform<double> X_2_lcs = RigidTransform<double>(
    drake::math::RotationMatrix<double>(), {-0.08, -0.04, 0.05});
  RigidTransform<double> X_3_lcs = RigidTransform<double>(
    drake::math::RotationMatrix<double>(), {-0.08, 0.04, 0.05});

  plant_for_lcs.WeldFrames(plant_for_lcs.world_frame(),
                          plant_for_lcs.GetFrameByName("base_link_1"), X_1_lcs);
  plant_for_lcs.WeldFrames(plant_for_lcs.world_frame(),
                          plant_for_lcs.GetFrameByName("base_link_2"), X_2_lcs);
  plant_for_lcs.WeldFrames(plant_for_lcs.world_frame(),
                          plant_for_lcs.GetFrameByName("base_link_3"), X_3_lcs);
  // plant_for_lcs.WeldFrames(plant_for_lcs.world_frame(),
  //                         plant_for_lcs.GetFrameByName("base_link_4"), X_4_lcs);                                                  
  plant_for_lcs.WeldFrames(plant_for_lcs.world_frame(),
                          plant_for_lcs.GetFrameByName("ground"), X_G_lcs);

  plant_for_lcs.Finalize();


  // Build the plant diagram.
 
  // Build the plant diagram.
  auto plant_diagram = plant_builder.Build();

  // Retrieve collision geometries for relevant bodies.
  GeometryId ground_collision_geom = 
    plant_for_lcs.GetCollisionGeometriesForBody(
          plant_for_lcs.GetBodyByName("ground"))[0];

  std::vector<GeometryId> fingertip_collision_geoms;
  // Index, middle, ring, thumb
  fingertip_collision_geoms.push_back(
    plant_for_lcs.GetCollisionGeometriesForBody(
        plant_for_lcs.GetBodyByName("fingertip_1"))[0]);
  fingertip_collision_geoms.push_back(
    plant_for_lcs.GetCollisionGeometriesForBody(
        plant_for_lcs.GetBodyByName("fingertip_2"))[0]);
  fingertip_collision_geoms.push_back(
    plant_for_lcs.GetCollisionGeometriesForBody(
        plant_for_lcs.GetBodyByName("fingertip_3"))[0]);
  // fingertip_collision_geoms.push_back(
  //   plant_for_lcs.GetCollisionGeometriesForBody(
  //       plant_for_lcs.GetBodyByName("fingertip_4"))[0]); 

	std::vector<GeometryId> cube_collision_geoms;
  for (int i = 0; i <= 8; i++) {
		cube_collision_geoms.push_back(
			plant_for_lcs.GetCollisionGeometriesForBody(
          plant_for_lcs.GetBodyByName("cube"))[i]);
	}

  // Define contact pairs for the LCS system.
  std::vector<SortedPair<GeometryId>> contact_pairs;

  // fingertip-cube contact pairs
	for (auto geom_id : fingertip_collision_geoms) {
		contact_pairs.emplace_back(cube_collision_geoms[0], geom_id);
  }

  for (auto geom_id : fingertip_collision_geoms) {
		contact_pairs.emplace_back(geom_id, ground_collision_geom);
  }
  // cube-ground contact pairs
  for (int i = 1; i < cube_collision_geoms.size(); i++) {
		contact_pairs.emplace_back(cube_collision_geoms[i], ground_collision_geom);
  }


  // Create contexts for the plant and LCS factory system.
  std::unique_ptr<drake::systems::Context<double>> plant_diagram_context =
      plant_diagram->CreateDefaultContext();
  auto plant_autodiff =
      drake::systems::System<double>::ToAutoDiffXd(plant_for_lcs);
  auto& plant_for_lcs_context = plant_diagram->GetMutableSubsystemContext(
      plant_for_lcs, plant_diagram_context.get());
  auto plant_context_autodiff = plant_autodiff->CreateDefaultContext();

	
  Eigen::VectorXd xd = toy_params.x_des;
	std::cout << "After xd" << std::endl;


  // Build the main diagram.
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0001);
  Parser parser(&plant, &scene_graph);

  const std::string ee_file = "examples/cube_flip/urdf/fingertips.sdf";
	const std::string cube_file = "examples/cube_flip/urdf/actual_cube.sdf";
	const std::string ground_file = "examples/cube_flip/urdf/ground.urdf";

  ModelInstanceIndex ee_idx = parser.AddModels(ee_file)[0];
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

	for (const auto& pname : plant.GetPositionNames()) {
		std::cout << pname << std::endl;
	}
	for (const auto& vname : plant.GetVelocityNames()) {
		std::cout << vname << std::endl;
	}

  auto ic3_x_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          "iC3_TRAJECTORY_X", &lcm));

  auto ic3_u_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          "iC3_TRAJECTORY_U", &lcm));

  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant, ee_idx);

  auto object_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_object_state>(
          "TOY_OBJECT_STATE", &lcm));
  auto object_state_receiver =
      builder.AddSystem<systems::ObjectStateReceiver>(plant, object_idx);      

  auto ee_command_sender =
    builder.AddSystem<systems::RobotCommandSender>(plant);
  auto ee_command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          "TOY_HAND_INPUT", &lcm,
          TriggerTypeSet({TriggerType::kForced})));

  auto hand_kinematics = builder.AddSystem<HandKinematics>(plant);

  auto u_passthrough =
      builder.AddSystem<drake::systems::PassThrough<double>>(plant.num_actuators());

  auto u_to_timestamped =
      builder.AddSystem<Vector2TimestampedVector>(plant.num_actuators());

  builder.Connect(object_state_sub->get_output_port(),
                  object_state_receiver->get_input_port());
  builder.Connect(state_receiver->get_output_port(),
                  hand_kinematics->get_input_port_x_hand());  
  builder.Connect(object_state_receiver->get_output_port(),
                  hand_kinematics->get_input_port_x_object());

  if (toy_params.input_type == "open") {
    auto fixed_input = builder.AddSystem<dairlib::FixedInput>(plant, ic3_options.N, ic3_options.dt, 
                                                      ic3_options.iter_to_use, toy_params.time_to_wait);
    builder.Connect(ic3_u_sub->get_output_port(),
                    fixed_input->get_input_port_trajectory());
    builder.Connect(fixed_input->get_output_port_u(),
                    u_passthrough->get_input_port());           

  } else if (toy_params.input_type == "lqr") {
    auto lqr_sub =
          builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_lqr_output>(
              "iC3_LQR", &lcm));

    auto lqr_controller = builder.AddSystem<dairlib::LQRInput>(plant, ic3_options.N, ic3_options.dt, 
                                              ic3_options.iter_to_use, toy_params.time_to_wait);
    

    builder.Connect(hand_kinematics->get_output_port_state(),
                    lqr_controller->get_input_port_x_curr());
    builder.Connect(ic3_x_sub->get_output_port(),
                    lqr_controller->get_input_port_ic3_x());
    builder.Connect(ic3_u_sub->get_output_port(),
                    lqr_controller->get_input_port_ic3_u());
    builder.Connect(lqr_sub->get_output_port(),
                    lqr_controller->get_input_port_lqr());    
          
    builder.Connect(lqr_controller->get_output_port_u(),
                    u_passthrough->get_input_port()); 
                                       
  } else if (toy_params.input_type == "c3") {

    auto lqr_sub =
          builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_lqr_output>(
              "iC3_LQR", &lcm));
    auto controller =
        builder.AddSystem<systems::iC3TrackingController>(plant_for_lcs, c3_options, ic3_options, toy_params.time_to_wait);
    auto vector_to_timestamped_vector =
      builder.AddSystem<Vector2TimestampedVector>(plant.num_positions() + plant.num_velocities());
    auto lcs_factory_system = builder.AddSystem<systems::LCSFactorySystem>(
      plant_for_lcs, plant_for_lcs_context, *plant_autodiff,
      *plant_context_autodiff, contact_pairs, c3_options);
    auto xdes =
      builder.AddSystem<drake::systems::ConstantVectorSource<double>>(toy_params.x_des);
    auto c3_input = builder.AddSystem<C3Solution2InputHand>(plant);


    builder.Connect(hand_kinematics->get_output_port_state(),
                    vector_to_timestamped_vector->get_input_port_state());
    builder.Connect(vector_to_timestamped_vector->get_output_port_timestamped_state(),
                    controller->get_input_port_lcs_state());
    builder.Connect(lcs_factory_system->get_output_port_lcs(),
                    controller->get_input_port_lcs());
    builder.Connect(xdes->get_output_port(),
                    controller->get_input_port_target());
    builder.Connect(ic3_x_sub->get_output_port(),
                    controller->get_input_port_ic3_x());
    builder.Connect(ic3_u_sub->get_output_port(),
                    controller->get_input_port_ic3_u());

    builder.Connect(vector_to_timestamped_vector->get_output_port_timestamped_state(),
                    lcs_factory_system->get_input_port_lcs_state());
    builder.Connect(lqr_sub->get_output_port(),
                    controller->get_input_port_lqr());       
    
    builder.Connect(hand_kinematics->get_output_port_state(),
                    c3_input->get_input_port_curr_x());
    builder.Connect(controller->get_output_port_c3_solution(),
                    c3_input->get_input_port_c3_solution());
    builder.Connect(c3_input->get_output_port_c3_input(),
                    u_passthrough->get_input_port());
  } else {
    std::cout << "UNKNOWN INPUT TYPE" << std::endl;
  }
  builder.Connect(u_passthrough->get_output_port(),
                  u_to_timestamped->get_input_port_state());   
  builder.Connect(u_to_timestamped->get_output_port_timestamped_state(),
                  ee_command_sender->get_input_port());   
  builder.Connect(ee_command_sender->get_output_port(),
                  ee_command_pub->get_input_port());


  // Visualize target
	Eigen::Vector4d q_vec = xd.segment(9, 4);
	Eigen::Quaterniond q(q_vec(0), q_vec(1), q_vec(2), q_vec(3));
	q.normalize();
  RotationMatrixd R_target(q);
	RigidTransformd X_WF(R_target, xd.segment(13, 3));


	const double axis_len = 0.2;
	const double radius = 0.01;

  // Build the diagram.
  auto diagram = builder.Build();
  std::shared_ptr<Diagram<double>> shared_diagram = std::move(diagram);
  shared_diagram->set_name(("toy_system"));
  DrawAndSaveDiagramGraph(*shared_diagram, "/home/ericcui/diagrams/toy_system_hand");

  auto diagram_context = shared_diagram->CreateDefaultContext();
  // Set default input to prevent drift
  std::unique_ptr<drake::systems::Context<double>> plant_context = plant.CreateDefaultContext();		
  VectorXd tau_g = plant.CalcGravityGeneralizedForces(*plant_context);
  VectorXd u_default(VectorXd::Zero(plant.num_actuators()));
  // HARDCODED
  auto& pass_context = u_passthrough->GetMyMutableContextFromRoot(diagram_context.get());
  u_passthrough->get_input_port().FixValue(&pass_context, u_default);


  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, shared_diagram, state_receiver,
      "TOY_HAND_STATE", true);
  loop.Simulate();


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
