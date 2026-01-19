#include <math.h>

#include <vector>

#include <dairlib/lcmt_radio_out.hpp>
#include <drake/common/find_resource.h>
#include <drake/common/yaml/yaml_io.h>
#include <drake/geometry/meshcat_visualizer.h>
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
#include <drake/multibody/tree/linear_bushing_roll_pitch_yaw.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/solvers/solve.h>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "common/find_resource.h"

#include "examples/cube_flip/trifinger_plate/parameter_headers/trifinger_plate_lcm_channels.h"
#include "examples/cube_flip/trifinger_plate/parameter_headers/trifinger_plate_sim_params.h"
#include "examples/cube_flip/trifinger_plate/systems/trifinger_plate_lcm_systems.h"

#include "examples/franka/systems/external_force_generator.h"
#include "multibody/multibody_utils.h"
#include "systems/primitives/radio_parser.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include <iostream>

namespace dairlib {

using dairlib::systems::SubvectorPassThrough;
using drake::geometry::GeometrySet;
using drake::geometry::SceneGraph;
using drake::math::RigidTransform;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::trajectories::PiecewisePolynomial;
using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;
using multibody::MakeNameToPositionsMap;
using multibody::MakeNameToVelocitiesMap;
using dairlib::AddActuationRecieverAndStateSenderTrifingerPlate;

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

DEFINE_string(lcm_channels,
              "examples/cube_flip/trifinger_plate/parameters/trifinger_lcm_channels_simulation.yaml",
              "Filepath containing lcm channels");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  // load parameters
  TrifingerPlateSimParams sim_params = drake::yaml::LoadYamlFile<TrifingerPlateSimParams>(
      "examples/cube_flip/trifinger_plate/parameters/trifinger_plate_sim_params.yaml");
  TrifingerPlateLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<TrifingerPlateLcmChannels>(FLAGS_lcm_channels);


  // load urdf and sphere
  DiagramBuilder<double> builder;
  double sim_dt = sim_params.dt;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, sim_dt);

  Parser parser(&plant);
  parser.SetAutoRenaming(true);

  parser.package_map().Add(
    "robot_properties_fingers", 
    "examples/cube_flip/trifinger_plate/robot_properties_fingers"
  );

  drake::multibody::ModelInstanceIndex trifinger_index = 
    parser.AddModels(FindResourceOrThrow(sim_params.trifinger_model))[0];
  drake::multibody::ModelInstanceIndex end_effector_index = 
      parser.AddModels(FindResourceOrThrow(sim_params.end_effector_model))[0];
  drake::multibody::ModelInstanceIndex object_index =
      parser.AddModels(FindResourceOrThrow(sim_params.object_model))[0];
  multibody::AddFlatTerrain(&plant, &scene_graph, 1.0, 1.0);

	const auto& plate_body = plant.GetBodyByName("plate", end_effector_index);

	// Attach each finger to the bottom of the plate
	std::vector<VectorXd> plate_socket_offsets = sim_params.finger_attachment_points;
	std::vector<std::string> tip_names = {
			"finger_tip_link_0", 
			"finger_tip_link_120", 
			"finger_tip_link_240"
	};

  // Link fingertips and plate
	for (int i = 0; i < 3; ++i) {
		drake::math::RigidTransformd X_ParentSocket(
			drake::math::RotationMatrixd::Identity(), 
			plate_socket_offsets[i]
		);

		const auto& tip_frame = plant.GetFrameByName(tip_names[i], trifinger_index);
		const drake::multibody::Frame<double>& plate_socket_frame = 
				plant.AddFrame(std::make_unique<drake::multibody::FixedOffsetFrame<double>>(
								"plate_socket_" + std::to_string(i),
								plate_body,
								X_ParentSocket 
				));

		// const Eigen::Vector3d trans_stiffness(1e6, 1e6, 1e6);
		// const Eigen::Vector3d trans_damping(1e3, 1e3, 1e3);

    const Eigen::Vector3d trans_stiffness(sim_params.translation_stiffness * Vector3d::Ones());
    const Eigen::Vector3d trans_damping(sim_params.translation_damping * Vector3d::Ones());

		const Eigen::Vector3d rot_stiffness(0, 0, 0); 
		const Eigen::Vector3d rot_damping(0.1, 0.1, 0.1); // Small damping for stability

		// Add the force element
		plant.AddForceElement<drake::multibody::LinearBushingRollPitchYaw>(
				tip_frame, 
				plate_socket_frame, 
				rot_stiffness, 
				rot_damping, 
				trans_stiffness, 
				trans_damping
		);
	}


//  const drake::geometry::GeometrySet& trifinger_only_geom_set =
//       plant.CollectRegisteredGeometries({
//           &plant.GetBodyByName("finger_base_link_0"),
//           &plant.GetBodyByName("finger_upper_link_0"),
//           &plant.GetBodyByName("finger_lower_link_0"),
//           &plant.GetBodyByName("finger_tip_link_0"),
//           &plant.GetBodyByName("finger_base_link_120"),
//           &plant.GetBodyByName("finger_upper_link_120"),
//           &plant.GetBodyByName("finger_lower_link_120"),
//           &plant.GetBodyByName("finger_tip_link_120"),
//           &plant.GetBodyByName("finger_base_link_240"),
//           &plant.GetBodyByName("finger_upper_link_240"),
//           &plant.GetBodyByName("finger_lower_link_240"),
//           &plant.GetBodyByName("finger_tip_link_240"),
//        });

//   auto plate_collision_set = GeometrySet(
//       plant.GetCollisionGeometriesForBody(plant.GetBodyByName("plate")));
//   plant.ExcludeCollisionGeometriesWithCollisionFilterGroupPair(
//       {"trifinger", trifinger_only_geom_set}, {"plate", plate_collision_set});
  

  // Weld trifinger base to world upside down
  Eigen::Vector3d base_translation(0.3 * Eigen::Vector3d::UnitZ());
	RigidTransformd X_WI(RotationMatrixd::MakeXRotation(M_PI), base_translation);
	plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link"), X_WI);
	plant.Finalize();

	std::vector<std::string> position_names = plant.GetPositionNames();
  std::cout << "--- Positions (q) ---" << std::endl;
  for (int i = 0; i < position_names.size(); ++i) {
      std::cout << "q[" << i << "]: " << position_names[i] << std::endl;
  }

  // 2. Get the names of all velocity variables (v)
  std::vector<std::string> velocity_names = plant.GetVelocityNames();
  std::cout << "\n--- Velocities (v) ---" << std::endl;
  for (int i = 0; i < velocity_names.size(); ++i) {
      std::cout << "v[" << i << "]: " << velocity_names[i] << std::endl;
  }
	std::cout << "\n--- Control Input Port (u) Names ---" << std::endl;
	for (const auto& name : plant.GetActuatorNames()) {
			std::cout << name << std::endl;
	}
/* -------------------------------------------------------------------------------------------*/
  
  drake::lcm::DrakeLcm drake_lcm;
  auto lcm =
      builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(&drake_lcm);

	int nq = plant.num_positions();
  int nv = plant.num_velocities();
	int nu = plant.num_actuators();
 
	std::cout << "nq: " << nq << std::endl;
	std::cout << "nv: " << nv << std::endl;
	std::cout << "nu: " << nu << std::endl;

	AddActuationRecieverAndStateSenderTrifingerPlate(
      &builder, plant, lcm, lcm_channel_params.trifinger_input_channel,
      lcm_channel_params.trifinger_state_channel, sim_params.trifinger_publish_rate,
      trifinger_index, end_effector_index,
      sim_params.publish_efforts, sim_params.actuator_delay);
	
	std::cout << "AFTER AddActuationRecieverAndStateSenderLcm" << std::endl;
  auto object_state_sender =
      builder.AddSystem<systems::ObjectStateSender>(plant, sim_params.publish_object_velocities, object_index);
  auto object_state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.object_state_channel, lcm,
          1.0 / sim_params.object_publish_rate));

	std::cout << "AFTER object_state_pub" << std::endl;

  builder.Connect(plant.get_state_output_port(object_index),
                  object_state_sender->get_input_port_state());
  builder.Connect(object_state_sender->get_output_port(),
                  object_state_pub->get_input_port());

  auto radio_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
          lcm_channel_params.radio_channel, &drake_lcm));
  auto radio_to_vector = builder.AddSystem<systems::RadioToVector>();
  builder.Connect(*radio_sub, *radio_to_vector);

	std::cout << "After radio" << std::endl;


  if (sim_params.visualize_drake_sim) {
    drake::visualization::AddDefaultVisualization(&builder);
  }

  auto diagram = builder.Build();
  drake::systems::Simulator<double> simulator(*diagram);

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(sim_params.realtime_rate);

  auto& plant_context = diagram->GetMutableSubsystemContext(
      plant, &simulator.get_mutable_context());


  VectorXd q_init_plate = sim_params.q_init_plate;
  Eigen::Quaterniond plate_rot(q_init_plate(0), q_init_plate(1), q_init_plate(2), q_init_plate(3));
  drake::math::RigidTransformd X_WPlate_target(plate_rot.normalized(), q_init_plate.tail(3));

  // Set up IK
  drake::multibody::InverseKinematics ik(plant);
  for (int i = 0; i < 3; ++i) {
    Eigen::Vector3d p_WS = X_WPlate_target * plate_socket_offsets[i];

    ik.AddPositionConstraint(
        plant.GetFrameByName(tip_names[i]), 
        Eigen::Vector3d::Zero(),           
        plant.world_frame(),                
        p_WS, p_WS                          
    );  
}

  // // Solve IK to get finger poses
  // VectorXd q_nominal = plant.GetPositions(plant_context);
  // q_nominal.tail(14).head(7) = sim_params.q_init_plate;
  // ik.get_mutable_prog()->AddQuadraticErrorCost(
  //     Eigen::MatrixXd::Identity(plant.num_positions(), plant.num_positions()),
  //     q_nominal, 
  //     ik.q()
  // );
  VectorXd q_seed(23);
  q_seed << 0.0, -1.0, -1.5, 0.0, -1.0, -1.5, 0.0, -1.0, -1.5, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;

  ik.get_mutable_prog()->SetInitialGuess(ik.q(), q_seed);
  auto result = drake::solvers::Solve(ik.prog());
  if (result.is_success()) {
    Eigen::VectorXd q_sol = result.GetSolution(ik.q());
    q_sol.tail(14).head(7) = sim_params.q_init_plate;
    q_sol.tail(7) = sim_params.q_init_object; 
    std::cout << "q_sol " << q_sol.transpose() << std::endl;
    plant.SetPositions(&plant_context, q_sol);
  } else {
    std::cout << "IK failed. Solver: " << result.get_solver_id().name() << std::endl;
    std::cout << "Solution Result: " << result.get_solution_result() << std::endl;  
  }
  
  auto x = plant.GetPositionsAndVelocities(plant_context);
  std::cout << "Max value in state: " << x.array().abs().maxCoeff() << std::endl;
  if (!x.allFinite()) {
      for (int i=0; i < x.size(); ++i) {
          if (!std::isfinite(x(i))) {
              std::cout << "NaN detected at index " << i << " (" << plant.GetPositionNames()[i] << ")" << std::endl;
          }
      }
  }
  simulator.Initialize();
  simulator.AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv); }
