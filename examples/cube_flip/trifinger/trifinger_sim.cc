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

#include "examples/cube_flip/trifinger/parameter_headers/trifinger_lcm_channels.h"
#include "examples/cube_flip/trifinger/parameter_headers/trifinger_sim_params.h"

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
using drake::multibody::ModelInstanceIndex;
using multibody::MakeNameToPositionsMap;
using multibody::MakeNameToVelocitiesMap;
using dairlib::systems::AddActuationRecieverAndStateSenderLcm;

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::vector;

DEFINE_string(lcm_channels,
              "examples/cube_flip/trifinger/parameters/trifinger_lcm_channels_simulation.yaml",
              "Filepath containing lcm channels");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  // load parameters
  TrifingerSimParams sim_params = drake::yaml::LoadYamlFile<TrifingerSimParams>(
      "examples/cube_flip/trifinger/parameters/trifinger_sim_params.yaml");
  TrifingerLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<TrifingerLcmChannels>(FLAGS_lcm_channels);


  // load urdf and sphere
  DiagramBuilder<double> builder;
  double sim_dt = sim_params.dt;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, sim_dt);

  Parser parser(&plant);
  parser.SetAutoRenaming(true);

  parser.package_map().Add(
    "robot_properties_fingers", 
    "examples/cube_flip/trifinger/robot_properties_fingers"
  );

  ModelInstanceIndex trifinger_index = 
    parser.AddModels(FindResourceOrThrow(sim_params.trifinger_model))[0];
  ModelInstanceIndex fingertips_index = 
    parser.AddModels(FindResourceOrThrow(sim_params.end_effector_model))[0];
  ModelInstanceIndex object_index =
      parser.AddModels(FindResourceOrThrow(sim_params.object_model))[0];
  multibody::AddFlatTerrain(&plant, &scene_graph, 1.0, 1.0);

  Eigen::Vector3d base_translation(-0 * Vector3d::UnitZ());
  RigidTransformd X_WI(drake::math::RotationMatrix<double>(), base_translation);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link"), X_WI);

  // HARDCODED
  RigidTransformd X_identity(drake::math::RotationMatrix<double>(), Eigen::Vector3d::Zero());
  vector<std::string> trifinger_tip_names = {
    "finger_tip_link_0", "finger_tip_link_120", "finger_tip_link_240"};
  vector<std::string> fingertip_names = {"fingertip_1", "fingertip_2", "fingertip_3"};

  for (int i = 0; i < 3; i++) {
    const auto& trifinger_tip_frame = plant.GetFrameByName(trifinger_tip_names[i], trifinger_index);
    const auto& fingertip_frame = plant.GetFrameByName(fingertip_names[i], fingertips_index);
    plant.WeldFrames(trifinger_tip_frame, fingertip_frame, X_identity);
  }
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

	AddActuationRecieverAndStateSenderLcm(
      &builder, plant, lcm, lcm_channel_params.trifinger_input_channel,
      lcm_channel_params.trifinger_state_channel, sim_params.trifinger_publish_rate,
      trifinger_index, sim_params.publish_efforts, sim_params.actuator_delay);
	
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

    
  VectorXd q = VectorXd::Zero(nq);
  q.head(plant.num_positions(trifinger_index)) = sim_params.q_init_trifinger;
  q.tail(plant.num_positions(object_index)) = sim_params.q_init_object;
  plant.SetPositions(&plant_context, q);
  
  VectorXd v = VectorXd::Zero(nv);
  plant.SetVelocities(&plant_context, v);
  
  simulator.Initialize();
  simulator.AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv); }
