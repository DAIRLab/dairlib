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
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "common/find_resource.h"
#include "examples/plate-balancing/parameters/lcm_channel_config.h"
#include "examples/plate-balancing/parameters/plate_balancing_config.h"
#include "examples/plate-balancing/parameters/simulation_config.h"
#include "examples/plate-balancing/parameters/simulation_scene_config.h"
#include "examples/plate-balancing/systems/external_force_generator.h"
#include "multibody/multibody_utils.h"
#include "systems/primitives/radio_parser.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

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

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace dairlib {

using multibody::MakeNameToPositionsMap;
using multibody::MakeNameToVelocitiesMap;
using systems::AddActuationRecieverAndStateSenderLcm;
using systems::ObjectStateSender;
using systems::RadioToVector;
using systems::SubvectorPassThrough;

namespace examples {
namespace plate_balancing {

DEFINE_string(plate_balancing_config,
              "examples/plate-balancing/config/plate_balancing_config.yaml",
              "Controller settings such as channels.");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Load parameters from YAML files.
  PlateBalancingConfig main_config =
      drake::yaml::LoadYamlFile<PlateBalancingConfig>(
          FLAGS_plate_balancing_config);
  SimulationConfig sim_params = drake::yaml::LoadYamlFile<SimulationConfig>(
      main_config.simulation_config_file);
  LcmChannelConfig lcm_channel_params =
      drake::yaml::LoadYamlFile<LcmChannelConfig>(
          main_config.lcm_simulation_settings_file);
  SimulationSceneConfig scene_params =
      drake::yaml::LoadYamlFile<SimulationSceneConfig>(
          main_config.get_simulation_scene_config_file());

  // Build the Drake diagram.
  DiagramBuilder<double> builder;
  double sim_dt = sim_params.dt;

  // Add MultibodyPlant and SceneGraph.
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, sim_dt);

  // Parse models and add them to the plant.
  Parser parser(&plant);
  parser.SetAutoRenaming(true);
  drake::multibody::ModelInstanceIndex franka_index =
      parser.AddModelsFromUrl(sim_params.franka_model)[0];
  drake::multibody::ModelInstanceIndex end_effector_index =
      parser.AddModels(FindResourceOrThrow(sim_params.end_effector_model))[0];
  drake::multibody::ModelInstanceIndex tray_index =
      parser.AddModels(FindResourceOrThrow(sim_params.tray_model))[0];
  drake::multibody::ModelInstanceIndex object_index =
      parser.AddModels(FindResourceOrThrow(sim_params.object_model))[0];
  multibody::AddFlatTerrain(&plant, &scene_graph, 1.0, 1.0);

  // Weld frames to position the robot and objects in the world.
  Vector3d franka_origin = Eigen::VectorXd::Zero(3);
  RigidTransform<double> T_X_W = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), franka_origin);
  RigidTransform<double> T_EE_W = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), sim_params.tool_attachment_frame);

  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"),
                   T_X_W);
  plant.WeldFrames(plant.GetFrameByName("panda_link7"),
                   plant.GetFrameByName("plate", end_effector_index), T_EE_W);

  // Define collision geometries and filter groups.
  const drake::geometry::GeometrySet& franka_geom_set =
      plant.CollectRegisteredGeometries({&plant.GetBodyByName("panda_link0"),
                                         &plant.GetBodyByName("panda_link1"),
                                         &plant.GetBodyByName("panda_link2"),
                                         &plant.GetBodyByName("panda_link3"),
                                         &plant.GetBodyByName("panda_link4")});

  drake::geometry::GeometrySet support_geom_set;
  std::vector<drake::multibody::ModelInstanceIndex> environment_model_indices;
  environment_model_indices.resize(scene_params.environment_models.size());
  for (int i = 0; i < scene_params.environment_models.size(); ++i) {
    environment_model_indices[i] = parser.AddModels(
        FindResourceOrThrow(scene_params.environment_models[i]))[0];
    RigidTransform<double> T_E_W =
        RigidTransform<double>(drake::math::RollPitchYaw<double>(
                                   scene_params.environment_orientations[i]),
                               scene_params.environment_positions[i]);
    plant.WeldFrames(plant.world_frame(),
                     plant.GetFrameByName("base", environment_model_indices[i]),
                     T_E_W);
    support_geom_set.Add(plant.GetCollisionGeometriesForBody(
        plant.GetBodyByName("base", environment_model_indices[i])));
  }
  plant.ExcludeCollisionGeometriesWithCollisionFilterGroupPair(
      {"supports", support_geom_set}, {"franka", franka_geom_set});

  const drake::geometry::GeometrySet& franka_only_geom_set =
      plant.CollectRegisteredGeometries({
          &plant.GetBodyByName("panda_link2"),
          &plant.GetBodyByName("panda_link3"),
          &plant.GetBodyByName("panda_link4"),
          &plant.GetBodyByName("panda_link5"),
          &plant.GetBodyByName("panda_link6"),
          &plant.GetBodyByName("panda_link7"),
          &plant.GetBodyByName("panda_link8"),
      });
  auto tray_collision_set = GeometrySet(
      plant.GetCollisionGeometriesForBody(plant.GetBodyByName("tray")));
  plant.ExcludeCollisionGeometriesWithCollisionFilterGroupPair(
      {"franka", franka_only_geom_set}, {"tray", tray_collision_set});

  plant.Finalize();

  // Add LCM interface system.
  drake::lcm::DrakeLcm drake_lcm;
  auto lcm =
      builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(&drake_lcm);

  // Add systems for sending and receiving robot actuation commands and state.
  AddActuationRecieverAndStateSenderLcm(
      &builder, plant, lcm, lcm_channel_params.franka_input_channel,
      lcm_channel_params.franka_state_channel, sim_params.franka_publish_rate,
      franka_index, sim_params.publish_efforts, sim_params.actuator_delay);

  // Add systems for sending tray state.
  auto tray_state_sender = builder.AddSystem<ObjectStateSender>(
      plant, sim_params.publish_object_velocities, tray_index);
  auto tray_state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.tray_state_channel, lcm,
          1.0 / sim_params.tray_publish_rate));

  // Add systems for sending object state.
  auto object_state_sender = builder.AddSystem<ObjectStateSender>(
      plant, sim_params.publish_object_velocities, object_index);
  auto object_state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.object_state_channel, lcm,
          1.0 / sim_params.object_publish_rate));

  // Connect tray state sender and publisher.
  builder.Connect(plant.get_state_output_port(tray_index),
                  tray_state_sender->get_input_port_state());
  builder.Connect(tray_state_sender->get_output_port(),
                  tray_state_pub->get_input_port());

  // Connect object state sender and publisher.
  builder.Connect(plant.get_state_output_port(object_index),
                  object_state_sender->get_input_port_state());
  builder.Connect(object_state_sender->get_output_port(),
                  object_state_pub->get_input_port());

  // Add external force generator and connect it to the radio subscriber.
  auto external_force_generator =
      builder.AddSystem<systems::ExternalForceGenerator>(
          plant.GetBodyByName("tray").index());
  external_force_generator->SetRemoteControlParameters(
      sim_params.external_force_scaling[0],
      sim_params.external_force_scaling[1],
      sim_params.external_force_scaling[2]);
  auto radio_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
          lcm_channel_params.radio_channel, &drake_lcm));
  auto radio_to_vector = builder.AddSystem<RadioToVector>();
  builder.Connect(*radio_sub, *radio_to_vector);
  builder.Connect(radio_to_vector->get_output_port(),
                  external_force_generator->get_input_port_radio());
  builder.Connect(external_force_generator->get_output_port_spatial_force(),
                  plant.get_applied_spatial_force_input_port());

  // Add visualizer if enabled.
  if (sim_params.visualize_drake_sim) {
    drake::visualization::AddDefaultVisualization(&builder);
  }

  // Build the diagram.
  auto diagram = builder.Build();

  // Create a simulator.
  drake::systems::Simulator<double> simulator(*diagram);

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(sim_params.realtime_rate);

  // Set initial state.
  auto& plant_context = diagram->GetMutableSubsystemContext(
      plant, &simulator.get_mutable_context());

  int nq = plant.num_positions();
  int nv = plant.num_velocities();
  VectorXd q = VectorXd::Zero(nq);

  q.head(plant.num_positions(franka_index)) = sim_params.q_init_franka;

  q.segment(plant.num_positions(franka_index),
            plant.num_positions(tray_index)) =
      sim_params.q_init_tray[main_config.scene_index];
  q.tail(plant.num_positions(object_index)) =
      sim_params.q_init_object[main_config.scene_index];

  plant.SetPositions(&plant_context, q);

  VectorXd v = VectorXd::Zero(nv);
  plant.SetVelocities(&plant_context, v);

  // Initialize and run the simulation.
  simulator.Initialize();
  simulator.AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}
}  // namespace plate_balancing
}  // namespace examples
}  // namespace dairlib

int main(int argc, char* argv[]) {
  dairlib::examples::plate_balancing::DoMain(argc, argv);
}
