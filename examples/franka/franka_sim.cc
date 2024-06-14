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
#include "examples/franka/parameters/franka_lcm_channels.h"
#include "examples/franka/parameters/franka_sim_params.h"
#include "examples/franka/parameters/franka_sim_scene_params.h"
#include "examples/franka/systems/external_force_generator.h"
#include "multibody/multibody_utils.h"
#include "systems/primitives/radio_parser.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

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
using multibody::MakeNameToPositionsMap;
using multibody::MakeNameToVelocitiesMap;
using systems::AddActuationRecieverAndStateSenderLcm;

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

DEFINE_string(lcm_channels,
              "examples/franka/parameters/lcm_channels_simulation.yaml",
              "Filepath containing lcm channels");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  // load parameters
  FrankaSimParams sim_params = drake::yaml::LoadYamlFile<FrankaSimParams>(
      "examples/franka/parameters/franka_sim_params.yaml");
  FrankaLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<FrankaLcmChannels>(FLAGS_lcm_channels);
  FrankaSimSceneParams scene_params =
      drake::yaml::LoadYamlFile<FrankaSimSceneParams>(
          sim_params.sim_scene_file[sim_params.scene_index]);

  // load urdf and sphere
  DiagramBuilder<double> builder;
  double sim_dt = sim_params.dt;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, sim_dt);

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

  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  Vector3d franka_origin = Eigen::VectorXd::Zero(3);

  RigidTransform<double> T_X_W = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), franka_origin);
  RigidTransform<double> T_EE_W = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), sim_params.tool_attachment_frame);

  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"),
                   T_X_W);
  plant.WeldFrames(plant.GetFrameByName("panda_link7"),
                   plant.GetFrameByName("plate", end_effector_index), T_EE_W);

  // we WANT to model collisions between link5 and the supports
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
  /* -------------------------------------------------------------------------------------------*/

  drake::lcm::DrakeLcm drake_lcm;
  auto lcm =
      builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(&drake_lcm);
  AddActuationRecieverAndStateSenderLcm(
      &builder, plant, lcm, lcm_channel_params.franka_input_channel,
      lcm_channel_params.franka_state_channel, sim_params.franka_publish_rate,
      franka_index, sim_params.publish_efforts, sim_params.actuator_delay);
  auto tray_state_sender =
      builder.AddSystem<systems::ObjectStateSender>(plant, sim_params.publish_object_velocities, tray_index);
  auto tray_state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.tray_state_channel, lcm,
          1.0 / sim_params.tray_publish_rate));
  auto object_state_sender =
      builder.AddSystem<systems::ObjectStateSender>(plant, sim_params.publish_object_velocities, object_index);
  auto object_state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.object_state_channel, lcm,
          1.0 / sim_params.object_publish_rate));

  builder.Connect(plant.get_state_output_port(tray_index),
                  tray_state_sender->get_input_port_state());
  builder.Connect(tray_state_sender->get_output_port(),
                  tray_state_pub->get_input_port());
  builder.Connect(plant.get_state_output_port(object_index),
                  object_state_sender->get_input_port_state());
  builder.Connect(object_state_sender->get_output_port(),
                  object_state_pub->get_input_port());

  auto external_force_generator = builder.AddSystem<ExternalForceGenerator>(
      plant.GetBodyByName("tray").index());
  external_force_generator->SetRemoteControlParameters(
      sim_params.external_force_scaling[0],
      sim_params.external_force_scaling[1],
      sim_params.external_force_scaling[2]);
  auto radio_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
          lcm_channel_params.radio_channel, &drake_lcm));
  auto radio_to_vector = builder.AddSystem<systems::RadioToVector>();
  builder.Connect(*radio_sub, *radio_to_vector);
  builder.Connect(radio_to_vector->get_output_port(),
                  external_force_generator->get_input_port_radio());
  builder.Connect(external_force_generator->get_output_port_spatial_force(),
                  plant.get_applied_spatial_force_input_port());

  int nq = plant.num_positions();
  int nv = plant.num_velocities();

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

  q.head(plant.num_positions(franka_index)) = sim_params.q_init_franka;

  q.segment(plant.num_positions(franka_index),
            plant.num_positions(tray_index)) =
      sim_params.q_init_tray[sim_params.scene_index];
  q.tail(plant.num_positions(object_index)) =
      sim_params.q_init_object[sim_params.scene_index];

  plant.SetPositions(&plant_context, q);

  VectorXd v = VectorXd::Zero(nv);
  plant.SetVelocities(&plant_context, v);

  simulator.Initialize();
  simulator.AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv); }
