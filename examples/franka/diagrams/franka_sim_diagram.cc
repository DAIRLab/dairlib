
#include "franka_sim_diagram.h"

#include <drake/common/find_resource.h>
#include <drake/common/yaml/yaml_io.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake/planning/robot_diagram_builder.h>
#include <drake/visualization/visualization_config_functions.h>

#include "common/find_resource.h"
#include "examples/franka/parameters/franka_lcm_channels.h"
#include "examples/franka/parameters/franka_sim_params.h"
#include "examples/franka/parameters/franka_sim_scene_params.h"
#include "systems/robot_lcm_systems.h"

namespace dairlib {
namespace examples {
using drake::systems::lcm::LcmPublisherSystem;

using drake::geometry::GeometrySet;
using drake::math::RigidTransform;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::systems::DiagramBuilder;

FrankaSimDiagram::FrankaSimDiagram(std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant,
                                   drake::lcm::DrakeLcm* lcm) {
  // load parameters
  DiagramBuilder<double> builder;
  scene_graph_ = builder.AddSystem<drake::geometry::SceneGraph>();
  scene_graph_->set_name("scene_graph");
  FrankaSimParams sim_params = drake::yaml::LoadYamlFile<FrankaSimParams>(
      "examples/franka/parameters/franka_sim_params.yaml");
  FrankaLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<FrankaLcmChannels>("examples/franka/parameters/lcm_channels_simulation.yaml");
  FrankaSimSceneParams scene_params =
      drake::yaml::LoadYamlFile<FrankaSimSceneParams>(
          sim_params.sim_scene_file[sim_params.scene_index]);
  /// Sim Start
  plant_ = builder.AddSystem(std::move(plant));

  auto parser = drake::multibody::Parser(plant_, scene_graph_);
//  lcs_diagram_builder.parser() lcs_diagram_builder.parser()(&plant);
  parser.SetAutoRenaming(true);
  drake::multibody::ModelInstanceIndex franka_index =
      parser.AddModelsFromUrl(sim_params.franka_model)[0];
  drake::multibody::ModelInstanceIndex c3_end_effector_index =
      parser.AddModels(FindResourceOrThrow(sim_params.end_effector_model))[0];
  drake::multibody::ModelInstanceIndex tray_index =
      parser.AddModels(FindResourceOrThrow(sim_params.tray_model))[0];

  RigidTransform<double> T_X_W = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), Eigen::VectorXd::Zero(3));
  RigidTransform<double> T_EE_W = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), sim_params.tool_attachment_frame);
  plant_->WeldFrames(plant_->world_frame(), plant_->GetFrameByName("panda_link0"),
                   T_X_W);
  plant_->WeldFrames(plant_->GetFrameByName("panda_link7"),
                   plant_->GetFrameByName("plate", c3_end_effector_index),
                   T_EE_W);

  // we WANT to model collisions between link5 and the supports
  const drake::geometry::GeometrySet& franka_geom_set =
      plant->CollectRegisteredGeometries({&plant->GetBodyByName("panda_link0"),
                                         &plant->GetBodyByName("panda_link1"),
                                         &plant->GetBodyByName("panda_link2"),
                                         &plant->GetBodyByName("panda_link3"),
                                         &plant->GetBodyByName("panda_link4")});
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
    plant->WeldFrames(
        plant->world_frame(),
        plant->GetFrameByName("base", environment_model_indices[i]),
        T_E_W);
    support_geom_set.Add(plant->GetCollisionGeometriesForBody(plant->GetBodyByName("base",
                                                                                 environment_model_indices[i])));
  }
  plant->ExcludeCollisionGeometriesWithCollisionFilterGroupPair(
      {"supports", support_geom_set}, {"franka", franka_geom_set});

  const drake::geometry::GeometrySet& paddle_geom_set =
      plant_->CollectRegisteredGeometries({
          &plant_->GetBodyByName("panda_link2"),
          &plant_->GetBodyByName("panda_link3"),
          &plant_->GetBodyByName("panda_link4"),
          &plant_->GetBodyByName("panda_link5"),
          &plant_->GetBodyByName("panda_link6"),
          &plant_->GetBodyByName("panda_link8"),
      });
  auto tray_collision_set = GeometrySet(
      plant_->GetCollisionGeometriesForBody(plant_->GetBodyByName("tray")));
  plant_->ExcludeCollisionGeometriesWithCollisionFilterGroupPair(
      {"paddle", paddle_geom_set}, {"tray", tray_collision_set});

  plant_->Finalize();
  auto tray_state_sender =
      builder.AddSystem<systems::ObjectStateSender>(*plant_, tray_index);
  auto franka_state_sender =
      builder.AddSystem<systems::RobotOutputSender>(*plant_, franka_index, false);

  // for lcm debugging
  auto state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          lcm_channel_params.franka_state_channel, lcm,
          drake::systems::TriggerTypeSet(
              {drake::systems::TriggerType::kForced})));
  auto tray_state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.tray_state_channel, lcm,
          drake::systems::TriggerTypeSet(
              {drake::systems::TriggerType::kForced})));
  builder.Connect(plant_->get_state_output_port(tray_index),
                  tray_state_sender->get_input_port_state());
  builder.Connect(plant_->get_state_output_port(franka_index),
                  franka_state_sender->get_input_port_state());
  builder.Connect(*franka_state_sender, *state_pub);
  builder.Connect(*tray_state_sender, *tray_state_pub);
  // end lcm debugging

  actuation_port_ = builder.ExportInput(plant_->get_actuation_input_port(),
                      "franka_sim: robot_efforts");
  tray_state_port_ = builder.ExportOutput(tray_state_sender->get_output_port(),
                       "franka_sim: tray_state");
  franka_state_port_ = builder.ExportOutput(franka_state_sender->get_output_port(),
                       "franka_sim: franka_state");

  builder.BuildInto(this);
  this->set_name("FrankaSim");

}
}  // namespace examples
}  // namespace dairlib