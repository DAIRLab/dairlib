
#include <iostream>
#include <vector>

#include <Eigen/Dense>
#include <dairlib/lcmt_radio_out.hpp>
#include <drake/common/find_resource.h>
#include <drake/common/yaml/yaml_io.h>
#include <drake/geometry/drake_visualizer.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/parsing/parser.h>
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
#include "examples/franka/diagrams/franka_c3_controller_diagram.h"
#include "examples/franka/diagrams/franka_osc_controller_diagram.h"
#include "examples/franka/parameters/franka_c3_controller_params.h"
#include "examples/franka/parameters/franka_lcm_channels.h"
#include "examples/franka/parameters/franka_sim_params.h"
#include "examples/franka/parameters/franka_sim_scene_params.h"
#include "multibody/multibody_utils.h"
#include "systems/primitives/radio_parser.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"
namespace dairlib {

using drake::geometry::GeometrySet;
using drake::math::RigidTransform;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using Eigen::VectorXd;
using examples::controllers::FrankaC3ControllerDiagram;
using examples::controllers::FrankaOSCControllerDiagram;
using systems::RobotInputReceiver;
using systems::RobotOutputSender;
using systems::SubvectorPassThrough;

int DoMain(int argc, char* argv[]) {
  drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0");

  drake::yaml::LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;
  FrankaLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<FrankaLcmChannels>(
          "examples/franka/parameters/lcm_channels_simulation.yaml");
  // load parameters
  FrankaSimParams sim_params = drake::yaml::LoadYamlFile<FrankaSimParams>(
      "examples/franka/parameters/franka_sim_params.yaml");
  FrankaSimSceneParams scene_params =
      drake::yaml::LoadYamlFile<FrankaSimSceneParams>(
          sim_params.sim_scene_file[sim_params.scene_index]);
  FrankaC3ControllerParams c3_params =
      drake::yaml::LoadYamlFile<FrankaC3ControllerParams>(
          "examples/franka/parameters/franka_c3_controller_params.yaml");
  C3Options c3_options = drake::yaml::LoadYamlFile<C3Options>(
      c3_params.c3_options_file[c3_params.scene_index]);

  DiagramBuilder<double> builder;

  /// Sim Start
  double sim_dt = sim_params.dt;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, sim_dt);

  Parser parser(&plant);
  parser.SetAutoRenaming(true);
  drake::multibody::ModelInstanceIndex franka_index =
      parser.AddModelsFromUrl(sim_params.franka_model)[0];
  drake::multibody::ModelInstanceIndex c3_end_effector_index =
      parser.AddModels(FindResourceOrThrow(sim_params.end_effector_model))[0];
  drake::multibody::ModelInstanceIndex tray_index =
      parser.AddModels(FindResourceOrThrow(sim_params.tray_model))[0];
  multibody::AddFlatTerrain(&plant, &scene_graph, 1.0, 1.0);

  RigidTransform<double> T_X_W = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), Eigen::VectorXd::Zero(3));
  RigidTransform<double> T_EE_W = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), sim_params.tool_attachment_frame);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"),
                   T_X_W);
  plant.WeldFrames(plant.GetFrameByName("panda_link7"),
                   plant.GetFrameByName("plate", c3_end_effector_index),
                   T_EE_W);

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
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("base", environment_model_indices[i]),
        T_E_W);
    support_geom_set.Add(plant.GetCollisionGeometriesForBody(plant.GetBodyByName("base",
                                                                                 environment_model_indices[i])));
  }
  plant.ExcludeCollisionGeometriesWithCollisionFilterGroupPair(
      {"supports", support_geom_set}, {"franka", franka_geom_set});


  const drake::geometry::GeometrySet& paddle_geom_set =
      plant.CollectRegisteredGeometries({
          &plant.GetBodyByName("panda_link2"),
          &plant.GetBodyByName("panda_link3"),
          &plant.GetBodyByName("panda_link4"),
          &plant.GetBodyByName("panda_link5"),
          &plant.GetBodyByName("panda_link6"),
          &plant.GetBodyByName("panda_link8"),
      });
  auto tray_collision_set = GeometrySet(
      plant.GetCollisionGeometriesForBody(plant.GetBodyByName("tray")));
  plant.ExcludeCollisionGeometriesWithCollisionFilterGroupPair(
      {"paddle", paddle_geom_set}, {"tray", tray_collision_set});

  plant.Finalize();

  /// OSC
  auto osc_controller = builder.AddSystem<FrankaOSCControllerDiagram>(
      "examples/franka/parameters/franka_osc_controller_params.yaml",
      "examples/franka/parameters/lcm_channels_simulation.yaml", &lcm);

  /// C3 plant
  auto c3_controller = builder.AddSystem<FrankaC3ControllerDiagram>(
      "examples/franka/parameters/franka_c3_controller_params.yaml", c3_options,
      "examples/franka/parameters/lcm_channels_simulation.yaml", &lcm);

  /* -------------------------------------------------------------------------------------------*/
  auto passthrough = builder.AddSystem<SubvectorPassThrough>(
      osc_controller->get_output_port_robot_input().size(), 0,
      plant.get_actuation_input_port().size());
  auto tray_state_sender =
      builder.AddSystem<systems::ObjectStateSender>(plant, tray_index);
  auto franka_state_sender =
      builder.AddSystem<RobotOutputSender>(plant, franka_index, false);
  auto state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          lcm_channel_params.franka_state_channel, &lcm,
          drake::systems::TriggerTypeSet(
              {drake::systems::TriggerType::kForced})));
  auto tray_state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.tray_state_channel, &lcm,
          drake::systems::TriggerTypeSet(
              {drake::systems::TriggerType::kForced})));
  auto radio_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
          lcm_channel_params.radio_channel, &lcm));

  //// OSC connections

  auto radio_to_vector = builder.AddSystem<systems::RadioToVector>();
  builder.Connect(radio_sub->get_output_port(),
                  radio_to_vector->get_input_port());
  // Diagram Connections
  builder.Connect(osc_controller->get_output_port_robot_input(),
                  passthrough->get_input_port());
  builder.Connect(c3_controller->get_output_port_mpc_plan(),
                  osc_controller->get_input_port_end_effector_position());
  builder.Connect(c3_controller->get_output_port_mpc_plan(),
                  osc_controller->get_input_port_end_effector_orientation());
  builder.Connect(c3_controller->get_output_port_mpc_plan(),
                  osc_controller->get_input_port_end_effector_force());

  builder.Connect(franka_state_sender->get_output_port(),
                  osc_controller->get_input_port_robot_state());
  builder.Connect(franka_state_sender->get_output_port(),
                  c3_controller->get_input_port_robot_state());
  builder.Connect(tray_state_sender->get_output_port(),
                  c3_controller->get_input_port_object_state());
  builder.Connect(radio_to_vector->get_output_port(),
                  c3_controller->get_input_port_radio());
  builder.Connect(radio_to_vector->get_output_port(),
                  osc_controller->get_input_port_radio());

  builder.Connect(*franka_state_sender, *state_pub);
  builder.Connect(tray_state_sender->get_output_port(),
                  tray_state_pub->get_input_port());
  builder.Connect(plant.get_state_output_port(franka_index),
                  franka_state_sender->get_input_port_state());
  builder.Connect(plant.get_state_output_port(tray_index),
                  tray_state_sender->get_input_port_state());
  builder.Connect(passthrough->get_output_port(),
                  plant.get_actuation_input_port());

  int nq = plant.num_positions();
  int nv = plant.num_velocities();

  if (sim_params.visualize_drake_sim) {
    drake::visualization::AddDefaultVisualization(&builder);
  }

  auto diagram = builder.Build();
  diagram->set_name("plate_balancing_full_diagram");
  DrawAndSaveDiagramGraph(*diagram);

  drake::systems::Simulator<double> simulator(*diagram);

  simulator.set_publish_every_time_step(true);
  simulator.set_publish_at_initialization(true);
  simulator.set_target_realtime_rate(sim_params.realtime_rate);

  auto& plant_context = diagram->GetMutableSubsystemContext(
      plant, &simulator.get_mutable_context());

  VectorXd q = VectorXd::Zero(nq);
  q.head(plant.num_positions(franka_index)) = sim_params.q_init_franka;

  q.tail(plant.num_positions(tray_index)) =
      sim_params.q_init_tray[sim_params.scene_index];

  plant.SetPositions(&plant_context, q);

  VectorXd v = VectorXd::Zero(nv);
  plant.SetVelocities(&plant_context, v);

  simulator.Initialize();
  simulator.AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv); }
