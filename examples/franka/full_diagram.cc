
#include <vector>
#include <Eigen/Dense>

#include "examples/franka/diagrams/franka_c3_controller_diagram.h"
#include "examples/franka/diagrams/franka_osc_controller_diagram.h"

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
#include <dairlib/lcmt_radio_out.hpp>

#include "common/eigen_utils.h"
#include "common/find_resource.h"
#include "examples/franka/parameters/franka_lcm_channels.h"
#include "examples/franka/parameters/franka_sim_params.h"
#include "multibody/multibody_utils.h"
#include "systems/robot_lcm_systems.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/system_utils.h"

namespace dairlib {

using examples::controllers::FrankaOSCControllerDiagram;
using examples::controllers::FrankaC3ControllerDiagram;
using drake::systems::DiagramBuilder;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::Parser;
using drake::math::RigidTransform;
using drake::geometry::GeometrySet;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using systems::RobotInputReceiver;
using systems::SubvectorPassThrough;
using systems::RobotOutputSender;
using Eigen::VectorXd;


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

  DiagramBuilder<double> builder;

  /// OSC
  auto osc_controller = builder.AddSystem<FrankaOSCControllerDiagram>("examples/franka/parameters/franka_osc_controller_params.yaml",
                                                   "examples/franka/parameters/lcm_channels_simulation.yaml",
                                                   &lcm);

  /// C3 plant
  auto c3_controller = builder.AddSystem<FrankaC3ControllerDiagram>("examples/franka/parameters/franka_c3_controller_params.yaml",
                                                   "examples/franka/parameters/lcm_channels_simulation.yaml",
                                                   &lcm);

  /// Sim Start
  double sim_dt = sim_params.dt;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, sim_dt);

  Parser parser(&plant);
  parser.SetAutoRenaming(true);
  drake::multibody::ModelInstanceIndex franka_index =
      parser.AddModels(drake::FindResourceOrThrow(sim_params.franka_model))[0];
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

  if (sim_params.scene_index > 0) {
    drake::multibody::ModelInstanceIndex left_support_index =
        parser.AddModels(FindResourceOrThrow(sim_params.left_support_model))[0];
    drake::multibody::ModelInstanceIndex right_support_index = parser.AddModels(
        FindResourceOrThrow(sim_params.right_support_model))[0];
    RigidTransform<double> T_S1_W = RigidTransform<double>(
        drake::math::RollPitchYaw<double>(sim_params.left_support_orientation),
        sim_params.left_support_position);
    RigidTransform<double> T_S2_W = RigidTransform<double>(
        drake::math::RollPitchYaw<double>(sim_params.right_support_orientation),
        sim_params.right_support_position);
    plant.WeldFrames(plant.world_frame(),
                     plant.GetFrameByName("support", left_support_index),
                     T_S1_W);
    plant.WeldFrames(plant.world_frame(),
                     plant.GetFrameByName("support", right_support_index),
                     T_S2_W);
    const drake::geometry::GeometrySet& support_geom_set =
        plant.CollectRegisteredGeometries({
            &plant.GetBodyByName("support", left_support_index),
            &plant.GetBodyByName("support", right_support_index),
        });
    // we WANT to model collisions between link5 and the supports
    const drake::geometry::GeometrySet& paddle_geom_set =
        plant.CollectRegisteredGeometries(
            {&plant.GetBodyByName("panda_link2"),
             &plant.GetBodyByName("panda_link3"),
             &plant.GetBodyByName("panda_link4"),
             &plant.GetBodyByName("panda_link6"),
             &plant.GetBodyByName("panda_link7"), &plant.GetBodyByName("plate"),
             &plant.GetBodyByName("panda_link8")});

    plant.ExcludeCollisionGeometriesWithCollisionFilterGroupPair(
        {"paddle", support_geom_set}, {"tray", paddle_geom_set});
  }

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
          lcm_channel_params.franka_state_channel, &lcm, 1.0 / sim_params.franka_publish_rate));
  auto radio_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
          lcm_channel_params.radio_channel, &lcm));

  //// OSC connections

  builder.Connect(plant.get_state_output_port(tray_index),
                  tray_state_sender->get_input_port_state());

  // Diagram Connections
  builder.Connect(osc_controller->get_output_port_robot_input(), passthrough->get_input_port());
  builder.Connect(c3_controller->get_output_port_mpc_plan(), osc_controller->get_input_port_end_effector_position());
  builder.Connect(c3_controller->get_output_port_mpc_plan(), osc_controller->get_input_port_end_effector_orientation());
  builder.Connect(c3_controller->get_output_port_mpc_plan(), osc_controller->get_input_port_end_effector_force());


  builder.Connect(franka_state_sender->get_output_port(), osc_controller->get_input_port_robot_state());
  builder.Connect(franka_state_sender->get_output_port(), c3_controller->get_input_port_robot_state());
  builder.Connect(tray_state_sender->get_output_port(), c3_controller->get_input_port_object_state());
  builder.Connect(radio_sub->get_output_port(), c3_controller->get_input_port_radio());
  builder.Connect(radio_sub->get_output_port(), osc_controller->get_input_port_radio());

  builder.Connect(*franka_state_sender, *state_pub);
  builder.Connect(plant.get_state_output_port(franka_index),
                   franka_state_sender->get_input_port_state());

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
      sim_params.q_init_plate[sim_params.scene_index];

  plant.SetPositions(&plant_context, q);

  VectorXd v = VectorXd::Zero(nv);
  plant.SetVelocities(&plant_context, v);

  simulator.Initialize();
  simulator.AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv); }
