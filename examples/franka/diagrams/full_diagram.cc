
#include <iostream>

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
#include <drake/visualization/visualization_config_functions.h>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "common/find_resource.h"
#include "examples/franka/diagrams/franka_c3_controller_diagram.h"
#include "examples/franka/diagrams/franka_osc_controller_diagram.h"
#include "examples/franka/diagrams/franka_sim_diagram.h"
#include "examples/franka/parameters/franka_lcm_channels.h"
#include "examples/franka/parameters/franka_sim_params.h"
#include "multibody/multibody_utils.h"
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
  FrankaSimParams sim_params = drake::yaml::LoadYamlFile<FrankaSimParams>(
      "examples/franka/parameters/franka_sim_params.yaml");

  DiagramBuilder<double> builder;
  std::unique_ptr<drake::multibody::MultibodyPlant<double>> sim_plant =
                                                                std::make_unique<drake::multibody::MultibodyPlant<double>>(sim_params.dt);

  auto franka_sim =
      builder.AddSystem<examples::FrankaSimDiagram>(std::move(sim_plant), &lcm);

  /// OSC
  auto osc_controller = builder.AddSystem<FrankaOSCControllerDiagram>(
      "examples/franka/parameters/franka_osc_controller_params.yaml",
      "examples/franka/parameters/lcm_channels_simulation.yaml", &lcm);

  /// C3 plant
  auto c3_controller = builder.AddSystem<FrankaC3ControllerDiagram>(
      "examples/franka/parameters/franka_c3_controller_params.yaml",
      "examples/franka/parameters/lcm_channels_simulation.yaml", &lcm);

  /* -------------------------------------------------------------------------------------------*/
  auto passthrough = builder.AddSystem<SubvectorPassThrough>(
      osc_controller->get_output_port_robot_input().size(), 0,
      franka_sim->get_input_port_actuation().size());
  auto radio_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
          lcm_channel_params.radio_channel, &lcm));

  //// OSC connections

  // Diagram Connections
  builder.Connect(osc_controller->get_output_port_robot_input(),
                  passthrough->get_input_port());
  builder.Connect(c3_controller->get_output_port_mpc_plan(),
                  osc_controller->get_input_port_end_effector_position());
  builder.Connect(c3_controller->get_output_port_mpc_plan(),
                  osc_controller->get_input_port_end_effector_orientation());
  builder.Connect(c3_controller->get_output_port_mpc_plan(),
                  osc_controller->get_input_port_end_effector_force());

  builder.Connect(franka_sim->get_output_port_franka_state(),
                  osc_controller->get_input_port_robot_state());
  builder.Connect(franka_sim->get_output_port_franka_state(),
                  c3_controller->get_input_port_robot_state());
  builder.Connect(franka_sim->get_output_port_tray_state(),
                  c3_controller->get_input_port_object_state());

  builder.Connect(radio_sub->get_output_port(),
                  c3_controller->get_input_port_radio());
  builder.Connect(radio_sub->get_output_port(),
                  osc_controller->get_input_port_radio());

  builder.Connect(passthrough->get_output_port(),
                  franka_sim->get_input_port_actuation());

  auto diagram = builder.Build();
  diagram->set_name("plate_balancing_full_diagram");
  DrawAndSaveDiagramGraph(*diagram);

  drake::systems::Simulator<double> simulator(*diagram);

  simulator.set_publish_every_time_step(true);
  simulator.set_publish_at_initialization(true);
  simulator.set_target_realtime_rate(sim_params.realtime_rate);


  auto diagram_context = diagram->CreateDefaultContext();
  auto& plant = franka_sim->get_plant();
  auto& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());
  int nq = plant.num_positions();
  int nv = plant.num_velocities();
  VectorXd q = VectorXd::Zero(nq);
  q.head(7) = sim_params.q_init_franka;

  q.tail(7) = sim_params.q_init_tray[sim_params.scene_index];

  plant.SetPositions(&plant_context, q);

  VectorXd v = VectorXd::Zero(nv);
  plant.SetVelocities(&plant_context, v);

  simulator.Initialize();
  simulator.AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv); }
