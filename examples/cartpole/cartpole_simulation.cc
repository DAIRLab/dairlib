/// Simulation of a cartpole as a drake demo for tutorial purposes

// misc includes
#include <gflags/gflags.h>
#include <drake/systems/lcm/lcm_publisher_system.h>

// dairlib includes
#include "common/find_resource.h"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "systems/robot_lcm_systems.h"
#include "systems/primitives/subvector_pass_through.h"

// drake includes
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/geometry/drake_visualizer.h"



DEFINE_string(channel_u, "CARTPOLE_INPUT",
    "channel to recieve cartpole input");
DEFINE_string(channel_x, "CARTPOLE_STATE",
    "channel to send ");
DEFINE_double(realtime_rate, 1.0, "realtime rate for simulation");

using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::DiagramBuilder;
using drake::systems::Diagram;
using drake::systems::Simulator;
using drake::systems::Context;

using drake::multibody::Parser;
using drake::multibody::MultibodyPlant;
using drake::multibody::PrismaticJoint;
using drake::multibody::RevoluteJoint;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::geometry::DrakeVisualizer;

using dairlib::systems::RobotInputReceiver;
using dairlib::systems::RobotOutputSender;
using dairlib::systems::SubvectorPassThrough;
using dairlib::FindResourceOrThrow;


int main(int argc, char* argv[]) {

  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Adding the MultibodyPlant
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(
      &builder, 0.001);
  Parser(&plant).AddModelFromFile(
      FindResourceOrThrow(
          "examples/cartpole/urdf/cartpole.urdf"));
  plant.Finalize();

  // Visualization
  DrakeVisualizer<double>::AddToBuilder(&builder, scene_graph);

  // setup lcm communications
  drake::lcm::DrakeLcm lcm;
  auto input_subscriber = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_channel_u, &lcm));
  auto input_reciever = builder.AddSystem<RobotInputReceiver>(plant);
  auto input_passthrough = builder.AddSystem<SubvectorPassThrough>(
      input_reciever->get_output_port().size(), 0,
      plant.get_actuation_input_port().size());

  auto state_sender = builder.AddSystem<RobotOutputSender>(plant);
  auto state_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          FLAGS_channel_x, &lcm));

  // wire up the diagram
  builder.Connect(*input_subscriber, *input_reciever);
  builder.Connect(*input_reciever, *input_passthrough);
  builder.Connect(input_passthrough->get_output_port(),
      plant.get_actuation_input_port());
  builder.Connect(input_passthrough->get_output_port(),
      state_sender->get_input_port_effort());
  builder.Connect(plant.get_state_output_port(),
      state_sender->get_input_port_state());
  builder.Connect(*state_sender, *state_publisher);
  auto diagram = builder.Build();

  // Set initial condition
  std::unique_ptr<Context<double>>
    diagram_context = diagram->CreateDefaultContext();
  Context<double>& plant_context = diagram->GetMutableSubsystemContext(
      plant, diagram_context.get());

  const PrismaticJoint<double>& slider =
      plant.GetJointByName<PrismaticJoint>("x");
  const RevoluteJoint<double>& pin =
      plant.GetJointByName<RevoluteJoint>("theta");
  slider.set_translation(&plant_context, 0.1);
  pin.set_angle(&plant_context, 0.4);

  // Simulate
  Simulator<double> simulator(*diagram, std::move(diagram_context));
  simulator.set_target_realtime_rate(FLAGS_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(10.0);
}