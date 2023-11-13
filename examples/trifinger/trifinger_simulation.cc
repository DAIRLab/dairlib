#pragma once

#include <iostream>
#include <memory>

#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "multibody/multibody_utils.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/discrete_time_delay.h"
#include "drake/systems/primitives/multiplexer.h"

// Simulation parameters.
DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(dt, 1e-3,
              "The step size to use for time_stepping, ignored for continuous");
DEFINE_double(end_time, std::numeric_limits<double>::infinity(),
              "End time for simulator");
DEFINE_double(publish_rate, 1000, "Publish rate for simulator");
DEFINE_string(channel_u, "TRIFINGER_INPUT",
              "LCM channel to receive controller inputs on");
DEFINE_string(channel_s, "TRIFINGER_STATE_SIMULATION",
              "LCM channel to publish the trifinger states");
DEFINE_double(actuator_delay, 0.0,
              "Duration of actuator delay. Set to 0.0 by default.");
DEFINE_bool(publish_efforts, true, "Flag to publish the efforts.");
DEFINE_double(start_time, 0.0,
              "Starting time of the simulator, useful for initializing the "
              "state at a particular configuration");
DEFINE_string(contact_solver, "SAP",
              "Contact solver to use. Either TAMSI or SAP.");

namespace dairlib {

using drake::geometry::SceneGraph;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::VectorXd;

int SimulateTrifinger(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  DiagramBuilder<double> builder;

  // Creates default plant and scene graph.
  const double time_step = FLAGS_dt;
  auto pair = AddMultibodyPlantSceneGraph(&builder, time_step);
  SceneGraph<double>& scene_graph = pair.scene_graph;
  scene_graph.set_name("scene_graph");
  MultibodyPlant<double>& plant = pair.plant;

  // Adds the URDF model to the trifinger plant.
  multibody::AddFlatTerrain(&plant, &scene_graph, .8, .8, {0, 0, 1}, false);
  std::string trifinger_urdf =
      "examples/trifinger/robot_properties_fingers/urdf/"
      "trifinger_minimal_collision.urdf";
  std::string cube_urdf =
      "examples/trifinger/robot_properties_fingers/cube/cube_v2.urdf";
  Parser parser(&plant, &scene_graph);
  parser.AddModelFromFile(FindResourceOrThrow(trifinger_urdf));
  parser.AddModelFromFile(FindResourceOrThrow(cube_urdf));

  // Fixes the trifinger base to the world frame.
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link"),
                   drake::math::RigidTransform<double>::Identity());

  // Sets up contact solver.
  if (FLAGS_contact_solver == "SAP") {
    plant.set_discrete_contact_solver(
        drake::multibody::DiscreteContactSolver::kSap);
  } else if (FLAGS_contact_solver == "TAMSI") {
    plant.set_discrete_contact_solver(
        drake::multibody::DiscreteContactSolver::kTamsi);
  } else {
    std::cerr << "Unknown contact solver setting." << std::endl;
  }

  plant.Finalize();

  // Creates LCM subscriber to controller and also LCM state publisher.
  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();
  systems::AddActuationRecieverAndStateSenderLcm(
      &builder, plant, lcm, FLAGS_channel_u, FLAGS_channel_s,
      FLAGS_publish_rate, FLAGS_publish_efforts, FLAGS_actuator_delay);

  // Builds diagram.
  auto diagram = builder.Build();
  diagram->set_name(("trifinger_sim"));

  // Create a context for this system:
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram_context->EnableCaching();
  diagram->SetDefaultContext(diagram_context.get());
  Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // Set initial conditions of the simulation
  VectorXd q_init(plant.num_positions());
  q_init << 0, -0.8, -1.3, 0, -0.8, -1.3, 0, -0.8, -1.3, 1, 0, 0, 0, 0, 0, 0.05;
  plant.SetPositions(&plant_context, q_init);
  plant.SetVelocities(&plant_context, VectorXd::Zero(plant.num_velocities()));
  diagram_context->SetTime(FLAGS_start_time);
  Simulator<double> simulator(*diagram, std::move(diagram_context));

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_end_time);

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::SimulateTrifinger(argc, argv);
}