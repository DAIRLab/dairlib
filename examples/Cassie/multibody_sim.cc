#include <memory>

#include <drake/systems/primitives/multiplexer.h>
#include <gflags/gflags.h>
#include <iostream>

#include "dairlib/lcmt_cassie_out.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_fixed_point_solver.h"
#include "examples/Cassie/cassie_utils.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/geared_motor.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/discrete_time_delay.h"

namespace dairlib {
using dairlib::systems::SubvectorPassThrough;
using drake::geometry::SceneGraph;
using drake::multibody::ContactResultsToLcmSystem;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

using drake::math::RotationMatrix;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::VectorXd;

// Simulation parameters.
DEFINE_bool(floating_base, true, "Fixed or floating base model");

DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(dt, 1e-3,
              "The step size to use for time_stepping, ignored for continuous");
DEFINE_double(end_time, std::numeric_limits<double>::infinity(),
              "End time for simulator");
DEFINE_double(publish_rate, 1000, "Publish rate for simulator");
DEFINE_double(init_height, .7,
              "Initial starting height of the pelvis above "
              "ground");
DEFINE_double(toe_spread, .15, "Initial toe spread in m.");
DEFINE_bool(spring_model, true, "Use a URDF with or without legs springs");
DEFINE_string(radio_channel, "CASSIE_VIRTUAL_RADIO",
              "LCM channel for virtual radio command");
DEFINE_string(channel_u, "CASSIE_INPUT",
              "LCM channel to receive controller inputs on");
DEFINE_double(actuator_delay, 0.0,
              "Duration of actuator delay. Set to 0.0 by default.");
DEFINE_bool(publish_efforts, true, "Flag to publish the efforts.");
DEFINE_double(start_time, 0.0,
              "Starting time of the simulator, useful for initializing the "
              "state at a particular configuration");
DEFINE_string(contact_solver, "SAP",
              "Contact solver to use. Either TAMSI or SAP.");

    int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Plant/System initialization
  DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  const double time_step = FLAGS_dt;
  MultibodyPlant<double>& plant = *builder.AddSystem<MultibodyPlant>(time_step);
  if (FLAGS_floating_base) {
    multibody::AddFlatTerrain(&plant, &scene_graph, .8, .8, {0, 0, 1}, false);
  }

  std::string urdf;
  if (FLAGS_spring_model) {
    urdf = "examples/Cassie/urdf/cassie_v2.urdf";
  } else {
    urdf = "examples/Cassie/urdf/cassie_fixed_springs.urdf";
  }

  if (FLAGS_contact_solver == "SAP") {
    plant.set_discrete_contact_solver(
        drake::multibody::DiscreteContactSolver::kSap);
  } else if (FLAGS_contact_solver == "TAMSI") {
    plant.set_discrete_contact_solver(
        drake::multibody::DiscreteContactSolver::kTamsi);
  } else {
    std::cerr << "Unknown contact solver setting." << std::endl;
  }
  AddCassieMultibody(&plant, &scene_graph, FLAGS_floating_base, urdf,
                     FLAGS_spring_model, true);
  plant.Finalize();

  // Create lcm systems.
  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();
  auto input_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_channel_u, lcm));
  auto input_receiver = builder.AddSystem<systems::RobotInputReceiver>(plant);
  auto passthrough = builder.AddSystem<SubvectorPassThrough>(
      input_receiver->get_output_port(0).size(), 0,
      plant.get_actuation_input_port().size());
  auto discrete_time_delay =
      builder.AddSystem<drake::systems::DiscreteTimeDelay>(
          1.0 / FLAGS_publish_rate, FLAGS_actuator_delay * FLAGS_publish_rate,
          plant.num_actuators() + 1);
  auto state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          "CASSIE_STATE_SIMULATION", lcm, 1.0 / FLAGS_publish_rate));
  auto state_sender = builder.AddSystem<systems::RobotOutputSender>(
      plant, FLAGS_publish_efforts);

  // Contact Information
  ContactResultsToLcmSystem<double>& contact_viz =
      *builder.template AddSystem<ContactResultsToLcmSystem<double>>(plant);
  contact_viz.set_name("contact_visualization");
  auto& contact_results_publisher = *builder.AddSystem(
      LcmPublisherSystem::Make<drake::lcmt_contact_results_for_viz>(
          "CASSIE_CONTACT_DRAKE", lcm, 1.0 / FLAGS_publish_rate));
  contact_results_publisher.set_name("contact_results_publisher");

  // Sensor aggregator and publisher of lcmt_cassie_out
  auto radio_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
          FLAGS_radio_channel, lcm));

  const auto& cassie_motor = AddMotorModel(&builder, plant);

  const auto& sensor_aggregator =
      AddImuAndAggregator(&builder, plant, cassie_motor.get_output_port());

  auto sensor_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_cassie_out>(
          "CASSIE_OUTPUT", lcm, 1.0 / FLAGS_publish_rate));

  // connect leaf systems
  builder.Connect(*input_sub, *input_receiver);
  builder.Connect(input_receiver->get_output_port(),
                  discrete_time_delay->get_input_port());
  builder.Connect(discrete_time_delay->get_output_port(),
                  passthrough->get_input_port());
  builder.Connect(passthrough->get_output_port(),
                  cassie_motor.get_input_port_command());
  builder.Connect(cassie_motor.get_output_port(),
                  plant.get_actuation_input_port());
  builder.Connect(plant.get_state_output_port(),
                  state_sender->get_input_port_state());
  builder.Connect(plant.get_state_output_port(),
                  cassie_motor.get_input_port_state());
  builder.Connect(cassie_motor.get_output_port(),
                  state_sender->get_input_port_effort());
  builder.Connect(*state_sender, *state_pub);
  builder.Connect(
      plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());
  builder.Connect(plant.get_contact_results_output_port(),
                  contact_viz.get_input_port(0));
  builder.Connect(contact_viz.get_output_port(0),
                  contact_results_publisher.get_input_port());
  builder.Connect(radio_sub->get_output_port(),
                  sensor_aggregator.get_input_port_radio());
  builder.Connect(sensor_aggregator.get_output_port(0),
                  sensor_pub->get_input_port());

  auto diagram = builder.Build();
  diagram->set_name(("multibody_sim"));
  //  DrawAndSaveDiagramGraph(*diagram);

  // Create a context for this system:
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram_context->EnableCaching();
  diagram->SetDefaultContext(diagram_context.get());
  Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // Set initial conditions of the simulation
  VectorXd q_init, u_init, lambda_init;
  double mu_fp = 0;
  double min_normal_fp = 70;
  double toe_spread = FLAGS_toe_spread;
  // Create a plant for CassieFixedPointSolver.
  // Note that we cannot use the plant from the above diagram, because after the
  // diagram is built, plant.get_actuation_input_port().HasValue(*context)
  // throws a segfault error
  drake::multibody::MultibodyPlant<double> plant_for_solver(0.0);
  AddCassieMultibody(&plant_for_solver, nullptr,
                     FLAGS_floating_base /*floating base*/, urdf,
                     FLAGS_spring_model, false);
  plant_for_solver.Finalize();
  if (FLAGS_floating_base) {
    CassieFixedPointSolver(plant_for_solver, FLAGS_init_height, mu_fp,
                           min_normal_fp, true, toe_spread, &q_init, &u_init,
                           &lambda_init);
  } else {
    CassieFixedBaseFixedPointSolver(plant_for_solver, &q_init, &u_init,
                                    &lambda_init);
  }
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

int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }
