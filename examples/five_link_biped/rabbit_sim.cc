#include <map>
#include <memory>
#include <string>
#include <gflags/gflags.h>
#include "common/find_resource.h"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "multibody/multibody_utils.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/sensors/accelerometer.h"
#include "drake/systems/sensors/gyroscope.h"

namespace dairlib {


using dairlib::systems::SubvectorPassThrough;
using drake::geometry::SceneGraph;
using drake::multibody::Body;
using drake::multibody::ContactResultsToLcmSystem;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::PrismaticJoint;
using drake::multibody::RevoluteJoint;
using drake::solvers::MathematicalProgramResult;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;


// Simulation parameters.
// DEFINE_double(timestep, 1e-4, "The simulator time step (s)");
DEFINE_double(x_initial, 0, "The initial x position of the torso");
DEFINE_bool(floating_base, true, "Fixed or floating base model");
DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_bool(time_stepping, false,
            "If 'true', the plant is modeled as a "
            "discrete system with periodic updates. "
            "If 'false', the plant is modeled as a continuous system.");
DEFINE_double(dt, 1e-4,
              "The step size to use for time_stepping, ignored for "
              "continuous");
DEFINE_string(state_simulation_channel, "RABBIT_STATE_SIMULATION",
              "Channel to publish/receive state from simulation");
DEFINE_string(input_channel, "RABBIT_INPUT",
              "Channel to publish/receive inputs from controller");
DEFINE_string(init_state, "Jumping",
              "The stored initial state for the simulator");
DEFINE_double(sim_time, std::numeric_limits<double>::infinity(),
              "The length of time to run the simulation");
DEFINE_bool(visualize, false, "Whether or not to connect to Drake visualizer");
DEFINE_double(penetration_allowance, 0.001,
    "Penetration allowance (m) for the contact model.");
int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  DiagramBuilder<double> builder;
  // drake::lcm::DrakeLcm lcm;
  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");
  const double time_step = FLAGS_time_stepping ? FLAGS_dt : 0.0;
  MultibodyPlant<double>& plant = *builder.AddSystem<MultibodyPlant>(time_step);
  Parser parser(&plant, &scene_graph);
  std::string full_name =
      FindResourceOrThrow("examples/jumping/five_link_biped.urdf");
  parser.AddModelFromFile(full_name);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"),
                   drake::math::RigidTransform<double>());
  plant.mutable_gravity_field().set_gravity_vector(-9.81 *
                                                   Eigen::Vector3d::UnitZ());
  multibody::addFlatTerrain(&plant, &scene_graph, .8, .8);  // Add ground
  plant.Finalize();
  plant.set_penetration_allowance(FLAGS_penetration_allowance);
  // Create input receiver.
  auto input_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_input_channel, lcm));
  auto input_receiver = builder.AddSystem<systems::RobotInputReceiver>(plant);
  builder.Connect(*input_sub, *input_receiver);
  // connect input receiver
  auto passthrough = builder.AddSystem<SubvectorPassThrough>(
      input_receiver->get_output_port(0).size(), 0,
      plant.get_actuation_input_port().size());
  builder.Connect(*input_receiver, *passthrough);
  builder.Connect(passthrough->get_output_port(),
                  plant.get_actuation_input_port());
  // Create state publisher.
  auto state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          "RABBIT_STATE_SIMULATION", lcm, 1.0 / 1000.0));
  ContactResultsToLcmSystem<double>& contact_viz =
      *builder.template AddSystem<ContactResultsToLcmSystem<double>>(plant);
  contact_viz.set_name("contact_visualization");
  auto& contact_results_publisher = *builder.AddSystem(
      LcmPublisherSystem::Make<drake::lcmt_contact_results_for_viz>(
          "CONTACT_RESULTS", lcm, 1.0 / 1000.0));
  contact_results_publisher.set_name("contact_results_publisher");
  // Contact results to lcm msg.
  builder.Connect(plant.get_contact_results_output_port(),
                  contact_viz.get_input_port(0));
  builder.Connect(contact_viz.get_output_port(0),
                  contact_results_publisher.get_input_port());
  //  auto contact_pub =
  //      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
  //          "RABBIT_STATE_SIMULATION", lcm, 1.0 / 10000.0));
  auto state_sender = builder.AddSystem<systems::RobotOutputSender>(plant);
  //  auto contact_sender = builder.AddSystem<systems::>(plant);
  // connect state publisher
  builder.Connect(plant.get_state_output_port(),
                  state_sender->get_input_port_state());
  builder.Connect(state_sender->get_output_port(0),
                  state_pub->get_input_port());
  builder.Connect(
      plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());
//  if (FLAGS_visualize) {
//    drake::geometry::ConnectDrakeVisualizer(&builder, scene_graph);
//  }

  auto diagram = builder.Build();
  // Create a context for this system:
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram_context->EnableCaching();
  diagram->SetDefaultContext(diagram_context.get());
  Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());


  Eigen::VectorXd x0(14);

  if (FLAGS_init_state == "Jumping") {
    x0 << FLAGS_x_initial, 0.778109, 0, -.3112, -.231, 0.427, 0.4689, 0, 0, 0,
        0, 0, 0, 0;
  } else if (FLAGS_init_state == "Walking") {
    x0 << 0, 0.798986, -0.00175796, -0.0541245, -0.320418, 0.1, 0.75, 0.225025,
        0.00132182, 0.145054, 0.136536, -0.746619, 9.46774e-05, -0.0115747;
    //    x0 << 0, 0.797399, 0.0435607, -0.0888324, 0.0707006, 0.15, 0.15,
    //        0, 0, 0, 0, 0, 0, 0;
  }
  plant.SetPositionsAndVelocities(&plant_context, x0);
  Simulator<double> simulator(*diagram, std::move(diagram_context));
  //  if (!FLAGS_time_stepping) {
  //    //
  //    simulator.reset_integrator<drake::systems::RungeKutta2Integrator<double>>(
  //    //        *diagram, FLAGS_dt, &simulator.get_mutable_context());
  //    simulator.reset_integrator<drake::systems::RungeKutta3Integrator<double>>(
  //        plant, &simulator.get_mutable_context());
  //  }
  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_sim_time);
  return 0;
}
}  // namespace dairlib
int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }