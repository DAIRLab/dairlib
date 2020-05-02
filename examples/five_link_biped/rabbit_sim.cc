#include <map>
#include <memory>
#include <string>
#include <drake/systems/primitives/signal_logger.h>
#include <gflags/gflags.h>
#include "common/find_resource.h"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "multibody/multibody_utils.h"
#include "systems/goldilocks_models/file_utils.h"
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
#include "lcm/lcm_trajectory.h"

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
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;
using Eigen::MatrixXd;

// Simulation parameters.
DEFINE_bool(floating_base, true, "Fixed or floating base model");
DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_bool(time_stepping, true,
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
DEFINE_string(init_state, "Walking",
              "The stored initial state for the simulator");
DEFINE_double(sim_time, std::numeric_limits<double>::infinity(),
              "The length of time to run the simulation");
DEFINE_double(start_time, 0.0, "Time to start the simulator at.");
DEFINE_double(penetration_allowance, 1e-5,
              "Penetration allowance for the contact model. It's a penalty "
              "method so there aren't any physical units");
DEFINE_double(stiction, 0.001, "Stiction tolerance for the contact model.");
DEFINE_string(trajectory_name, "", "Filename for the trajectory that contains"
                                   " the initial state.");
DEFINE_string(folder_path, "", "Folder path for the folder that contains the "
                               "saved trajectory");
DEFINE_int32(error_idx, 0, "Index in the state vector to inject error into");
DEFINE_double(error, 0.0, "Value fo the error, see error_idx");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  DiagramBuilder<double> builder;
  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  const double time_step = FLAGS_time_stepping ? FLAGS_dt : 0.0;
  MultibodyPlant<double>& plant = *builder.AddSystem<MultibodyPlant>(time_step);
  Parser parser(&plant, &scene_graph);
  std::string full_name =
      FindResourceOrThrow("examples/five_link_biped/five_link_biped.urdf");
  parser.AddModelFromFile(full_name);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"),
                   drake::math::RigidTransform<double>());
  plant.mutable_gravity_field().set_gravity_vector(-9.81 *
                                                   Eigen::Vector3d::UnitZ());
  multibody::addFlatTerrain(&plant, &scene_graph, .8, .8);  // Add ground
  plant.Finalize();

  // Contact model parameters
  plant.set_stiction_tolerance(FLAGS_stiction);
  plant.set_penetration_allowance(FLAGS_penetration_allowance);

  int nx = plant.num_positions() + plant.num_velocities();
  const LcmTrajectory& loaded_traj = LcmTrajectory(
      FLAGS_folder_path + FLAGS_trajectory_name);
  const LcmTrajectory::Trajectory& state_and_input =
      loaded_traj.getTrajectory("walking_trajectory_x_u0");
  PiecewisePolynomial<double> state_traj = PiecewisePolynomial<double>::CubicHermite(
      state_and_input.time_vector,
      state_and_input.datapoints.topRows(nx),
      state_and_input.datapoints.topRows(2*nx).bottomRows(nx));
  // Create input receiver.
  auto input_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_input_channel, lcm));
  auto input_receiver = builder.AddSystem<systems::RobotInputReceiver>(plant);
  // connect input receiver
  auto passthrough = builder.AddSystem<SubvectorPassThrough>(
      input_receiver->get_output_port(0).size(), 0,
      plant.get_actuation_input_port().size());
  // Create state publisher.
  auto state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          "RABBIT_STATE_SIMULATION", lcm, 1.0 / 4000.0));
  ContactResultsToLcmSystem<double>& contact_viz =
      *builder.template AddSystem<ContactResultsToLcmSystem<double>>(plant);
  contact_viz.set_name("contact_visualization");
  auto& contact_results_publisher = *builder.AddSystem(
      LcmPublisherSystem::Make<drake::lcmt_contact_results_for_viz>(
          "CONTACT_RESULTS", lcm, 1.0 / 4000.0));
  contact_results_publisher.set_name("contact_results_publisher");
  auto state_sender = builder.AddSystem<systems::RobotOutputSender>(plant);
  auto input_logger =
      drake::systems::LogOutput(passthrough->get_output_port(), &builder);
  input_logger->set_publish_period(0.00025);  // 1000Hz

  // Contact results to lcm msg.
  //  auto contact_pub =
  //      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
  //          "RABBIT_STATE_SIMULATION", lcm, 1.0 / 10000.0));
  //  auto contact_sender = builder.AddSystem<systems::>(plant);
  // connect state publisher
  builder.Connect(*input_sub, *input_receiver);
  builder.Connect(*input_receiver, *passthrough);
  builder.Connect(passthrough->get_output_port(),
                  plant.get_actuation_input_port());
  builder.Connect(plant.get_contact_results_output_port(),
                  contact_viz.get_input_port(0));
  builder.Connect(contact_viz.get_output_port(0),
                  contact_results_publisher.get_input_port());
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
    x0 << 0, 0.778109, 0, -.3112, -.231, 0.427, 0.4689, 0, 0, 0, 0, 0, 0, 0;
  } else if (FLAGS_init_state == "Walking") {
    x0 << state_traj.value(FLAGS_start_time);
    x0[FLAGS_error_idx] += FLAGS_error;
//    x0 << 0, 0.798986, -0.00175796, -0.0541245, -0.320418, 0.1, 0.75, 0.225025,
//        0.00132182, 0.145054, 0.136536, -0.746619, 9.46774e-05, -0.0115747;
  } else if (FLAGS_init_state == "Walking_2") {
    x0 << 0.075032, 0.79286, -0.0200404, 0.0081442, -0.184221, 0.200742,
        0.215107, 0.501089, -0.148784, -0.340658, -0.312183, 2.19227, 2.59285,
        -5.28952;
  } else if (FLAGS_init_state == "Walking_3") {
    x0 << 0.17696, 0.798739, 0.00405198, -0.337437, -0.0280155, 0.648008,
        0.099669, 0.33931, -0.00910572, -0.061163, 0.472764, 0.477749, -2.2539,
        0.016462;
  }
  plant.SetPositionsAndVelocities(&plant_context, x0);
  diagram_context->SetTime(FLAGS_start_time);
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
  simulator.AdvanceTo(FLAGS_start_time + FLAGS_sim_time);

  MatrixXd input_matrix = input_logger->data().transpose();
  MatrixXd times = input_logger->sample_times();
  DRAKE_ASSERT(input_matrix.rows() == times.rows());
  MatrixXd inputs_and_times(input_matrix.rows(),
                            input_matrix.cols() + times.cols());
  inputs_and_times << times , input_matrix;
  goldilocks_models::writeCSV(
      "../projects/five_link_biped/hybrid_lqr/plotting/inputs.csv",
      inputs_and_times);
  return 0;
}
}  // namespace dairlib
int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }