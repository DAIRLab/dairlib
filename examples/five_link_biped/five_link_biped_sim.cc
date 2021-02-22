#include <map>
#include <memory>
#include <string>
#include "lcm/dircon_saved_trajectory.h"
#include "lcm/lcm_trajectory.h"
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
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/plant/multibody_plant.h"
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
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Simulation parameters.
DEFINE_bool(floating_base, true, "Fixed or floating base model");
DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(publish_rate, 2000.0,
              "Publish rate in Hz.");
DEFINE_bool(time_stepping, true,
            "If 'true', the plant is modeled as a "
            "discrete system with periodic updates. "
            "If 'false', the plant is modeled as a continuous system.");
DEFINE_double(dt, 1e-4,
              "The step size to use for time_stepping, ignored for "
              "continuous");
DEFINE_string(channel_x, "RABBIT_STATE",
              "Channel to publish/receive state from simulation");
DEFINE_string(channel_u, "RABBIT_INPUT",
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

  int nv = plant.num_velocities();
  int nu = plant.num_actuators();

  // Contact model parameters
  plant.set_stiction_tolerance(FLAGS_stiction);
  plant.set_penetration_allowance(FLAGS_penetration_allowance);

  int nx = plant.num_positions() + plant.num_velocities();
  const DirconTrajectory& loaded_traj = DirconTrajectory(
      FLAGS_folder_path + FLAGS_trajectory_name);
  auto state_traj = loaded_traj.ReconstructStateTrajectory();

  // Create input receiver.
  auto input_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_channel_u, lcm));
  auto input_receiver = builder.AddSystem<systems::RobotInputReceiver>(plant);
  // connect input receiver
  auto passthrough = builder.AddSystem<SubvectorPassThrough>(
      input_receiver->get_output_port(0).size(), 0,
      plant.get_actuation_input_port().size());
  // Create state publisher.
  auto state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          FLAGS_channel_x, lcm, 1.0 / FLAGS_publish_rate));
//          "RABBIT_STATE_SIMULATION", lcm, 1.0 / 4000.0));
  ContactResultsToLcmSystem<double>& contact_viz =
  *builder.template AddSystem<ContactResultsToLcmSystem<double>>(plant);
  contact_viz.set_name("contact_visualization");
  auto& contact_results_publisher = *builder.AddSystem(
      LcmPublisherSystem::Make<drake::lcmt_contact_results_for_viz>(
          "CONTACT_RESULTS", lcm, 1.0 / FLAGS_publish_rate));
  contact_results_publisher.set_name("contact_results_publisher");
  auto state_sender = builder.AddSystem<systems::RobotOutputSender>(plant);

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

  auto diagram = builder.Build();
  // Create a context for this system:
  std::unique_ptr<Context<double>> diagram_context =
                                       diagram->CreateDefaultContext();
  diagram_context->EnableCaching();
  diagram->SetDefaultContext(diagram_context.get());
  Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  Eigen::VectorXd x0 = state_traj.value(0);

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
//  simulator.AdvanceTo(FLAGS_start_time + FLAGS_sim_time);
  simulator.AdvanceTo(FLAGS_sim_time);

  return 0;
}


}  // namespace dairlib
int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }