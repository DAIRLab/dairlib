#include <memory>
#include <string>

#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "lcm/dircon_saved_trajectory.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"

#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

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
DEFINE_string(trajectory_name, "",
              "Filename for the trajectory that contains"
              " the initial state.");
DEFINE_string(folder_path, "",
              "Folder path for the folder that contains the "
              "saved trajectory");
DEFINE_double(start_time, 0.0, "Time to start the simulator at.");
DEFINE_double(sim_time, std::numeric_limits<double>::infinity(),
              "The length of time to run the simulation");
DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(dt, 1e-4, "The step size for the time-stepping simulator.");
DEFINE_double(publish_rate, 2000.0, "Publish rate of the robot's state in Hz.");
DEFINE_string(channel_x, "RABBIT_STATE",
              "Channel to publish/receive state from simulation");
DEFINE_string(channel_u, "RABBIT_INPUT",
              "Channel to publish/receive inputs from controller");
DEFINE_double(penetration_allowance, 1e-5,
              "Penetration allowance for the contact model.");
DEFINE_double(stiction, 0.001, "Stiction tolerance for the contact model.");
DEFINE_double(error, 0.0,
              "Initial velocity error of the swing leg in global coordinates.");

VectorXd calcStateOffset(MultibodyPlant<double>& plant,
                         Context<double>& context, VectorXd& x0);
int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  DiagramBuilder<double> builder;
  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  MultibodyPlant<double>& plant = *builder.AddSystem<MultibodyPlant>(FLAGS_dt);
  Parser parser(&plant, &scene_graph);
  std::string full_name = FindResourceOrThrow(
      "examples/impact_invariant_control/five_link_biped.urdf");
  parser.AddModelFromFile(full_name);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"),
                   drake::math::RigidTransform<double>());
  multibody::AddFlatTerrain(&plant, &scene_graph, .8, .8);  // Add ground
  plant.Finalize();

  int nv = plant.num_velocities();

  // Contact model parameters
  plant.set_stiction_tolerance(FLAGS_stiction);
  plant.set_penetration_allowance(FLAGS_penetration_allowance);

  const DirconTrajectory& loaded_traj =
      DirconTrajectory(plant, FLAGS_folder_path + FLAGS_trajectory_name);
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
  ContactResultsToLcmSystem<double>& contact_viz =
      *builder.template AddSystem<ContactResultsToLcmSystem<double>>(plant);
  contact_viz.set_name("contact_visualization");
  auto& contact_results_publisher = *builder.AddSystem(
      LcmPublisherSystem::Make<drake::lcmt_contact_results_for_viz>(
          "CONTACT_RESULTS", lcm, 1.0 / FLAGS_publish_rate));
  contact_results_publisher.set_name("contact_results_publisher");
  auto state_sender = builder.AddSystem<systems::RobotOutputSender>(plant);

  // Contact results to lcm msg.
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
  VectorXd vel_offset = calcStateOffset(plant, plant_context, x0);
  x0.tail(nv) += vel_offset;
  plant.SetPositionsAndVelocities(&plant_context, x0);
  diagram_context->SetTime(FLAGS_start_time);
  Simulator<double> simulator(*diagram, std::move(diagram_context));

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_start_time + FLAGS_sim_time);

  return 0;
}

VectorXd calcStateOffset(MultibodyPlant<double>& plant,
                         Context<double>& context, VectorXd& x0) {
  plant.SetPositionsAndVelocities(&context, x0);

  // common frames
  auto right_foot_frame = &plant.GetBodyByName("right_foot").body_frame();
  auto world = &plant.world_frame();
  // conversion to planar x-z
  MatrixXd TXZ = MatrixXd(2, 3);
  TXZ << 1, 0, 0, 0, 0, 1;

  MatrixXd J_foot_3d = MatrixXd::Zero(3, plant.num_velocities());
  plant.CalcJacobianTranslationalVelocity(
      context, drake::multibody::JacobianWrtVariable::kV, *right_foot_frame,
      Eigen::Vector3d::Zero(), *world, *world, &J_foot_3d);
  Eigen::Vector2d foot_vel_offset = Eigen::Vector2d::Zero();
  foot_vel_offset(1) = FLAGS_error;
  MatrixXd J_rfoot_angles = MatrixXd(2, 2);
  // Taking only the Jacobian wrt right leg angles
  J_rfoot_angles << (TXZ * J_foot_3d).col(4), (TXZ * J_foot_3d).col(6);
  VectorXd joint_rate_offsets =
      J_rfoot_angles.colPivHouseholderQr().solve(foot_vel_offset);
  // Remove floating base offsets
  VectorXd v_offset = VectorXd::Zero(plant.num_velocities());
  v_offset(4) = joint_rate_offsets(0);
  v_offset(6) = joint_rate_offsets(1);
  return v_offset;
}

}  // namespace dairlib
int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }