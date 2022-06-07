#include <memory>

#include <gflags/gflags.h>

#include "dairlib/lcmt_cassie_out.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "lcm/dircon_saved_trajectory.h"
#include "multibody/multibody_utils.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"

#include "drake/geometry/drake_visualizer.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/discrete_time_delay.h"

using dairlib::systems::SubvectorPassThrough;
using drake::geometry::DrakeVisualizer;
using drake::geometry::SceneGraph;
using drake::multibody::ContactResultsToLcmSystem;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;

using drake::systems::lcm::LcmPublisherSystem;

using drake::systems::lcm::LcmSubscriberSystem;

using drake::math::RotationMatrix;
using drake::trajectories::PiecewisePolynomial;

namespace dairlib {

// Simulation parameters.
DEFINE_bool(floating_base, true, "Fixed or floating base model");
DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(dt, 8e-5,
              "The step size to use for time_stepping, ignored for continuous");
DEFINE_double(v_stiction, 1e-3, "Stiction tolernace (m/s)");
DEFINE_double(penetration_allowance, 1e-5,
              "Penetration allowance for the contact model. Nearly equivalent"
              " to (m)");
DEFINE_double(end_time, std::numeric_limits<double>::infinity(),
              "End time for simulator");
DEFINE_double(publish_rate, 2000, "Publish rate for simulator");
DEFINE_double(init_height, .7,
              "Initial starting height of the pelvis above "
              "ground");
DEFINE_double(platform_height, 0.0, "Height of the platform");
DEFINE_double(platform_x, 0.0, "x location of the  platform");
DEFINE_double(start_time, 0.0,
              "Starting time of the simulator, useful for initializing the "
              "state at a particular configuration");
DEFINE_string(traj_name, "", "Name of the saved trajectory");
DEFINE_string(folder_path, "examples/Cassie/saved_trajectories/",
              "Folder path for where the trajectory names are stored");
DEFINE_bool(spring_model, true, "Use a URDF with or without legs springs");
DEFINE_bool(visualize, true, "Set to true to visualize the platform");
DEFINE_double(actuator_delay, 0.0,
              "Duration of actuator delay. Set to 0.0 by default.");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Plant/System initialization
  DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  const double time_step = FLAGS_dt;
  MultibodyPlant<double>& plant = *builder.AddSystem<MultibodyPlant>(time_step);
  if (FLAGS_floating_base) {
    multibody::AddFlatTerrain(&plant, &scene_graph, .8, .8);
  }

  std::string urdf = FLAGS_spring_model
                         ? "examples/Cassie/urdf/cassie_v2.urdf"
                         : "examples/Cassie/urdf/cassie_fixed_springs.urdf";

  if (FLAGS_platform_height != 0) {
    Parser parser(&plant, &scene_graph);
    std::string terrain_name =
        FindResourceOrThrow("examples/impact_invariant_control/platform.urdf");
    parser.AddModelFromFile(terrain_name);
    Eigen::Vector3d offset;
    offset << FLAGS_platform_x, 0, FLAGS_platform_height;
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"),
                     drake::math::RigidTransform<double>(offset));
  }

  plant.set_penetration_allowance(FLAGS_penetration_allowance);
  plant.set_stiction_tolerance(FLAGS_v_stiction);

  AddCassieMultibody(&plant, &scene_graph, FLAGS_floating_base, urdf,
                     FLAGS_spring_model, true);

  plant.Finalize();

  int nq = plant.num_positions();
  int nv = plant.num_velocities();
  int nx = nq + nv;

  // Create maps for joints
  std::map<std::string, int> pos_map =
      multibody::MakeNameToPositionsMap(plant);
  std::map<std::string, int> vel_map =
      multibody::MakeNameToVelocitiesMap(plant);
  std::map<std::string, int> act_map = multibody::MakeNameToActuatorsMap(plant);

  // Create lcm systems.
  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();
  auto input_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_input>(
          "CASSIE_INPUT", lcm));
  auto input_receiver = builder.AddSystem<systems::RobotInputReceiver>(plant);
  auto passthrough = builder.AddSystem<SubvectorPassThrough>(
      input_receiver->get_output_port(0).size(), 0,
      plant.get_actuation_input_port().size());
  auto discrete_time_delay =
      builder.AddSystem<drake::systems::DiscreteTimeDelay>(
          1.0 / FLAGS_publish_rate, FLAGS_actuator_delay * FLAGS_publish_rate,
          plant.num_actuators());
  auto state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          "CASSIE_STATE_SIMULATION", lcm, 1.0 / FLAGS_publish_rate));
  auto state_sender =
      builder.AddSystem<systems::RobotOutputSender>(plant, true);

  // Contact Information
  ContactResultsToLcmSystem<double>& contact_viz =
      *builder.template AddSystem<ContactResultsToLcmSystem<double>>(plant);
  contact_viz.set_name("contact_visualization");
  auto& contact_results_publisher = *builder.AddSystem(
      LcmPublisherSystem::Make<drake::lcmt_contact_results_for_viz>(
          "CASSIE_CONTACT_DRAKE", lcm, 1.0 / FLAGS_publish_rate));
  contact_results_publisher.set_name("contact_results_publisher");

  // Sensor aggregator and publisher of lcmt_cassie_out
  const auto& sensor_aggregator =
      AddImuAndAggregator(&builder, plant, passthrough->get_output_port());
  auto sensor_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_cassie_out>(
          "CASSIE_OUTPUT", lcm, 1.0 / FLAGS_publish_rate));

  // connect leaf systems
  builder.Connect(*input_sub, *input_receiver);
  builder.Connect(*input_receiver, *passthrough);
  builder.Connect(passthrough->get_output_port(),
                  discrete_time_delay->get_input_port());
  builder.Connect(discrete_time_delay->get_output_port(),
                  plant.get_actuation_input_port());
  builder.Connect(plant.get_state_output_port(),
                  state_sender->get_input_port_state());
  builder.Connect(discrete_time_delay->get_output_port(),
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
  builder.Connect(sensor_aggregator.get_output_port(0),
                  sensor_pub->get_input_port());

  if (FLAGS_visualize) {
    DrakeVisualizer<double>::AddToBuilder(&builder, scene_graph);
  }

  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram_context->EnableCaching();
  diagram->SetDefaultContext(diagram_context.get());
  Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  const DirconTrajectory& dircon_trajectory =
      DirconTrajectory(plant, FLAGS_folder_path + FLAGS_traj_name);

  PiecewisePolynomial<double> state_traj =
      dircon_trajectory.ReconstructStateTrajectory();

  Eigen::VectorXd x_traj_init = state_traj.value(FLAGS_start_time);

  plant.SetPositionsAndVelocities(&plant_context, x_traj_init);

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
