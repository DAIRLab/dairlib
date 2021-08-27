#include <memory>

#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "common/file_utils.h"
#include "dairlib/lcmt_cassie_out.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "lcm/dircon_saved_trajectory.h"
#include "multibody/multibody_utils.h"
#include "systems/controllers/trajectory_playback.h"
#include "systems/framework/contact_results_to_force_vector.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"

#include "drake/common/yaml/yaml_read_archive.h"
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
#include "drake/systems/primitives/signal_logger.h"

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
DEFINE_double(stiction_tol, 1e-3, "Stiction tolernace (m/s)");
DEFINE_double(penetration_allowance, 1e-5,
              "Penetration allowance for the contact model. Nearly equivalent"
              " to (m)");
DEFINE_double(stiffness, 1e4, "Stiffness value the contact model");
DEFINE_double(dissipation_rate, 0.5,
              "Penetration allowance for the contact model. Nearly equivalent"
              " to (m)");
DEFINE_double(mu_static, 0.8, "Coefficient of static friction.");
DEFINE_double(mu_kinetic, 0.8, "Coefficient of kinematic friction.");
DEFINE_double(end_time, std::numeric_limits<double>::infinity(),
              "End time for simulator");
DEFINE_double(publish_rate, 2000, "Publish rate for simulator");
DEFINE_double(terrain_height, 0.0, "Height of the landing terrain");
DEFINE_double(platform_x, 0.0, "x location of the  landing terrain");
DEFINE_double(start_time, 0.0,
              "Starting time of the simulator, useful for initializing the "
              "state at a particular configuration");
DEFINE_string(log_num, "", "Name of the saved initial state");
DEFINE_string(
    folder_path,
    "/home/yangwill/Documents/research/projects/impact_uncertainty/data/",
    "Name of the saved initial state");
DEFINE_string(initial_state_file, "examples/Cassie/data/initial_state.yaml",
              "YAML file containing the initial state.");
DEFINE_bool(spring_model, true, "Use a URDF with or without legs springs");

struct InitialState {
  std::vector<double> x_init;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(x_init));
  }
};

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Plant/System initialization
  DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  const double time_step = FLAGS_dt;
  MultibodyPlant<double>& plant = *builder.AddSystem<MultibodyPlant>(time_step);
  if (FLAGS_floating_base) {
    multibody::addFlatTerrain(&plant, &scene_graph, FLAGS_mu_static,
                              FLAGS_mu_kinetic, Eigen::Vector3d(0, 0, 1),
                              FLAGS_stiffness, FLAGS_dissipation_rate);
  }

  if (FLAGS_terrain_height != 0) {
    Parser parser(&plant, &scene_graph);
    std::string terrain_name =
        FindResourceOrThrow("examples/impact_invariant_control/platform.urdf");
    parser.AddModelFromFile(terrain_name);
    Eigen::Vector3d offset;
    offset << FLAGS_platform_x, 0, FLAGS_terrain_height;
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"),
                     drake::math::RigidTransform<double>(offset));
  }
  std::string urdf;
  if (FLAGS_spring_model) {
    urdf = "examples/Cassie/urdf/cassie_v2.urdf";
  } else {
    urdf = "examples/Cassie/urdf/cassie_fixed_springs.urdf";
  }

  //  // Drake models the normal compliant force fn as: fn = fₙ = k⋅(1−d⋅vₙ)⋅ϕ
  //  Where
  //  // ϕ
  //  //   is the signed distance between the geometries (positive if not in
  //  contact
  //  //   and negative if overlaping), vₙ = dϕ/dt the normal separation
  //  velocity
  //  //   (positive if separating, negative if getting closer), k is the linear
  //  //   stiffness in N/m and d is the Hung & Crossley dissipation in s/m.
  //  const double stiffness = 1.0e4;  // k. Linear stiffness in N/m.
  //  const double dissipation = 0.5;  // d. Hunt & Crossley dissipation in s/m.
  //
  //  // Drake uses a Coulomb friction model, where μ is the coefficient of
  //  dynamic
  //  // friction (static friction equals dynamics friction).
  //  const double friction = 0.2;  // μ. Coefficient of friction,
  //  dimensionless.
  //
  //  drake::geometry::ProximityProperties props;
  //  props.AddProperty("material", "point_contact_stiffness", FLAGS_stiffness);
  //  props.AddProperty("material", "hunt_crossley_dissipation",
  //                    FLAGS_dissipation_rate);
  //  props.AddProperty(drake::geometry::internal::kMaterialGroup,
  //                    drake::geometry::internal::kFriction,
  //                    drake::multibody::CoulombFriction<double>(friction,
  //                    friction));
  //
  //  // When you programatically register your geometry you do:
  //  // const auto& body = plant.AddRigidBody(...) or,
  //  // const auto& body = plant.GetBodyByName("BodyName");
  //  // N.B. Here we use "props" created above to specify the properties of the
  //  new
  //  // geometry register for "body".
  //  plant.RegisterCollisionGeometry(body,
  //  drake::math::RigidTransformd::Identity(),
  //                                  drake::geometry::Box(...),
  //                                  "CollisionGeometryName", props);

  //  plant.set_penetration_allowance(FLAGS_penetration_allowance);
  plant.set_stiction_tolerance(FLAGS_stiction_tol);

  addCassieMultibody(&plant, &scene_graph, FLAGS_floating_base, urdf,
                     FLAGS_spring_model, true);

  plant.Finalize();

  int nq = plant.num_positions();
  int nv = plant.num_velocities();

  auto left_toe_front = LeftToeFront(plant);
  auto right_toe_front = RightToeFront(plant);
  auto left_toe_rear = LeftToeRear(plant);
  auto right_toe_rear = RightToeRear(plant);

  // Create maps for joints
  std::map<std::string, int> pos_map = multibody::makeNameToPositionsMap(plant);
  std::map<std::string, int> vel_map =
      multibody::makeNameToVelocitiesMap(plant);
  std::map<std::string, int> act_map = multibody::makeNameToActuatorsMap(plant);

  auto lcm_trajectory =
      LcmTrajectory(FLAGS_folder_path + "u_traj_" + FLAGS_log_num);
  const LcmTrajectory::Trajectory u_traj =
      lcm_trajectory.GetTrajectory("controller_inputs");
  auto u_init_traj = PiecewisePolynomial<double>::FirstOrderHold(
      u_traj.time_vector, u_traj.datapoints);
  auto controller_playback = builder.AddSystem<systems::TrajectoryPlayback>(
      u_init_traj, plant.num_actuators());

  // Create lcm systems.
  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();
  auto command_sender = builder.AddSystem<systems::RobotCommandSender>(plant);
  auto input_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          "CASSIE_INPUT", lcm, 1.0 / FLAGS_publish_rate));
  auto passthrough = builder.AddSystem<SubvectorPassThrough>(
      controller_playback->get_output_port(0).size(), 0,
      plant.get_actuation_input_port().size());
  auto state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          "CASSIE_STATE_SIMULATION", lcm, 1.0 / FLAGS_publish_rate));
  auto state_sender =
      builder.AddSystem<systems::RobotOutputSender>(plant, true);
  auto contact_results_to_vector =
      builder.AddSystem<systems::ContactResultsToForceVector>(plant);

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
  builder.Connect(*controller_playback, *command_sender);
  builder.Connect(*command_sender, *input_pub);
  builder.Connect(controller_playback->get_output_port(),
                  passthrough->get_input_port());
  builder.Connect(passthrough->get_output_port(),
                  plant.get_actuation_input_port());
  builder.Connect(plant.get_state_output_port(),
                  state_sender->get_input_port_state());
  builder.Connect(passthrough->get_output_port(),
                  state_sender->get_input_port_effort());
  builder.Connect(*state_sender, *state_pub);
  builder.Connect(
      plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());
  builder.Connect(plant.get_contact_results_output_port(),
                  contact_viz.get_input_port(0));
  builder.Connect(plant.get_contact_results_output_port(),
                  contact_results_to_vector->get_contact_result_input_port());
  builder.Connect(contact_viz.get_output_port(0),
                  contact_results_publisher.get_input_port());
  builder.Connect(sensor_aggregator.get_output_port(0),
                  sensor_pub->get_input_port());

  auto state_logger =
      drake::systems::LogOutput(plant.get_state_output_port(), &builder);
  state_logger->set_publish_period(1.0 / FLAGS_publish_rate);
  auto lambda_logger = drake::systems::LogOutput(
      contact_results_to_vector->get_contact_forces_output_port(), &builder);
  lambda_logger->set_publish_period(1.0 / FLAGS_publish_rate);
  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram_context->EnableCaching();
  diagram->SetDefaultContext(diagram_context.get());
  Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  MultibodyPlant<double> plant_wo_spr(FLAGS_dt);  // non-zero timestep to avoid
  //  Parser parser_wo_spr(&plant_wo_spr, &scene_graph);
  addCassieMultibody(&plant_wo_spr, &scene_graph, FLAGS_floating_base,
                     "examples/Cassie/urdf/cassie_fixed_springs.urdf", false,
                     true);
  plant_wo_spr.Finalize();

  InitialState init_state;
  const YAML::Node& root =
      YAML::LoadFile(FindResourceOrThrow(FLAGS_initial_state_file));
  drake::yaml::YamlReadArchive(root).Accept(&init_state);

  Eigen::VectorXd x_init = Eigen::Map<Eigen::VectorXd>(
      init_state.x_init.data(), init_state.x_init.size());

  plant.SetPositionsAndVelocities(&plant_context, x_init);

  Eigen::Vector3d left_toe_front_pt;
  Eigen::Vector3d right_toe_front_pt;
  Eigen::Vector3d left_toe_rear_pt;
  Eigen::Vector3d right_toe_rear_pt;
  plant.CalcPointsPositions(plant_context, left_toe_front.second,
                            left_toe_front.first, plant.world_frame(),
                            &left_toe_front_pt);
  plant.CalcPointsPositions(plant_context, right_toe_front.second,
                            right_toe_front.first, plant.world_frame(),
                            &right_toe_front_pt);
  plant.CalcPointsPositions(plant_context, left_toe_rear.second,
                            left_toe_rear.first, plant.world_frame(),
                            &left_toe_rear_pt);
  plant.CalcPointsPositions(plant_context, right_toe_rear.second,
                            right_toe_rear.first, plant.world_frame(),
                            &right_toe_rear_pt);

  if ((left_toe_front_pt(2) < 0) || (right_toe_front_pt(2) < 0) ||
      (left_toe_rear_pt(2) < 0) || (right_toe_rear_pt(2) < 0)) {
    writeCSV("x_traj.csv", Eigen::MatrixXd::Zero(1, 1));
    writeCSV("lambda_traj.csv", Eigen::MatrixXd::Zero(1, 1));
    writeCSV("t_x.csv", Eigen::VectorXd::Zero(1));
    return 1;
  }

  diagram_context->SetTime(FLAGS_start_time);
  Simulator<double> simulator(*diagram, std::move(diagram_context));

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_end_time);

  //  auto x_traj = logger->data();
  auto timestamps = state_logger->sample_times();
  //  std::cout << x_traj.cols() << std::endl;
  //  std::cout << x_traj.rows() << std::endl;
  writeCSV("x_traj.csv", state_logger->data());
  writeCSV("lambda_traj.csv", lambda_logger->data());
  writeCSV("t_x.csv", timestamps);

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }
