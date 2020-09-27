#include <drake/multibody/parsing/parser.h>
#include <gflags/gflags.h>

#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "lcm/dircon_saved_trajectory.h"
#include "lcm/lcm_trajectory.h"
#include "systems/controllers/joint_level_controller.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "yaml-cpp/yaml.h"

#include "drake/common/yaml/yaml_read_archive.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace dairlib {

using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::trajectories::PiecewisePolynomial;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

// lcm channels
DEFINE_string(channel_x, "CASSIE_STATE_SIMULATION",
              "LCM channel for receiving state. "
              "Use CASSIE_STATE_SIMULATION to get state from simulator, and "
              "use CASSIE_STATE_DISPATCHER to get state from state estimator");
DEFINE_string(channel_u, "CASSIE_INPUT",
              "The name of the channel which publishes command");
DEFINE_string(folder_path, "examples/Cassie/saved_trajectories/",
              "Folder path for where the trajectory names are stored");
DEFINE_string(traj_name, "", "File to load saved trajectories from");
DEFINE_string(mode_name, "state_input_trajectory",
              "Base name of each trajectory");
// Cassie model parameter
DEFINE_bool(floating_base, true, "Fixed or floating base model");
DEFINE_string(gains, "examples/Cassie/data/joint_gains.yaml",
              "Filename of the yaml file containing the gains");

struct JointPdGains {
  int rows;
  int cols;
  std::vector<double> K;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(rows));
    a->Visit(DRAKE_NVP(cols));
    a->Visit(DRAKE_NVP(K));
  }
};

int doMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  DiagramBuilder<double> builder;
  DiagramBuilder<double> builder_null;

  //  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(
  //      "udpm://239.255.76.67:7667?ttl=0");
  drake::lcm::DrakeLcm lcm;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  SceneGraph<double>& scene_graph_null = *builder_null.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  MultibodyPlant<double>& plant = *builder_null.AddSystem<MultibodyPlant>(1.0);
  addCassieMultibody(&plant, &scene_graph_null, FLAGS_floating_base);
  plant.Finalize();

  int nq = plant.num_positions();
  int nx = plant.num_positions() + plant.num_velocities();
  int nu = plant.num_actuators();
//  const LcmTrajectory& original_traj =
//      LcmTrajectory(FLAGS_folder_path + FLAGS_traj_name);
//  const LcmTrajectory::Trajectory& lcm_state_traj =
//      original_traj.GetTrajectory("state_trajectory");
//  const LcmTrajectory::Trajectory& lcm_input_traj =
//      original_traj.GetTrajectory("input_trajectory");

  const DirconTrajectory& dircon_trajectory = DirconTrajectory(
      FindResourceOrThrow(FLAGS_folder_path + FLAGS_traj_name));
  PiecewisePolynomial<double> state_traj =
      dircon_trajectory.ReconstructStateTrajectory();
  PiecewisePolynomial<double> input_traj =
      dircon_trajectory.ReconstructInputTrajectory();

  const std::string channel_x = FLAGS_channel_x;
  const std::string channel_u = FLAGS_channel_u;
  //  const std::string channel_config = "PD_CONFIG";

  // Create state receiver.
  auto state_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(channel_x, &lcm));
  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);
  builder.Connect(state_sub->get_output_port(),
                  state_receiver->get_input_port(0));

  // Create config receiver.
  // Create command sender.
  auto command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          channel_u, &lcm, 1.0 / 1000.0));
  auto command_sender = builder.AddSystem<systems::RobotCommandSender>(plant);

  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port());

  JointPdGains result;
  const YAML::Node& root = YAML::LoadFile(FindResourceOrThrow(FLAGS_gains));
  drake::yaml::YamlReadArchive(root).Accept(&result);

  MatrixXd K = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      result.K.data(), result.rows, result.cols);

//  std::cout << K << std::endl;
  MultibodyPlant<double> plant_wo_spr(0.0);  // non-zero timestep to avoid
  Parser parser_wo_spr(&plant_wo_spr, &scene_graph);
  parser_wo_spr.AddModelFromFile(
      FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf"));
  plant_wo_spr.Finalize();
  Eigen::MatrixXd map_no_spring_to_spring =
      multibody::createWithSpringsToWithoutSpringsMap(plant, plant_wo_spr);

  auto controller = builder.AddSystem<systems::JointLevelController>(
      plant, state_traj, input_traj, K, map_no_spring_to_spring);

  builder.Connect(state_receiver->get_output_port(0),
                  controller->get_input_port_info());

  builder.Connect(controller->get_output_port(0),
                  command_sender->get_input_port(0));

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  /// Use the simulator to drive at a fixed rate
  /// If set_publish_every_time_step is true, this publishes twice
  /// Set realtime rate. Otherwise, runs as fast as possible
  //  auto stepper = std::make_unique<drake::systems::Simulator<double>>(
  //      *diagram, std::move(context));
  //  stepper->set_publish_every_time_step(false);
  //  stepper->set_publish_at_initialization(false);
  //  stepper->set_target_realtime_rate(1.0);
  //  stepper->Initialize();

  drake::log()->info("controller started");

  //  stepper->AdvanceTo(std::numeric_limits<double>::infinity());
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, std::move(diagram), state_receiver, FLAGS_channel_x, true);
  loop.Simulate();

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::doMain(argc, argv); }
