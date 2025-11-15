#include <dairlib/lcmt_franka_hand_target_position.hpp>
#include <dairlib/lcmt_radio_out.hpp>
#include <dairlib/lcmt_timestamped_saved_traj.hpp>
#include <drake/lcmt_schunk_wsg_command.hpp>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "common/find_resource.h"
#include "examples/magna/parameter_headers/lcm_channel_params.h"
#include "examples/magna/systems/utils/simulation_utils.h"
#include "solvers/solver_options_io.h"
#include "systems/controllers/osc/joint_space_tracking_data.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace dairlib {
namespace examples {
namespace magna {
static constexpr const char* kFrankaHand =
    "package://drake_models/franka_description/urdf/"
    "panda_hand_with_long_fingers.urdf";

using drake::math::RigidTransform;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::Parser;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::systems::controllers::JointSpaceTrackingData;
using dairlib::systems::controllers::OperationalSpaceControl;
using examples::magna::systems::simulation_utils::FrankaHandCommandToTrajectory;

DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");
int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::lcm::DrakeLcm lcm(FLAGS_lcm_url);

  // Load parameters.
  MagnaLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<MagnaLcmChannels>(
          "examples/magna/parameters/lcm_channels.yaml");
  drake::solvers::SolverOptions solver_options =
      drake::yaml::LoadYamlFile<dairlib::solvers::SolverOptionsFromYaml>(
          dairlib::FindResourceOrThrow(
              "examples/magna/parameters/osc_qp_settings.yaml"))
          .GetAsSolverOptions(drake::solvers::OsqpSolver::id());

  // Create MultibodyPlant for the Franka hand.
  drake::multibody::MultibodyPlant<double> plant(0.0);
  Parser parser(&plant);
  parser.SetAutoRenaming(true);
  parser.AddModelsFromUrl(kFrankaHand);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_hand"),
                   RigidTransform<double>::Identity());
  plant.Finalize();
  auto plant_context = plant.CreateDefaultContext();

  DiagramBuilder<double> builder;
  auto state_receiver =
      builder.AddSystem<dairlib::systems::RobotOutputReceiver>(plant);
  auto franka_hand_input_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          lcm_channel_params.franka_hand_input_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto franka_hand_input_sender =
      builder.AddSystem<dairlib::systems::RobotCommandSender>(plant);

  /// Add a system to receive the target hand position and convert it to a
  /// trajectory.
  auto target_hand_position_receiver =
      builder.AddSystem<FrankaHandCommandToTrajectory>();
  auto target_hand_position_subscriber = builder.AddSystem(
      LcmSubscriberSystem::Make<drake::lcmt_schunk_wsg_command>(
          lcm_channel_params.franka_hand_target_position_channel, &lcm));
  builder.Connect(target_hand_position_subscriber->get_output_port(),
                  target_hand_position_receiver->get_input_port());

  /// Add OSC controller for the hand and some subsystems to publish and send
  /// commands.
  auto osc = builder.AddSystem<OperationalSpaceControl>(
      plant, plant, plant_context.get(), plant_context.get(), false);
  auto osc_debug_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
          lcm_channel_params.osc_debug_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto joint_position_tracking_data = std::make_unique<JointSpaceTrackingData>(
      "hand_joint_position_target", Eigen::MatrixXd::Identity(2, 2) * 500,
      Eigen::MatrixXd::Identity(2, 2) * 20, Eigen::MatrixXd::Identity(2, 2),
      plant, plant);
  std::vector<std::string> finger_position_names = {"panda_finger_joint1",
                                                    "panda_finger_joint2"};
  std::vector<std::string> joint_velocity_names = {"panda_finger_joint1dot",
                                                   "panda_finger_joint2dot"};
  joint_position_tracking_data->AddJointsToTrack(finger_position_names,
                                                 joint_velocity_names);
  osc->AddTrackingData(std::move(joint_position_tracking_data));
  osc->SetAccelerationCostWeights(Eigen::MatrixXd::Identity(2, 2) * 1e-2);
  osc->SetInputCostWeights(Eigen::MatrixXd::Identity(2, 2));
  osc->SetInputSmoothingCostWeights(
      Eigen::MatrixXd::Zero(2, 2));  // No input smoothing cost
  osc->SetAccelerationConstraints(false);
  osc->SetContactFriction(0.4615);
  osc->SetOsqpSolverOptions(solver_options);
  osc->Build();

  /// Wire up the system.
  builder.Connect(osc->get_output_port_osc_debug(),
                  osc_debug_pub->get_input_port());
  builder.Connect(
      target_hand_position_receiver->get_output_port_hand_position_trajectory(),
      osc->get_input_port_tracking_data("hand_joint_position_target"));
  builder.Connect(state_receiver->get_output_port(0),
                  osc->get_input_port_robot_output());
  builder.Connect(osc->get_output_port_osc_command(),
                  franka_hand_input_sender->get_input_port(0));
  builder.Connect(franka_hand_input_sender->get_output_port(),
                  franka_hand_input_pub->get_input_port());

  /// Build the diagram and run the simulation.
  auto owned_diagram = builder.Build();
  std::shared_ptr<Diagram<double>> shared_diagram = std::move(owned_diagram);
  shared_diagram->set_name(("franka_hand_osc_controller"));
  dairlib::DrawAndSaveDiagramGraph(*shared_diagram);
  // Run lcm-driven simulation
  dairlib::systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, shared_diagram, state_receiver,
      lcm_channel_params.franka_hand_state_channel, true);
  loop.Simulate();
  return 0;
}

}  // namespace magna
}  // namespace examples
}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::examples::magna::DoMain(argc, argv);
}
