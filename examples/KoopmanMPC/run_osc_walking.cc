#include <drake/multibody/parsing/parser.h>
#include <gflags/gflags.h>
#include <drake/systems/lcm/lcm_subscriber_system.h>

#include "common/find_resource.h"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_dairlib_signal.hpp"
#include "lcm/lcm_trajectory.h"

#include "multibody/multibody_utils.h"

#include "examples/KoopmanMPC/PlanarWalker/planar_walker_model_utils.h"

#include "systems/controllers/osc/operational_space_control.h"
#include "systems/controllers/osc/com_tracking_data.h"
#include "systems/controllers/osc/trans_space_tracking_data.h"
#include "systems/controllers/osc/joint_space_tracking_data.h"
#include "systems/controllers/time_based_fsm.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"

#include "examples/KoopmanMPC/PlanarWalker/planar_osc_walking_gains.h"
#include "systems/controllers/mpc/mpc_trajectory_reciever.h"
#include "examples/KoopmanMPC/koopman_mpc.h"

#include "drake/common/yaml/yaml_read_archive.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "systems/dairlib_signal_lcm_systems.h"

namespace dairlib {

using std::map;
using std::pair;
using std::string;
using std::vector;

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::geometry::SceneGraph;
using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::TriggerTypeSet;
using drake::trajectories::PiecewisePolynomial;

using systems::controllers::ComTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;
using systems::controllers::JointSpaceTrackingData;
using systems::TimeBasedFiniteStateMachine;
using systems::DairlibSignalReceiver;

namespace examples {

DEFINE_string(channel_x, "PLANAR_STATE",
              "The name of the channel which receives state");
DEFINE_string(channel_u, "PLANAR_INPUT",
              "The name of the channel which publishes command");
DEFINE_string(channel_fsm, "FSM", "the name of the channel with the time-based fsm");

DEFINE_string(mpc_channel, "KOOPMAN_MPC_OUT", "channel to recieve koopman mpc message");


DEFINE_string(
    gains_filename,
    "examples/KoopmanMPC/planar_osc_walking_gains.yaml",
    "Filepath containing gains");

DEFINE_bool(track_com, false,
    "use com tracking data (otherwise uses trans space)");

DEFINE_bool(print_osc_debug, false, "print osc_debug to the terminal");

void print_gains(const OSCWalkingGains& gains);

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Build the controller diagram
  DiagramBuilder<double> builder;

  // Built the MBP
  drake::multibody::MultibodyPlant<double> plant(0.0);

  SceneGraph<double>& scene_graph = *(builder.AddSystem<SceneGraph>());
  LoadPlanarWalkerFromFile(plant, &scene_graph);
  plant.Finalize();

  auto left_pt = std::pair<const drake::multibody::BodyFrame<double> &, Eigen::Vector3d>(
      plant.GetBodyByName("left_lower_leg").body_frame(), Vector3d(0, 0, -0.5));

  auto right_pt = std::pair<const drake::multibody::BodyFrame<double> &, Eigen::Vector3d>(
      plant.GetBodyByName("right_lower_leg").body_frame(), Vector3d(0, 0, -0.5));

  auto plant_context = plant.CreateDefaultContext();

  // Get contact frames and position (doesn't matter whether we use
  // plant or plant_wo_springs because the contact frames exit in both
  // plants)
  int nv = plant.num_velocities();

  // Create maps for joints
  map<string, int> pos_map = multibody::makeNameToPositionsMap(plant);
  map<string, int> vel_map = multibody::makeNameToVelocitiesMap(plant);
  map<string, int> act_map = multibody::makeNameToActuatorsMap(plant);

  /**** Convert the gains from the yaml struct to Eigen Matrices ****/
  OSCWalkingGains gains;
  const YAML::Node& root =
      YAML::LoadFile(FindResourceOrThrow(FLAGS_gains_filename));
  drake::yaml::YamlReadArchive(root).Accept(&gains);


  /**** Initialize all the leaf systems ****/
  drake::lcm::DrakeLcm lcm_local;

  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);

  /* auto fsm = builder.AddSystem<TimeBasedFiniteStateMachine>(
      plant, fsm_states, state_durations); */

  auto mpc_subscriber = builder.AddSystem(
      LcmSubscriberSystem::Make<lcmt_saved_traj>(FLAGS_mpc_channel, &lcm_local));

  auto mpc_reciever = builder.AddSystem<MpcTrajectoryReceiver>(
      TrajectoryType::kCubicHermite, TrajectoryType::kCubicHermite,
      TrajectoryType::kCubicHermite, true);

  auto command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_channel_u, &lcm_local, TriggerTypeSet({TriggerType::kForced})));

  auto command_sender = builder.AddSystem<systems::RobotCommandSender>(plant);

  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      plant, plant, plant_context.get(), plant_context.get(), true, FLAGS_print_osc_debug);

  auto osc_debug_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
          "OSC_DEBUG_WALKING", &lcm_local, TriggerTypeSet({TriggerType::kForced})));

  auto fsm_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_dairlib_signal>(
          FLAGS_channel_fsm, &lcm_local));

  auto fsm_receiver = builder.AddSystem<DairlibSignalReceiver>(1);

  /**** OSC setup ****/
  // Cost
  MatrixXd Q_accel = gains.w_accel * MatrixXd::Identity(nv, nv);
  osc->SetAccelerationCostForAllJoints(Q_accel);

  // Soft constraint on contacts
  osc->SetWeightOfSoftContactConstraint(gains.w_soft_constraint);

  // Contact information for OSC
  osc->SetContactFriction(gains.mu);

  Vector3d foot_contact_disp(0, 0, 0);

  auto left_foot_evaluator = multibody::WorldPointEvaluator(plant,
      left_pt.second, left_pt.first, Matrix3d::Identity(),
      foot_contact_disp,{0, 2});

  auto right_foot_evaluator = multibody::WorldPointEvaluator(plant,
      right_pt.second, right_pt.first,  Matrix3d::Identity(),
      foot_contact_disp, {0, 2});

  osc->AddStateAndContactPoint(koopMpcStance::kLeft, &left_foot_evaluator);
  osc->AddStateAndContactPoint(koopMpcStance::kRight, &right_foot_evaluator);


  /*** tracking data ***/
  TransTaskSpaceTrackingData swing_foot_traj("swing_ft_traj",
      gains.K_p_swing_foot, gains.K_d_swing_foot, gains.W_swing_foot, plant, plant);
  
  swing_foot_traj.AddStateAndPointToTrack(koopMpcStance::kLeft, "right_lower_leg", right_pt.second);
  swing_foot_traj.AddStateAndPointToTrack(koopMpcStance::kRight, "left_lower_leg", left_pt.second);

  osc->AddTrackingData(&swing_foot_traj);

  ComTrackingData com_traj("com_traj", gains.K_p_com,
                           gains.K_d_com, gains.W_com, plant, plant);

  TransTaskSpaceTrackingData torso_traj("com_traj", gains.K_p_com,
                                      gains.K_d_com, gains.W_com, plant, plant);
  torso_traj.AddPointToTrack("torso_mass");

  if (FLAGS_track_com) {
    osc->AddTrackingData(&com_traj);
  } else {
    osc->AddTrackingData(&torso_traj);
  }

  JointSpaceTrackingData angular_traj("base_angle", gains.K_p_orientation,
      gains.K_d_orientation, gains.W_orientation, plant, plant);

  angular_traj.AddJointToTrack("planar_roty", "planar_rotydot");

  osc->AddTrackingData(&angular_traj);

  // Build OSC problem
  osc->Build();
  std::cout << "Built OSC" << std::endl;

  /*****Connect ports*****/

  // OSC connections
  builder.Connect(fsm_sub->get_output_port(), fsm_receiver->get_input_port());

  builder.Connect(fsm_receiver->get_output_port(), osc->get_fsm_input_port());

  builder.Connect(state_receiver->get_output_port(0),
                  osc->get_robot_output_input_port());

  builder.Connect(mpc_subscriber->get_output_port(),
      mpc_reciever->get_input_port());

  builder.Connect(mpc_reciever->get_com_traj_output_port(),
      osc->get_tracking_data_input_port("com_traj"));

  builder.Connect(mpc_reciever->get_angular_traj_output_port(),
      osc->get_tracking_data_input_port("base_angle"));

  builder.Connect(mpc_reciever->get_swing_ft_traj_output_port(),
      osc->get_tracking_data_input_port("swing_ft_traj"));

  // Publisher connections
  builder.Connect(osc->get_osc_output_port(),
                  command_sender->get_input_port(0));
  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port());
  builder.Connect(osc->get_osc_debug_port(), osc_debug_pub->get_input_port());

  // Run lcm-driven simulation
  // Create the diagram
  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("id_walking_controller"));

  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm_local, std::move(owned_diagram), state_receiver, FLAGS_channel_x, true);

  LcmHandleSubscriptionsUntil(&lcm_local, [&]() {
    return mpc_subscriber->GetInternalMessageCount() > 1; });

  auto& loop_context = loop.get_diagram_mutable_context();
  mpc_subscriber->Publish(loop.get_diagram()->
  GetMutableSubsystemContext(*mpc_subscriber, &loop_context));

  loop.Simulate();


  return 0;
}

void print_gains(const OSCWalkingGains& gains) {
  std::cout <<"======== OSC WALKING GAINS ==========\n";
}

}  // namespace examples
}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::examples::DoMain(argc, argv);
}
