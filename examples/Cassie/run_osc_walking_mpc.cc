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

#include "systems/controllers/mpc/srbd_cmpc.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/controllers/osc/osc_tracking_data.h"
#include "systems/controllers/time_based_fsm.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"

#include "examples/Cassie/mpc/cassie_mpc_osc_walking_gains.h"
#include "systems/controllers/mpc/mpc_trajectory_reciever.h"

#include "drake/common/yaml/yaml_read_archive.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "systems/dairlib_signal_lcm_systems.h"
#include "examples/Cassie/cassie_utils.h"

namespace dairlib {

using std::map;
using std::pair;
using std::string;
using std::vector;

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

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
using systems::controllers::RotTaskSpaceTrackingData;
using systems::TimeBasedFiniteStateMachine;
using systems::DairlibSignalReceiver;

namespace examples {

DEFINE_string(channel_x, "CASSIE_STATE_DISPATCHER",
              "The name of the channel which receives state");
DEFINE_string(channel_u, "OSC_MPC_WALKING",
              "The name of the channel which publishes command");
DEFINE_string(channel_fsm, "FSM", "the name of the channel with the time-based fsm");

DEFINE_string(mpc_channel, "SRBD_MPC_OUT", "channel to recieve koopman mpc message");


DEFINE_string(
    gains_filename,
    "examples/Cassie/mpc/cassie_mpc_osc_walking_gains.yaml","Filepath containing gains");

DEFINE_bool(track_com, false,
            "use com tracking data (otherwise uses trans space)");

DEFINE_bool(print_osc_debug, false, "print osc_debug to the terminal");

void print_gains(const OSCWalkingGains& gains);

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Build the controller diagram
  DiagramBuilder<double> builder;

  // Built the MBP
  drake::multibody::MultibodyPlant<double> plant_w_springs(0.0);
  addCassieMultibody(&plant_w_springs, nullptr, true,
      "examples/Cassie/urdf/cassie_v2.urdf", true, false);
  plant_w_springs.Finalize();

  // Get contact frames and position (doesn't matter whether we use
  // plant_w_spr or plant_wo_spr because the contact frames exit in both
  // plants)
  auto left_toe = LeftToeFront(plant_w_springs);
  auto left_heel = LeftToeRear(plant_w_springs);

  // Get body frames and points
  Vector3d mid_contact_point = (left_toe.first + left_heel.first) / 2.0;
  auto left_toe_mid = std::pair<const Vector3d, const Frame<double>&>(
      mid_contact_point, plant_w_springs.GetFrameByName("toe_left"));
  auto right_toe_mid = std::pair<const Vector3d, const Frame<double>&>(
      mid_contact_point, plant_w_springs.GetFrameByName("toe_right"));
  auto left_toe_origin = std::pair<const Vector3d, const Frame<double>&>(
      Vector3d::Zero(), plant_w_springs.GetFrameByName("toe_left"));
  auto right_toe_origin = std::pair<const Vector3d, const Frame<double>&>(
      Vector3d::Zero(), plant_w_springs.GetFrameByName("toe_right"));

  auto plant_context = plant_w_springs.CreateDefaultContext();

  // Get contact frames and position (doesn't matter whether we use
  // plant_w_springs or plant_wo_springs because the contact frames exit in both
  // plants)
  int nv = plant_w_springs.num_velocities();

  // Create maps for joints
  map<string, int> pos_map = multibody::makeNameToPositionsMap(plant_w_springs);
  map<string, int> vel_map = multibody::makeNameToVelocitiesMap(plant_w_springs);
  map<string, int> act_map = multibody::makeNameToActuatorsMap(plant_w_springs);

  /**** Convert the gains from the yaml struct to Eigen Matrices ****/
  OSCWalkingGains gains;
  const YAML::Node& root =
      YAML::LoadFile(FindResourceOrThrow(FLAGS_gains_filename));
  drake::yaml::YamlReadArchive(root).Accept(&gains);


  /**** Initialize all the leaf systems ****/
  drake::lcm::DrakeLcm lcm_local;

  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant_w_springs);

  /* auto fsm = builder.AddSystem<TimeBasedFiniteStateMachine>(
      plant_w_springs, fsm_states, state_durations); */

  auto mpc_subscriber = builder.AddSystem(
      LcmSubscriberSystem::Make<lcmt_saved_traj>(FLAGS_mpc_channel, &lcm_local));

  auto mpc_reciever = builder.AddSystem<MpcTrajectoryReceiver>(
      TrajectoryType::kCubicHermite, TrajectoryType::kCubicHermite,
      TrajectoryType::kCubicHermite, true);

  auto command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_channel_u, &lcm_local, TriggerTypeSet({TriggerType::kForced})));

  auto command_sender = builder.AddSystem<systems::RobotCommandSender>(plant_w_springs);

  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      plant_w_springs, plant_w_springs, plant_context.get(), plant_context.get(), true, FLAGS_print_osc_debug);

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

  auto left_foot_evaluator = multibody::WorldPointEvaluator(plant_w_springs,
                                                            left_toe_mid.first, left_toe_mid.second, Matrix3d::Identity(),
                                                            foot_contact_disp, {0, 2});

  auto right_foot_evaluator = multibody::WorldPointEvaluator(plant_w_springs,
                                                             right_toe_mid.first, right_toe_mid.second, Matrix3d::Identity(),
                                                             foot_contact_disp, {0, 2});

  osc->AddStateAndContactPoint(BipedStance::kLeft, &left_foot_evaluator);
  osc->AddStateAndContactPoint(BipedStance::kRight, &right_foot_evaluator);


  /*** tracking data ***/
  TransTaskSpaceTrackingData swing_foot_traj("swing_ft_traj",
                                             gains.K_p_swing_foot, gains.K_d_swing_foot, gains.W_swing_foot, plant_w_springs, plant_w_springs);

  swing_foot_traj.AddStateAndPointToTrack(BipedStance::kRight, "toe_left", left_toe_mid.first);
  swing_foot_traj.AddStateAndPointToTrack(BipedStance::kLeft, "toe_right", right_toe_mid.first);

  osc->AddTrackingData(&swing_foot_traj);

  // CoM offset
  Vector3d com_offset = {0, 0, -0.128};

  ComTrackingData com_traj("com_traj", gains.K_p_com,
                           gains.K_d_com, gains.W_com, plant_w_springs, plant_w_springs);

  TransTaskSpaceTrackingData pelvis_traj("com_traj", gains.K_p_com,
                                        gains.K_d_com, gains.W_com, plant_w_springs, plant_w_springs);
  pelvis_traj.AddPointToTrack("pelvis", com_offset);

  if (FLAGS_track_com) {
    osc->AddTrackingData(&com_traj);
  } else {
    osc->AddTrackingData(&pelvis_traj);
  }

  RotTaskSpaceTrackingData angular_traj("orientation_traj", gains.K_p_orientation,
                                      gains.K_d_orientation, gains.W_orientation, plant_w_springs, plant_w_springs);

  angular_traj.AddFrameToTrack("pelvis");

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
                  osc->get_tracking_data_input_port("orientation_traj"));

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
  owned_diagram->set_name(("osc_walking_controller"));

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