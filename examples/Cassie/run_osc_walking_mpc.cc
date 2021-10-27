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
#include "systems/controllers/osc/com_tracking_data.h"
#include "systems/controllers/osc/trans_space_tracking_data.h"
#include "systems/controllers/osc/joint_space_tracking_data.h"
#include "systems/controllers/osc/rpy_space_tracking_data.h"

#include "systems/controllers/time_based_fsm.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"

#include "examples/Cassie/mpc/cassie_mpc_osc_walking_gains.h"
#include "systems/controllers/mpc/mpc_trajectory_reciever.h"
#include "systems/system_utils.h"

#include "drake/common/yaml/yaml_read_archive.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "systems/dairlib_signal_lcm_systems.h"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/osc/swing_toe_traj_generator.h"
#include "multibody/kinematic/fixed_joint_evaluator.h"

namespace dairlib {

using std::map;
using std::pair;
using std::string;
using std::vector;

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using multibody::FixedJointEvaluator;

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
using systems::controllers::RpyTaskSpaceTrackingData;
using systems::TimeBasedFiniteStateMachine;
using systems::DairlibSignalReceiver;

namespace examples {

DEFINE_string(channel_x, "CASSIE_STATE_DISPATCHER",
              "The name of the channel which receives state");
DEFINE_string(channel_u, "CASSIE_INPUT",
              "The name of the channel which publishes command");
DEFINE_string(channel_fsm, "FSM", "the name of the channel with the time-based fsm");

DEFINE_string(mpc_channel, "SRBD_MPC_OUT", "channel to recieve koopman mpc message");

DEFINE_string(
    gains_filename,
    "examples/Cassie/mpc/cassie_mpc_osc_walking_gains.yaml","Filepath containing gains");

DEFINE_bool(track_com, false,
            "use com tracking data (otherwise uses trans space)");

DEFINE_bool(print_osc_debug, false, "print osc_debug to the terminal");

DEFINE_bool(make_srbd_approx, false, "modify plant to closer approximate single rigid body assumption");

DEFINE_bool(print_diagram, false, "whether to print the graphviz diagram");

void print_gains(const CassieMpcOSCWalkingGains& gains);

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
  auto right_toe = RightToeFront(plant_w_springs);
  auto right_heel = RightToeRear(plant_w_springs);

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
  Vector3d com_offset = {0, 0, -0.128};

  if (FLAGS_make_srbd_approx) {
    std::vector<std::string> links = {"yaw_left", "yaw_right", "hip_left", "hip_right",
                                      "thigh_left", "thigh_right", "knee_left", "knee_right", "shin_left",
                                      "shin_right"};
    drake::multibody::RotationalInertia I_rot(
        0.91, 0.55, 0.89, 0.04, 0.09, -.001);
    double mass = 30.0218;

    multibody::MakePlantApproximateRigidBody(plant_context.get(), plant_w_springs,
                                             "pelvis", links, com_offset, I_rot, mass, 0.02);
  }
  // Get contact frames and position (doesn't matter whether we use
  // plant_w_springs or plant_wo_springs because the contact frames exit in both
  // plants)
  int nv = plant_w_springs.num_velocities();

  // Create maps for joints
  map<string, int> pos_map = multibody::makeNameToPositionsMap(plant_w_springs);
  map<string, int> vel_map = multibody::makeNameToVelocitiesMap(plant_w_springs);
  map<string, int> act_map = multibody::makeNameToActuatorsMap(plant_w_springs);

  /**** Convert the gains from the yaml struct to Eigen Matrices ****/
  CassieMpcOSCWalkingGains gains;
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
      TrajectoryType::kCubicHermite, false);

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

   // Constraints in OSC
  multibody::KinematicEvaluatorSet<double> evaluators(plant_w_springs);
  // 1. fourbar constraint
  auto left_loop = LeftLoopClosureEvaluator(plant_w_springs);
  auto right_loop = RightLoopClosureEvaluator(plant_w_springs);
  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);
  // 2. fixed spring constraint
  // Note that we set the position value to 0, but this is not used in OSC,
  // because OSC constraint only use JdotV and J.
  auto pos_idx_map = multibody::makeNameToPositionsMap(plant_w_springs);
  auto vel_idx_map = multibody::makeNameToVelocitiesMap(plant_w_springs);
  auto left_fixed_knee_spring =
      FixedJointEvaluator(plant_w_springs, pos_idx_map.at("knee_joint_left"),
                          vel_idx_map.at("knee_joint_leftdot"), 0);
  auto right_fixed_knee_spring =
      FixedJointEvaluator(plant_w_springs, pos_idx_map.at("knee_joint_right"),
                          vel_idx_map.at("knee_joint_rightdot"), 0);
  auto left_fixed_ankle_spring = FixedJointEvaluator(
      plant_w_springs, pos_idx_map.at("ankle_spring_joint_left"),
      vel_idx_map.at("ankle_spring_joint_leftdot"), 0);
  auto right_fixed_ankle_spring = FixedJointEvaluator(
      plant_w_springs, pos_idx_map.at("ankle_spring_joint_right"),
      vel_idx_map.at("ankle_spring_joint_rightdot"), 0);
  evaluators.add_evaluator(&left_fixed_knee_spring);
  evaluators.add_evaluator(&right_fixed_knee_spring);
  evaluators.add_evaluator(&left_fixed_ankle_spring);
  evaluators.add_evaluator(&right_fixed_ankle_spring);
  osc->AddKinematicConstraint(&evaluators);

  // Soft constraint on contacts
  osc->SetWeightOfSoftContactConstraint(gains.w_soft_constraint);

  // Contact information for OSC
  osc->SetContactFriction(gains.mu);

  // Add contact points (The position doesn't matter. It's not used in OSC)
  auto left_toe_evaluator = multibody::WorldPointEvaluator(
      plant_w_springs, left_toe.first, left_toe.second, Matrix3d::Identity(),
      Vector3d::Zero(), {1, 2});
  auto left_heel_evaluator = multibody::WorldPointEvaluator(
      plant_w_springs, left_heel.first, left_heel.second, Matrix3d::Identity(),
      Vector3d::Zero(), {0, 1, 2});
  auto right_toe_evaluator = multibody::WorldPointEvaluator(
      plant_w_springs, right_toe.first, right_toe.second, Matrix3d::Identity(),
      Vector3d::Zero(), {1, 2});
  auto right_heel_evaluator = multibody::WorldPointEvaluator(
      plant_w_springs, right_heel.first, right_heel.second, Matrix3d::Identity(),
      Vector3d::Zero(), {0, 1, 2});

  Vector3d foot_contact_disp(0, 0, 0);

  osc->AddStateAndContactPoint(BipedStance::kLeft, &left_toe_evaluator);
  osc->AddStateAndContactPoint(BipedStance::kLeft, &left_heel_evaluator);
  osc->AddStateAndContactPoint(BipedStance::kRight, &right_toe_evaluator);
  osc->AddStateAndContactPoint(BipedStance::kRight, &right_heel_evaluator);

  // Swing toe joint trajectory
  vector<std::pair<const Vector3d, const Frame<double>&>> left_foot_points = {
      left_heel, left_toe};
  vector<std::pair<const Vector3d, const Frame<double>&>> right_foot_points = {
      right_heel, right_toe};
  auto left_toe_angle_traj_gen = builder.AddSystem<cassie::osc::SwingToeTrajGenerator>(
      plant_w_springs, plant_context.get(), pos_map["toe_left"], left_foot_points,
      "left_toe_angle_traj");
  auto right_toe_angle_traj_gen = builder.AddSystem<cassie::osc::SwingToeTrajGenerator>(
      plant_w_springs, plant_context.get(), pos_map["toe_right"], right_foot_points,
      "right_toe_angle_traj");


  // Swing toe joint tracking
  JointSpaceTrackingData swing_toe_traj_left(
      "left_toe_angle_traj", gains.K_p_swing_toe, gains.K_d_swing_toe,
      gains.W_swing_toe, plant_w_springs, plant_w_springs);
  JointSpaceTrackingData swing_toe_traj_right(
      "right_toe_angle_traj", gains.K_p_swing_toe, gains.K_d_swing_toe,
      gains.W_swing_toe, plant_w_springs, plant_w_springs);
  swing_toe_traj_right.AddStateAndJointToTrack(BipedStance::kLeft, "toe_right",
                                               "toe_rightdot");
  swing_toe_traj_left.AddStateAndJointToTrack(BipedStance::kRight, "toe_left",
                                              "toe_leftdot");
  osc->AddTrackingData(&swing_toe_traj_left);
  osc->AddTrackingData(&swing_toe_traj_right);

  /*** tracking data ***/
  TransTaskSpaceTrackingData swing_foot_traj("swing_ft_traj",
                                             gains.K_p_swing_foot, gains.K_d_swing_foot, gains.W_swing_foot, plant_w_springs, plant_w_springs);

  swing_foot_traj.AddStateAndPointToTrack(BipedStance::kLeft, "toe_right", right_toe_mid.first);
  swing_foot_traj.AddStateAndPointToTrack(BipedStance::kRight, "toe_left", left_toe_mid.first);

  osc->AddTrackingData(&swing_foot_traj);


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

  RpyTaskSpaceTrackingData angular_traj("orientation_traj", gains.K_p_orientation,
                                      gains.K_d_orientation, gains.W_orientation, plant_w_springs, plant_w_springs);

  angular_traj.AddFrameToTrack("pelvis");

  osc->AddTrackingData(&angular_traj);

  // Swing hip yaw joint tracking
  JointSpaceTrackingData swing_hip_yaw_traj(
      "swing_hip_yaw_traj", gains.K_p_hip_yaw, gains.K_d_hip_yaw,
      gains.W_hip_yaw, plant_w_springs, plant_w_springs);
  swing_hip_yaw_traj.AddStateAndJointToTrack(
      BipedStance::kLeft, "hip_yaw_right","hip_yaw_rightdot");
  swing_hip_yaw_traj.AddStateAndJointToTrack(
      BipedStance::kRight, "hip_yaw_left","hip_yaw_leftdot");
  osc->AddConstTrackingData(&swing_hip_yaw_traj, VectorXd::Zero(1));

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

  //toe angles
  builder.Connect(left_toe_angle_traj_gen->get_output_port(0),
                  osc->get_tracking_data_input_port("left_toe_angle_traj"));
  builder.Connect(right_toe_angle_traj_gen->get_output_port(0),
                  osc->get_tracking_data_input_port("right_toe_angle_traj"));
  builder.Connect(state_receiver->get_output_port(0),
                  left_toe_angle_traj_gen->get_state_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  right_toe_angle_traj_gen->get_state_input_port());

  // Publisher connections
  builder.Connect(osc->get_osc_output_port(),
                  command_sender->get_input_port(0));
  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port());
  builder.Connect(osc->get_osc_debug_port(), osc_debug_pub->get_input_port());

  // Run lcm-driven simulation
  // Create the diagram
  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("osc_walking_controller_mpc"));

  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm_local, std::move(owned_diagram), state_receiver, FLAGS_channel_x, true);

  if (FLAGS_print_diagram) {
    DrawAndSaveDiagramGraph(*loop.get_diagram());
  }

  LcmHandleSubscriptionsUntil(&lcm_local, [&]() {
    return mpc_subscriber->GetInternalMessageCount() > 1; });

  auto& loop_context = loop.get_diagram_mutable_context();
  mpc_subscriber->Publish(loop.get_diagram()->
      GetMutableSubsystemContext(*mpc_subscriber, &loop_context));

  loop.Simulate();


  return 0;
}

void print_gains(const CassieMpcOSCWalkingGains& gains) {
  std::cout <<"======== OSC WALKING GAINS ==========\n";
}

}  // namespace examples
}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::examples::DoMain(argc, argv);
}