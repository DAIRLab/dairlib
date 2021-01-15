#include <drake/lcmt_contact_results_for_viz.hpp>
#include <drake/multibody/parsing/parser.h>
#include <gflags/gflags.h>

#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/osc_jump/com_traj_generator.h"
#include "examples/Cassie/osc_jump/flight_foot_traj_generator.h"
#include "examples/Cassie/osc_jump/flight_toe_angle_traj_generator.h"
#include "examples/Cassie/osc_jump/jumping_event_based_fsm.h"
#include "examples/Cassie/osc_jump/osc_jumping_gains.h"
#include "examples/Cassie/osc_jump/pelvis_orientation_traj_generator.h"
#include "lcm/dircon_saved_trajectory.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/kinematic/fixed_joint_evaluator.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/controllers/osc/osc_tracking_data.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/primitives/gaussian_noise_pass_through.h"
#include "systems/robot_lcm_systems.h"
#include "yaml-cpp/yaml.h"

#include "drake/common/yaml/yaml_read_archive.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace dairlib {

using std::cout;
using std::endl;
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
using examples::osc_jump::COMTrajGenerator;
using examples::osc_jump::FlightFootTrajGenerator;
using examples::osc_jump::JumpingEventFsm;
using examples::osc_jump::PelvisOrientationTrajGenerator;
using multibody::FixedJointEvaluator;
using systems::controllers::ComTrackingData;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;

namespace examples {

DEFINE_string(channel_x, "CASSIE_STATE_SIMULATION",
              "The name of the channel which receives state");
DEFINE_string(channel_u, "OSC_JUMPING",
              "The name of the channel which publishes command");
DEFINE_bool(print_osc, false, "whether to print the osc debug message or not");
DEFINE_string(folder_path, "examples/Cassie/saved_trajectories/",
              "Folder path for where the trajectory names are stored");
DEFINE_string(traj_name, "", "File to load saved trajectories from");
DEFINE_string(mode_name, "state_input_trajectory",
              "Base name of each trajectory");
DEFINE_double(delay_time, 0.0, "time to wait before executing jump");
DEFINE_bool(contact_based_fsm, true,
            "The contact based fsm transitions "
            "between states using contact data.");
DEFINE_double(transition_delay, 0.0,
              "Time to wait after trigger to "
              "transition between FSM states.");
DEFINE_string(simulator, "DRAKE",
              "Simulator used, important for determining how to interpret "
              "contact information. Other options include MUJOCO and soon to "
              "include contact results from the GM contact estimator.");
DEFINE_int32(init_fsm_state, osc_jump::BALANCE, "Initial state of the FSM");
DEFINE_string(gains_filename, "examples/Cassie/osc_jump/osc_jumping_gains.yaml",
              "Filepath containing gains");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Build the controller diagram
  DiagramBuilder<double> builder;

  // Built the Cassie MBPs
  drake::multibody::MultibodyPlant<double> plant_w_spr(0.0);
  addCassieMultibody(&plant_w_spr, nullptr, true,
                     "examples/Cassie/urdf/cassie_v2.urdf",
                     false /*spring model*/, false /*loop closure*/);
  plant_w_spr.Finalize();

  auto context_w_spr = plant_w_spr.CreateDefaultContext();

  // Get contact frames and position (doesn't matter whether we use
  // plant_w_spr or plant_wo_springs because the contact frames exit in both
  // plants)
  auto left_toe = LeftToeFront(plant_w_spr);
  auto left_heel = LeftToeRear(plant_w_spr);
  auto right_toe = RightToeFront(plant_w_spr);
  auto right_heel = RightToeRear(plant_w_spr);

  int nq = plant_w_spr.num_positions();
  int nv = plant_w_spr.num_velocities();
  int nx = nq + nv;

  // Create maps for joints
  map<string, int> pos_map = multibody::makeNameToPositionsMap(plant_w_spr);
  map<string, int> vel_map = multibody::makeNameToVelocitiesMap(plant_w_spr);
  map<string, int> act_map = multibody::makeNameToActuatorsMap(plant_w_spr);

  std::vector<std::pair<const Vector3d, const drake::multibody::Frame<double>&>>
      feet_contact_points = {left_toe, right_toe};

  /**** Convert the gains from the yaml struct to Eigen Matrices ****/
  OSCJumpingGains gains;
  const YAML::Node& root =
      YAML::LoadFile(FindResourceOrThrow(FLAGS_gains_filename));
  drake::yaml::YamlReadArchive(root).Accept(&gains);



  /**** Get trajectory from optimization ****/
  const DirconTrajectory& dircon_trajectory = DirconTrajectory(
      FindResourceOrThrow(FLAGS_folder_path + FLAGS_traj_name));
  const LcmTrajectory& processed_trajs = LcmTrajectory(
      FindResourceOrThrow(FLAGS_folder_path + FLAGS_traj_name + "_processed"));

  const LcmTrajectory::Trajectory lcm_com_traj =
      processed_trajs.GetTrajectory("center_of_mass_trajectory");
  const LcmTrajectory::Trajectory lcm_l_foot_traj =
      processed_trajs.GetTrajectory("left_foot_trajectory");
  const LcmTrajectory::Trajectory lcm_r_foot_traj =
      processed_trajs.GetTrajectory("right_foot_trajectory");
  const LcmTrajectory::Trajectory lcm_pelvis_rot_traj =
      processed_trajs.GetTrajectory("pelvis_rot_trajectory");

  std::cout << "Loading output trajectories: " << std::endl;
  PiecewisePolynomial<double> com_traj =
      PiecewisePolynomial<double>::CubicHermite(
          lcm_com_traj.time_vector, lcm_com_traj.datapoints.topRows(3),
          lcm_com_traj.datapoints.bottomRows(3));
  PiecewisePolynomial<double> l_foot_trajectory =
      PiecewisePolynomial<double>::CubicHermite(
          lcm_l_foot_traj.time_vector, lcm_l_foot_traj.datapoints.topRows(3),
          lcm_l_foot_traj.datapoints.bottomRows(3));
  PiecewisePolynomial<double> r_foot_trajectory =
      PiecewisePolynomial<double>::CubicHermite(
          lcm_r_foot_traj.time_vector, lcm_r_foot_traj.datapoints.topRows(3),
          lcm_r_foot_traj.datapoints.bottomRows(3));
  PiecewisePolynomial<double> pelvis_rot_trajectory;
  pelvis_rot_trajectory = PiecewisePolynomial<double>::FirstOrderHold(
      lcm_pelvis_rot_traj.time_vector,
      lcm_pelvis_rot_traj.datapoints.topRows(4));

  // For the time-based FSM (squatting by default)
  double flight_time = FLAGS_delay_time + 100;
  double land_time = FLAGS_delay_time + 200;
  if (dircon_trajectory.GetNumModes() == 3) {  // Override for jumping
    flight_time = FLAGS_delay_time + dircon_trajectory.GetStateBreaks(1)(0);
    land_time = FLAGS_delay_time + dircon_trajectory.GetStateBreaks(2)(0);
  }
  std::vector<double> transition_times = {0.0, FLAGS_delay_time, flight_time,
                                          land_time};

  Vector3d support_center_offset;
  support_center_offset << gains.x_offset, 0.0, 0.0;
  std::vector<double> breaks = com_traj.get_segment_times();
  std::vector<double> ft_breaks = l_foot_trajectory.get_segment_times();
  VectorXd breaks_vector = Eigen::Map<VectorXd>(breaks.data(), breaks.size());
  VectorXd ft_breaks_vector =
      Eigen::Map<VectorXd>(ft_breaks.data(), ft_breaks.size());
  MatrixXd offset_points = support_center_offset.replicate(1, breaks.size());
  MatrixXd ft_offset_points =
      support_center_offset.replicate(1, ft_breaks_vector.size());
  PiecewisePolynomial<double> offset_traj =
      PiecewisePolynomial<double>::ZeroOrderHold(breaks_vector, offset_points);
  com_traj = com_traj + offset_traj;
  l_foot_trajectory = l_foot_trajectory - ft_offset_points;
  r_foot_trajectory = r_foot_trajectory - ft_offset_points;

  /**** Initialize all the leaf systems ****/
  drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0");

  auto state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_w_spr);
  auto com_traj_generator = builder.AddSystem<COMTrajGenerator>(
      plant_w_spr, context_w_spr.get(), com_traj, feet_contact_points,
      FLAGS_delay_time);
  auto l_foot_traj_generator = builder.AddSystem<FlightFootTrajGenerator>(
      plant_w_spr, context_w_spr.get(), "hip_left", true, l_foot_trajectory,
      FLAGS_delay_time);
  auto r_foot_traj_generator = builder.AddSystem<FlightFootTrajGenerator>(
      plant_w_spr, context_w_spr.get(), "hip_right", false, r_foot_trajectory,
      FLAGS_delay_time);
  auto pelvis_rot_traj_generator =
      builder.AddSystem<PelvisOrientationTrajGenerator>(
          pelvis_rot_trajectory, "pelvis_rot_tracking_data", FLAGS_delay_time);
  auto fsm = builder.AddSystem<JumpingEventFsm>(
      plant_w_spr, transition_times, FLAGS_contact_based_fsm,
      gains.impact_threshold, (osc_jump::FSM_STATE)FLAGS_init_fsm_state);
  auto command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_channel_u, &lcm, TriggerTypeSet({TriggerType::kForced})));
  auto command_sender =
      builder.AddSystem<systems::RobotCommandSender>(plant_w_spr);
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      plant_w_spr, plant_w_spr, context_w_spr.get(), context_w_spr.get(), true,
      FLAGS_print_osc); /*print_tracking_info*/
  auto osc_debug_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
          "OSC_DEBUG_JUMPING", &lcm, TriggerTypeSet({TriggerType::kForced})));

  LcmSubscriberSystem* contact_results_sub = nullptr;
  if (FLAGS_simulator == "DRAKE") {
    contact_results_sub = builder.AddSystem(
        LcmSubscriberSystem::Make<drake::lcmt_contact_results_for_viz>(
            "CASSIE_CONTACT_DRAKE", &lcm));
  } else if (FLAGS_simulator == "MUJOCO") {
    contact_results_sub = builder.AddSystem(
        LcmSubscriberSystem::Make<drake::lcmt_contact_results_for_viz>(
            "CASSIE_CONTACT_MUJOCO", &lcm));
  } else if (FLAGS_simulator == "DISPATCHER") {
    contact_results_sub = builder.AddSystem(
        LcmSubscriberSystem::Make<drake::lcmt_contact_results_for_viz>(
            "CASSIE_CONTACT_FOR_FSM_DISPATCHER", &lcm));
    // TODO(yangwill): Add PR for GM contact observer, currently in
    // gm_contact_estimator branch
  } else {
    std::cerr << "Unknown simulator type!" << std::endl;
  }

  /**** OSC setup ****/
  // Cost
  MatrixXd Q_accel = gains.w_accel * MatrixXd::Identity(nv, nv);
  osc->SetAccelerationCostForAllJoints(Q_accel);
  // Soft constraint on contacts
  double w_contact_relax = gains.w_soft_constraint;
  osc->SetWeightOfSoftContactConstraint(w_contact_relax);

  // Contact information for OSC
  double mu = 0.6;
  osc->SetContactFriction(mu);

  auto left_toe_evaluator = multibody::WorldPointEvaluator(
      plant_w_spr, left_toe.first, left_toe.second, Matrix3d::Identity(),
      Vector3d::Zero(), {1, 2});
  auto left_heel_evaluator = multibody::WorldPointEvaluator(
      plant_w_spr, left_heel.first, left_heel.second, Matrix3d::Identity(),
      Vector3d::Zero(), {0, 1, 2});
  auto right_toe_evaluator = multibody::WorldPointEvaluator(
      plant_w_spr, right_toe.first, right_toe.second, Matrix3d::Identity(),
      Vector3d::Zero(), {1, 2});
  auto right_heel_evaluator = multibody::WorldPointEvaluator(
      plant_w_spr, right_heel.first, right_heel.second, Matrix3d::Identity(),
      Vector3d::Zero(), {0, 1, 2});
  vector<osc_jump::FSM_STATE> stance_modes = {osc_jump::BALANCE,
                                              osc_jump::CROUCH, osc_jump::LAND};
  for (auto mode : stance_modes) {
    osc->AddStateAndContactPoint(mode, &left_toe_evaluator);
    osc->AddStateAndContactPoint(mode, &left_heel_evaluator);
    osc->AddStateAndContactPoint(mode, &right_toe_evaluator);
    osc->AddStateAndContactPoint(mode, &right_heel_evaluator);
  }

  multibody::KinematicEvaluatorSet<double> evaluators(plant_w_spr);
  auto left_loop = LeftLoopClosureEvaluator(plant_w_spr);
  auto right_loop = RightLoopClosureEvaluator(plant_w_spr);
  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);

  auto pos_idx_map = multibody::makeNameToPositionsMap(plant_w_spr);
  auto vel_idx_map = multibody::makeNameToVelocitiesMap(plant_w_spr);
  auto left_fixed_knee_spring =
      FixedJointEvaluator(plant_w_spr, pos_idx_map.at("knee_joint_left"),
                          vel_idx_map.at("knee_joint_leftdot"), 0);
  auto right_fixed_knee_spring =
      FixedJointEvaluator(plant_w_spr, pos_idx_map.at("knee_joint_right"),
                          vel_idx_map.at("knee_joint_rightdot"), 0);
  auto left_fixed_ankle_spring = FixedJointEvaluator(
      plant_w_spr, pos_idx_map.at("ankle_spring_joint_left"),
      vel_idx_map.at("ankle_spring_joint_leftdot"), 0);
  auto right_fixed_ankle_spring = FixedJointEvaluator(
      plant_w_spr, pos_idx_map.at("ankle_spring_joint_right"),
      vel_idx_map.at("ankle_spring_joint_rightdot"), 0);
  evaluators.add_evaluator(&left_fixed_knee_spring);
  evaluators.add_evaluator(&right_fixed_knee_spring);
  evaluators.add_evaluator(&left_fixed_ankle_spring);
  evaluators.add_evaluator(&right_fixed_ankle_spring);

  osc->AddKinematicConstraint(&evaluators);

  /**** Tracking Data for OSC *****/
  TransTaskSpaceTrackingData com_tracking_data("com_traj", gains.K_p_com, gains.K_d_com,
                                               gains.W_com, plant_w_spr, plant_w_spr);
  for (auto mode : stance_modes) {
    com_tracking_data.AddStateAndPointToTrack(mode, "pelvis");
  }
  osc->AddTrackingData(&com_tracking_data);

  TransTaskSpaceTrackingData left_foot_tracking_data(
      "left_ft_traj", gains.K_p_flight_foot, gains.K_d_flight_foot, gains.W_flight_foot,
      plant_w_spr, plant_w_spr);
  TransTaskSpaceTrackingData right_foot_tracking_data(
      "right_ft_traj", gains.K_p_flight_foot, gains.K_d_flight_foot, gains.W_flight_foot,
      plant_w_spr, plant_w_spr);
  left_foot_tracking_data.AddStateAndPointToTrack(osc_jump::FLIGHT, "toe_left");
  right_foot_tracking_data.AddStateAndPointToTrack(osc_jump::FLIGHT,
                                                   "toe_right");

  RotTaskSpaceTrackingData pelvis_rot_tracking_data(
      "pelvis_rot_tracking_data", gains.K_p_pelvis, gains.K_d_pelvis, gains.W_pelvis, plant_w_spr,
      plant_w_spr);

  for (auto mode : stance_modes) {
    pelvis_rot_tracking_data.AddStateAndFrameToTrack(mode, "pelvis");
  }

  // Yaw tracking
  MatrixXd W_hip_yaw = gains.w_hip_yaw * MatrixXd::Identity(1, 1);
  MatrixXd K_p_hip_yaw = gains.hip_yaw_kp * MatrixXd::Identity(1, 1);
  MatrixXd K_d_hip_yaw = gains.hip_yaw_kd * MatrixXd::Identity(1, 1);
  JointSpaceTrackingData swing_hip_yaw_left_traj(
      "swing_hip_yaw_left_traj", K_p_hip_yaw, K_d_hip_yaw, W_hip_yaw,
      plant_w_spr, plant_w_spr);
  JointSpaceTrackingData swing_hip_yaw_right_traj(
      "swing_hip_yaw_right_traj", K_p_hip_yaw, K_d_hip_yaw, W_hip_yaw,
      plant_w_spr, plant_w_spr);
  swing_hip_yaw_left_traj.AddStateAndJointToTrack(
      osc_jump::FLIGHT, "hip_yaw_left", "hip_yaw_leftdot");
  swing_hip_yaw_right_traj.AddStateAndJointToTrack(
      osc_jump::FLIGHT, "hip_yaw_right", "hip_yaw_rightdot");
  osc->AddConstTrackingData(&swing_hip_yaw_left_traj, VectorXd::Zero(1));
  osc->AddConstTrackingData(&swing_hip_yaw_right_traj, VectorXd::Zero(1));

  // Toe tracking flight phase
  MatrixXd W_swing_toe = gains.w_swing_toe * MatrixXd::Identity(1, 1);
  MatrixXd K_p_swing_toe = gains.swing_toe_kp * MatrixXd::Identity(1, 1);
  MatrixXd K_d_swing_toe = gains.swing_toe_kd * MatrixXd::Identity(1, 1);
  JointSpaceTrackingData left_toe_angle_traj(
      "left_toe_angle_traj", K_p_swing_toe, K_d_swing_toe, W_swing_toe,
      plant_w_spr, plant_w_spr);
  JointSpaceTrackingData right_toe_angle_traj(
      "right_toe_angle_traj", K_p_swing_toe, K_d_swing_toe, W_swing_toe,
      plant_w_spr, plant_w_spr);

  vector<std::pair<const Vector3d, const Frame<double>&>> left_foot_points = {
      left_heel, left_toe};
  vector<std::pair<const Vector3d, const Frame<double>&>> right_foot_points = {
      right_heel, right_toe};

  auto left_toe_angle_traj_gen =
      builder.AddSystem<cassie::osc_jump::FlightToeAngleTrajGenerator>(
          plant_w_spr, context_w_spr.get(), pos_map["toe_left"],
          left_foot_points, "left_toe_angle_traj");
  auto right_toe_angle_traj_gen =
      builder.AddSystem<cassie::osc_jump::FlightToeAngleTrajGenerator>(
          plant_w_spr, context_w_spr.get(), pos_map["toe_right"],
          right_foot_points, "right_toe_angle_traj");

  //  left_toe_angle_traj.AddStateAndJointToTrack(osc_jump::CROUCH, "toe_left",
  //                                              "toe_leftdot");
  //  right_toe_angle_traj.AddStateAndJointToTrack(osc_jump::CROUCH,
  //  "toe_right",
  //                                               "toe_rightdot");
  left_toe_angle_traj.AddStateAndJointToTrack(osc_jump::FLIGHT, "toe_left",
                                              "toe_leftdot");
  right_toe_angle_traj.AddStateAndJointToTrack(osc_jump::FLIGHT, "toe_right",
                                               "toe_rightdot");
  //  left_toe_angle_traj.AddStateAndJointToTrack(osc_jump::LAND, "toe_left",
  //                                              "toe_leftdot");
  //  right_toe_angle_traj.AddStateAndJointToTrack(osc_jump::LAND, "toe_right",
  //                                               "toe_rightdot");

  osc->AddTrackingData(&pelvis_rot_tracking_data);
  osc->AddTrackingData(&left_foot_tracking_data);
  osc->AddTrackingData(&right_foot_tracking_data);
  osc->AddTrackingData(&left_toe_angle_traj);
  osc->AddTrackingData(&right_toe_angle_traj);

  // Build OSC problem
  osc->Build();
  std::cout << "Built OSC" << std::endl;

  /*****Connect ports*****/

  // OSC connections
  builder.Connect(fsm->get_fsm_output_port(), osc->get_fsm_input_port());
  builder.Connect(fsm->get_impact_output_port(),
                  osc->get_near_impact_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  osc->get_robot_output_input_port());
  builder.Connect(com_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("com_traj"));
  builder.Connect(l_foot_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("left_ft_traj"));
  builder.Connect(r_foot_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("right_ft_traj"));
  builder.Connect(left_toe_angle_traj_gen->get_output_port(0),
                  osc->get_tracking_data_input_port("left_toe_angle_traj"));
  builder.Connect(right_toe_angle_traj_gen->get_output_port(0),
                  osc->get_tracking_data_input_port("right_toe_angle_traj"));
  builder.Connect(
      pelvis_rot_traj_generator->get_output_port(0),
      osc->get_tracking_data_input_port("pelvis_rot_tracking_data"));

  // FSM connections
  builder.Connect(contact_results_sub->get_output_port(),
                  fsm->get_contact_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  fsm->get_state_input_port());
  //  builder.Connect(controller_switch_receiver->get_output_port(),
  //                  fsm->get_switch_input_port());

  // Trajectory generator connections
  builder.Connect(state_receiver->get_output_port(0),
                  com_traj_generator->get_state_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  l_foot_traj_generator->get_state_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  r_foot_traj_generator->get_state_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  left_toe_angle_traj_gen->get_state_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  right_toe_angle_traj_gen->get_state_input_port());
  builder.Connect(fsm->get_output_port(0),
                  com_traj_generator->get_fsm_input_port());
  builder.Connect(fsm->get_output_port(0),
                  l_foot_traj_generator->get_fsm_input_port());
  builder.Connect(fsm->get_output_port(0),
                  r_foot_traj_generator->get_fsm_input_port());
  builder.Connect(fsm->get_output_port(0),
                  left_toe_angle_traj_gen->get_fsm_input_port());
  builder.Connect(fsm->get_output_port(0),
                  right_toe_angle_traj_gen->get_fsm_input_port());

  // Publisher connections
  builder.Connect(osc->get_osc_output_port(),
                  command_sender->get_input_port(0));
  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port());
  builder.Connect(osc->get_osc_debug_port(), osc_debug_pub->get_input_port());

  // Run lcm-driven simulation
  // Create the diagram
  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("osc jumping controller"));

  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, std::move(owned_diagram), state_receiver, FLAGS_channel_x, true);
  loop.Simulate();

  return 0;
}
}  // namespace examples
}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::examples::DoMain(argc, argv);
}
