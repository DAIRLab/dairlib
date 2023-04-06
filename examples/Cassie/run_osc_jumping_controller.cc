
#include <drake/common/yaml/yaml_io.h>
#include <drake/common/yaml/yaml_read_archive.h>
#include <drake/lcmt_contact_results_for_viz.hpp>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <gflags/gflags.h>

#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/osc_jump/basic_trajectory_passthrough.h"
#include "examples/Cassie/osc_jump/flight_foot_traj_generator.h"
#include "examples/Cassie/osc_jump/jumping_event_based_fsm.h"
#include "examples/Cassie/osc_jump/osc_jumping_gains.h"
#include "examples/Cassie/osc_jump/pelvis_trans_traj_generator.h"
#include "examples/Cassie/osc_jump/toe_angle_traj_generator.h"
#include "lcm/dircon_saved_trajectory.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/kinematic/fixed_joint_evaluator.h"
#include "systems/controllers/controller_failure_aggregator.h"
#include "systems/controllers/osc/joint_space_tracking_data.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/controllers/osc/osc_tracking_data.h"
#include "systems/controllers/osc/relative_translation_tracking_data.h"
#include "systems/controllers/osc/rot_space_tracking_data.h"
#include "systems/controllers/osc/trans_space_tracking_data.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

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
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::trajectories::PiecewisePolynomial;
using examples::osc_jump::BasicTrajectoryPassthrough;
using examples::osc_jump::FlightFootTrajGenerator;
using examples::osc_jump::JumpingEventFsm;
using examples::osc_jump::PelvisTransTrajGenerator;
using multibody::FixedJointEvaluator;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::RelativeTranslationTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;

namespace examples {

DEFINE_string(
    channel_x, "CASSIE_STATE_SIMULATION",
    "The name of the channel where state estimation is published. Set to "
    "CASSIE_STATE_DISPATCHER for use on hardware with the state estimator");
DEFINE_string(channel_u, "OSC_JUMPING",
              "The name of the channel where control efforts are published");
DEFINE_double(delay_time, 0.0,
              "Time to wait before executing jump. Useful for getting the "
              "robot state into the desired initial state.");
DEFINE_bool(contact_based_fsm, false,
            "The contact based fsm transitions "
            "between states using contact data.");
DEFINE_string(simulator, "DRAKE",
              "Simulator used, important for determining how to interpret "
              "contact information. Other options include MUJOCO and soon to "
              "include contact results from the GM contact estimator.");
DEFINE_int32(init_fsm_state, osc_jump::BALANCE, "Initial state of the FSM");
DEFINE_string(folder_path, "examples/Cassie/saved_trajectories/",
              "Folder path for where the trajectory names are stored");
DEFINE_string(traj_name, "jumping_0.15h_0.3d",
              "File to load saved trajectories from");
DEFINE_string(gains_filename, "examples/Cassie/osc_jump/osc_jumping_gains.yaml",
              "Filepath containing gains");
DEFINE_string(osqp_settings,
              "examples/Cassie/osc_run/osc_running_qp_settings.yaml",
              "Filepath containing qp settings");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Build the controller diagram
  DiagramBuilder<double> builder;

  // Built the Cassie MBPs
  drake::multibody::MultibodyPlant<double> plant_w_spr(0.0);
  AddCassieMultibody(&plant_w_spr, nullptr, true,
                     "examples/Cassie/urdf/cassie_v2_conservative.urdf",
                     true /*spring model*/, false /*loop closure*/);
  plant_w_spr.Finalize();

  auto context_w_spr = plant_w_spr.CreateDefaultContext();

  // Get contact frames and position
  auto left_toe = LeftToeFront(plant_w_spr);
  auto left_heel = LeftToeRear(plant_w_spr);
  auto right_toe = RightToeFront(plant_w_spr);
  auto right_heel = RightToeRear(plant_w_spr);

  // Create maps for joints
  map<string, int> pos_map = multibody::MakeNameToPositionsMap(plant_w_spr);
  map<string, int> vel_map = multibody::MakeNameToVelocitiesMap(plant_w_spr);
  map<string, int> act_map = multibody::MakeNameToActuatorsMap(plant_w_spr);

  std::vector<std::pair<const Vector3d, const drake::multibody::Frame<double>&>>
      feet_contact_points = {left_toe, right_toe};

  /**** Convert the gains from the yaml struct to Eigen Matrices ****/
  drake::yaml::LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;

  OSCGains gains = drake::yaml::LoadYamlFile<OSCGains>(
      FindResourceOrThrow(FLAGS_gains_filename), {}, {}, yaml_options);
  OSCJumpingGains osc_gains = drake::yaml::LoadYamlFile<OSCJumpingGains>(
      FindResourceOrThrow(FLAGS_gains_filename));

  /**** Get trajectory from optimization ****/
  const DirconTrajectory& dircon_trajectory = DirconTrajectory(
      plant_w_spr, FindResourceOrThrow(FLAGS_folder_path + FLAGS_traj_name));
  string output_traj_path = FLAGS_folder_path + FLAGS_traj_name + "_processed";
  if (osc_gains.relative_feet) {
    output_traj_path += "_rel";
  }
  const LcmTrajectory& output_trajs =
      LcmTrajectory(FindResourceOrThrow(output_traj_path));

  PiecewisePolynomial<double> state_traj =
      dircon_trajectory.ReconstructStateTrajectory();

  PiecewisePolynomial<double> pelvis_trans_traj;
  PiecewisePolynomial<double> l_foot_trajectory;
  PiecewisePolynomial<double> r_foot_trajectory;
  PiecewisePolynomial<double> l_hip_trajectory;
  PiecewisePolynomial<double> r_hip_trajectory;
  PiecewisePolynomial<double> l_toe_trajectory;
  PiecewisePolynomial<double> r_toe_trajectory;
  PiecewisePolynomial<double> pelvis_rot_trajectory;

  for (int mode = 0; mode < dircon_trajectory.GetNumModes(); ++mode) {
    const LcmTrajectory::Trajectory lcm_pelvis_trans_trajectory =
        output_trajs.GetTrajectory("pelvis_trans_trajectory" +
                                   std::to_string(mode));
    const LcmTrajectory::Trajectory lcm_left_foot_traj =
        output_trajs.GetTrajectory("left_foot_trajectory" +
                                   std::to_string(mode));
    const LcmTrajectory::Trajectory lcm_right_foot_traj =
        output_trajs.GetTrajectory("right_foot_trajectory" +
                                   std::to_string(mode));
    const LcmTrajectory::Trajectory lcm_left_hip_traj =
        output_trajs.GetTrajectory("left_hip_trajectory" +
                                   std::to_string(mode));
    const LcmTrajectory::Trajectory lcm_right_hip_traj =
        output_trajs.GetTrajectory("right_hip_trajectory" +
                                   std::to_string(mode));
    const LcmTrajectory::Trajectory lcm_left_toe_traj =
        output_trajs.GetTrajectory("left_toe_trajectory" +
                                   std::to_string(mode));
    const LcmTrajectory::Trajectory lcm_right_toe_traj =
        output_trajs.GetTrajectory("right_toe_trajectory" +
                                   std::to_string(mode));
    const LcmTrajectory::Trajectory lcm_pelvis_rot_traj =
        output_trajs.GetTrajectory("pelvis_rot_trajectory" +
                                   std::to_string(mode));
    pelvis_trans_traj.ConcatenateInTime(
        PiecewisePolynomial<double>::CubicHermite(
            lcm_pelvis_trans_trajectory.time_vector,
            lcm_pelvis_trans_trajectory.datapoints.topRows(3),
            lcm_pelvis_trans_trajectory.datapoints.bottomRows(3)));
    l_foot_trajectory.ConcatenateInTime(
        PiecewisePolynomial<double>::CubicHermite(
            lcm_left_foot_traj.time_vector,
            lcm_left_foot_traj.datapoints.topRows(6),
            lcm_left_foot_traj.datapoints.bottomRows(6)));
    r_foot_trajectory.ConcatenateInTime(
        PiecewisePolynomial<double>::CubicHermite(
            lcm_right_foot_traj.time_vector,
            lcm_right_foot_traj.datapoints.topRows(6),
            lcm_right_foot_traj.datapoints.bottomRows(6)));
    l_hip_trajectory.ConcatenateInTime(
        PiecewisePolynomial<double>::CubicHermite(
            lcm_left_hip_traj.time_vector,
            lcm_left_hip_traj.datapoints.topRows(6),
            lcm_left_hip_traj.datapoints.bottomRows(6)));
    r_hip_trajectory.ConcatenateInTime(
        PiecewisePolynomial<double>::CubicHermite(
            lcm_right_hip_traj.time_vector,
            lcm_right_hip_traj.datapoints.topRows(6),
            lcm_right_hip_traj.datapoints.bottomRows(6)));
    l_toe_trajectory.ConcatenateInTime(
        PiecewisePolynomial<double>::CubicHermite(
            lcm_left_toe_traj.time_vector,
            lcm_left_toe_traj.datapoints.topRows(1),
            lcm_left_toe_traj.datapoints.bottomRows(1)));
    r_toe_trajectory.ConcatenateInTime(
        PiecewisePolynomial<double>::CubicHermite(
            lcm_right_toe_traj.time_vector,
            lcm_right_toe_traj.datapoints.topRows(1),
            lcm_right_toe_traj.datapoints.bottomRows(1)));
    pelvis_rot_trajectory.ConcatenateInTime(
        PiecewisePolynomial<double>::FirstOrderHold(
            lcm_pelvis_rot_traj.time_vector,
            lcm_pelvis_rot_traj.datapoints.topRows(4)));
  }

  // For the time-based FSM (squatting by default)
  double flight_time =
      FLAGS_delay_time + dircon_trajectory.GetStateBreaks(1)(0);
  double land_time = FLAGS_delay_time + dircon_trajectory.GetStateBreaks(2)(0) +
                     osc_gains.landing_delay;
  std::vector<double> transition_times = {0.0, FLAGS_delay_time, flight_time,
                                          land_time};

  // Offset the output trajectories to account for the starting global position
  // of the robot
  Vector3d support_center_offset;
  support_center_offset << osc_gains.crouch_x_offset, 0.0, 0.0;
  std::vector<double> breaks = pelvis_trans_traj.get_segment_times();
  VectorXd breaks_vector = Eigen::Map<VectorXd>(breaks.data(), breaks.size());
  MatrixXd offset_points = support_center_offset.replicate(1, breaks.size());
  PiecewisePolynomial<double> offset_traj =
      PiecewisePolynomial<double>::ZeroOrderHold(breaks_vector, offset_points);
  pelvis_trans_traj = pelvis_trans_traj + offset_traj;

  /**** Initialize all the leaf systems ****/
  drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0");

  auto state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_w_spr);
  // This actually outputs the target position of the pelvis not the true
  // center of mass
  auto pelvis_trans_traj_generator =
      builder.AddSystem<PelvisTransTrajGenerator>(
          plant_w_spr, context_w_spr.get(), pelvis_trans_traj,
          feet_contact_points, FLAGS_delay_time);
  auto l_foot_traj_generator = builder.AddSystem<FlightFootTrajGenerator>(
      plant_w_spr, context_w_spr.get(), "hip_left", true, l_foot_trajectory,
      l_hip_trajectory, FLAGS_delay_time);
  auto r_foot_traj_generator = builder.AddSystem<FlightFootTrajGenerator>(
      plant_w_spr, context_w_spr.get(), "hip_right", false, r_foot_trajectory,
      r_hip_trajectory, FLAGS_delay_time);
  auto pelvis_rot_traj_generator =
      builder.AddSystem<BasicTrajectoryPassthrough>(
          pelvis_rot_trajectory, "pelvis_rot_traj", FLAGS_delay_time);
  auto fsm = builder.AddSystem<JumpingEventFsm>(
      plant_w_spr, transition_times, FLAGS_contact_based_fsm,
      gains.impact_threshold,
      (osc_jump::JUMPING_FSM_STATE)FLAGS_init_fsm_state);
  auto command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_channel_u, &lcm, TriggerTypeSet({TriggerType::kForced})));
  auto command_sender =
      builder.AddSystem<systems::RobotCommandSender>(plant_w_spr);
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      plant_w_spr, plant_w_spr, context_w_spr.get(), context_w_spr.get(), true);
  auto osc_debug_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
          "OSC_DEBUG_JUMPING", &lcm, TriggerTypeSet({TriggerType::kForced})));
  auto failure_aggregator =
      builder.AddSystem<systems::ControllerFailureAggregator>(FLAGS_channel_u,
                                                              1);
  auto controller_failure_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_controller_failure>(
          "CONTROLLER_ERROR", &lcm, TriggerTypeSet({TriggerType::kForced})));

  //   For contact-based fsm
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
  } else {
    std::cerr << "Unknown simulator type!" << std::endl;
  }

  pelvis_trans_traj_generator->SetLandingOffset(osc_gains.land_x_offset);

  /**** OSC setup ****/
  // Cost
  osc->SetAccelerationCostWeights(gains.w_accel * gains.W_acceleration);
  osc->SetInputSmoothingCostWeights(gains.w_input *
                                    gains.W_input_regularization);
  osc->SetInputCostWeights(gains.w_input * gains.W_input_regularization);
  osc->SetLambdaContactRegularizationWeight(gains.w_lambda *
                                            gains.W_lambda_c_regularization);
  osc->SetLambdaHolonomicRegularizationWeight(gains.w_lambda *
                                              gains.W_lambda_h_regularization);
  // Soft constraint on contacts
  osc->SetContactSoftConstraintWeight(gains.w_soft_constraint);

  // Contact information for OSC
  osc->SetContactFriction(gains.mu);

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
  vector<osc_jump::JUMPING_FSM_STATE> stance_modes = {
      osc_jump::BALANCE, osc_jump::CROUCH, osc_jump::LAND};
  for (auto mode : stance_modes) {
    osc->AddStateAndContactPoint(mode, &left_toe_evaluator);
    osc->AddStateAndContactPoint(mode, &left_heel_evaluator);
    osc->AddStateAndContactPoint(mode, &right_toe_evaluator);
    osc->AddStateAndContactPoint(mode, &right_heel_evaluator);
  }

  multibody::KinematicEvaluatorSet<double> evaluators(plant_w_spr);

  // Fix the springs in the dynamics
  auto pos_idx_map = multibody::MakeNameToPositionsMap(plant_w_spr);
  auto vel_idx_map = multibody::MakeNameToVelocitiesMap(plant_w_spr);
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

  auto left_loop = LeftLoopClosureEvaluator(plant_w_spr);
  auto right_loop = RightLoopClosureEvaluator(plant_w_spr);
  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);

  osc->AddKinematicConstraint(&evaluators);

  /**** Tracking Data for OSC *****/
  auto pelvis_tracking_data = std::make_unique<TransTaskSpaceTrackingData>(
      "pelvis_trans_traj", osc_gains.K_p_com, osc_gains.K_d_com,
      osc_gains.W_com, plant_w_spr, plant_w_spr);
  auto pelvis_rot_tracking_data = std::make_unique<RotTaskSpaceTrackingData>(
      "pelvis_rot_traj", osc_gains.K_p_pelvis, osc_gains.K_d_pelvis,
      osc_gains.W_pelvis, plant_w_spr, plant_w_spr);
  for (auto mode : stance_modes) {
    pelvis_tracking_data->AddStateAndPointToTrack(mode, "pelvis");
    pelvis_rot_tracking_data->AddStateAndFrameToTrack(mode, "pelvis");
  }
  pelvis_rot_tracking_data->SetImpactInvariantProjection(true);
  pelvis_tracking_data->SetImpactInvariantProjection(true);
  VectorXd pelvis_acc_lb = Vector3d::Ones();
  pelvis_acc_lb << -10000, -10000, osc_gains.min_pelvis_acc;
  VectorXd pelvis_acc_ub = osc_gains.max_pelvis_acc * Vector3d::Ones();
  pelvis_tracking_data->SetCmdAccelerationBounds(pelvis_acc_lb, pelvis_acc_ub);

  auto left_foot_tracking_data = std::make_unique<TransTaskSpaceTrackingData>(
      "left_ft_traj", osc_gains.K_p_flight_foot, osc_gains.K_d_flight_foot,
      osc_gains.W_flight_foot, plant_w_spr, plant_w_spr);
  auto right_foot_tracking_data = std::make_unique<TransTaskSpaceTrackingData>(
      "right_ft_traj", osc_gains.K_p_flight_foot, osc_gains.K_d_flight_foot,
      osc_gains.W_flight_foot, plant_w_spr, plant_w_spr);
  left_foot_tracking_data->AddStateAndPointToTrack(osc_jump::FLIGHT,
                                                   "toe_left");
  right_foot_tracking_data->AddStateAndPointToTrack(osc_jump::FLIGHT,
                                                    "toe_right");

  auto left_hip_tracking_data = std::make_unique<TransTaskSpaceTrackingData>(
      "left_hip_traj", osc_gains.K_p_flight_foot, osc_gains.K_d_flight_foot,
      osc_gains.W_flight_foot, plant_w_spr, plant_w_spr);
  auto right_hip_tracking_data = std::make_unique<TransTaskSpaceTrackingData>(
      "right_hip_traj", osc_gains.K_p_flight_foot, osc_gains.K_d_flight_foot,
      osc_gains.W_flight_foot, plant_w_spr, plant_w_spr);
  left_hip_tracking_data->AddStateAndPointToTrack(osc_jump::FLIGHT, "hip_left");
  right_hip_tracking_data->AddStateAndPointToTrack(osc_jump::FLIGHT,
                                                   "hip_right");

  auto left_foot_rel_tracking_data =
      std::make_unique<RelativeTranslationTrackingData>(
          "left_ft_traj", osc_gains.K_p_flight_foot, osc_gains.K_d_flight_foot,
          osc_gains.W_flight_foot, plant_w_spr, plant_w_spr,
          left_foot_tracking_data.get(), left_hip_tracking_data.get());
  auto right_foot_rel_tracking_data =
      std::make_unique<RelativeTranslationTrackingData>(
          "right_ft_traj", osc_gains.K_p_flight_foot, osc_gains.K_d_flight_foot,
          osc_gains.W_flight_foot, plant_w_spr, plant_w_spr,
          right_foot_tracking_data.get(), right_hip_tracking_data.get());

  // Flight phase hip yaw tracking
  auto left_hip_yaw_tracking_data = std::make_unique<JointSpaceTrackingData>(
      "hip_yaw_left_traj", osc_gains.K_p_hip_yaw, osc_gains.K_d_hip_yaw,
      osc_gains.W_hip_yaw, plant_w_spr, plant_w_spr);
  auto right_hip_yaw_tracking_data = std::make_unique<JointSpaceTrackingData>(
      "hip_yaw_right_traj", osc_gains.K_p_hip_yaw, osc_gains.K_d_hip_yaw,
      osc_gains.W_hip_yaw, plant_w_spr, plant_w_spr);
  left_hip_yaw_tracking_data->AddStateAndJointToTrack(
      osc_jump::FLIGHT, "hip_yaw_left", "hip_yaw_leftdot");
  right_hip_yaw_tracking_data->AddStateAndJointToTrack(
      osc_jump::FLIGHT, "hip_yaw_right", "hip_yaw_rightdot");
  left_hip_yaw_tracking_data->SetImpactInvariantProjection(true);
  right_hip_yaw_tracking_data->SetImpactInvariantProjection(true);
  osc->AddConstTrackingData(std::move(left_hip_yaw_tracking_data),
                            VectorXd::Zero(1));
  osc->AddConstTrackingData(std::move(right_hip_yaw_tracking_data),
                            VectorXd::Zero(1));

  // Flight phase toe pitch tracking
  MatrixXd W_swing_toe = osc_gains.w_swing_toe * MatrixXd::Identity(1, 1);
  MatrixXd K_p_swing_toe = osc_gains.swing_toe_kp * MatrixXd::Identity(1, 1);
  MatrixXd K_d_swing_toe = osc_gains.swing_toe_kd * MatrixXd::Identity(1, 1);
  auto left_toe_angle_tracking_data = std::make_unique<JointSpaceTrackingData>(
      "left_toe_angle_traj", K_p_swing_toe, K_d_swing_toe, W_swing_toe,
      plant_w_spr, plant_w_spr);
  auto right_toe_angle_tracking_data = std::make_unique<JointSpaceTrackingData>(
      "right_toe_angle_traj", K_p_swing_toe, K_d_swing_toe, W_swing_toe,
      plant_w_spr, plant_w_spr);

  vector<std::pair<const Vector3d, const Frame<double>&>> left_foot_points = {
      left_heel, left_toe};
  vector<std::pair<const Vector3d, const Frame<double>&>> right_foot_points = {
      right_heel, right_toe};

  auto left_toe_angle_traj_gen =
      builder.AddSystem<cassie::osc_jump::FlightToeAngleTrajGenerator>(
          plant_w_spr, context_w_spr.get(), l_toe_trajectory,
          pos_map["toe_left"], left_foot_points, "left_toe_angle_traj");
  auto right_toe_angle_traj_gen =
      builder.AddSystem<cassie::osc_jump::FlightToeAngleTrajGenerator>(
          plant_w_spr, context_w_spr.get(), r_toe_trajectory,
          pos_map["toe_right"], right_foot_points, "right_toe_angle_traj");

  left_toe_angle_tracking_data->AddStateAndJointToTrack(
      osc_jump::FLIGHT, "toe_left", "toe_leftdot");
  right_toe_angle_tracking_data->AddStateAndJointToTrack(
      osc_jump::FLIGHT, "toe_right", "toe_rightdot");

  osc->AddTrackingData(std::move(pelvis_tracking_data));
  osc->AddTrackingData(std::move(pelvis_rot_tracking_data));
  if (osc_gains.relative_feet) {
    left_foot_rel_tracking_data->SetImpactInvariantProjection(true);
    right_foot_rel_tracking_data->SetImpactInvariantProjection(true);
    osc->AddTrackingData(std::move(left_foot_rel_tracking_data));
    osc->AddTrackingData(std::move(right_foot_rel_tracking_data));
  } else {
    left_foot_tracking_data->SetImpactInvariantProjection(true);
    right_foot_tracking_data->SetImpactInvariantProjection(true);
    osc->AddTrackingData(std::move(left_foot_tracking_data));
    osc->AddTrackingData(std::move(right_foot_tracking_data));
  }
  left_toe_angle_tracking_data->SetImpactInvariantProjection(true);
  right_toe_angle_tracking_data->SetImpactInvariantProjection(true);
  osc->AddTrackingData(std::move(left_toe_angle_tracking_data));
  osc->AddTrackingData(std::move(right_toe_angle_tracking_data));

  osc->SetOsqpSolverOptionsFromYaml(FLAGS_osqp_settings);
  // Build OSC problem
  osc->Build();
  std::cout << "Built OSC" << std::endl;

  /*****Connect ports*****/

  // OSC connections
  builder.Connect(fsm->get_fsm_output_port(), osc->get_input_port_fsm());
  builder.Connect(fsm->get_clock_output_port(), osc->get_input_port_clock());
  builder.Connect(fsm->get_impact_output_port(),
                  osc->get_input_port_impact_info());
  builder.Connect(state_receiver->get_output_port(0),
                  osc->get_input_port_robot_output());
  builder.Connect(pelvis_trans_traj_generator->get_output_port(0),
                  osc->get_input_port_tracking_data("pelvis_trans_traj"));
  builder.Connect(l_foot_traj_generator->get_output_port(0),
                  osc->get_input_port_tracking_data("left_ft_traj"));
  builder.Connect(r_foot_traj_generator->get_output_port(0),
                  osc->get_input_port_tracking_data("right_ft_traj"));
  builder.Connect(left_toe_angle_traj_gen->get_output_port(0),
                  osc->get_input_port_tracking_data("left_toe_angle_traj"));
  builder.Connect(right_toe_angle_traj_gen->get_output_port(0),
                  osc->get_input_port_tracking_data("right_toe_angle_traj"));
  builder.Connect(pelvis_rot_traj_generator->get_output_port(0),
                  osc->get_input_port_tracking_data("pelvis_rot_traj"));

  // FSM connections
  builder.Connect(contact_results_sub->get_output_port(),
                  fsm->get_contact_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  fsm->get_state_input_port());

  // Trajectory generator connections
  builder.Connect(state_receiver->get_output_port(0),
                  pelvis_trans_traj_generator->get_state_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  l_foot_traj_generator->get_state_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  r_foot_traj_generator->get_state_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  left_toe_angle_traj_gen->get_state_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  right_toe_angle_traj_gen->get_state_input_port());
  builder.Connect(fsm->get_output_port(0),
                  pelvis_trans_traj_generator->get_fsm_input_port());
  builder.Connect(fsm->get_output_port(0),
                  l_foot_traj_generator->get_fsm_input_port());
  builder.Connect(fsm->get_output_port(0),
                  r_foot_traj_generator->get_fsm_input_port());
  builder.Connect(fsm->get_output_port(0),
                  left_toe_angle_traj_gen->get_fsm_input_port());
  builder.Connect(fsm->get_output_port(0),
                  right_toe_angle_traj_gen->get_fsm_input_port());

  // Publisher connections
  builder.Connect(osc->get_output_port_osc_command(),
                  command_sender->get_input_port(0));
  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port());
  builder.Connect(osc->get_output_port_osc_debug(),
                  osc_debug_pub->get_input_port());
  builder.Connect(osc->get_output_port_failure(),
                  failure_aggregator->get_input_port(0));
  builder.Connect(failure_aggregator->get_status_output_port(),
                  controller_failure_pub->get_input_port());

  // Run lcm-driven simulation
  // Create the diagram
  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("osc_jumping_controller"));

  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, std::move(owned_diagram), state_receiver, FLAGS_channel_x, true);
  DrawAndSaveDiagramGraph(*loop.get_diagram());
  loop.Simulate();

  return 0;
}
}  // namespace examples
}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::examples::DoMain(argc, argv);
}
