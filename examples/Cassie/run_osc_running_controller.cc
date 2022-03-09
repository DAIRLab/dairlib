#include <fstream>

#include <drake/common/yaml/yaml_io.h>
#include <drake/multibody/parsing/parser.h>
#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/osc/heading_traj_generator.h"
#include "examples/Cassie/osc/high_level_command.h"
#include "examples/Cassie/osc/swing_toe_traj_generator.h"
#include "examples/Cassie/osc_jump/basic_trajectory_passthrough.h"
#include "examples/Cassie/osc_jump/toe_angle_traj_generator.h"
#include "examples/Cassie/osc_run/foot_traj_generator.h"
#include "examples/Cassie/osc_run/osc_running_gains.h"
#include "examples/Cassie/osc_run/pelvis_pitch_traj_generator.h"
#include "examples/Cassie/osc_run/pelvis_roll_traj_generator.h"
#include "examples/Cassie/osc_run/pelvis_trans_traj_generator.h"
#include "examples/impact_invariant_control/impact_aware_time_based_fsm.h"
#include "lcm/dircon_saved_trajectory.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/kinematic/fixed_joint_evaluator.h"
#include "multibody/multibody_utils.h"
#include "systems/controllers/controller_failure_aggregator.h"
#include "systems/controllers/osc/joint_space_tracking_data.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/controllers/osc/relative_translation_tracking_data.h"
#include "systems/controllers/osc/rot_space_tracking_data.h"
#include "systems/controllers/osc/trans_space_tracking_data.h"
#include "systems/filters/floating_base_velocity_filter.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"
#include "yaml-cpp/yaml.h"

#include "drake/common/yaml/yaml_read_archive.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace dairlib {

using std::map;
using std::pair;
using std::string;
using std::vector;

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using cassie::osc::SwingToeTrajGenerator;
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
using examples::osc::PelvisPitchTrajGenerator;
using examples::osc::PelvisRollTrajGenerator;
using examples::osc::PelvisTransTrajGenerator;
using examples::osc_jump::BasicTrajectoryPassthrough;
using examples::osc_run::FootTrajGenerator;
using multibody::FixedJointEvaluator;
using multibody::WorldYawViewFrame;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::RelativeTranslationTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;

namespace examples {

DEFINE_string(channel_x, "CASSIE_STATE_SIMULATION",
              "The name of the channel which receives state");
DEFINE_string(channel_u, "CASSIE_INPUT",
              "The name of the channel which publishes command");
DEFINE_string(gains_filename, "examples/Cassie/osc_run/osc_running_gains.yaml",
              "Filepath containing gains");
DEFINE_string(osqp_settings,
              "examples/Cassie/osc_run/osc_running_qp_settings.yaml",
              "Filepath containing qp settings");
DEFINE_string(
    channel_cassie_out, "CASSIE_OUTPUT_ECHO",
    "The name of the channel to receive the cassie out structure from.");
DEFINE_double(
    fsm_time_offset, 0.0,
    "Time (s) in the fsm to move from the stance phase to the flight phase");
DEFINE_double(qp_time_limit, 0.0, "Time limit (s) for the OSC QP");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Build the controller diagram
  DiagramBuilder<double> builder;

  // Built the Cassie MBPs
  drake::multibody::MultibodyPlant<double> plant(0.0);
  addCassieMultibody(&plant, nullptr, true,
                     "examples/Cassie/urdf/cassie_v2_conservative.urdf",
                     false /*spring model*/, false /*loop closure*/);
  plant.Finalize();

  auto plant_context = plant.CreateDefaultContext();

  // Get contact frames and position
  auto left_toe = LeftToeFront(plant);
  auto left_heel = LeftToeRear(plant);
  auto right_toe = RightToeFront(plant);
  auto right_heel = RightToeRear(plant);

  int nv = plant.num_velocities();

  // Create maps for joints
  map<string, int> pos_map = multibody::makeNameToPositionsMap(plant);
  map<string, int> vel_map = multibody::makeNameToVelocitiesMap(plant);
  map<string, int> act_map = multibody::makeNameToActuatorsMap(plant);

  std::unordered_map<
      int, std::vector<std::pair<const Vector3d,
                                 const drake::multibody::Frame<double>&>>>
      feet_contact_points;
  feet_contact_points[0] = std::vector<
      std::pair<const Vector3d, const drake::multibody::Frame<double>&>>(
      {left_toe, left_heel});
  feet_contact_points[1] = std::vector<
      std::pair<const Vector3d, const drake::multibody::Frame<double>&>>(
      {right_toe, right_heel});

  /**** Get trajectory from optimization ****/

  /**** OSC Gains ****/
  drake::yaml::YamlReadArchive::Options yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;

  OSCGains gains = drake::yaml::LoadYamlFile<OSCGains>(
      FindResourceOrThrow(FLAGS_gains_filename), {}, {}, yaml_options);
  OSCRunningGains osc_gains = drake::yaml::LoadYamlFile<OSCRunningGains>(
      FindResourceOrThrow(FLAGS_gains_filename));
  solvers::OSQPSettingsYaml osqp_settings =
      drake::yaml::LoadYamlFile<solvers::OSQPSettingsYaml>(
          FindResourceOrThrow(FLAGS_osqp_settings));

  /**** FSM and contact mode configuration ****/
  int left_stance_state = 0;
  int right_stance_state = 1;
  int right_touchdown_air_phase = 2;
  int left_touchdown_air_phase = 3;

  vector<int> fsm_states = {left_stance_state, right_touchdown_air_phase,
                            right_stance_state, left_touchdown_air_phase,
                            left_stance_state};

  vector<double> state_durations = {
      osc_gains.stance_duration, osc_gains.flight_duration,
      osc_gains.stance_duration, osc_gains.flight_duration, 0.0};
  vector<double> accumulated_state_durations;
  accumulated_state_durations.push_back(0);
  std::cout << accumulated_state_durations.back() << std::endl;
  for (double state_duration : state_durations) {
    accumulated_state_durations.push_back(accumulated_state_durations.back() +
                                          state_duration);
    std::cout << accumulated_state_durations.back() << std::endl;
  }
  accumulated_state_durations.pop_back();

  auto fsm = builder.AddSystem<ImpactTimeBasedFiniteStateMachine>(
      plant, fsm_states, state_durations, 0.0, gains.impact_threshold);

  /**** Initialize all the leaf systems ****/
  drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0");

  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);
  auto command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_channel_u, &lcm, TriggerTypeSet({TriggerType::kForced})));
  auto command_sender = builder.AddSystem<systems::RobotCommandSender>(plant);
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      plant, plant, plant_context.get(), plant_context.get(), true, false,
      FLAGS_qp_time_limit);
  auto osc_debug_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
          "OSC_DEBUG_RUNNING", &lcm, TriggerTypeSet({TriggerType::kForced})));
  auto failure_aggregator =
      builder.AddSystem<systems::ControllerFailureAggregator>(FLAGS_channel_u,
                                                              1);
  auto controller_failure_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_controller_failure>(
          "CONTROLLER_ERROR", &lcm, TriggerTypeSet({TriggerType::kForced})));
  std::vector<double> tau = {.001, .01, .001};
  auto ekf_filter =
      builder.AddSystem<systems::FloatingBaseVelocityFilter>(plant, tau);

  /**** OSC setup ****/
  // Cost
  /// REGULARIZATION COSTS
  osc->SetAccelerationCostWeights(gains.w_accel * gains.W_acceleration);
  //  osc->SetInputSmoothingWeights(1e-3 * gains.W_input_regularization);
  osc->SetInputCostWeights(gains.w_input * gains.W_input_regularization);
  //  osc->SetLambdaContactRegularizationWeight(1e-4 *
  //  gains.W_lambda_c_regularization);
  osc->SetLambdaHolonomicRegularizationWeight(1e-5 *
                                              gains.W_lambda_h_regularization);

  // Soft constraint on contacts
  osc->SetSoftConstraintWeight(gains.w_soft_constraint);

  // Contact information for OSC
  osc->SetContactFriction(gains.mu);

  const auto& pelvis = plant.GetBodyByName("pelvis");
  multibody::WorldYawViewFrame view_frame(pelvis);
  auto left_toe_evaluator = multibody::WorldPointEvaluator(
      plant, left_toe.first, left_toe.second, view_frame, Matrix3d::Identity(),
      Vector3d::Zero(), {1, 2});
  auto left_heel_evaluator = multibody::WorldPointEvaluator(
      plant, left_heel.first, left_heel.second, view_frame,
      Matrix3d::Identity(), Vector3d::Zero(), {0, 1, 2});
  auto right_toe_evaluator = multibody::WorldPointEvaluator(
      plant, right_toe.first, right_toe.second, view_frame,
      Matrix3d::Identity(), Vector3d::Zero(), {1, 2});
  auto right_heel_evaluator = multibody::WorldPointEvaluator(
      plant, right_heel.first, right_heel.second, view_frame,
      Matrix3d::Identity(), Vector3d::Zero(), {0, 1, 2});

  osc->AddStateAndContactPoint(left_stance_state, &left_toe_evaluator);
  osc->AddStateAndContactPoint(left_stance_state, &left_heel_evaluator);
  osc->AddStateAndContactPoint(right_stance_state, &right_toe_evaluator);
  osc->AddStateAndContactPoint(right_stance_state, &right_heel_evaluator);

  multibody::KinematicEvaluatorSet<double> evaluators(plant);
  auto left_loop = LeftLoopClosureEvaluator(plant);
  auto right_loop = RightLoopClosureEvaluator(plant);
  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);

  // Fix the springs in the dynamics
  auto pos_idx_map = multibody::makeNameToPositionsMap(plant);
  auto vel_idx_map = multibody::makeNameToVelocitiesMap(plant);
  auto left_fixed_knee_spring =
      FixedJointEvaluator(plant, pos_idx_map.at("knee_joint_left"),
                          vel_idx_map.at("knee_joint_leftdot"), 0);
  auto right_fixed_knee_spring =
      FixedJointEvaluator(plant, pos_idx_map.at("knee_joint_right"),
                          vel_idx_map.at("knee_joint_rightdot"), 0);
  auto left_fixed_ankle_spring =
      FixedJointEvaluator(plant, pos_idx_map.at("ankle_spring_joint_left"),
                          vel_idx_map.at("ankle_spring_joint_leftdot"), 0);
  auto right_fixed_ankle_spring =
      FixedJointEvaluator(plant, pos_idx_map.at("ankle_spring_joint_right"),
                          vel_idx_map.at("ankle_spring_joint_rightdot"), 0);
  evaluators.add_evaluator(&left_fixed_knee_spring);
  evaluators.add_evaluator(&right_fixed_knee_spring);
  evaluators.add_evaluator(&left_fixed_ankle_spring);
  evaluators.add_evaluator(&right_fixed_ankle_spring);

  osc->AddKinematicConstraint(&evaluators);

  /**** Tracking Data *****/

  std::cout << "Creating tracking data. " << std::endl;

  auto cassie_out_receiver =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_cassie_out>(
          FLAGS_channel_cassie_out, &lcm));
  cassie::osc::HighLevelCommand* high_level_command;
  high_level_command = builder.AddSystem<cassie::osc::HighLevelCommand>(
      plant, plant_context.get(), osc_gains.vel_scale_rot,
      osc_gains.vel_scale_trans_sagital, osc_gains.vel_scale_trans_lateral);
  builder.Connect(cassie_out_receiver->get_output_port(),
                  high_level_command->get_cassie_out_input_port());

  auto default_traj = PiecewisePolynomial<double>(Vector3d{0, 0, 0});
  auto pelvis_trans_traj_generator =
      builder.AddSystem<PelvisTransTrajGenerator>(
          plant, plant_context.get(), default_traj, feet_contact_points,
          osc_gains.relative_pelvis);
  pelvis_trans_traj_generator->SetSLIPParams(osc_gains.rest_length);
  auto l_foot_traj_generator = builder.AddSystem<FootTrajGenerator>(
      plant, plant_context.get(), "toe_left", "pelvis", osc_gains.relative_feet,
      0, accumulated_state_durations);
  auto r_foot_traj_generator = builder.AddSystem<FootTrajGenerator>(
      plant, plant_context.get(), "toe_right", "pelvis",
      osc_gains.relative_feet, 1, accumulated_state_durations);
  l_foot_traj_generator->SetFootstepGains(osc_gains.K_d_footstep);
  r_foot_traj_generator->SetFootstepGains(osc_gains.K_d_footstep);
  l_foot_traj_generator->SetFootPlacementOffsets(osc_gains.rest_length,
                                                 osc_gains.center_line_offset,
                                                 osc_gains.footstep_offset);
  r_foot_traj_generator->SetFootPlacementOffsets(osc_gains.rest_length,
                                                 osc_gains.center_line_offset,
                                                 osc_gains.footstep_offset);

  TransTaskSpaceTrackingData pelvis_tracking_data(
      "pelvis_trans_traj", osc_gains.K_p_pelvis, osc_gains.K_d_pelvis,
      osc_gains.W_pelvis, plant, plant);
  TransTaskSpaceTrackingData stance_foot_tracking_data(
      "stance_ft_traj", osc_gains.K_p_swing_foot, osc_gains.K_d_swing_foot,
      osc_gains.W_swing_foot, plant, plant);
  TransTaskSpaceTrackingData left_foot_tracking_data(
      "left_ft_traj", osc_gains.K_p_swing_foot, osc_gains.K_d_swing_foot,
      osc_gains.W_swing_foot, plant, plant);
  TransTaskSpaceTrackingData right_foot_tracking_data(
      "right_ft_traj", osc_gains.K_p_swing_foot, osc_gains.K_d_swing_foot,
      osc_gains.W_swing_foot, plant, plant);
  pelvis_tracking_data.AddStateAndPointToTrack(left_stance_state, "pelvis");
  pelvis_tracking_data.AddStateAndPointToTrack(right_stance_state, "pelvis");
  stance_foot_tracking_data.AddStateAndPointToTrack(left_stance_state,
                                                    "toe_left");
  stance_foot_tracking_data.AddStateAndPointToTrack(right_stance_state,
                                                    "toe_right");
  left_foot_tracking_data.AddStateAndPointToTrack(right_stance_state,
                                                  "toe_left");
  right_foot_tracking_data.AddStateAndPointToTrack(left_stance_state,
                                                   "toe_right");
  left_foot_tracking_data.AddStateAndPointToTrack(left_touchdown_air_phase,
                                                  "toe_left");
  right_foot_tracking_data.AddStateAndPointToTrack(right_touchdown_air_phase,
                                                   "toe_right");

  TransTaskSpaceTrackingData left_foot_yz_tracking_data(
      "left_ft_traj", osc_gains.K_p_swing_foot, osc_gains.K_d_swing_foot,
      osc_gains.W_swing_foot, plant, plant);
  TransTaskSpaceTrackingData right_foot_yz_tracking_data(
      "right_ft_traj", osc_gains.K_p_swing_foot, osc_gains.K_d_swing_foot,
      osc_gains.W_swing_foot, plant, plant);
  left_foot_yz_tracking_data.AddStateAndPointToTrack(right_touchdown_air_phase,
                                                     "toe_left");
  right_foot_yz_tracking_data.AddStateAndPointToTrack(left_touchdown_air_phase,
                                                      "toe_right");

  TransTaskSpaceTrackingData left_hip_tracking_data(
      "left_hip_traj", osc_gains.K_p_swing_foot, osc_gains.K_d_swing_foot,
      osc_gains.W_swing_foot, plant, plant);
  TransTaskSpaceTrackingData right_hip_tracking_data(
      "right_hip_traj", osc_gains.K_p_swing_foot, osc_gains.K_d_swing_foot,
      osc_gains.W_swing_foot, plant, plant);
  left_hip_tracking_data.AddStateAndPointToTrack(right_stance_state, "pelvis");
  right_hip_tracking_data.AddStateAndPointToTrack(left_stance_state, "pelvis");
  right_hip_tracking_data.AddStateAndPointToTrack(right_touchdown_air_phase,
                                                  "pelvis");
  left_hip_tracking_data.AddStateAndPointToTrack(left_touchdown_air_phase,
                                                 "pelvis");

  TransTaskSpaceTrackingData left_hip_yz_tracking_data(
      "left_hip_traj", osc_gains.K_p_swing_foot, osc_gains.K_d_swing_foot,
      osc_gains.W_swing_foot, plant, plant);
  TransTaskSpaceTrackingData right_hip_yz_tracking_data(
      "right_hip_traj", osc_gains.K_p_swing_foot, osc_gains.K_d_swing_foot,
      osc_gains.W_swing_foot, plant, plant);
  left_hip_yz_tracking_data.AddStateAndPointToTrack(right_touchdown_air_phase,
                                                    "hip_left");
  right_hip_yz_tracking_data.AddStateAndPointToTrack(left_touchdown_air_phase,
                                                     "hip_right");

  RelativeTranslationTrackingData left_foot_rel_tracking_data(
      "left_ft_traj", osc_gains.K_p_swing_foot, osc_gains.K_d_swing_foot,
      osc_gains.W_swing_foot, plant, plant, &left_foot_tracking_data,
      &left_hip_tracking_data);
  RelativeTranslationTrackingData right_foot_rel_tracking_data(
      "right_ft_traj", osc_gains.K_p_swing_foot, osc_gains.K_d_swing_foot,
      osc_gains.W_swing_foot, plant, plant, &right_foot_tracking_data,
      &right_hip_tracking_data);
  RelativeTranslationTrackingData left_foot_yz_rel_tracking_data(
      "left_ft_z_traj", osc_gains.K_p_liftoff_swing_foot,
      osc_gains.K_d_liftoff_swing_foot, osc_gains.W_liftoff_swing_foot, plant,
      plant, &left_foot_yz_tracking_data, &left_hip_yz_tracking_data);
  RelativeTranslationTrackingData right_foot_yz_rel_tracking_data(
      "right_ft_z_traj", osc_gains.K_p_liftoff_swing_foot,
      osc_gains.K_d_liftoff_swing_foot, osc_gains.W_liftoff_swing_foot, plant,
      plant, &right_foot_yz_tracking_data, &right_hip_yz_tracking_data);
  RelativeTranslationTrackingData pelvis_trans_rel_tracking_data(
      "pelvis_trans_traj", osc_gains.K_p_pelvis, osc_gains.K_d_pelvis,
      osc_gains.W_pelvis, plant, plant, &pelvis_tracking_data,
      &stance_foot_tracking_data);
  left_foot_rel_tracking_data.SetViewFrame(view_frame);
  right_foot_rel_tracking_data.SetViewFrame(view_frame);
  left_foot_yz_rel_tracking_data.SetViewFrame(view_frame);
  right_foot_yz_rel_tracking_data.SetViewFrame(view_frame);
  pelvis_trans_rel_tracking_data.SetViewFrame(view_frame);

  left_foot_rel_tracking_data.SetImpactInvariantProjection(true);
  right_foot_rel_tracking_data.SetImpactInvariantProjection(true);
  //  left_foot_yz_rel_tracking_data.SetImpactInvariantProjection(true);
  //  right_foot_yz_rel_tracking_data.SetImpactInvariantProjection(true);
  pelvis_trans_rel_tracking_data.SetImpactInvariantProjection(true);
  //  left_foot_yz_rel_tracking_data.DisableFeedforwardAccel({0, 1, 2});
  //  right_foot_yz_rel_tracking_data.DisableFeedforwardAccel({0, 1, 2});

  osc->AddTrackingData(&pelvis_trans_rel_tracking_data);
  osc->AddTrackingData(&left_foot_rel_tracking_data);
  osc->AddTrackingData(&right_foot_rel_tracking_data);
  osc->AddTrackingData(&left_foot_yz_rel_tracking_data);
  osc->AddTrackingData(&right_foot_yz_rel_tracking_data);

  auto heading_traj_generator =
      builder.AddSystem<cassie::osc::HeadingTrajGenerator>(plant,
                                                           plant_context.get());

  RotTaskSpaceTrackingData pelvis_rot_tracking_data(
      "pelvis_rot_traj", osc_gains.K_p_pelvis_rot, osc_gains.K_d_pelvis_rot,
      osc_gains.W_pelvis_rot, plant, plant);
  pelvis_rot_tracking_data.AddStateAndFrameToTrack(left_stance_state, "pelvis");
  pelvis_rot_tracking_data.AddStateAndFrameToTrack(right_stance_state,
                                                   "pelvis");
  pelvis_rot_tracking_data.AddStateAndFrameToTrack(left_touchdown_air_phase,
                                                   "pelvis");
  pelvis_rot_tracking_data.AddStateAndFrameToTrack(right_touchdown_air_phase,
                                                   "pelvis");
  pelvis_rot_tracking_data.SetImpactInvariantProjection(true);
  osc->AddTrackingData(&pelvis_rot_tracking_data);

  // Swing toe joint trajectory
  vector<std::pair<const Vector3d, const Frame<double>&>> left_foot_points = {
      left_heel, left_toe};
  vector<std::pair<const Vector3d, const Frame<double>&>> right_foot_points = {
      right_heel, right_toe};
  auto left_toe_angle_traj_gen = builder.AddSystem<SwingToeTrajGenerator>(
      plant, plant_context.get(), pos_map["toe_left"], left_foot_points,
      "left_toe_angle_traj");
  auto right_toe_angle_traj_gen = builder.AddSystem<SwingToeTrajGenerator>(
      plant, plant_context.get(), pos_map["toe_right"], right_foot_points,
      "right_toe_angle_traj");

  // Swing toe joint tracking
  JointSpaceTrackingData left_toe_angle_tracking_data(
      "left_toe_angle_traj", osc_gains.K_p_swing_toe, osc_gains.K_d_swing_toe,
      osc_gains.W_swing_toe, plant, plant);
  JointSpaceTrackingData right_toe_angle_tracking_data(
      "right_toe_angle_traj", osc_gains.K_p_swing_toe, osc_gains.K_d_swing_toe,
      osc_gains.W_swing_toe, plant, plant);
  left_toe_angle_tracking_data.AddStateAndJointToTrack(
      right_stance_state, "toe_left", "toe_leftdot");
  left_toe_angle_tracking_data.AddStateAndJointToTrack(
      right_touchdown_air_phase, "toe_left", "toe_leftdot");
  left_toe_angle_tracking_data.AddStateAndJointToTrack(
      left_touchdown_air_phase, "toe_left", "toe_leftdot");
  right_toe_angle_tracking_data.AddStateAndJointToTrack(
      left_stance_state, "toe_right", "toe_rightdot");
  right_toe_angle_tracking_data.AddStateAndJointToTrack(
      right_touchdown_air_phase, "toe_right", "toe_rightdot");
  right_toe_angle_tracking_data.AddStateAndJointToTrack(
      left_touchdown_air_phase, "toe_right", "toe_rightdot");
  left_toe_angle_tracking_data.SetImpactInvariantProjection(true);
  right_toe_angle_tracking_data.SetImpactInvariantProjection(true);
  osc->AddTrackingData(&left_toe_angle_tracking_data);
  osc->AddTrackingData(&right_toe_angle_tracking_data);

  // Swing hip yaw joint tracking
  JointSpaceTrackingData left_hip_yaw_tracking_data(
      "hip_yaw_left_traj", osc_gains.K_p_hip_yaw, osc_gains.K_d_hip_yaw,
      osc_gains.W_hip_yaw, plant, plant);
  JointSpaceTrackingData right_hip_yaw_tracking_data(
      "hip_yaw_right_traj", osc_gains.K_p_hip_yaw, osc_gains.K_d_hip_yaw,
      osc_gains.W_hip_yaw, plant, plant);
  left_hip_yaw_tracking_data.AddJointToTrack("hip_yaw_left", "hip_yaw_leftdot");
  right_hip_yaw_tracking_data.AddJointToTrack("hip_yaw_right",
                                              "hip_yaw_rightdot");
  //  left_hip_yaw_tracking_data.SetImpactInvariantProjection(true);
  //  right_hip_yaw_tracking_data.SetImpactInvariantProjection(true);
  osc->AddConstTrackingData(&left_hip_yaw_tracking_data, VectorXd::Zero(1));
  osc->AddConstTrackingData(&right_hip_yaw_tracking_data, VectorXd::Zero(1));

  // Build OSC problem
  osc->Build(osqp_settings);
  std::cout << "Built OSC" << std::endl;

  /*****Connect ports*****/

  // OSC connections
  builder.Connect(fsm->get_output_port_fsm(), osc->get_fsm_input_port());
  builder.Connect(fsm->get_output_port_impact(),
                  osc->get_near_impact_input_port());
  builder.Connect(fsm->get_output_port_clock(), osc->get_clock_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  ekf_filter->get_input_port());
  builder.Connect(ekf_filter->get_output_port(),
                  osc->get_robot_output_input_port());
  // FSM connections
  builder.Connect(state_receiver->get_output_port(0),
                  fsm->get_input_port_state());

  // Trajectory generator connections
  builder.Connect(state_receiver->get_output_port(0),
                  pelvis_trans_traj_generator->get_state_input_port());
  builder.Connect(fsm->get_output_port_fsm(),
                  pelvis_trans_traj_generator->get_fsm_input_port());
  builder.Connect(fsm->get_output_port_clock(),
                  pelvis_trans_traj_generator->get_clock_input_port());
  builder.Connect(high_level_command->get_xy_output_port(),
                  l_foot_traj_generator->get_target_vel_input_port());
  builder.Connect(high_level_command->get_xy_output_port(),
                  r_foot_traj_generator->get_target_vel_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  l_foot_traj_generator->get_state_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  r_foot_traj_generator->get_state_input_port());
  builder.Connect(fsm->get_output_port_fsm(),
                  l_foot_traj_generator->get_fsm_input_port());
  builder.Connect(fsm->get_output_port_fsm(),
                  r_foot_traj_generator->get_fsm_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  left_toe_angle_traj_gen->get_state_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  right_toe_angle_traj_gen->get_state_input_port());
  // OSC connections
  builder.Connect(pelvis_trans_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("pelvis_trans_traj"));
  builder.Connect(state_receiver->get_output_port(0),
                  heading_traj_generator->get_state_input_port());
  builder.Connect(high_level_command->get_yaw_output_port(),
                  heading_traj_generator->get_yaw_input_port());
  builder.Connect(heading_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("pelvis_rot_traj"));
  builder.Connect(l_foot_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("left_ft_traj"));
  builder.Connect(r_foot_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("right_ft_traj"));
  builder.Connect(l_foot_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("left_ft_z_traj"));
  builder.Connect(r_foot_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("right_ft_z_traj"));
  builder.Connect(left_toe_angle_traj_gen->get_output_port(0),
                  osc->get_tracking_data_input_port("left_toe_angle_traj"));
  builder.Connect(right_toe_angle_traj_gen->get_output_port(0),
                  osc->get_tracking_data_input_port("right_toe_angle_traj"));

  // Publisher connections
  builder.Connect(osc->get_osc_output_port(),
                  command_sender->get_input_port(0));
  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port());
  builder.Connect(osc->get_osc_debug_port(), osc_debug_pub->get_input_port());
  builder.Connect(osc->get_failure_output_port(),
                  failure_aggregator->get_input_port(0));
  builder.Connect(failure_aggregator->get_status_output_port(),
                  controller_failure_pub->get_input_port());

  // Run lcm-driven simulation
  // Create the diagram
  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("osc_running_controller"));

  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, std::move(owned_diagram), state_receiver, FLAGS_channel_x, true);
  DrawAndSaveDiagramGraph(*loop.get_diagram());
  loop.Simulate();

  return 0;
}  // namespace examples
}  // namespace examples
}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::examples::DoMain(argc, argv);
}
