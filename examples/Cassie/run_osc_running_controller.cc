#include <fstream>

#include <drake/common/yaml/yaml_io.h>
#include <drake/multibody/parsing/parser.h>
#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/contact_scheduler/contact_scheduler.h"
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
#include "systems/controllers/cassie_out_to_radio.h"
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

#include "drake/common/yaml/yaml_io.h"
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

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Build the controller diagram
  DiagramBuilder<double> builder;

  // Built the Cassie MBPs
  drake::multibody::MultibodyPlant<double> plant(0.0);
  AddCassieMultibody(&plant, nullptr, true,
                     "examples/Cassie/urdf/cassie_v2_conservative.urdf",
                     true /*spring model*/, false /*loop closure*/);
  plant.Finalize();

  auto plant_context = plant.CreateDefaultContext();

  // Get contact frames and position
  auto left_toe = LeftToeFront(plant);
  auto left_heel = LeftToeRear(plant);
  auto right_toe = RightToeFront(plant);
  auto right_heel = RightToeRear(plant);

  int nv = plant.num_velocities();

  // Create maps for joints
  map<string, int> pos_map = multibody::MakeNameToPositionsMap(plant);
  map<string, int> vel_map = multibody::MakeNameToVelocitiesMap(plant);
  map<string, int> act_map = multibody::MakeNameToActuatorsMap(plant);

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
  drake::yaml::LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;

  OSCGains gains = drake::yaml::LoadYamlFile<OSCGains>(
      FindResourceOrThrow(FLAGS_gains_filename), {}, {}, yaml_options);
  OSCRunningGains osc_gains = drake::yaml::LoadYamlFile<OSCRunningGains>(
      FindResourceOrThrow(FLAGS_gains_filename));
  //  solvers::OSQPSettingsYaml osqp_settings =
  //      drake::yaml::LoadYamlFile<solvers::OSQPSettingsYaml>(
  //          FindResourceOrThrow(FLAGS_osqp_settings));

  /**** FSM and contact mode configuration ****/

  vector<int> fsm_states = {
      RUNNING_FSM_STATE::LEFT_STANCE, RUNNING_FSM_STATE::LEFT_FLIGHT,
      RUNNING_FSM_STATE::RIGHT_STANCE, RUNNING_FSM_STATE::RIGHT_FLIGHT,
      RUNNING_FSM_STATE::LEFT_STANCE};

  vector<double> state_durations = {
      osc_gains.stance_duration, osc_gains.flight_duration,
      osc_gains.stance_duration, osc_gains.flight_duration, 0.0};
  vector<double> accumulated_state_durations;
  accumulated_state_durations.push_back(0);
  for (double state_duration : state_durations) {
    accumulated_state_durations.push_back(accumulated_state_durations.back() +
                                          state_duration);
  }
  accumulated_state_durations.pop_back();

  std::set<RUNNING_FSM_STATE> impact_states = {LEFT_STANCE, RIGHT_STANCE};
  auto contact_scheduler = builder.AddSystem<ContactScheduler>(
      plant, plant_context.get(), impact_states, gains.impact_threshold,
      gains.impact_tau);
  contact_scheduler->SetSLIPParams(osc_gains.rest_length);
  //  auto fsm = builder.AddSystem<ImpactTimeBasedFiniteStateMachine>(
  //      plant, fsm_states, state_durations, 0.0, gains.impact_threshold,
  //      gains.impact_tau);

  /**** Initialize all the leaf systems ****/
  drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0");

  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);
  auto command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_channel_u, &lcm, TriggerTypeSet({TriggerType::kForced})));
  auto command_sender = builder.AddSystem<systems::RobotCommandSender>(plant);
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      plant, plant, plant_context.get(), plant_context.get(), true);
  auto osc_debug_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
          "OSC_DEBUG_RUNNING", &lcm, TriggerTypeSet({TriggerType::kForced})));
  auto failure_aggregator =
      builder.AddSystem<systems::ControllerFailureAggregator>(FLAGS_channel_u,
                                                              1);
  auto cassie_out_to_radio = builder.AddSystem<systems::CassieOutToRadio>();
  auto controller_failure_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_controller_failure>(
          "CONTROLLER_ERROR", &lcm, TriggerTypeSet({TriggerType::kForced})));
  auto contact_scheduler_debug_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_contact_timing>(
          "CONTACT_TIMING", &lcm, TriggerTypeSet({TriggerType::kForced})));
  //  std::vector<double> tau = {.05, .01, .01};
  auto ekf_filter = builder.AddSystem<systems::FloatingBaseVelocityFilter>(
      plant, osc_gains.ekf_filter_tau);

  /**** OSC setup ****/
  // Cost
  /// REGULARIZATION COSTS
  osc->SetAccelerationCostWeights(osc_gains.w_accel * osc_gains.W_acceleration);
  osc->SetInputSmoothingWeights(osc_gains.w_input_reg * osc_gains.W_input_regularization);
  osc->SetInputCostWeights(osc_gains.w_input * osc_gains.W_input_regularization);
  osc->SetLambdaContactRegularizationWeight(osc_gains.w_lambda * osc_gains.W_lambda_c_regularization);
  osc->SetLambdaHolonomicRegularizationWeight(osc_gains.w_lambda * osc_gains.W_lambda_h_regularization);
  // Soft constraint on contacts
  osc->SetSoftConstraintWeight(osc_gains.w_soft_constraint);

  // Contact information for OSC
  osc->SetContactFriction(osc_gains.mu);

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

  osc->AddStateAndContactPoint(RUNNING_FSM_STATE::LEFT_STANCE,
                               &left_toe_evaluator);
  osc->AddStateAndContactPoint(RUNNING_FSM_STATE::LEFT_STANCE,
                               &left_heel_evaluator);
  osc->AddStateAndContactPoint(RUNNING_FSM_STATE::RIGHT_STANCE,
                               &right_toe_evaluator);
  osc->AddStateAndContactPoint(RUNNING_FSM_STATE::RIGHT_STANCE,
                               &right_heel_evaluator);

  multibody::KinematicEvaluatorSet<double> evaluators(plant);
  auto left_loop = LeftLoopClosureEvaluator(plant);
  auto right_loop = RightLoopClosureEvaluator(plant);
  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);

  // Fix the springs in the dynamics
  auto pos_idx_map = multibody::MakeNameToPositionsMap(plant);
  auto vel_idx_map = multibody::MakeNameToVelocitiesMap(plant);
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
  //  osc->AddStateAndContactPoint(RUNNING_FSM_STATE::LEFT_STANCE,
  //  &left_fixed_knee_spring);
  //  osc->AddStateAndContactPoint(RUNNING_FSM_STATE::RIGHT_STANCE,
  //  &right_fixed_knee_spring);
  //  osc->AddStateAndContactPoint(RUNNING_FSM_STATE::LEFT_STANCE,
  //  &left_fixed_ankle_spring);
  //  osc->AddStateAndContactPoint(RUNNING_FSM_STATE::RIGHT_STANCE,
  //  &right_fixed_ankle_spring);

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

  auto default_traj = PiecewisePolynomial<double>(Vector3d{0, 0, 0});
  auto pelvis_trans_traj_generator =
      builder.AddSystem<PelvisTransTrajGenerator>(
          plant, plant_context.get(), default_traj, feet_contact_points,
          osc_gains.relative_pelvis);
  pelvis_trans_traj_generator->SetSLIPParams(osc_gains.rest_length);

  auto l_foot_traj_generator = builder.AddSystem<FootTrajGenerator>(
      plant, plant_context.get(), "toe_left", "pelvis", osc_gains.relative_feet,
      LEFT_STANCE);
  auto r_foot_traj_generator = builder.AddSystem<FootTrajGenerator>(
      plant, plant_context.get(), "toe_right", "pelvis",
      osc_gains.relative_feet, RIGHT_STANCE);
  l_foot_traj_generator->SetFootstepGains(osc_gains.K_d_footstep);
  r_foot_traj_generator->SetFootstepGains(osc_gains.K_d_footstep);
  l_foot_traj_generator->SetFootPlacementOffsets(
      osc_gains.rest_length, osc_gains.footstep_lateral_offset,
      osc_gains.footstep_sagital_offset, osc_gains.mid_foot_height);
  r_foot_traj_generator->SetFootPlacementOffsets(
      osc_gains.rest_length, osc_gains.footstep_lateral_offset,
      osc_gains.footstep_sagital_offset, osc_gains.mid_foot_height);

  auto pelvis_tracking_data = std::make_unique<TransTaskSpaceTrackingData>(
      "pelvis_trans_traj", osc_gains.K_p_pelvis, osc_gains.K_d_pelvis,
      osc_gains.W_pelvis, plant, plant);
  auto stance_foot_tracking_data = std::make_unique<TransTaskSpaceTrackingData>(
      "stance_ft_traj", osc_gains.K_p_swing_foot, osc_gains.K_d_swing_foot,
      osc_gains.W_swing_foot, plant, plant);
  auto left_foot_tracking_data = std::make_unique<TransTaskSpaceTrackingData>(
      "left_ft_traj", osc_gains.K_p_swing_foot, osc_gains.K_d_swing_foot,
      osc_gains.W_swing_foot, plant, plant);
  auto right_foot_tracking_data = std::make_unique<TransTaskSpaceTrackingData>(
      "right_ft_traj", osc_gains.K_p_swing_foot, osc_gains.K_d_swing_foot,
      osc_gains.W_swing_foot, plant, plant);
  pelvis_tracking_data->AddStateAndPointToTrack(RUNNING_FSM_STATE::LEFT_STANCE,
                                                "pelvis");
  pelvis_tracking_data->AddStateAndPointToTrack(RUNNING_FSM_STATE::RIGHT_STANCE,
                                                "pelvis");
  stance_foot_tracking_data->AddStateAndPointToTrack(
      RUNNING_FSM_STATE::LEFT_STANCE, "toe_left");
  stance_foot_tracking_data->AddStateAndPointToTrack(
      RUNNING_FSM_STATE::RIGHT_STANCE, "toe_right");
  left_foot_tracking_data->AddStateAndPointToTrack(
      RUNNING_FSM_STATE::RIGHT_STANCE, "toe_left");
  right_foot_tracking_data->AddStateAndPointToTrack(
      RUNNING_FSM_STATE::LEFT_STANCE, "toe_right");
  left_foot_tracking_data->AddStateAndPointToTrack(
      RUNNING_FSM_STATE::RIGHT_FLIGHT, "toe_left");
  right_foot_tracking_data->AddStateAndPointToTrack(
      RUNNING_FSM_STATE::LEFT_FLIGHT, "toe_right");

  auto left_foot_yz_tracking_data =
      std::make_unique<TransTaskSpaceTrackingData>(
          "left_ft_traj", osc_gains.K_p_swing_foot, osc_gains.K_d_swing_foot,
          osc_gains.W_swing_foot, plant, plant);
  auto right_foot_yz_tracking_data =
      std::make_unique<TransTaskSpaceTrackingData>(
          "right_ft_traj", osc_gains.K_p_swing_foot, osc_gains.K_d_swing_foot,
          osc_gains.W_swing_foot, plant, plant);
  left_foot_yz_tracking_data->AddStateAndPointToTrack(
      RUNNING_FSM_STATE::LEFT_FLIGHT, "toe_left");
  right_foot_yz_tracking_data->AddStateAndPointToTrack(
      RUNNING_FSM_STATE::RIGHT_FLIGHT, "toe_right");

  auto left_hip_tracking_data = std::make_unique<TransTaskSpaceTrackingData>(
      "left_hip_traj", osc_gains.K_p_swing_foot, osc_gains.K_d_swing_foot,
      osc_gains.W_swing_foot, plant, plant);
  auto right_hip_tracking_data = std::make_unique<TransTaskSpaceTrackingData>(
      "right_hip_traj", osc_gains.K_p_swing_foot, osc_gains.K_d_swing_foot,
      osc_gains.W_swing_foot, plant, plant);
  left_hip_tracking_data->AddStateAndPointToTrack(
      RUNNING_FSM_STATE::RIGHT_STANCE, "pelvis");
  right_hip_tracking_data->AddStateAndPointToTrack(
      RUNNING_FSM_STATE::LEFT_STANCE, "pelvis");
  right_hip_tracking_data->AddStateAndPointToTrack(
      RUNNING_FSM_STATE::LEFT_FLIGHT, "pelvis");
  left_hip_tracking_data->AddStateAndPointToTrack(
      RUNNING_FSM_STATE::RIGHT_FLIGHT, "pelvis");

  auto left_hip_yz_tracking_data = std::make_unique<TransTaskSpaceTrackingData>(
      "left_hip_traj", osc_gains.K_p_swing_foot, osc_gains.K_d_swing_foot,
      osc_gains.W_swing_foot, plant, plant);
  auto right_hip_yz_tracking_data =
      std::make_unique<TransTaskSpaceTrackingData>(
          "right_hip_traj", osc_gains.K_p_swing_foot, osc_gains.K_d_swing_foot,
          osc_gains.W_swing_foot, plant, plant);
  left_hip_yz_tracking_data->AddStateAndPointToTrack(
      RUNNING_FSM_STATE::LEFT_FLIGHT, "pelvis");
  right_hip_yz_tracking_data->AddStateAndPointToTrack(
      RUNNING_FSM_STATE::RIGHT_FLIGHT, "pelvis");

  auto left_foot_rel_tracking_data =
      std::make_unique<RelativeTranslationTrackingData>(
          "left_ft_traj", osc_gains.K_p_swing_foot, osc_gains.K_d_swing_foot,
          osc_gains.W_swing_foot, plant, plant, left_foot_tracking_data.get(),
          left_hip_tracking_data.get());
  auto right_foot_rel_tracking_data =
      std::make_unique<RelativeTranslationTrackingData>(
          "right_ft_traj", osc_gains.K_p_swing_foot, osc_gains.K_d_swing_foot,
          osc_gains.W_swing_foot, plant, plant, right_foot_tracking_data.get(),
          right_hip_tracking_data.get());
  auto left_foot_yz_rel_tracking_data =
      std::make_unique<RelativeTranslationTrackingData>(
          "left_ft_z_traj", osc_gains.K_p_liftoff_swing_foot,
          osc_gains.K_d_liftoff_swing_foot, osc_gains.W_liftoff_swing_foot,
          plant, plant, left_foot_yz_tracking_data.get(),
          left_hip_yz_tracking_data.get());
  auto right_foot_yz_rel_tracking_data =
      std::make_unique<RelativeTranslationTrackingData>(
          "right_ft_z_traj", osc_gains.K_p_liftoff_swing_foot,
          osc_gains.K_d_liftoff_swing_foot, osc_gains.W_liftoff_swing_foot,
          plant, plant, right_foot_yz_tracking_data.get(),
          right_hip_yz_tracking_data.get());
  auto pelvis_trans_rel_tracking_data =
      std::make_unique<RelativeTranslationTrackingData>(
          "pelvis_trans_traj", osc_gains.K_p_pelvis, osc_gains.K_d_pelvis,
          osc_gains.W_pelvis, plant, plant, pelvis_tracking_data.get(),
          stance_foot_tracking_data.get());
  left_foot_rel_tracking_data->SetViewFrame(view_frame);
  right_foot_rel_tracking_data->SetViewFrame(view_frame);
  left_foot_yz_rel_tracking_data->SetViewFrame(view_frame);
  right_foot_yz_rel_tracking_data->SetViewFrame(view_frame);
  pelvis_trans_rel_tracking_data->SetViewFrame(view_frame);

  //  pelvis_trans_rel_tracking_data->AddJointAndStateToIgnoreInJacobian(vel_map["hip_roll_leftdot"],
  //  RUNNING_FSM_STATE::LEFT_STANCE);
  //  pelvis_trans_rel_tracking_data->AddJointAndStateToIgnoreInJacobian(vel_map["hip_roll_rightdot"],
  //  RUNNING_FSM_STATE::RIGHT_STANCE);
  //  pelvis_trans_rel_tracking_data->AddJointAndStateToIgnoreInJacobian(vel_map["hip_pitch_leftdot"],
  //  RUNNING_FSM_STATE::LEFT_STANCE);
  //  pelvis_trans_rel_tracking_data->AddJointAndStateToIgnoreInJacobian(vel_map["hip_pitch_rightdot"],
  //  RUNNING_FSM_STATE::RIGHT_STANCE);
  //  pelvis_trans_rel_tracking_data->AddJointAndStateToIgnoreInJacobian(vel_map["knee_joint_leftdot"],
  //  RUNNING_FSM_STATE::LEFT_STANCE);
  //  pelvis_trans_rel_tracking_data->AddJointAndStateToIgnoreInJacobian(vel_map["knee_joint_rightdot"],
  //  RUNNING_FSM_STATE::RIGHT_STANCE);

  //  left_foot_rel_tracking_data->DisableFeedforwardAccel({2});
  //  right_foot_rel_tracking_data->DisableFeedforwardAccel({2});
  pelvis_trans_rel_tracking_data->DisableFeedforwardAccel({2});
  //  left_foot_yz_rel_tracking_data->DisableFeedforwardAccel({2});
  //  right_foot_yz_rel_tracking_data->DisableFeedforwardAccel({2});
  //  left_foot_yz_rel_tracking_data->DisableFeedforwardAccel({0, 1, 2});
  //  right_foot_yz_rel_tracking_data->DisableFeedforwardAccel({0, 1, 2});

  left_foot_rel_tracking_data->SetImpactInvariantProjection(true);
  right_foot_rel_tracking_data->SetImpactInvariantProjection(true);
  left_foot_yz_rel_tracking_data->SetImpactInvariantProjection(true);
  right_foot_yz_rel_tracking_data->SetImpactInvariantProjection(true);
  pelvis_trans_rel_tracking_data->SetImpactInvariantProjection(true);

  osc->AddTrackingData(std::move(pelvis_trans_rel_tracking_data));
  osc->AddTrackingData(std::move(left_foot_rel_tracking_data));
  osc->AddTrackingData(std::move(right_foot_rel_tracking_data));
  osc->AddTrackingData(std::move(left_foot_yz_rel_tracking_data));
  osc->AddTrackingData(std::move(right_foot_yz_rel_tracking_data));

  auto heading_traj_generator =
      builder.AddSystem<cassie::osc::HeadingTrajGenerator>(plant,
                                                           plant_context.get());

  auto pelvis_rot_tracking_data = std::make_unique<RotTaskSpaceTrackingData>(
      "pelvis_rot_traj", osc_gains.K_p_pelvis_rot, osc_gains.K_d_pelvis_rot,
      osc_gains.W_pelvis_rot, plant, plant);
  pelvis_rot_tracking_data->AddStateAndFrameToTrack(
      RUNNING_FSM_STATE::LEFT_STANCE, "pelvis");
  pelvis_rot_tracking_data->AddStateAndFrameToTrack(
      RUNNING_FSM_STATE::RIGHT_STANCE, "pelvis");
  pelvis_rot_tracking_data->AddStateAndFrameToTrack(
      RUNNING_FSM_STATE::RIGHT_FLIGHT, "pelvis");
  pelvis_rot_tracking_data->AddStateAndFrameToTrack(
      RUNNING_FSM_STATE::LEFT_FLIGHT, "pelvis");
  pelvis_rot_tracking_data->SetImpactInvariantProjection(true);
  osc->AddTrackingData(std::move(pelvis_rot_tracking_data));

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
  auto left_toe_angle_tracking_data = std::make_unique<JointSpaceTrackingData>(
      "left_toe_angle_traj", osc_gains.K_p_swing_toe, osc_gains.K_d_swing_toe,
      osc_gains.W_swing_toe, plant, plant);
  auto right_toe_angle_tracking_data = std::make_unique<JointSpaceTrackingData>(
      "right_toe_angle_traj", osc_gains.K_p_swing_toe, osc_gains.K_d_swing_toe,
      osc_gains.W_swing_toe, plant, plant);
  left_toe_angle_tracking_data->AddStateAndJointToTrack(
      RUNNING_FSM_STATE::RIGHT_STANCE, "toe_left", "toe_leftdot");
  left_toe_angle_tracking_data->AddStateAndJointToTrack(
      RUNNING_FSM_STATE::LEFT_FLIGHT, "toe_left", "toe_leftdot");
  left_toe_angle_tracking_data->AddStateAndJointToTrack(
      RUNNING_FSM_STATE::RIGHT_FLIGHT, "toe_left", "toe_leftdot");
  right_toe_angle_tracking_data->AddStateAndJointToTrack(
      RUNNING_FSM_STATE::LEFT_STANCE, "toe_right", "toe_rightdot");
  right_toe_angle_tracking_data->AddStateAndJointToTrack(
      RUNNING_FSM_STATE::LEFT_FLIGHT, "toe_right", "toe_rightdot");
  right_toe_angle_tracking_data->AddStateAndJointToTrack(
      RUNNING_FSM_STATE::RIGHT_FLIGHT, "toe_right", "toe_rightdot");
  left_toe_angle_tracking_data->SetImpactInvariantProjection(true);
  right_toe_angle_tracking_data->SetImpactInvariantProjection(true);
  osc->AddTrackingData(std::move(left_toe_angle_tracking_data));
  osc->AddTrackingData(std::move(right_toe_angle_tracking_data));

  // Swing hip yaw joint tracking
  auto left_hip_yaw_tracking_data = std::make_unique<JointSpaceTrackingData>(
      "hip_yaw_left_traj", osc_gains.K_p_hip_yaw, osc_gains.K_d_hip_yaw,
      osc_gains.W_hip_yaw, plant, plant);
  auto right_hip_yaw_tracking_data = std::make_unique<JointSpaceTrackingData>(
      "hip_yaw_right_traj", osc_gains.K_p_hip_yaw, osc_gains.K_d_hip_yaw,
      osc_gains.W_hip_yaw, plant, plant);
  left_hip_yaw_tracking_data->AddJointToTrack("hip_yaw_left",
                                              "hip_yaw_leftdot");
  right_hip_yaw_tracking_data->AddJointToTrack("hip_yaw_right",
                                               "hip_yaw_rightdot");
  left_hip_yaw_tracking_data->SetImpactInvariantProjection(true);
  right_hip_yaw_tracking_data->SetImpactInvariantProjection(true);
  osc->AddConstTrackingData(std::move(left_hip_yaw_tracking_data),
                            VectorXd::Zero(1));
  osc->AddConstTrackingData(std::move(right_hip_yaw_tracking_data),
                            VectorXd::Zero(1));

  osc->SetOsqpSolverOptionsFromYaml(FLAGS_osqp_settings);
  // Build OSC problem
  osc->Build();
  std::cout << "Built OSC" << std::endl;

  /*****Connect ports*****/

  // OSC connections
  builder.Connect(contact_scheduler->get_output_port_fsm(),
                  osc->get_input_port_fsm());
  builder.Connect(contact_scheduler->get_output_port_impact_info(),
                  osc->get_input_port_impact_info());
  builder.Connect(contact_scheduler->get_output_port_clock(),
                  osc->get_input_port_clock());

  builder.Connect(state_receiver->get_output_port(0),
                  ekf_filter->get_input_port());
  builder.Connect(ekf_filter->get_output_port(),
                  osc->get_robot_output_input_port());
  //  builder.Connect(state_receiver->get_output_port(0),
  //                  osc->get_robot_output_input_port());

  // FSM connections
  builder.Connect(state_receiver->get_output_port(0),
                  contact_scheduler->get_input_port_state());

  // Trajectory generator connections

  builder.Connect(
      contact_scheduler->get_output_port_contact_scheduler(),
      pelvis_trans_traj_generator->get_contact_scheduler_input_port());
  builder.Connect(contact_scheduler->get_output_port_contact_scheduler(),
                  l_foot_traj_generator->get_input_port_contact_scheduler());
  builder.Connect(contact_scheduler->get_output_port_contact_scheduler(),
                  r_foot_traj_generator->get_input_port_contact_scheduler());

  builder.Connect(state_receiver->get_output_port(0),
                  pelvis_trans_traj_generator->get_state_input_port());
  builder.Connect(contact_scheduler->get_output_port_fsm(),
                  pelvis_trans_traj_generator->get_fsm_input_port());
  builder.Connect(contact_scheduler->get_output_port_clock(),
                  pelvis_trans_traj_generator->get_clock_input_port());
  builder.Connect(high_level_command->get_xy_output_port(),
                  l_foot_traj_generator->get_input_port_target_vel());
  builder.Connect(high_level_command->get_xy_output_port(),
                  r_foot_traj_generator->get_input_port_target_vel());
  builder.Connect(state_receiver->get_output_port(0),
                  l_foot_traj_generator->get_input_port_state());
  builder.Connect(state_receiver->get_output_port(0),
                  r_foot_traj_generator->get_input_port_state());
  builder.Connect(cassie_out_to_radio->get_output_port(),
                  l_foot_traj_generator->get_input_port_radio());
  builder.Connect(cassie_out_to_radio->get_output_port(),
                  r_foot_traj_generator->get_input_port_radio());

  builder.Connect(contact_scheduler->get_output_port_clock(),
                  l_foot_traj_generator->get_input_port_clock());
  builder.Connect(contact_scheduler->get_output_port_clock(),
                  r_foot_traj_generator->get_input_port_clock());
  builder.Connect(contact_scheduler->get_output_port_fsm(),
                  l_foot_traj_generator->get_input_port_fsm());
  builder.Connect(contact_scheduler->get_output_port_fsm(),
                  r_foot_traj_generator->get_input_port_fsm());
  builder.Connect(state_receiver->get_output_port(0),
                  left_toe_angle_traj_gen->get_input_port_state());
  builder.Connect(state_receiver->get_output_port(0),
                  right_toe_angle_traj_gen->get_input_port_state());
  // OSC connections
  builder.Connect(pelvis_trans_traj_generator->get_output_port(0),
                  osc->get_input_port_tracking_data("pelvis_trans_traj"));
  builder.Connect(state_receiver->get_output_port(0),
                  heading_traj_generator->get_state_input_port());
  builder.Connect(high_level_command->get_yaw_output_port(),
                  heading_traj_generator->get_yaw_input_port());
  builder.Connect(heading_traj_generator->get_output_port(0),
                  osc->get_input_port_tracking_data("pelvis_rot_traj"));
  builder.Connect(l_foot_traj_generator->get_output_port(0),
                  osc->get_input_port_tracking_data("left_ft_traj"));
  builder.Connect(r_foot_traj_generator->get_output_port(0),
                  osc->get_input_port_tracking_data("right_ft_traj"));
  builder.Connect(l_foot_traj_generator->get_output_port(0),
                  osc->get_input_port_tracking_data("left_ft_z_traj"));
  builder.Connect(r_foot_traj_generator->get_output_port(0),
                  osc->get_input_port_tracking_data("right_ft_z_traj"));
  builder.Connect(left_toe_angle_traj_gen->get_output_port(0),
                  osc->get_input_port_tracking_data("left_toe_angle_traj"));
  builder.Connect(right_toe_angle_traj_gen->get_output_port(0),
                  osc->get_input_port_tracking_data("right_toe_angle_traj"));

  // Publisher connections
  builder.Connect(cassie_out_receiver->get_output_port(),
                  cassie_out_to_radio->get_input_port());
  builder.Connect(cassie_out_to_radio->get_output_port(),
                  high_level_command->get_radio_input_port());
  builder.Connect(osc->get_osc_output_port(),
                  command_sender->get_input_port(0));
  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port());
  builder.Connect(osc->get_osc_debug_port(), osc_debug_pub->get_input_port());
  builder.Connect(osc->get_failure_output_port(),
                  failure_aggregator->get_input_port(0));
  builder.Connect(failure_aggregator->get_status_output_port(),
                  controller_failure_pub->get_input_port());
  builder.Connect(contact_scheduler->get_output_port_debug_info(),
                  contact_scheduler_debug_publisher->get_input_port());

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
}
}  // namespace examples
}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::examples::DoMain(argc, argv);
}
