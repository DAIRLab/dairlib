#include <fstream>

#include <drake/multibody/parsing/parser.h>
#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_utils.h"
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
DEFINE_string(folder_path, "examples/Cassie/saved_trajectories/",
              "Folder path for where the trajectory names are stored");
DEFINE_string(traj_name, "running_0.00",
              "File to load saved trajectories from");
DEFINE_string(gains_filename, "examples/Cassie/osc_run/osc_running_gains.yaml",
              "Filepath containing gains");
DEFINE_string(
    channel_cassie_out, "CASSIE_OUTPUT_ECHO",
    "The name of the channel to receive the cassie out structure from.");
DEFINE_double(
    fsm_time_offset, 0.0,
    "Time (s) in the fsm to move from the stance phase to the flight phase");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Build the controller diagram
  DiagramBuilder<double> builder;

  // Built the Cassie MBPs
  drake::multibody::MultibodyPlant<double> plant(0.0);
  addCassieMultibody(&plant, nullptr, true,
                     "examples/Cassie/urdf/cassie_v2_conservative.urdf",
                     false /*spring model*/, false /*loop closure*/);
  drake::multibody::MultibodyPlant<double> plant_wo_spr(0.0);
  addCassieMultibody(&plant_wo_spr, nullptr, true,
                     "examples/Cassie/urdf/cassie_fixed_springs.urdf",
                     false /*spring model*/, false /*loop closure*/);
  plant.Finalize();
  plant_wo_spr.Finalize();

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

  map<string, int> pos_map_wo_spr =
      multibody::makeNameToPositionsMap(plant_wo_spr);

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
  const DirconTrajectory& dircon_trajectory = DirconTrajectory(
      FindResourceOrThrow(FLAGS_folder_path + FLAGS_traj_name));

  PiecewisePolynomial<double> state_traj =
      dircon_trajectory.ReconstructStateTrajectory();
  state_traj.ConcatenateInTime(
      dircon_trajectory.ReconstructMirrorStateTrajectory(
          state_traj.end_time()));

  /**** OSC Gains ****/
  OSCGains gains{};
  const YAML::Node& root =
      YAML::LoadFile(FindResourceOrThrow(FLAGS_gains_filename));
  drake::yaml::YamlReadArchive::Options yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;
  drake::yaml::YamlReadArchive(root, yaml_options).Accept(&gains);

  /**** FSM and contact mode configuration ****/
  int left_stance_state = 0;
  int right_stance_state = 1;
  int right_touchdown_air_phase = 2;
  int left_touchdown_air_phase = 3;
  double left_support_duration =
      dircon_trajectory.GetStateBreaks(1)(0) * 2 - FLAGS_fsm_time_offset;
  double right_support_duration = left_support_duration;
  double air_phase_duration = dircon_trajectory.GetStateBreaks(2)(0) -
                              dircon_trajectory.GetStateBreaks(1)(0) +
                              FLAGS_fsm_time_offset;
  vector<int> fsm_states = {left_stance_state, right_touchdown_air_phase,
                            right_stance_state, left_touchdown_air_phase,
                            left_stance_state};
  std::cout << "left support duration: " << left_support_duration << std::endl;
  std::cout << "flight duration: " << air_phase_duration << std::endl;
  std::cout << "right support duration: " << right_support_duration
            << std::endl;
  vector<double> state_durations = {
      left_support_duration / 2, air_phase_duration, right_support_duration,
      air_phase_duration, left_support_duration / 2};

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
      plant, plant, plant_context.get(), plant_context.get(), true);
  auto osc_debug_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
          "OSC_DEBUG_RUNNING", &lcm, TriggerTypeSet({TriggerType::kForced})));
  auto failure_aggregator =
      builder.AddSystem<systems::ControllerFailureAggregator>(FLAGS_channel_u,
                                                              1);
  auto controller_failure_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_controller_failure>(
          "CONTROLLER_ERROR", &lcm, TriggerTypeSet({TriggerType::kForced})));

  /**** OSC setup ****/
  // Cost
  MatrixXd Q_accel = gains.w_accel * MatrixXd::Identity(nv, nv);
  osc->SetAccelerationCostForAllJoints(Q_accel);
  osc->SetInputRegularizationWeight(gains.w_input_reg);

  // Soft constraint on contacts
  double w_contact_relax = gains.w_soft_constraint;
  osc->SetWeightOfSoftContactConstraint(w_contact_relax);

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
  OSCRunningGains osc_gains;
  drake::yaml::YamlReadArchive(root).Accept(&osc_gains);

  auto cassie_out_receiver =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_cassie_out>(
          FLAGS_channel_cassie_out, &lcm));
  cassie::osc::HighLevelCommand* high_level_command;
  high_level_command = builder.AddSystem<cassie::osc::HighLevelCommand>(
      plant, plant_context.get(), osc_gains.vel_scale_rot,
      osc_gains.vel_scale_trans_sagital, osc_gains.vel_scale_trans_lateral);
  builder.Connect(cassie_out_receiver->get_output_port(),
                  high_level_command->get_cassie_output_port());

  string output_traj_path = FLAGS_folder_path + FLAGS_traj_name + "_processed";
  if (osc_gains.relative_feet) {
    output_traj_path += "_rel";
  }
  const LcmTrajectory& output_trajs =
      LcmTrajectory(FindResourceOrThrow(output_traj_path));
  PiecewisePolynomial<double> pelvis_trans_traj;
  PiecewisePolynomial<double> l_hip_trajectory;
  PiecewisePolynomial<double> r_hip_trajectory;
  PiecewisePolynomial<double> l_foot_trajectory;
  PiecewisePolynomial<double> r_foot_trajectory;
  PiecewisePolynomial<double> pelvis_rot_trajectory;

  //  for (auto name : output_trajs.GetTrajectoryNames()) {
  //    std::cout << name << std::endl;
  //  }
  for (int mode = 0; mode < dircon_trajectory.GetNumModes() * 2; ++mode) {
    const LcmTrajectory::Trajectory lcm_pelvis_trans_trajectory =
        output_trajs.GetTrajectory("pelvis_trans_trajectory" +
                                   std::to_string(mode));
    const LcmTrajectory::Trajectory lcm_left_hip_traj =
        output_trajs.GetTrajectory("left_hip_trajectory" +
                                   std::to_string(mode));
    const LcmTrajectory::Trajectory lcm_right_hip_traj =
        output_trajs.GetTrajectory("right_hip_trajectory" +
                                   std::to_string(mode));
    const LcmTrajectory::Trajectory lcm_left_foot_traj =
        output_trajs.GetTrajectory("left_foot_trajectory" +
                                   std::to_string(mode));
    const LcmTrajectory::Trajectory lcm_right_foot_traj =
        output_trajs.GetTrajectory("right_foot_trajectory" +
                                   std::to_string(mode));
    const LcmTrajectory::Trajectory lcm_pelvis_rot_traj =
        output_trajs.GetTrajectory("pelvis_rot_trajectory" +
                                   std::to_string(mode));
    pelvis_trans_traj.ConcatenateInTime(
        PiecewisePolynomial<double>::CubicHermite(
            lcm_pelvis_trans_trajectory.time_vector,
            lcm_pelvis_trans_trajectory.datapoints.topRows(6),
            lcm_pelvis_trans_trajectory.datapoints.bottomRows(6)));
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
    pelvis_rot_trajectory.ConcatenateInTime(
        PiecewisePolynomial<double>::FirstOrderHold(
            lcm_pelvis_rot_traj.time_vector,
            lcm_pelvis_rot_traj.datapoints.topRows(4)));
  }

  auto pelvis_trans_traj_generator =
      builder.AddSystem<PelvisTransTrajGenerator>(
          plant, plant_context.get(), pelvis_trans_traj, feet_contact_points,
          osc_gains.relative_pelvis);
  pelvis_trans_traj_generator->SetSLIPParams(osc_gains.rest_length);
  //                                             osc_gains.k_leg,
  //                                             osc_gains.b_leg);
  auto l_foot_traj_generator = builder.AddSystem<FootTrajGenerator>(
      plant, plant_context.get(), "hip_left", true, l_foot_trajectory,
      l_hip_trajectory, osc_gains.relative_feet);
  auto r_foot_traj_generator = builder.AddSystem<FootTrajGenerator>(
      plant, plant_context.get(), "hip_right", false, r_foot_trajectory,
      r_hip_trajectory, osc_gains.relative_feet);
  l_foot_traj_generator->SetFootstepGains(osc_gains.K_p_footstep,
                                          osc_gains.K_d_footstep);
  r_foot_traj_generator->SetFootstepGains(osc_gains.K_p_footstep,
                                          osc_gains.K_d_footstep);
  l_foot_traj_generator->SetFootPlacementOffsets(osc_gains.center_line_offset,
                                                 osc_gains.footstep_offset);
  r_foot_traj_generator->SetFootPlacementOffsets(osc_gains.center_line_offset,
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
  left_hip_tracking_data.AddStateAndPointToTrack(right_stance_state,
                                                 "hip_left");
  right_hip_tracking_data.AddStateAndPointToTrack(left_stance_state,
                                                  "hip_right");
  right_hip_tracking_data.AddStateAndPointToTrack(right_touchdown_air_phase,
                                                  "hip_right");
  left_hip_tracking_data.AddStateAndPointToTrack(left_touchdown_air_phase,
                                                 "hip_left");

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
  pelvis_trans_rel_tracking_data.SetImpactInvariantProjection(true);
  left_foot_yz_rel_tracking_data.DisableFeedforwardAccel({0, 1, 2});
  right_foot_yz_rel_tracking_data.DisableFeedforwardAccel({0, 1, 2});

  if (osc_gains.relative_pelvis) {
    osc->AddTrackingData(&pelvis_trans_rel_tracking_data);
  } else {
    osc->AddTrackingData(&pelvis_tracking_data);
  }

  if (osc_gains.relative_feet) {
    left_foot_rel_tracking_data.SetImpactInvariantProjection(true);
    right_foot_rel_tracking_data.SetImpactInvariantProjection(true);
    osc->AddTrackingData(&left_foot_rel_tracking_data);
    osc->AddTrackingData(&right_foot_rel_tracking_data);
    //    left_foot_yz_rel_tracking_data.SetImpactInvariantProjection(true);
    //    right_foot_yz_rel_tracking_data.SetImpactInvariantProjection(true);
    osc->AddTrackingData(&left_foot_yz_rel_tracking_data);
    osc->AddTrackingData(&right_foot_yz_rel_tracking_data);
  } else {
    left_foot_tracking_data.SetImpactInvariantProjection(true);
    right_foot_tracking_data.SetImpactInvariantProjection(true);
    osc->AddTrackingData(&left_foot_tracking_data);
    osc->AddTrackingData(&right_foot_tracking_data);
  }

  // Stance hip pitch trajectory
  auto hip_pitch_left_traj = dircon_trajectory.ReconstructJointTrajectory(
      pos_map_wo_spr["hip_pitch_left"]);
  auto hip_pitch_left_traj_mir =
      dircon_trajectory.ReconstructMirrorJointTrajectory(
          pos_map_wo_spr["hip_pitch_left"]);
  auto hip_pitch_right_traj = dircon_trajectory.ReconstructJointTrajectory(
      pos_map_wo_spr["hip_pitch_right"]);
  auto hip_pitch_right_traj_mir =
      dircon_trajectory.ReconstructMirrorJointTrajectory(
          pos_map_wo_spr["hip_pitch_right"]);
  PiecewisePolynomial<double> pelvis_pitch_traj =
      PiecewisePolynomial<double>(VectorXd::Zero(1));
  hip_pitch_left_traj_mir.shiftRight(hip_pitch_left_traj.end_time());
  hip_pitch_right_traj_mir.shiftRight(hip_pitch_right_traj.end_time());
  hip_pitch_left_traj.ConcatenateInTime(hip_pitch_left_traj_mir);
  hip_pitch_right_traj.ConcatenateInTime(hip_pitch_right_traj_mir);
  //  auto hip_pitch_left_traj_generator =
  //      builder.AddSystem<BasicTrajectoryPassthrough>(
  //          hip_pitch_left_traj, "hip_pitch_left_traj_generator");
  //  auto hip_pitch_right_traj_generator =
  //      builder.AddSystem<BasicTrajectoryPassthrough>(
  //          hip_pitch_right_traj, "hip_pitch_right_traj_generator");
  auto hip_pitch_left_traj_generator =
      builder.AddSystem<PelvisPitchTrajGenerator>(
          plant, plant_context.get(), hip_pitch_left_traj, pelvis_pitch_traj, 1,
          "hip_pitch_left_traj_generator");
  auto hip_pitch_right_traj_generator =
      builder.AddSystem<PelvisPitchTrajGenerator>(
          plant, plant_context.get(), hip_pitch_right_traj, pelvis_pitch_traj,
          1, "hip_pitch_right_traj_generator");

  JointSpaceTrackingData left_hip_pitch_tracking_data(
      "hip_pitch_left_traj", osc_gains.K_p_hip_pitch, osc_gains.K_d_hip_pitch,
      osc_gains.W_hip_pitch, plant, plant);
  JointSpaceTrackingData right_hip_pitch_tracking_data(
      "hip_pitch_right_traj", osc_gains.K_p_hip_pitch, osc_gains.K_d_hip_pitch,
      osc_gains.W_hip_pitch, plant, plant);
  left_hip_pitch_tracking_data.AddStateAndJointToTrack(
      left_stance_state, "hip_pitch_left", "hip_pitch_leftdot");
  left_hip_pitch_tracking_data.AddStateAndJointToTrack(
      right_touchdown_air_phase, "hip_pitch_left", "hip_pitch_leftdot");
  right_hip_pitch_tracking_data.AddStateAndJointToTrack(
      right_stance_state, "hip_pitch_right", "hip_pitch_rightdot");
  right_hip_pitch_tracking_data.AddStateAndJointToTrack(
      left_touchdown_air_phase, "hip_pitch_right", "hip_pitch_rightdot");
  left_hip_pitch_tracking_data.DisableFeedforwardAccel({0});
  right_hip_pitch_tracking_data.DisableFeedforwardAccel({0});
  left_hip_pitch_tracking_data.SetImpactInvariantProjection(true);
  right_hip_pitch_tracking_data.SetImpactInvariantProjection(true);
  //  osc->AddConstTrackingData(&hip_pitch_left_tracking_data,
  //                            0.6 * VectorXd::Ones(1));
  //  osc->AddConstTrackingData(&hip_pitch_right_tracking_data,
  //                            0.6 * VectorXd::Ones(1));
  osc->AddTrackingData(&left_hip_pitch_tracking_data);
  osc->AddTrackingData(&right_hip_pitch_tracking_data);

  // Stance hip roll trajectory
  auto hip_roll_left_traj = dircon_trajectory.ReconstructJointTrajectory(
      pos_map_wo_spr["hip_roll_left"]);
  auto hip_roll_left_traj_mir =
      dircon_trajectory.ReconstructMirrorJointTrajectory(
          pos_map_wo_spr["hip_roll_left"]);
  auto hip_roll_right_traj = dircon_trajectory.ReconstructJointTrajectory(
      pos_map_wo_spr["hip_roll_right"]);
  auto hip_roll_right_traj_mir =
      dircon_trajectory.ReconstructMirrorJointTrajectory(
          pos_map_wo_spr["hip_roll_right"]);
  hip_roll_left_traj_mir.shiftRight(hip_roll_left_traj.end_time());
  hip_roll_right_traj_mir.shiftRight(hip_roll_right_traj.end_time());
  hip_roll_left_traj.ConcatenateInTime(hip_roll_left_traj_mir);
  hip_roll_right_traj.ConcatenateInTime(hip_roll_right_traj_mir);
  PiecewisePolynomial<double> pelvis_roll_traj =
      PiecewisePolynomial<double>(VectorXd::Zero(1));
  auto hip_roll_left_traj_generator =
      builder.AddSystem<PelvisRollTrajGenerator>(
          plant, plant_context.get(), hip_roll_left_traj, pelvis_roll_traj, 1,
          "hip_roll_left_traj_generator");
  auto hip_roll_right_traj_generator =
      builder.AddSystem<PelvisRollTrajGenerator>(
          plant, plant_context.get(), hip_roll_right_traj, pelvis_roll_traj, 1,
          "hip_roll_right_traj_generator");
  JointSpaceTrackingData left_hip_roll_tracking_data(
      "hip_roll_left_traj", osc_gains.K_p_hip_roll, osc_gains.K_d_hip_roll,
      osc_gains.W_hip_roll, plant, plant);
  JointSpaceTrackingData right_hip_roll_tracking_data(
      "hip_roll_right_traj", osc_gains.K_p_hip_roll, osc_gains.K_d_hip_roll,
      osc_gains.W_hip_roll, plant, plant);
  left_hip_roll_tracking_data.AddStateAndJointToTrack(
      left_stance_state, "hip_roll_left", "hip_roll_leftdot");
  left_hip_roll_tracking_data.AddStateAndJointToTrack(
      right_touchdown_air_phase, "hip_roll_left", "hip_roll_leftdot");
  right_hip_roll_tracking_data.AddStateAndJointToTrack(
      right_stance_state, "hip_roll_right", "hip_roll_rightdot");
  right_hip_roll_tracking_data.AddStateAndJointToTrack(
      left_touchdown_air_phase, "hip_roll_right", "hip_roll_rightdot");
  left_hip_roll_tracking_data.DisableFeedforwardAccel({0});
  right_hip_roll_tracking_data.DisableFeedforwardAccel({0});
  left_hip_roll_tracking_data.SetImpactInvariantProjection(true);
  right_hip_roll_tracking_data.SetImpactInvariantProjection(true);
  osc->AddTrackingData(&left_hip_roll_tracking_data);
  osc->AddTrackingData(&right_hip_roll_tracking_data);

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
  right_toe_angle_tracking_data.AddStateAndJointToTrack(
      left_stance_state, "toe_right", "toe_rightdot");
  left_toe_angle_tracking_data.AddStateAndJointToTrack(
      right_stance_state, "toe_left", "toe_leftdot");
  right_toe_angle_tracking_data.AddStateAndJointToTrack(
      right_touchdown_air_phase, "toe_right", "toe_rightdot");
  right_toe_angle_tracking_data.AddStateAndJointToTrack(
      left_touchdown_air_phase, "toe_right", "toe_rightdot");
  left_toe_angle_tracking_data.AddStateAndJointToTrack(
      right_touchdown_air_phase, "toe_left", "toe_leftdot");
  left_toe_angle_tracking_data.AddStateAndJointToTrack(
      left_touchdown_air_phase, "toe_left", "toe_leftdot");
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
  osc->Build();
  std::cout << "Built OSC" << std::endl;

  /*****Connect ports*****/

  // OSC connections
  builder.Connect(fsm->get_output_port_fsm(), osc->get_fsm_input_port());
  builder.Connect(fsm->get_output_port_impact(),
                  osc->get_near_impact_input_port());
  builder.Connect(fsm->get_output_port_clock(), osc->get_clock_input_port());
  builder.Connect(state_receiver->get_output_port(0),
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
  builder.Connect(hip_pitch_left_traj_generator->get_output_port(),
                  osc->get_tracking_data_input_port("hip_pitch_left_traj"));
  builder.Connect(hip_pitch_right_traj_generator->get_output_port(),
                  osc->get_tracking_data_input_port("hip_pitch_right_traj"));
  builder.Connect(hip_roll_left_traj_generator->get_output_port(),
                  osc->get_tracking_data_input_port("hip_roll_left_traj"));
  builder.Connect(hip_roll_right_traj_generator->get_output_port(),
                  osc->get_tracking_data_input_port("hip_roll_right_traj"));
  // OSC connections
  builder.Connect(pelvis_trans_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("pelvis_trans_traj"));
  builder.Connect(state_receiver->get_output_port(0),
                  hip_roll_left_traj_generator->get_state_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  hip_roll_right_traj_generator->get_state_input_port());
  builder.Connect(fsm->get_output_port_fsm(),
                  hip_roll_left_traj_generator->get_fsm_input_port());
  builder.Connect(fsm->get_output_port_fsm(),
                  hip_roll_right_traj_generator->get_fsm_input_port());
  builder.Connect(fsm->get_output_port_clock(),
                  hip_roll_left_traj_generator->get_clock_input_port());
  builder.Connect(fsm->get_output_port_clock(),
                  hip_roll_right_traj_generator->get_clock_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  hip_pitch_left_traj_generator->get_state_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  hip_pitch_right_traj_generator->get_state_input_port());
  builder.Connect(fsm->get_output_port_fsm(),
                  hip_pitch_left_traj_generator->get_fsm_input_port());
  builder.Connect(fsm->get_output_port_fsm(),
                  hip_pitch_right_traj_generator->get_fsm_input_port());
  builder.Connect(fsm->get_output_port_clock(),
                  hip_pitch_left_traj_generator->get_clock_input_port());
  builder.Connect(fsm->get_output_port_clock(),
                  hip_pitch_right_traj_generator->get_clock_input_port());
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
