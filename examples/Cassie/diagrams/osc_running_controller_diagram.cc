#include "osc_running_controller_diagram.h"

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
#include "systems/primitives/subvector_pass_through.h"
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
using examples::osc_run::FootTrajGenerator;
using multibody::FixedJointEvaluator;
using multibody::WorldYawViewFrame;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::RelativeTranslationTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;

namespace examples {
namespace controllers {

OSCRunningControllerDiagram::OSCRunningControllerDiagram(
    drake::multibody::MultibodyPlant<double>& plant, const OSCGains& osc_gains,
    const OSCRunningGains& osc_running_gains)
    : pos_map(multibody::makeNameToPositionsMap(plant)),
      vel_map(multibody::makeNameToVelocitiesMap(plant)),
      act_map(multibody::makeNameToActuatorsMap(plant)),
      left_toe(LeftToeFront(plant)),
      left_heel(LeftToeRear(plant)),
      right_toe(RightToeFront(plant)),
      right_heel(RightToeRear(plant)),
      left_foot_points({left_toe, left_heel}),
      right_foot_points({right_toe, right_heel}),
      view_frame(
          multibody::WorldYawViewFrame<double>(plant.GetBodyByName("pelvis"))),
      left_toe_evaluator(multibody::WorldPointEvaluator(
          plant, left_toe.first, left_toe.second, view_frame,
          Matrix3d::Identity(), Vector3d::Zero(), {1, 2})),
      left_heel_evaluator(multibody::WorldPointEvaluator(
          plant, left_heel.first, left_heel.second, view_frame,
          Matrix3d::Identity(), Vector3d::Zero(), {0, 1, 2})),
      right_toe_evaluator(multibody::WorldPointEvaluator(
          plant, right_toe.first, right_toe.second, view_frame,
          Matrix3d::Identity(), Vector3d::Zero(), {1, 2})),
      right_heel_evaluator(multibody::WorldPointEvaluator(
          plant, right_heel.first, right_heel.second, view_frame,
          Matrix3d::Identity(), Vector3d::Zero(), {0, 1, 2})),
      left_loop(LeftLoopClosureEvaluator(plant)),
      right_loop(RightLoopClosureEvaluator(plant)),
      evaluators(multibody::KinematicEvaluatorSet<double>(plant)),
      left_fixed_knee_spring(
          FixedJointEvaluator(plant, pos_map.at("knee_joint_left"),
                              vel_map.at("knee_joint_leftdot"), 0)),
      right_fixed_knee_spring(
          FixedJointEvaluator(plant, pos_map.at("knee_joint_right"),
                              vel_map.at("knee_joint_rightdot"), 0)),
      left_fixed_ankle_spring(
          FixedJointEvaluator(plant, pos_map.at("ankle_spring_joint_left"),
                              vel_map.at("ankle_spring_joint_leftdot"), 0)),
      right_fixed_ankle_spring(
          FixedJointEvaluator(plant, pos_map.at("ankle_spring_joint_right"),
                              vel_map.at("ankle_spring_joint_rightdot"), 0)),
      osc_gains_(osc_gains),
      osc_running_gains_(osc_running_gains){

  // Build the controller diagram
  DiagramBuilder<double> builder;
  plant_context = plant.CreateDefaultContext();
  feet_contact_points[0] = std::vector<
      std::pair<const Vector3d, const drake::multibody::Frame<double>&>>(
      {left_toe, left_heel});
  feet_contact_points[1] = std::vector<
      std::pair<const Vector3d, const drake::multibody::Frame<double>&>>(
      {right_toe, right_heel});

  /**** OSC Gains ****/
  drake::yaml::YamlReadArchive::Options yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;

  /**** FSM and contact mode configuration ****/
  int left_stance_state = 0;
  int right_stance_state = 1;
  int right_touchdown_air_phase = 2;
  int left_touchdown_air_phase = 3;

  fsm_states = vector<int>();
  fsm_states.push_back(left_stance_state);
  fsm_states.push_back(right_touchdown_air_phase);
  fsm_states.push_back(right_stance_state);
  fsm_states.push_back(left_touchdown_air_phase);
  fsm_states.push_back(left_stance_state);

  state_durations = vector<double>();
  state_durations.push_back(osc_running_gains_.stance_duration);
  state_durations.push_back(osc_running_gains_.flight_duration);
  state_durations.push_back(osc_running_gains_.stance_duration);
  state_durations.push_back(osc_running_gains_.flight_duration);
  state_durations.push_back(0.0);

  accumulated_state_durations = vector<double>();
  accumulated_state_durations.push_back(0);
  std::cout << accumulated_state_durations.back() << std::endl;
  for (double state_duration : state_durations) {
    accumulated_state_durations.push_back(accumulated_state_durations.back() +
                                          state_duration);
    std::cout << accumulated_state_durations.back() << std::endl;
  }
  accumulated_state_durations.pop_back();

  auto fsm = builder.AddSystem<ImpactTimeBasedFiniteStateMachine>(
      plant, fsm_states, state_durations, 0.0, osc_gains_.impact_threshold);

  /**** Initialize all the leaf systems ****/
  drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0");

  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);
  auto command_sender = builder.AddSystem<systems::RobotCommandSender>(plant);
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      plant, plant, plant_context.get(), plant_context.get(), true, false);
  //  auto failure_aggregator =
  //      builder.AddSystem<systems::ControllerFailureAggregator>(
  //          control_channel_name_, 1);
  //  auto passthrough = builder.AddSystem<systems::SubvectorPassThrough>(
  //      osc->get_output_port(0).size(), 0,
  //      plant.get_actuation_input_port().size());
  std::vector<double> tau = {.05, .05, .01};
  auto ekf_filter =
      builder.AddSystem<systems::FloatingBaseVelocityFilter>(plant, tau);

  /**** OSC setup ****/
  // Cost
  /// REGULARIZATION COSTS
  osc->SetAccelerationCostWeights(osc_gains_.w_accel *
                                  osc_gains_.W_acceleration);
  osc->SetInputCostWeights(osc_gains_.w_input *
                           osc_gains_.W_input_regularization);
  osc->SetLambdaHolonomicRegularizationWeight(
      1e-5 * osc_gains_.W_lambda_h_regularization);

  // Soft constraint on contacts
  osc->SetSoftConstraintWeight(osc_gains_.w_soft_constraint);

  // Contact information for OSC
  osc->SetContactFriction(osc_gains_.mu);

  osc->AddStateAndContactPoint(left_stance_state, &left_toe_evaluator);
  osc->AddStateAndContactPoint(left_stance_state, &left_heel_evaluator);
  osc->AddStateAndContactPoint(right_stance_state, &right_toe_evaluator);
  osc->AddStateAndContactPoint(right_stance_state, &right_heel_evaluator);

  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);

  // Fix the springs in the dynamics
  evaluators.add_evaluator(&left_fixed_knee_spring);
  evaluators.add_evaluator(&right_fixed_knee_spring);
  evaluators.add_evaluator(&left_fixed_ankle_spring);
  evaluators.add_evaluator(&right_fixed_ankle_spring);

  osc->AddKinematicConstraint(&evaluators);

  /**** Tracking Data *****/

  std::cout << "Creating tracking data. " << std::endl;

  //  auto cassie_out_receiver =
  //      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_cassie_out>(
  //          FLAGS_channel_cassie_out, &lcm));
  cassie::osc::HighLevelCommand* high_level_command;
  high_level_command = builder.AddSystem<cassie::osc::HighLevelCommand>(
      plant, plant_context.get(), osc_running_gains_.vel_scale_rot,
      osc_running_gains_.vel_scale_trans_sagital,
      osc_running_gains_.vel_scale_trans_lateral);

  auto default_traj = PiecewisePolynomial<double>(Vector3d{0, 0, 0});
  auto pelvis_trans_traj_generator =
      builder.AddSystem<PelvisTransTrajGenerator>(
          plant, plant_context.get(), default_traj, feet_contact_points,
          osc_running_gains_.relative_pelvis);
  pelvis_trans_traj_generator->SetSLIPParams(osc_running_gains_.rest_length);
  auto l_foot_traj_generator = builder.AddSystem<FootTrajGenerator>(
      plant, plant_context.get(), "toe_left", "pelvis",
      osc_running_gains_.relative_feet, 0, accumulated_state_durations);
  auto r_foot_traj_generator = builder.AddSystem<FootTrajGenerator>(
      plant, plant_context.get(), "toe_right", "pelvis",
      osc_running_gains_.relative_feet, 1, accumulated_state_durations);
  l_foot_traj_generator->SetFootstepGains(osc_running_gains_.K_d_footstep);
  r_foot_traj_generator->SetFootstepGains(osc_running_gains_.K_d_footstep);
  l_foot_traj_generator->SetFootPlacementOffsets(
      osc_running_gains_.rest_length, osc_running_gains_.center_line_offset,
      osc_running_gains_.footstep_offset);
  r_foot_traj_generator->SetFootPlacementOffsets(
      osc_running_gains_.rest_length, osc_running_gains_.center_line_offset,
      osc_running_gains_.footstep_offset);

  pelvis_tracking_data = std::make_unique<TransTaskSpaceTrackingData>(
      "pelvis_trans_traj", osc_running_gains_.K_p_pelvis,
      osc_running_gains_.K_d_pelvis, osc_running_gains_.W_pelvis, plant, plant);
  stance_foot_tracking_data = std::make_unique<TransTaskSpaceTrackingData>(
      "stance_ft_traj", osc_running_gains_.K_p_swing_foot,
      osc_running_gains_.K_d_swing_foot, osc_running_gains_.W_swing_foot, plant,
      plant);
  left_foot_tracking_data = std::make_unique<TransTaskSpaceTrackingData>(
      "left_ft_traj", osc_running_gains_.K_p_swing_foot,
      osc_running_gains_.K_d_swing_foot, osc_running_gains_.W_swing_foot, plant,
      plant);
  right_foot_tracking_data = std::make_unique<TransTaskSpaceTrackingData>(
      "right_ft_traj", osc_running_gains_.K_p_swing_foot,
      osc_running_gains_.K_d_swing_foot, osc_running_gains_.W_swing_foot, plant,
      plant);
  pelvis_tracking_data->AddStateAndPointToTrack(left_stance_state, "pelvis");
  pelvis_tracking_data->AddStateAndPointToTrack(right_stance_state, "pelvis");
  stance_foot_tracking_data->AddStateAndPointToTrack(left_stance_state,
                                                     "toe_left");
  stance_foot_tracking_data->AddStateAndPointToTrack(right_stance_state,
                                                     "toe_right");
  left_foot_tracking_data->AddStateAndPointToTrack(right_stance_state,
                                                   "toe_left");
  right_foot_tracking_data->AddStateAndPointToTrack(left_stance_state,
                                                    "toe_right");
  left_foot_tracking_data->AddStateAndPointToTrack(left_touchdown_air_phase,
                                                   "toe_left");
  right_foot_tracking_data->AddStateAndPointToTrack(right_touchdown_air_phase,
                                                    "toe_right");

  left_foot_yz_tracking_data = std::make_unique<TransTaskSpaceTrackingData>(
      "left_ft_traj", osc_running_gains_.K_p_swing_foot,
      osc_running_gains_.K_d_swing_foot, osc_running_gains_.W_swing_foot, plant,
      plant);
  right_foot_yz_tracking_data = std::make_unique<TransTaskSpaceTrackingData>(
      "right_ft_traj", osc_running_gains_.K_p_swing_foot,
      osc_running_gains_.K_d_swing_foot, osc_running_gains_.W_swing_foot, plant,
      plant);
  left_foot_yz_tracking_data->AddStateAndPointToTrack(right_touchdown_air_phase,
                                                      "toe_left");
  right_foot_yz_tracking_data->AddStateAndPointToTrack(left_touchdown_air_phase,
                                                       "toe_right");

  left_hip_tracking_data = std::make_unique<TransTaskSpaceTrackingData>(
      "left_hip_traj", osc_running_gains_.K_p_swing_foot,
      osc_running_gains_.K_d_swing_foot, osc_running_gains_.W_swing_foot, plant,
      plant);
  right_hip_tracking_data = std::make_unique<TransTaskSpaceTrackingData>(
      "right_hip_traj", osc_running_gains_.K_p_swing_foot,
      osc_running_gains_.K_d_swing_foot, osc_running_gains_.W_swing_foot, plant,
      plant);
  left_hip_tracking_data->AddStateAndPointToTrack(right_stance_state, "pelvis");
  right_hip_tracking_data->AddStateAndPointToTrack(left_stance_state, "pelvis");
  right_hip_tracking_data->AddStateAndPointToTrack(right_touchdown_air_phase,
                                                   "pelvis");
  left_hip_tracking_data->AddStateAndPointToTrack(left_touchdown_air_phase,
                                                  "pelvis");

  left_hip_yz_tracking_data = std::make_unique<TransTaskSpaceTrackingData>(
      "left_hip_traj", osc_running_gains_.K_p_swing_foot,
      osc_running_gains_.K_d_swing_foot, osc_running_gains_.W_swing_foot, plant,
      plant);
  right_hip_yz_tracking_data = std::make_unique<TransTaskSpaceTrackingData>(
      "right_hip_traj", osc_running_gains_.K_p_swing_foot,
      osc_running_gains_.K_d_swing_foot, osc_running_gains_.W_swing_foot, plant,
      plant);
  left_hip_yz_tracking_data->AddStateAndPointToTrack(right_touchdown_air_phase,
                                                    "hip_left");
  right_hip_yz_tracking_data->AddStateAndPointToTrack(left_touchdown_air_phase,
                                                     "hip_right");

  left_foot_rel_tracking_data =
      std::make_unique<RelativeTranslationTrackingData>(
          "left_ft_traj", osc_running_gains_.K_p_swing_foot,
          osc_running_gains_.K_d_swing_foot, osc_running_gains_.W_swing_foot,
          plant, plant, left_foot_tracking_data.get(), left_hip_tracking_data.get());
  right_foot_rel_tracking_data =
      std::make_unique<RelativeTranslationTrackingData>(
          "right_ft_traj", osc_running_gains_.K_p_swing_foot,
          osc_running_gains_.K_d_swing_foot, osc_running_gains_.W_swing_foot,
          plant, plant, right_foot_tracking_data.get(), right_hip_tracking_data.get());
  left_foot_yz_rel_tracking_data =
      std::make_unique<RelativeTranslationTrackingData>(
          "left_ft_z_traj", osc_running_gains_.K_p_liftoff_swing_foot,
          osc_running_gains_.K_d_liftoff_swing_foot,
          osc_running_gains_.W_liftoff_swing_foot, plant, plant,
          left_foot_yz_tracking_data.get(), left_hip_yz_tracking_data.get());
  right_foot_yz_rel_tracking_data =
      std::make_unique<RelativeTranslationTrackingData>(
          "right_ft_z_traj", osc_running_gains_.K_p_liftoff_swing_foot,
          osc_running_gains_.K_d_liftoff_swing_foot,
          osc_running_gains_.W_liftoff_swing_foot, plant, plant,
          right_foot_yz_tracking_data.get(), right_hip_yz_tracking_data.get());
  pelvis_trans_rel_tracking_data =
      std::make_unique<RelativeTranslationTrackingData>(
          "pelvis_trans_traj", osc_running_gains_.K_p_pelvis,
          osc_running_gains_.K_d_pelvis, osc_running_gains_.W_pelvis, plant,
          plant, pelvis_tracking_data.get(), stance_foot_tracking_data.get());
  left_foot_rel_tracking_data->SetViewFrame(view_frame);
  right_foot_rel_tracking_data->SetViewFrame(view_frame);
  left_foot_yz_rel_tracking_data->SetViewFrame(view_frame);
  right_foot_yz_rel_tracking_data->SetViewFrame(view_frame);
  pelvis_trans_rel_tracking_data->SetViewFrame(view_frame);

  left_foot_rel_tracking_data->SetImpactInvariantProjection(true);
  right_foot_rel_tracking_data->SetImpactInvariantProjection(true);
  pelvis_trans_rel_tracking_data->SetImpactInvariantProjection(true);

//  std::cout << "here: " << std::endl;
  osc->AddTrackingData(std::move(pelvis_trans_rel_tracking_data));
  osc->AddTrackingData(std::move(left_foot_rel_tracking_data));
  osc->AddTrackingData(std::move(right_foot_rel_tracking_data));
  osc->AddTrackingData(std::move(left_foot_yz_rel_tracking_data));
  osc->AddTrackingData(std::move(right_foot_yz_rel_tracking_data));

//  std::cout << "here 1: " << std::endl;
  auto heading_traj_generator =
      builder.AddSystem<cassie::osc::HeadingTrajGenerator>(plant,
                                                           plant_context.get());

  pelvis_rot_tracking_data = std::make_unique<RotTaskSpaceTrackingData>(
      "pelvis_rot_traj", osc_running_gains_.K_p_pelvis_rot,
      osc_running_gains_.K_d_pelvis_rot, osc_running_gains_.W_pelvis_rot, plant,
      plant);
  pelvis_rot_tracking_data->AddStateAndFrameToTrack(left_stance_state, "pelvis");
  pelvis_rot_tracking_data->AddStateAndFrameToTrack(right_stance_state,
                                                   "pelvis");
  pelvis_rot_tracking_data->AddStateAndFrameToTrack(left_touchdown_air_phase,
                                                   "pelvis");
  pelvis_rot_tracking_data->AddStateAndFrameToTrack(right_touchdown_air_phase,
                                                   "pelvis");
  pelvis_rot_tracking_data->SetImpactInvariantProjection(true);
  osc->AddTrackingData(std::move(pelvis_rot_tracking_data));

  // Swing toe joint trajectory
//  vector<std::pair<const Vector3d, const Frame<double>&>> left_foot_points = {
//      left_heel, left_toe};
//  vector<std::pair<const Vector3d, const Frame<double>&>> right_foot_points = {
//      right_heel, right_toe};
  auto left_toe_angle_traj_gen = builder.AddSystem<SwingToeTrajGenerator>(
      plant, plant_context.get(), pos_map["toe_left"], left_foot_points,
      "left_toe_angle_traj");
  auto right_toe_angle_traj_gen = builder.AddSystem<SwingToeTrajGenerator>(
      plant, plant_context.get(), pos_map["toe_right"], right_foot_points,
      "right_toe_angle_traj");

  // Swing toe joint tracking
   left_toe_angle_tracking_data = std::make_unique<JointSpaceTrackingData>(
      "left_toe_angle_traj", osc_running_gains_.K_p_swing_toe,
      osc_running_gains_.K_d_swing_toe, osc_running_gains_.W_swing_toe, plant,
      plant);
   right_toe_angle_tracking_data = std::make_unique<JointSpaceTrackingData>(
      "right_toe_angle_traj", osc_running_gains_.K_p_swing_toe,
      osc_running_gains_.K_d_swing_toe, osc_running_gains_.W_swing_toe, plant,
      plant);
  left_toe_angle_tracking_data->AddStateAndJointToTrack(
      right_stance_state, "toe_left", "toe_leftdot");
  left_toe_angle_tracking_data->AddStateAndJointToTrack(
      right_touchdown_air_phase, "toe_left", "toe_leftdot");
  left_toe_angle_tracking_data->AddStateAndJointToTrack(
      left_touchdown_air_phase, "toe_left", "toe_leftdot");
  right_toe_angle_tracking_data->AddStateAndJointToTrack(
      left_stance_state, "toe_right", "toe_rightdot");
  right_toe_angle_tracking_data->AddStateAndJointToTrack(
      right_touchdown_air_phase, "toe_right", "toe_rightdot");
  right_toe_angle_tracking_data->AddStateAndJointToTrack(
      left_touchdown_air_phase, "toe_right", "toe_rightdot");
  left_toe_angle_tracking_data->SetImpactInvariantProjection(true);
  right_toe_angle_tracking_data->SetImpactInvariantProjection(true);
  osc->AddTrackingData(std::move(left_toe_angle_tracking_data));
  osc->AddTrackingData(std::move(right_toe_angle_tracking_data));

  // Swing hip yaw joint tracking
   left_hip_yaw_tracking_data = std::make_unique<JointSpaceTrackingData>(
      "hip_yaw_left_traj", osc_running_gains_.K_p_hip_yaw,
      osc_running_gains_.K_d_hip_yaw, osc_running_gains_.W_hip_yaw, plant,
      plant);
   right_hip_yaw_tracking_data = std::make_unique<JointSpaceTrackingData>(
      "hip_yaw_right_traj", osc_running_gains_.K_p_hip_yaw,
      osc_running_gains_.K_d_hip_yaw, osc_running_gains_.W_hip_yaw, plant,
      plant);
  left_hip_yaw_tracking_data->AddJointToTrack("hip_yaw_left", "hip_yaw_leftdot");
  right_hip_yaw_tracking_data->AddJointToTrack("hip_yaw_right",
                                              "hip_yaw_rightdot");
  //  left_hip_yaw_tracking_data.SetImpactInvariantProjection(true);
  //  right_hip_yaw_tracking_data.SetImpactInvariantProjection(true);
  osc->AddConstTrackingData(std::move(left_hip_yaw_tracking_data), VectorXd::Zero(1));
  osc->AddConstTrackingData(std::move(right_hip_yaw_tracking_data), VectorXd::Zero(1));

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
  builder.Connect(osc->get_osc_output_port(),
                  command_sender->get_input_port(0));

  // Publisher connections
  builder.ExportInput(state_receiver->get_input_port(), "x, u, t");
  builder.ExportInput(high_level_command->get_cassie_out_input_port(),
                      "lcmt_cassie_out");
  builder.ExportOutput(command_sender->get_output_port(), "u, t");
  //  builder.ExportOutput(failure_aggregator->get_status_output_port(),
  //  "failure_status");

  builder.BuildInto(this);
  this->set_name(("osc_running_controller_diagram"));
  DrawAndSaveDiagramGraph(*this);
  std::cout << "Built controller" << std::endl;
}

}  // namespace controllers
}  // namespace examples
}  // namespace dairlib
