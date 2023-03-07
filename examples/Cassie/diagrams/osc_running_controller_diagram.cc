#include "osc_running_controller_diagram.h"

#include <drake/common/yaml/yaml_io.h>
#include <drake/multibody/parsing/parser.h>
#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/contact_scheduler/contact_scheduler.h"
#include "examples/Cassie/osc/heading_traj_generator.h"
#include "examples/Cassie/osc/high_level_command.h"
#include "examples/Cassie/osc/swing_toe_traj_generator.h"
#include "examples/Cassie/osc_run/foot_traj_generator.h"
#include "examples/Cassie/osc_run/osc_running_gains.h"
#include "examples/Cassie/osc_run/pelvis_trans_traj_generator.h"
#include "multibody/kinematic/fixed_joint_evaluator.h"
#include "multibody/multibody_utils.h"
#include "systems/controllers/controller_failure_aggregator.h"
#include "systems/controllers/osc/joint_space_tracking_data.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/controllers/osc/relative_translation_tracking_data.h"
#include "systems/controllers/osc/rot_space_tracking_data.h"
#include "systems/controllers/osc/trans_space_tracking_data.h"
#include "systems/filters/floating_base_velocity_filter.h"
#include "systems/primitives/radio_parser.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"
#include "yaml-cpp/yaml.h"

#include "drake/common/yaml/yaml_read_archive.h"
#include "drake/systems/framework/diagram_builder.h"

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
using drake::trajectories::PiecewisePolynomial;
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
    drake::multibody::MultibodyPlant<double>& plant,
    const string& osc_running_gains_filename, const string& osqp_settings_filename)
    : plant_(&plant),
      pos_map(multibody::MakeNameToPositionsMap(plant)),
      vel_map(multibody::MakeNameToVelocitiesMap(plant)),
      act_map(multibody::MakeNameToActuatorsMap(plant)),
      left_toe(LeftToeFront(plant)),
      left_heel(LeftToeRear(plant)),
      right_toe(RightToeFront(plant)),
      right_heel(RightToeRear(plant)),
      left_foot_points({left_heel, left_toe}),
      right_foot_points({right_heel, right_toe}),
      view_frame(std::make_shared<multibody::WorldYawViewFrame<double>>(
          plant.GetBodyByName("pelvis"))),
      left_toe_evaluator(multibody::WorldPointEvaluator(
          plant, left_toe.first, left_toe.second, *view_frame,
          Matrix3d::Identity(), Vector3d::Zero(), {1, 2})),
      left_heel_evaluator(multibody::WorldPointEvaluator(
          plant, left_heel.first, left_heel.second, *view_frame,
          Matrix3d::Identity(), Vector3d::Zero(), {0, 1, 2})),
      right_toe_evaluator(multibody::WorldPointEvaluator(
          plant, right_toe.first, right_toe.second, *view_frame,
          Matrix3d::Identity(), Vector3d::Zero(), {1, 2})),
      right_heel_evaluator(multibody::WorldPointEvaluator(
          plant, right_heel.first, right_heel.second, *view_frame,
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
                              vel_map.at("ankle_spring_joint_rightdot"), 0)) {
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
  drake::yaml::LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;

  OSCGains gains = drake::yaml::LoadYamlFile<OSCGains>(
      FindResourceOrThrow(osc_running_gains_filename), {}, {}, yaml_options);
  OSCRunningGains osc_running_gains =
      drake::yaml::LoadYamlFile<OSCRunningGains>(
          FindResourceOrThrow(osc_running_gains_filename));

  /**** FSM and contact mode configuration ****/
  vector<int> fsm_states = {
      RUNNING_FSM_STATE::LEFT_STANCE, RUNNING_FSM_STATE::LEFT_FLIGHT,
      RUNNING_FSM_STATE::RIGHT_STANCE, RUNNING_FSM_STATE::RIGHT_FLIGHT,
      RUNNING_FSM_STATE::LEFT_STANCE};

  vector<double> state_durations = {osc_running_gains.stance_duration,
                                    osc_running_gains.flight_duration,
                                    osc_running_gains.stance_duration,
                                    osc_running_gains.flight_duration, 0.0};
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
  contact_scheduler->SetSLIPParams(osc_running_gains.rest_length);
  contact_scheduler->SetNominalStepTimings(osc_running_gains.stance_duration,
                                           osc_running_gains.flight_duration);
  contact_scheduler->SetMaxStepTimingVariance(
      osc_running_gains.stance_variance, osc_running_gains.flight_variance);

  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);
  auto command_sender = builder.AddSystem<systems::RobotCommandSender>(plant);
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      plant, plant, plant_context.get(), plant_context.get(), true);
  auto radio_parser = builder.AddSystem<systems::RadioParser>();
  auto failure_aggregator =
      builder.AddSystem<systems::ControllerFailureAggregator>(
          control_channel_name_, 1);

  /**** OSC setup ****/
  /// REGULARIZATION COSTS
  osc->SetAccelerationCostWeights(osc_running_gains.w_accel * osc_running_gains.W_acceleration);
  osc->SetInputSmoothingCostWeights(osc_running_gains.w_input_reg *
      osc_running_gains.W_input_regularization);
  osc->SetInputCostWeights(osc_running_gains.w_input *
      osc_running_gains.W_input_regularization);
  osc->SetLambdaContactRegularizationWeight(
      osc_running_gains.w_lambda * osc_running_gains.W_lambda_c_regularization);
  osc->SetLambdaHolonomicRegularizationWeight(
      osc_running_gains.w_lambda * osc_running_gains.W_lambda_h_regularization);
  // Soft constraint on contacts
  osc->SetContactSoftConstraintWeight(osc_running_gains.w_soft_constraint);
  osc->SetJointLimitWeight(osc_running_gains.w_joint_limit);

  // Contact information for OSC
  osc->SetContactFriction(osc_running_gains.mu);

  osc->AddStateAndContactPoint(RUNNING_FSM_STATE::LEFT_STANCE,
                               &left_toe_evaluator);
  osc->AddStateAndContactPoint(RUNNING_FSM_STATE::LEFT_STANCE,
                               &left_heel_evaluator);
  osc->AddStateAndContactPoint(RUNNING_FSM_STATE::RIGHT_STANCE,
                               &right_toe_evaluator);
  osc->AddStateAndContactPoint(RUNNING_FSM_STATE::RIGHT_STANCE,
                               &right_heel_evaluator);

  // Fix the springs in the dynamics
  evaluators.add_evaluator(&left_fixed_knee_spring);
  evaluators.add_evaluator(&right_fixed_knee_spring);
  evaluators.add_evaluator(&left_fixed_ankle_spring);
  evaluators.add_evaluator(&right_fixed_ankle_spring);
  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);


  osc->AddKinematicConstraint(&evaluators);

  /**** Tracking Data *****/

  cassie::osc::HighLevelCommand* high_level_command;
  high_level_command = builder.AddSystem<cassie::osc::HighLevelCommand>(
      plant, plant_context.get(), osc_running_gains.vel_scale_rot,
      osc_running_gains.vel_scale_trans_sagital,
      osc_running_gains.vel_scale_trans_lateral);

  auto pelvis_trans_traj_generator =
      builder.AddSystem<PelvisTransTrajGenerator>(
          plant, plant_context.get(), feet_contact_points,
          osc_running_gains.relative_pelvis);
  pelvis_trans_traj_generator->SetSLIPParams(
      osc_running_gains.rest_length, osc_running_gains.rest_length_offset);
  auto l_foot_traj_generator = builder.AddSystem<FootTrajGenerator>(
      plant, plant_context.get(), "toe_left", "pelvis", LEFT_STANCE);
  auto r_foot_traj_generator = builder.AddSystem<FootTrajGenerator>(
      plant, plant_context.get(), "toe_right", "pelvis", RIGHT_STANCE);
  l_foot_traj_generator->SetFootstepGains(osc_running_gains.K_d_footstep);
  r_foot_traj_generator->SetFootstepGains(osc_running_gains.K_d_footstep);
  l_foot_traj_generator->SetFootPlacementOffsets(
      osc_running_gains.rest_length, osc_running_gains.rest_length_offset,
      osc_running_gains.footstep_lateral_offset,
      osc_running_gains.footstep_sagital_offset,
      osc_running_gains.mid_foot_height);
  r_foot_traj_generator->SetFootPlacementOffsets(
      osc_running_gains.rest_length, osc_running_gains.rest_length_offset,
      osc_running_gains.footstep_lateral_offset,
      osc_running_gains.footstep_sagital_offset,
      osc_running_gains.mid_foot_height);

  pelvis_tracking_data = std::make_unique<TransTaskSpaceTrackingData>(
      "pelvis_trans_traj", osc_running_gains.K_p_pelvis,
      osc_running_gains.K_d_pelvis, osc_running_gains.W_pelvis, plant, plant);
  stance_foot_tracking_data = std::make_unique<TransTaskSpaceTrackingData>(
      "stance_ft_traj", osc_running_gains.K_p_swing_foot,
      osc_running_gains.K_d_swing_foot, osc_running_gains.W_swing_foot, plant,
      plant);
  left_foot_tracking_data = std::make_unique<TransTaskSpaceTrackingData>(
      "left_ft_traj", osc_running_gains.K_p_swing_foot,
      osc_running_gains.K_d_swing_foot, osc_running_gains.W_swing_foot, plant,
      plant);
  right_foot_tracking_data = std::make_unique<TransTaskSpaceTrackingData>(
      "right_ft_traj", osc_running_gains.K_p_swing_foot,
      osc_running_gains.K_d_swing_foot, osc_running_gains.W_swing_foot, plant,
      plant);
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

  left_foot_tracking_data->AddStateAndPointToTrack(
      RUNNING_FSM_STATE::LEFT_FLIGHT, "toe_left");
  right_foot_tracking_data->AddStateAndPointToTrack(
      RUNNING_FSM_STATE::RIGHT_FLIGHT, "toe_right");

  left_hip_tracking_data = std::make_unique<TransTaskSpaceTrackingData>(
      "left_hip_traj", osc_running_gains.K_p_swing_foot,
      osc_running_gains.K_d_swing_foot, osc_running_gains.W_swing_foot, plant,
      plant);
  right_hip_tracking_data = std::make_unique<TransTaskSpaceTrackingData>(
      "right_hip_traj", osc_running_gains.K_p_swing_foot,
      osc_running_gains.K_d_swing_foot, osc_running_gains.W_swing_foot, plant,
      plant);
  left_hip_tracking_data->AddStateAndPointToTrack(
      RUNNING_FSM_STATE::RIGHT_STANCE, "pelvis");
  right_hip_tracking_data->AddStateAndPointToTrack(
      RUNNING_FSM_STATE::LEFT_STANCE, "pelvis");
  right_hip_tracking_data->AddStateAndPointToTrack(
      RUNNING_FSM_STATE::LEFT_FLIGHT, "pelvis");
  left_hip_tracking_data->AddStateAndPointToTrack(
      RUNNING_FSM_STATE::RIGHT_FLIGHT, "pelvis");

  left_hip_tracking_data->AddStateAndPointToTrack(
      RUNNING_FSM_STATE::LEFT_FLIGHT, "pelvis");
  right_hip_tracking_data->AddStateAndPointToTrack(
      RUNNING_FSM_STATE::RIGHT_FLIGHT, "pelvis");

  left_foot_rel_tracking_data =
      std::make_unique<RelativeTranslationTrackingData>(
          "left_ft_traj", osc_running_gains.K_p_swing_foot,
          osc_running_gains.K_d_swing_foot, osc_running_gains.W_swing_foot,
          plant, plant, left_foot_tracking_data.get(),
          left_hip_tracking_data.get());
  right_foot_rel_tracking_data =
      std::make_unique<RelativeTranslationTrackingData>(
          "right_ft_traj", osc_running_gains.K_p_swing_foot,
          osc_running_gains.K_d_swing_foot, osc_running_gains.W_swing_foot,
          plant, plant, right_foot_tracking_data.get(),
          right_hip_tracking_data.get());
  pelvis_trans_rel_tracking_data =
      std::make_unique<RelativeTranslationTrackingData>(
          "pelvis_trans_traj", osc_running_gains.K_p_pelvis,
          osc_running_gains.K_d_pelvis, osc_running_gains.W_pelvis, plant,
          plant, pelvis_tracking_data.get(), stance_foot_tracking_data.get());
  left_foot_rel_tracking_data->SetViewFrame(view_frame);
  right_foot_rel_tracking_data->SetViewFrame(view_frame);
  pelvis_trans_rel_tracking_data->SetViewFrame(view_frame);

  auto foot_traj_weight_trajectory =
      std::make_shared<PiecewisePolynomial<double>>(
          PiecewisePolynomial<double>::FirstOrderHold(
              {0, osc_running_gains.stance_duration +
                      2 * osc_running_gains.flight_duration},
              {0.5 * VectorXd::Ones(1), 4.0 * VectorXd::Ones(1)}));
  auto foot_traj_gain_trajectory =
      std::make_shared<PiecewisePolynomial<double>>(
          PiecewisePolynomial<double>::FirstOrderHold(
              {0, osc_running_gains.stance_duration +
                      2 * osc_running_gains.flight_duration},
              {0.5 * MatrixXd::Identity(3, 3),
               1.5 * MatrixXd::Identity(3, 3)}));
  left_foot_rel_tracking_data->SetTimeVaryingWeights(
      foot_traj_weight_trajectory);
  right_foot_rel_tracking_data->SetTimeVaryingWeights(
      foot_traj_weight_trajectory);
  left_foot_rel_tracking_data->SetTimeVaryingPDGainMultiplier(
      foot_traj_gain_trajectory);
  right_foot_rel_tracking_data->SetTimeVaryingPDGainMultiplier(
      foot_traj_gain_trajectory);

  auto heading_traj_generator =
      builder.AddSystem<cassie::osc::HeadingTrajGenerator>(plant,
                                                           plant_context.get());

  pelvis_rot_tracking_data = std::make_unique<RotTaskSpaceTrackingData>(
      "pelvis_rot_traj", osc_running_gains.K_p_pelvis_rot,
      osc_running_gains.K_d_pelvis_rot, osc_running_gains.W_pelvis_rot, plant,
      plant);
  pelvis_rot_tracking_data->AddStateAndFrameToTrack(
      RUNNING_FSM_STATE::LEFT_STANCE, "pelvis");
  pelvis_rot_tracking_data->AddStateAndFrameToTrack(
      RUNNING_FSM_STATE::RIGHT_STANCE, "pelvis");
  pelvis_rot_tracking_data->AddStateAndFrameToTrack(
      RUNNING_FSM_STATE::RIGHT_FLIGHT, "pelvis");
  pelvis_rot_tracking_data->AddStateAndFrameToTrack(
      RUNNING_FSM_STATE::LEFT_FLIGHT, "pelvis");

  if (osc_running_gains.rot_filter_tau > 0) {
    pelvis_rot_tracking_data->SetLowPassFilter(osc_running_gains.rot_filter_tau,
                                               {0, 1, 2});
  }
  pelvis_rot_tracking_data->SetViewFrame(view_frame);

  // Swing toe joint trajectory
  auto left_toe_angle_traj_gen = builder.AddSystem<SwingToeTrajGenerator>(
      plant, plant_context.get(), pos_map["toe_left"], left_foot_points,
      "left_toe_angle_traj");
  auto right_toe_angle_traj_gen = builder.AddSystem<SwingToeTrajGenerator>(
      plant, plant_context.get(), pos_map["toe_right"], right_foot_points,
      "right_toe_angle_traj");

  // Swing toe joint tracking
  left_toe_angle_tracking_data = std::make_unique<JointSpaceTrackingData>(
      "left_toe_angle_traj", osc_running_gains.K_p_swing_toe,
      osc_running_gains.K_d_swing_toe, osc_running_gains.W_swing_toe, plant,
      plant);
  right_toe_angle_tracking_data = std::make_unique<JointSpaceTrackingData>(
      "right_toe_angle_traj", osc_running_gains.K_p_swing_toe,
      osc_running_gains.K_d_swing_toe, osc_running_gains.W_swing_toe, plant,
      plant);
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

  // Swing hip yaw joint tracking
  left_hip_yaw_tracking_data = std::make_unique<JointSpaceTrackingData>(
      "hip_yaw_left_traj", osc_running_gains.K_p_hip_yaw,
      osc_running_gains.K_d_hip_yaw, osc_running_gains.W_hip_yaw, plant, plant);
  right_hip_yaw_tracking_data = std::make_unique<JointSpaceTrackingData>(
      "hip_yaw_right_traj", osc_running_gains.K_p_hip_yaw,
      osc_running_gains.K_d_hip_yaw, osc_running_gains.W_hip_yaw, plant, plant);
  left_hip_yaw_tracking_data->AddJointToTrack("hip_yaw_left",
                                              "hip_yaw_leftdot");
  right_hip_yaw_tracking_data->AddJointToTrack("hip_yaw_right",
                                               "hip_yaw_rightdot");

  left_foot_rel_tracking_data->SetImpactInvariantProjection(true);
  right_foot_rel_tracking_data->SetImpactInvariantProjection(true);
  pelvis_trans_rel_tracking_data->SetImpactInvariantProjection(true);
  left_hip_yaw_tracking_data->SetImpactInvariantProjection(true);
  right_hip_yaw_tracking_data->SetImpactInvariantProjection(true);
  pelvis_rot_tracking_data->SetImpactInvariantProjection(true);
  left_toe_angle_tracking_data->SetImpactInvariantProjection(true);
  right_toe_angle_tracking_data->SetImpactInvariantProjection(true);
  osc->AddTrackingData(std::move(pelvis_rot_tracking_data));
  osc->AddTrackingData(std::move(pelvis_trans_rel_tracking_data));
  osc->AddTrackingData(std::move(left_foot_rel_tracking_data));
  osc->AddTrackingData(std::move(right_foot_rel_tracking_data));
  osc->AddTrackingData(std::move(left_toe_angle_tracking_data));
  osc->AddTrackingData(std::move(right_toe_angle_tracking_data));
  osc->AddConstTrackingData(std::move(left_hip_yaw_tracking_data),
                            VectorXd::Zero(1));
  osc->AddConstTrackingData(std::move(right_hip_yaw_tracking_data),
                            VectorXd::Zero(1));

  // Build OSC problem
  osc->SetOsqpSolverOptionsFromYaml(osqp_settings_filename);
  osc->Build();

  /*****Connect ports*****/

  // OSC connections
  builder.Connect(contact_scheduler->get_output_port_fsm(),
                  osc->get_input_port_fsm());
  builder.Connect(contact_scheduler->get_output_port_impact_info(),
                  osc->get_input_port_impact_info());
  builder.Connect(contact_scheduler->get_output_port_clock(),
                  osc->get_input_port_clock());
  builder.Connect(state_receiver->get_output_port(0),
                  osc->get_input_port_robot_output());
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
  builder.Connect(radio_parser->get_output_port(),
                  l_foot_traj_generator->get_input_port_radio());
  builder.Connect(radio_parser->get_output_port(),
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
  //  builder.Connect(l_foot_traj_generator->get_output_port(0),
  //                  osc->get_input_port_tracking_data("left_ft_z_traj"));
  //  builder.Connect(r_foot_traj_generator->get_output_port(0),
  //                  osc->get_input_port_tracking_data("right_ft_z_traj"));
  builder.Connect(left_toe_angle_traj_gen->get_output_port(0),
                  osc->get_input_port_tracking_data("left_toe_angle_traj"));
  builder.Connect(right_toe_angle_traj_gen->get_output_port(0),
                  osc->get_input_port_tracking_data("right_toe_angle_traj"));
  builder.Connect(osc->get_output_port_osc_command(),
                  command_sender->get_input_port(0));
  builder.Connect(radio_parser->get_output_port(),
                  high_level_command->get_radio_input_port());

  // Publisher connections
  builder.ExportInput(state_receiver->get_input_port(), "lcmt_robot_output");
  builder.ExportInput(radio_parser->get_input_port(), "raw_radio");
  builder.ExportOutput(command_sender->get_output_port(), "lcmt_robot_input");
  builder.ExportOutput(osc->get_output_port_osc_command(), "u, t");
  builder.ExportOutput(failure_aggregator->get_status_output_port(),
                       "lcmt_controller_failure");

  builder.BuildInto(this);
  this->set_name(("osc_running_controller_diagram"));
  DrawAndSaveDiagramGraph(*this);
}

}  // namespace controllers
}  // namespace examples
}  // namespace dairlib
