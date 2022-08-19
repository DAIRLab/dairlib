#include "footstep_target_controller_diagram.h"

#include "common/find_resource.h"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/osc/hip_yaw_traj_gen.h"
#include "examples/Cassie/osc/heading_traj_generator.h"
#include "examples/Cassie/osc/high_level_command.h"
#include "examples/Cassie/osc/osc_walking_gains_alip.h"
#include "examples/Cassie/osc/swing_toe_traj_generator.h"
#include "multibody/kinematic/fixed_joint_evaluator.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/multibody_utils.h"
#include "systems/controllers/fsm_event_time.h"
#include "systems/controllers/alip_traj_gen.h"
#include "systems/controllers/osc/com_tracking_data.h"
#include "systems/controllers/osc/joint_space_tracking_data.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/controllers/osc/options_tracking_data.h"
#include "systems/controllers/osc/relative_translation_tracking_data.h"
#include "systems/controllers/osc/rot_space_tracking_data.h"
#include "systems/controllers/osc/trans_space_tracking_data.h"
#include "systems/controllers/alip_footstep_planner.h"
#include "systems/controllers/swing_foot_target_traj_gen.h"
#include "systems/controllers/time_based_fsm.h"
#include "systems/primitives/radio_parser.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/systems/framework/diagram_builder.h"

namespace dairlib {

using std::cout;
using std::endl;
using std::map;
using std::string;
using std::vector;

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::multibody::Frame;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;

using multibody::WorldYawViewFrame;
using systems::controllers::ComTrackingData;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::RelativeTranslationTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;

using multibody::FixedJointEvaluator;

using drake::trajectories::PiecewisePolynomial;

namespace examples {
namespace controllers {

FootstepTargetControllerDiagram::FootstepTargetControllerDiagram(
    drake::multibody::MultibodyPlant<double> &plant,
    bool has_double_stance,
    const std::string &osc_gains_filename,
    const std::string &osqp_settings_filename,
    double single_stance_time_override)
    : plant_(&plant),
      pos_map(multibody::MakeNameToPositionsMap(plant)),
      vel_map(multibody::MakeNameToVelocitiesMap(plant)),
      act_map(multibody::MakeNameToActuatorsMap(plant)),
      left_toe(LeftToeFront(plant)),
      left_heel(LeftToeRear(plant)),
      right_toe(RightToeFront(plant)),
      right_heel(RightToeRear(plant)),
      left_toe_mid(std::pair<const Vector3d, const Frame<double>&>(
          (left_toe.first + left_heel.first) / 2, plant.GetFrameByName("toe_left"))),
      right_toe_mid(std::pair<const Vector3d, const Frame<double>&>(
          (left_toe.first + left_heel.first) / 2, plant.GetFrameByName("toe_right"))),
      left_toe_origin(std::pair<const Vector3d, const Frame<double>&>(
          Vector3d::Zero(), plant.GetFrameByName("toe_left"))),
      right_toe_origin(std::pair<const Vector3d, const Frame<double>&>(
          Vector3d::Zero(), plant.GetFrameByName("toe_right"))),
      left_right_foot({left_toe_origin, right_toe_origin}),
      left_foot_points({left_heel, left_toe}),
      right_foot_points({right_heel, right_toe}),
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
                              vel_map.at("ankle_spring_joint_rightdot"), 0))
{

  /*** Build the controller diagram ***/
  DiagramBuilder<double> builder;
  plant_context = plant.CreateDefaultContext();

  auto gains = drake::yaml::LoadYamlFile<OSCWalkingGainsALIP>(
      FindResourceOrThrow(osc_gains_filename));

  if (single_stance_time_override > 0) {
    gains.ss_time = single_stance_time_override;
  }

  /*** FSM and contact mode configuration ***/
  int left_stance_state = 0;
  int right_stance_state = 1;
  int post_left_double_support_state = 3;
  int post_right_double_support_state = 4;
  double left_support_duration = gains.ss_time;
  double right_support_duration = gains.ss_time;
  double double_support_duration = gains.ds_time;
  if (has_double_stance) {
    fsm_states = {left_stance_state, post_left_double_support_state,
                  right_stance_state, post_right_double_support_state};
    state_durations = {left_support_duration, double_support_duration,
                       right_support_duration, double_support_duration};
    unordered_fsm_states = {left_stance_state, right_stance_state,
                            post_left_double_support_state,
                            post_right_double_support_state};
    unordered_state_durations = {left_support_duration, right_support_duration,
                                 double_support_duration,
                                 double_support_duration};
    contact_points_in_each_state.push_back({left_toe_mid});
    contact_points_in_each_state.push_back({right_toe_mid});
    contact_points_in_each_state.push_back({left_toe_mid});
    contact_points_in_each_state.push_back({right_toe_mid});
  } else {
    fsm_states = {left_stance_state, right_stance_state};
    state_durations = {left_support_duration, right_support_duration};
    unordered_fsm_states = {left_stance_state, right_stance_state};
    unordered_state_durations = {left_support_duration, right_support_duration};
    contact_points_in_each_state.push_back({left_toe_mid});
    contact_points_in_each_state.push_back({right_toe_mid});
  }
  single_support_states = {left_stance_state, right_stance_state};
  double_support_states = {post_left_double_support_state,
                           post_right_double_support_state};
  left_right_support_fsm_states = {left_stance_state, right_stance_state};
  left_right_support_state_durations =
      {left_support_duration, right_support_duration};

  // Swing foot tracking
  swing_ft_gain_multiplier_breaks = {0, left_support_duration / 2,
                                     left_support_duration};
  swing_ft_gain_multiplier_samples =
      {3, drake::MatrixX<double>::Identity(3, 3)};
  swing_ft_gain_multiplier_samples[2](2, 2) *= 0.3;
  swing_ft_gain_multiplier_gain_multiplier =
      PiecewisePolynomial<double>::FirstOrderHold(
          swing_ft_gain_multiplier_breaks, swing_ft_gain_multiplier_samples);
  swing_ft_accel_gain_multiplier_breaks = {0, left_support_duration / 2,
                                           left_support_duration * 3 / 4,
                                           left_support_duration};
  swing_ft_accel_gain_multiplier_samples = {
      4, drake::MatrixX<double>::Identity(3, 3)};
  swing_ft_accel_gain_multiplier_samples[2](2, 2) *= 0;
  swing_ft_accel_gain_multiplier_samples[3](2, 2) *= 0;
  swing_ft_accel_gain_multiplier_gain_multiplier =
      PiecewisePolynomial<double>::FirstOrderHold(
          swing_ft_accel_gain_multiplier_breaks,
          swing_ft_accel_gain_multiplier_samples);

  /*** Leaf systems ***/
  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);
  auto command_sender = builder.AddSystem<systems::RobotCommandSender>(plant);
  auto high_level_command =
      builder.AddSystem<cassie::osc::HighLevelCommand>(
          plant, plant_context.get(), gains.vel_scale_rot,
          gains.vel_scale_trans_sagital, gains.vel_scale_trans_lateral);
  auto fsm = builder.AddSystem<systems::TimeBasedFiniteStateMachine>(
      plant, fsm_states, state_durations);
  auto liftoff_event_time =
      builder.AddSystem<systems::FiniteStateMachineEventTime>(
          plant, single_support_states);
  auto touchdown_event_time =
      builder.AddSystem<systems::FiniteStateMachineEventTime>(
          plant, double_support_states);
  auto radio_parser = builder.AddSystem<systems::RadioParser>();
  touchdown_event_time->set_name("touchdown_time");
  liftoff_event_time->set_name("liftoff_time");

  // Trajectory Generators
  auto head_traj_gen = builder.AddSystem<cassie::osc::HeadingTrajGenerator>(
      plant, plant_context.get());
  auto alip_traj_generator = builder.AddSystem<systems::ALIPTrajGenerator>(
      plant, plant_context.get(), gains.lipm_height,
      unordered_fsm_states, unordered_state_durations,
      contact_points_in_each_state, gains.Q_alip_kalman_filter.asDiagonal(),
      gains.R_alip_kalman_filter.asDiagonal(), false, true);
  auto footstep_planner =
      builder.AddSystem<systems::AlipFootstepPlanner>(
          plant, plant_context.get(), left_right_support_fsm_states,
          left_right_support_state_durations, left_right_foot,
          double_support_duration,
          gains.max_CoM_to_footstep_dist, gains.footstep_offset,
          gains.center_line_offset);
  auto swing_ft_traj_generator =
      builder.AddSystem<systems::SwingFootTargetTrajGen>(
          plant, plant_context.get(), left_right_support_fsm_states,
          left_right_support_state_durations, left_right_foot,
          gains.mid_foot_height, gains.final_foot_height,
          gains.final_foot_velocity_z, false);
  auto left_toe_angle_traj_gen =
      builder.AddSystem<cassie::osc::SwingToeTrajGenerator>(
          plant, plant_context.get(), pos_map["toe_left"],
          left_foot_points, "left_toe_angle_traj");
  auto right_toe_angle_traj_gen =
      builder.AddSystem<cassie::osc::SwingToeTrajGenerator>(
          plant, plant_context.get(), pos_map["toe_right"],
          right_foot_points, "right_toe_angle_traj");
  auto hip_yaw_traj_gen =
      builder.AddSystem<cassie::HipYawTrajGen>(left_stance_state);

  // Create Operational space control
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      plant, plant, plant_context.get(), plant_context.get(), true,
      false, 0);

  // Cost
  int n_v = plant.num_velocities();
  int n_u = plant.num_actuators();
  MatrixXd Q_accel = gains.w_accel * MatrixXd::Identity(n_v, n_v);
  osc->SetAccelerationCostWeights(Q_accel);
  osc->SetInputSmoothingWeights(
      gains.w_input_reg * MatrixXd::Identity(n_u, n_u));

  // Constraints in OSC
  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);
  evaluators.add_evaluator(&left_fixed_knee_spring);
  evaluators.add_evaluator(&right_fixed_knee_spring);
  evaluators.add_evaluator(&left_fixed_ankle_spring);
  evaluators.add_evaluator(&right_fixed_ankle_spring);
  osc->AddKinematicConstraint(&evaluators);

  // Soft constraint
  // w_contact_relax shouldn't be too big, cause we want tracking error to be
  // important
  osc->SetContactSoftConstraintWeight(gains.w_soft_constraint);
  // Friction coefficient
  osc->SetContactFriction(gains.mu);

  // Add contact points (The position doesn't matter. It's not used in OSC)
  const auto& pelvis = plant.GetBodyByName("pelvis");
  osc->AddStateAndContactPoint(left_stance_state, &left_toe_evaluator);
  osc->AddStateAndContactPoint(left_stance_state, &left_heel_evaluator);
  osc->AddStateAndContactPoint(right_stance_state, &right_toe_evaluator);
  osc->AddStateAndContactPoint(right_stance_state, &right_heel_evaluator);

  if (has_double_stance) {
    osc->AddStateAndContactPoint(post_left_double_support_state,
                                 &left_toe_evaluator);
    osc->AddStateAndContactPoint(post_left_double_support_state,
                                 &left_heel_evaluator);
    osc->AddStateAndContactPoint(post_left_double_support_state,
                                 &right_toe_evaluator);
    osc->AddStateAndContactPoint(post_left_double_support_state,
                                 &right_heel_evaluator);
    osc->AddStateAndContactPoint(post_right_double_support_state,
                                 &left_toe_evaluator);
    osc->AddStateAndContactPoint(post_right_double_support_state,
                                 &left_heel_evaluator);
    osc->AddStateAndContactPoint(post_right_double_support_state,
                                 &right_toe_evaluator);
    osc->AddStateAndContactPoint(post_right_double_support_state,
                                 &right_heel_evaluator);
  }

  /*** Tracking Datas ***/
  swing_foot_data = std::make_unique<TransTaskSpaceTrackingData>(
      "swing_ft_traj", gains.K_p_swing_foot, gains.K_d_swing_foot,
      gains.W_swing_foot, plant, plant);
  swing_foot_data->AddStateAndPointToTrack(left_stance_state, "toe_right");
  swing_foot_data->AddStateAndPointToTrack(right_stance_state, "toe_left");

  swing_foot_data->SetTimeVaryingGains(
      swing_ft_gain_multiplier_gain_multiplier);
  swing_foot_data->SetFeedforwardAccelMultiplier(
      swing_ft_accel_gain_multiplier_gain_multiplier);

  /*
  com_data = std::make_unique<ComTrackingData>(
      "com_data", gains.K_p_swing_foot, gains.K_d_swing_foot,
      gains.W_swing_foot, plant, plant);
  com_data->AddFiniteStateToTrack(left_stance_state);
  com_data->AddFiniteStateToTrack(right_stance_state);
  swing_ft_traj_local = std::make_unique<RelativeTranslationTrackingData>(
      "swing_ft_traj", gains.K_p_swing_foot, gains.K_d_swing_foot,
      gains.W_swing_foot, plant, plant, swing_foot_data.get(),
      com_data.get());
  WorldYawViewFrame pelvis_view_frame(plant.GetBodyByName("pelvis"));
  swing_ft_traj_local->SetViewFrame(view_frame);

  swing_ft_traj_local->SetTimeVaryingGains(
      swing_ft_gain_multiplier_gain_multiplier);
  swing_ft_traj_local->SetFeedforwardAccelMultiplier(
      swing_ft_accel_gain_multiplier_gain_multiplier); */
  osc->AddTrackingData(swing_foot_data.get());

  center_of_mass_traj = std::make_unique<ComTrackingData>(
      "alip_com_traj", gains.K_p_com, gains.K_d_com, gains.W_com, plant, plant);

  // FiniteStatesToTrack cannot be empty
  center_of_mass_traj->AddFiniteStateToTrack(-1);
  osc->AddTrackingData(center_of_mass_traj.get());

  // Pelvis rotation tracking (pitch and roll)
  pelvis_balance_traj = std::make_unique<RotTaskSpaceTrackingData>(
      "pelvis_balance_traj", gains.K_p_pelvis_balance, gains.K_d_pelvis_balance,
      gains.W_pelvis_balance, plant, plant);
  pelvis_balance_traj->AddFrameToTrack("pelvis");
  osc->AddTrackingData(pelvis_balance_traj.get());
  // Pelvis rotation tracking (yaw)
  pelvis_heading_traj = std::make_unique<RotTaskSpaceTrackingData>(
      "pelvis_heading_traj", gains.K_p_pelvis_heading, gains.K_d_pelvis_heading,
      gains.W_pelvis_heading, plant, plant);
  pelvis_heading_traj->AddFrameToTrack("pelvis");
  osc->AddTrackingData(pelvis_heading_traj.get(),
                       gains.period_of_no_heading_control);

  // Swing toe joint tracking
  swing_toe_traj_left = std::make_unique<JointSpaceTrackingData>(
      "left_toe_angle_traj", gains.K_p_swing_toe, gains.K_d_swing_toe,
      gains.W_swing_toe, plant, plant);
  swing_toe_traj_right = std::make_unique<JointSpaceTrackingData>(
      "right_toe_angle_traj", gains.K_p_swing_toe, gains.K_d_swing_toe,
      gains.W_swing_toe, plant, plant);
  swing_toe_traj_right->AddStateAndJointToTrack(
      left_stance_state, "toe_right","toe_rightdot");
  swing_toe_traj_left->AddStateAndJointToTrack(
      right_stance_state, "toe_left","toe_leftdot");
  osc->AddTrackingData(swing_toe_traj_left.get());
  osc->AddTrackingData(swing_toe_traj_right.get());

  // Swing hip yaw joint tracking
  swing_hip_yaw_traj = std::make_unique<JointSpaceTrackingData>(
      "swing_hip_yaw_traj", gains.K_p_hip_yaw, gains.K_d_hip_yaw,
      gains.W_hip_yaw, plant, plant);
  swing_hip_yaw_traj->AddStateAndJointToTrack(left_stance_state,
                                              "hip_yaw_right","hip_yaw_rightdot");
  swing_hip_yaw_traj->AddStateAndJointToTrack(right_stance_state,
                                              "hip_yaw_left","hip_yaw_leftdot");
  osc->AddTrackingData(swing_hip_yaw_traj.get());

  // Set double support duration for force blending
  osc->SetUpDoubleSupportPhaseBlending(
      double_support_duration, left_stance_state, right_stance_state,
      {post_left_double_support_state, post_right_double_support_state});

  // BUILD OSC
  osc->SetOsqpSolverOptionsFromYaml(
      "examples/Cassie/osc/solver_settings/osqp_options_walking.yaml");

  if (gains.W_com(0,0) == 0){
    osc->SetInputCostWeightForJointAndFsmState(
        "toe_left_motor", left_stance_state, 1.0);
    osc->SetInputCostWeightForJointAndFsmState(
        "toe_left_motor", post_right_double_support_state, 1.0);
    osc->SetInputCostWeightForJointAndFsmState(
        "toe_right_motor", right_stance_state, 1.0);
    osc->SetInputCostWeightForJointAndFsmState(
        "toe_right_motor", post_left_double_support_state, 1.0);
  }
  osc->Build();

  /*** Connect input and output ports ***/
  // Connect state receiver output
  builder.Connect(state_receiver->get_output_port(),
                  osc->get_robot_output_input_port());
  builder.Connect(state_receiver->get_output_port(),
                  fsm->get_input_port_state());
  builder.Connect(state_receiver->get_output_port(),
                  alip_traj_generator->get_input_port_state());
  builder.Connect(state_receiver->get_output_port(),
                  swing_ft_traj_generator->get_input_port_state());
  builder.Connect(state_receiver->get_output_port(),
                  high_level_command->get_state_input_port());
  builder.Connect(state_receiver->get_output_port(),
                  left_toe_angle_traj_gen->get_state_input_port());
  builder.Connect(state_receiver->get_output_port(),
                  right_toe_angle_traj_gen->get_state_input_port());
  builder.Connect(state_receiver->get_output_port(),
                  head_traj_gen->get_state_input_port());
  builder.Connect(state_receiver->get_output_port(),
                  liftoff_event_time->get_input_port_state());
  builder.Connect(state_receiver->get_output_port(),
                  touchdown_event_time->get_input_port_state());
  builder.Connect(state_receiver->get_output_port(),
                  footstep_planner->get_input_port_state());

  // Connect fsm output port
  builder.Connect(fsm->get_output_port_fsm(),
                  osc->get_fsm_input_port());
  builder.Connect(fsm->get_output_port_fsm(),
                  liftoff_event_time->get_input_port_fsm());
  builder.Connect(fsm->get_output_port_fsm(),
                  touchdown_event_time->get_input_port_fsm());
  builder.Connect(fsm->get_output_port_fsm(),
                  alip_traj_generator->get_input_port_fsm());
  builder.Connect(fsm->get_output_port_fsm(),
                  swing_ft_traj_generator->get_input_port_fsm());
  builder.Connect(fsm->get_output_port_fsm(),
                  hip_yaw_traj_gen->get_fsm_input_port());
  builder.Connect(fsm->get_output_port_fsm(),
                  footstep_planner->get_input_port_fsm());

  // Connect radio
  builder.Connect(radio_parser->get_output_port(),
                  high_level_command->get_radio_port());
  builder.Connect(radio_parser->get_output_port(),
                  hip_yaw_traj_gen->get_radio_input_port());

  // Connect footstep planning pipeline
  builder.Connect(touchdown_event_time->get_output_port_event_time(),
                  alip_traj_generator->get_input_port_touchdown_time());
  builder.Connect(liftoff_event_time->get_output_port_event_time_of_interest(),
                  swing_ft_traj_generator->get_input_port_fsm_switch_time());
  builder.Connect(high_level_command->get_yaw_output_port(),
                  head_traj_gen->get_yaw_input_port());
  builder.Connect(alip_traj_generator->get_output_port_alip_state(),
                  footstep_planner->get_input_port_alip_state());
  builder.Connect(high_level_command->get_xy_output_port(),
                  footstep_planner->get_input_port_vdes());
  builder.Connect(liftoff_event_time->get_output_port_event_time_of_interest(),
                  footstep_planner->get_input_port_fsm_switch_time());


  builder.Connect(alip_traj_generator->get_output_port_com(),
                  osc->get_tracking_data_input_port("alip_com_traj"));
  builder.Connect(swing_ft_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("swing_ft_traj"));
  builder.Connect(head_traj_gen->get_output_port(0),
                  osc->get_tracking_data_input_port("pelvis_heading_traj"));
  builder.Connect(head_traj_gen->get_output_port(0),
                  osc->get_tracking_data_input_port("pelvis_balance_traj"));
  builder.Connect(left_toe_angle_traj_gen->get_output_port(0),
                  osc->get_tracking_data_input_port("left_toe_angle_traj"));
  builder.Connect(right_toe_angle_traj_gen->get_output_port(0),
                  osc->get_tracking_data_input_port("right_toe_angle_traj"));
  builder.Connect(hip_yaw_traj_gen->get_hip_yaw_output_port(),
                  osc->get_tracking_data_input_port("swing_hip_yaw_traj"));
  builder.Connect(osc->get_output_port(0),
                  command_sender->get_input_port(0));

  builder.ExportInput(state_receiver->get_input_port(), "x, u, t");
  builder.ExportInput(radio_parser->get_input_port(), "raw_radio");
  builder.ExportInput(swing_ft_traj_generator->get_input_port_footstep_target(), "footstep_target");
  builder.ExportInput(alip_traj_generator->get_input_port_target_com_z(), "target_com_z_at_touchdown");
  builder.ExportOutput(command_sender->get_output_port(), "lcmt_robot_input");
  builder.ExportOutput(osc->get_osc_output_port(), "u, t");
  builder.ExportOutput(fsm->get_output_port_fsm(), "fsm");
  builder.ExportOutput(footstep_planner->get_output_port(), "alip_target");

  // Create the diagram
  builder.BuildInto(this);
  this->set_name(("ALIP_walking_controller_diagram"));
  //  std::cout << "Built controller" << std::endl;

}
}
}
}