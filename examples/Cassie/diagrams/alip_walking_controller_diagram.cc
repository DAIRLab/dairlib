#include "alip_walking_controller_diagram.h"

#include "common/find_resource.h"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_utils.h"
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
#include "systems/filters/floating_base_velocity_filter.h"
#include "systems/primitives/radio_parser.h"
#include "systems/robot_lcm_systems.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_scope_system.h"

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
//using drake::systems::lcm::LcmPublisherSystem;
//using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::LcmScopeSystem;
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

AlipWalkingControllerDiagram::AlipWalkingControllerDiagram(
    drake::multibody::MultibodyPlant<double> &plant,
    bool has_double_stance,
    const std::string &osc_gains_filename,
    const std::string &osqp_settings_filename)
    : plant_(&plant),
      pos_map(multibody::makeNameToPositionsMap(plant)),
      vel_map(multibody::makeNameToVelocitiesMap(plant)),
      act_map(multibody::makeNameToActuatorsMap(plant)),
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
      left_right_foot({left_toe_mid, right_toe_mid}),
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
  left_right_foot = {left_toe_origin, right_toe_origin};
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
      gains.R_alip_kalman_filter.asDiagonal());
  auto footstep_planner =
      builder.AddSystem<systems::AlipFootstepPlanner>(
          plant, plant_context.get(), left_right_support_fsm_states,
          left_right_support_state_durations, left_right_foot, "pelvis",
          double_support_duration, gains.mid_foot_height,
          gains.final_foot_height, gains.final_foot_velocity_z,
          gains.max_CoM_to_footstep_dist, gains.footstep_offset,
          gains.center_line_offset);

  builder.Connect(fsm->get_output_port(0),
                  alip_traj_generator->get_input_port_fsm());
  builder.Connect(touchdown_event_time->get_output_port_event_time(),
                  alip_traj_generator->get_input_port_touchdown_time());
  builder.Connect(fsm->get_output_port(0),
                  touchdown_event_time->get_input_port_fsm());
  builder.Connect(fsm->get_output_port(0),
                  liftoff_event_time->get_input_port_fsm());
  builder.Connect(high_level_command->get_yaw_output_port(),
                  head_traj_gen->get_yaw_input_port());


  builder.Connect(fsm->get_output_port(0),
                  swing_ft_traj_generator->get_input_port_fsm());
  builder.Connect(liftoff_event_time->get_output_port_event_time_of_interest(),
                  swing_ft_traj_generator->get_input_port_fsm_switch_time());

  builder.Connect(alip_traj_generator->get_output_port_alip_state(),
                  swing_ft_traj_generator->get_input_port_alip_state());
  builder.Connect(high_level_command->get_xy_output_port(),
                  swing_ft_traj_generator->get_input_port_vdes());

  auto left_toe_angle_traj_gen =
      builder.AddSystem<cassie::osc::SwingToeTrajGenerator>(
          plant, plant_context.get(), pos_map["toe_left"],
          left_foot_points, "left_toe_angle_traj");
  auto right_toe_angle_traj_gen =
      builder.AddSystem<cassie::osc::SwingToeTrajGenerator>(
          plant, plant_context.get(), pos_map["toe_right"],
          right_foot_points, "right_toe_angle_traj");
  builder.Connect(pelvis_filt->get_output_port(0),
                  left_toe_angle_traj_gen->get_state_input_port());
  builder.Connect(pelvis_filt->get_output_port(0),
                  right_toe_angle_traj_gen->get_state_input_port());

  // Create Operational space control
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      plant, plant, plant_context.get(), plant_context.get(), true,
      false, 0);

  // Cost
  int n_v = plant.num_velocities();
  MatrixXd Q_accel = gains.w_accel * MatrixXd::Identity(n_v, n_v);
  osc->SetAccelerationCostForAllJoints(Q_accel);
  osc->SetInputRegularizationWeight(gains.w_input_reg);

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
  osc->SetWeightOfSoftContactConstraint(gains.w_soft_constraint);
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



  TransTaskSpaceTrackingData swing_foot_data(
      "swing_ft_data", gains.K_p_swing_foot, gains.K_d_swing_foot,
      gains.W_swing_foot, plant, plant);
  swing_foot_data.AddStateAndPointToTrack(left_stance_state, "toe_right");
  swing_foot_data.AddStateAndPointToTrack(right_stance_state, "toe_left");

  swing_foot_data.AddJointAndStateToIgnoreInJacobian(
      vel_map["hip_yaw_right"], left_stance_state);
  swing_foot_data.AddJointAndStateToIgnoreInJacobian(
      vel_map["hip_yaw_left"], right_stance_state);

  ComTrackingData com_data("com_data", gains.K_p_swing_foot,
                           gains.K_d_swing_foot, gains.W_swing_foot,
                           plant, plant);
  com_data.AddFiniteStateToTrack(left_stance_state);
  com_data.AddFiniteStateToTrack(right_stance_state);
  RelativeTranslationTrackingData swing_ft_traj_local(
      "swing_ft_traj", gains.K_p_swing_foot, gains.K_d_swing_foot,
      gains.W_swing_foot, plant, plant, &swing_foot_data,
      &com_data);
  WorldYawViewFrame pelvis_view_frame(plant.GetBodyByName("pelvis"));
  swing_ft_traj_local.SetViewFrame(pelvis_view_frame);

  TransTaskSpaceTrackingData swing_ft_traj_global(
      "swing_ft_traj", gains.K_p_swing_foot, gains.K_d_swing_foot,
      gains.W_swing_foot, plant, plant);
  swing_ft_traj_global.AddStateAndPointToTrack(left_stance_state, "toe_right");
  swing_ft_traj_global.AddStateAndPointToTrack(right_stance_state, "toe_left");

  swing_ft_traj_local.SetTimeVaryingGains(
      swing_ft_gain_multiplier_gain_multiplier);
  swing_ft_traj_local.SetFeedforwardAccelMultiplier(
      swing_ft_accel_gain_multiplier_gain_multiplier);
  osc->AddTrackingData(&swing_ft_traj_local);

  ComTrackingData center_of_mass_traj("alip_com_traj", gains.K_p_com, gains.K_d_com,
                                      gains.W_com, plant, plant);
  // FiniteStatesToTrack cannot be empty
  center_of_mass_traj.AddFiniteStateToTrack(-1);
  osc->AddTrackingData(&center_of_mass_traj);

  // Pelvis rotation tracking (pitch and roll)
  RotTaskSpaceTrackingData pelvis_balance_traj(
      "pelvis_balance_traj", gains.K_p_pelvis_balance, gains.K_d_pelvis_balance,
      gains.W_pelvis_balance, plant, plant);
  pelvis_balance_traj.AddFrameToTrack("pelvis");
  osc->AddTrackingData(&pelvis_balance_traj);
  // Pelvis rotation tracking (yaw)
  RotTaskSpaceTrackingData pelvis_heading_traj(
      "pelvis_heading_traj", gains.K_p_pelvis_heading, gains.K_d_pelvis_heading,
      gains.W_pelvis_heading, plant, plant);
  pelvis_heading_traj.AddFrameToTrack("pelvis");
  osc->AddTrackingData(&pelvis_heading_traj,
                       gains.period_of_no_heading_control);

  // Swing toe joint tracking
  JointSpaceTrackingData swing_toe_traj_left(
      "left_toe_angle_traj", gains.K_p_swing_toe, gains.K_d_swing_toe,
      gains.W_swing_toe, plant, plant);
  JointSpaceTrackingData swing_toe_traj_right(
      "right_toe_angle_traj", gains.K_p_swing_toe, gains.K_d_swing_toe,
      gains.W_swing_toe, plant, plant);
  swing_toe_traj_right.AddStateAndJointToTrack(left_stance_state, "toe_right",
                                               "toe_rightdot");
  swing_toe_traj_left.AddStateAndJointToTrack(right_stance_state, "toe_left",
                                              "toe_leftdot");
  osc->AddTrackingData(&swing_toe_traj_left);
  osc->AddTrackingData(&swing_toe_traj_right);


  auto hip_yaw_traj_gen =
      builder.AddSystem<cassie::HipYawTrajGen>(left_stance_state);

  // Swing hip yaw joint tracking
  JointSpaceTrackingData swing_hip_yaw_traj(
      "swing_hip_yaw_traj", gains.K_p_hip_yaw, gains.K_d_hip_yaw,
      gains.W_hip_yaw, plant, plant);
  swing_hip_yaw_traj.AddStateAndJointToTrack(left_stance_state, "hip_yaw_right",
                                             "hip_yaw_rightdot");
  swing_hip_yaw_traj.AddStateAndJointToTrack(right_stance_state, "hip_yaw_left",
                                             "hip_yaw_leftdot");

  if (FLAGS_use_radio) {
    builder.Connect(cassie_out_to_radio->get_output_port(),
                    hip_yaw_traj_gen->get_radio_input_port());
    builder.Connect(fsm->get_output_port_fsm(),
                    hip_yaw_traj_gen->get_fsm_input_port());
    osc->AddTrackingData(&swing_hip_yaw_traj);
  } else {
    osc->AddConstTrackingData(&swing_hip_yaw_traj, VectorXd::Zero(1));
  }

  // Set double support duration for force blending
  osc->SetUpDoubleSupportPhaseBlending(
      double_support_duration, left_stance_state, right_stance_state,
      {post_left_double_support_state, post_right_double_support_state});

  osc->SetOsqpSolverOptionsFromYaml(
      "examples/Cassie/osc/solver_settings/osqp_options_walking.yaml");

  if (gains.W_com(0,0) == 0){
    osc->AddInputCostByJointAndFsmState(
        "toe_left_motor", left_stance_state, 1.0);
    osc->AddInputCostByJointAndFsmState(
        "toe_left_motor", post_right_double_support_state, 1.0);
    osc->AddInputCostByJointAndFsmState(
        "toe_right_motor", right_stance_state, 1.0);
    osc->AddInputCostByJointAndFsmState(
        "toe_right_motor", post_left_double_support_state, 1.0);
  }
  osc->Build();

  // Connect ports
  builder.Connect(simulator_drift->get_output_port(0),
                  osc->get_robot_output_input_port());
  builder.Connect(fsm->get_output_port(0), osc->get_fsm_input_port());
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
  if (FLAGS_use_radio) {
    builder.Connect(hip_yaw_traj_gen->get_hip_yaw_output_port(),
                    osc->get_tracking_data_input_port("swing_hip_yaw_traj"));
  }
  builder.Connect(osc->get_output_port(0), command_sender->get_input_port(0));
  if (FLAGS_publish_osc_data) {
    // Create osc debug sender.
    auto osc_debug_pub =
        builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
            "OSC_DEBUG_WALKING", &lcm_local,
            TriggerTypeSet({TriggerType::kForced})));
    builder.Connect(osc->get_osc_debug_port(), osc_debug_pub->get_input_port());
  }

  // Create the diagram
  auto owned_diagram = builder.Build();
  owned_diagram->set_name("osc walking controller");

}
}
}
}