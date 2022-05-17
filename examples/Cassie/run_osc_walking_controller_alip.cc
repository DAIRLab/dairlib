#include <gflags/gflags.h>

#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/simulator_drift.h"
#include "examples/Cassie/osc/heading_traj_generator.h"
#include "examples/Cassie/osc/high_level_command.h"
#include "examples/Cassie/osc/osc_walking_gains_alip.h"
#include "examples/Cassie/osc/swing_toe_traj_generator.h"
#include "examples/Cassie/systems/cassie_out_to_radio.h"
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
#include "systems/controllers/alip_swing_ft_traj_gen.h"
#include "systems/controllers/time_based_fsm.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/filters/floating_base_velocity_filter.h"
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
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::LcmScopeSystem;
using drake::systems::TriggerTypeSet;

using multibody::WorldYawViewFrame;
using systems::controllers::ComTrackingData;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::RelativeTranslationTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;

using multibody::FixedJointEvaluator;
using multibody::makeNameToVelocitiesMap;
using multibody::makeNameToPositionsMap;

using drake::trajectories::PiecewisePolynomial;

DEFINE_double(drift_rate, 0.0, "Drift rate for floating-base state");
DEFINE_string(channel_x, "CASSIE_STATE_SIMULATION",
              "LCM channel for receiving state. "
              "Use CASSIE_STATE_SIMULATION to get state from simulator, and "
              "use CASSIE_STATE_DISPATCHER to get state from state estimator");
DEFINE_string(channel_u, "CASSIE_INPUT",
              "The name of the channel which publishes command");
DEFINE_bool(use_radio, false,
            "Set to true if sending high level commands from radio controller");
DEFINE_string(
    cassie_out_channel, "CASSIE_OUTPUT_ECHO",
    "The name of the channel to receive the cassie out structure from.");
DEFINE_string(gains_filename, "examples/Cassie/osc/osc_walking_gains_alip.yaml",
              "Filepath containing gains");
DEFINE_bool(publish_osc_data, true,
            "whether to publish lcm messages for OscTrackData");
DEFINE_bool(print_osc, false, "whether to print the osc debug message or not");

DEFINE_bool(is_two_phase, false,
            "true: only right/left single support"
            "false: both double and single support");
DEFINE_double(qp_time_limit, 0.002, "maximum qp solve time");

DEFINE_bool(spring_model, true, "");
DEFINE_bool(publish_filtered_state, false,
            "whether to publish the low pass filtered state");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Read-in the parameters
  auto gains = drake::yaml::LoadYamlFile<OSCWalkingGainsALIP>(FLAGS_gains_filename);

  // Build Cassie MBP
  drake::multibody::MultibodyPlant<double> plant_w_spr(0.0);
  if (FLAGS_spring_model) {
    addCassieMultibody(&plant_w_spr, nullptr, true /*floating base*/,
                       "examples/Cassie/urdf/cassie_v2.urdf",
                       true /*spring model*/, false /*loop closure*/);
  } else {
    addCassieMultibody(&plant_w_spr, nullptr, true /*floating base*/,
                       "examples/Cassie/urdf/cassie_fixed_springs.urdf",
                       false /*spring model*/, false /*loop closure*/);
  }
  plant_w_spr.Finalize();

  auto context_w_spr = plant_w_spr.CreateDefaultContext();

  // Build the controller diagram
  DiagramBuilder<double> builder;

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");

  // Get contact frames and position (doesn't matter whether we use
  // plant_w_spr or plant_wospr because the contact frames exit in both
  // plants)
  auto left_toe = LeftToeFront(plant_w_spr);
  auto left_heel = LeftToeRear(plant_w_spr);
  auto right_toe = RightToeFront(plant_w_spr);
  auto right_heel = RightToeRear(plant_w_spr);

  // Get body frames and points
  Vector3d mid_contact_point = left_heel.first +
      gains.contact_point_pos * (left_toe.first - left_heel.first);
  auto left_toe_mid = std::pair<const Vector3d, const Frame<double>&>(
      mid_contact_point, plant_w_spr.GetFrameByName("toe_left"));
  auto right_toe_mid = std::pair<const Vector3d, const Frame<double>&>(
      mid_contact_point, plant_w_spr.GetFrameByName("toe_right"));
  auto left_toe_origin = std::pair<const Vector3d, const Frame<double>&>(
      Vector3d::Zero(), plant_w_spr.GetFrameByName("toe_left"));
  auto right_toe_origin = std::pair<const Vector3d, const Frame<double>&>(
      Vector3d::Zero(), plant_w_spr.GetFrameByName("toe_right"));

  // Create state receiver.
  auto state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_w_spr);
  auto pelvis_filt =
      builder.AddSystem<systems::FloatingBaseVelocityFilter>(
          plant_w_spr, gains.pelvis_xyz_vel_filter_tau);
  builder.Connect(*state_receiver, *pelvis_filt);

  if (FLAGS_publish_filtered_state) {
    auto [filtered_state_scope, filtered_state_sender]=
    // AddToBuilder will add the systems to the diagram and connect their ports
    LcmScopeSystem::AddToBuilder(
        &builder, &lcm_local,pelvis_filt->get_output_port(),
        "CASSIE_STATE_FB_FILTERED", 0);
  }

  // Create command sender.
  auto command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_channel_u, &lcm_local, TriggerTypeSet({TriggerType::kForced})));
  auto command_sender =
      builder.AddSystem<systems::RobotCommandSender>(plant_w_spr);

  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port());

  // Add emulator for floating base drift
  Eigen::VectorXd drift_mean =
      Eigen::VectorXd::Zero(plant_w_spr.num_positions());
  Eigen::MatrixXd drift_cov = Eigen::MatrixXd::Zero(
      plant_w_spr.num_positions(), plant_w_spr.num_positions());
  drift_cov(4, 4) = FLAGS_drift_rate;  // x
  drift_cov(5, 5) = FLAGS_drift_rate;  // y
  drift_cov(6, 6) = FLAGS_drift_rate;  // z
  // Note that we didn't add drift to yaw angle here because it requires
  // changing SimulatorDrift.

  auto simulator_drift =
      builder.AddSystem<SimulatorDrift>(plant_w_spr, drift_mean, drift_cov);
  builder.Connect(pelvis_filt->get_output_port(0),
                  simulator_drift->get_input_port_state());

  auto cassie_out_to_radio =
      builder.AddSystem<systems::CassieOutToRadio>();

  // Create human high-level control
  Eigen::Vector2d global_target_position(gains.global_target_position_x,
                                         gains.global_target_position_y);
  Eigen::Vector2d params_of_no_turning(gains.yaw_deadband_blur,
                                       gains.yaw_deadband_radius);
  cassie::osc::HighLevelCommand* high_level_command;
  if (FLAGS_use_radio) {
    high_level_command = builder.AddSystem<cassie::osc::HighLevelCommand>(
        plant_w_spr, context_w_spr.get(), gains.vel_scale_rot,
        gains.vel_scale_trans_sagital, gains.vel_scale_trans_lateral, 0.4);
    auto cassie_out_receiver =
        builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_cassie_out>(
            FLAGS_cassie_out_channel, &lcm_local));
    builder.Connect(*cassie_out_receiver, *cassie_out_to_radio);
    builder.Connect(cassie_out_to_radio->get_output_port(),
                    high_level_command->get_radio_port());
  } else {
    high_level_command = builder.AddSystem<cassie::osc::HighLevelCommand>(
        plant_w_spr, context_w_spr.get(), gains.kp_yaw, gains.kd_yaw,
        gains.vel_max_yaw, gains.kp_pos_sagital, gains.kd_pos_sagital,
        gains.vel_max_sagital, gains.kp_pos_lateral, gains.kd_pos_lateral,
        gains.vel_max_lateral, gains.target_pos_offset, global_target_position,
        params_of_no_turning);
  }
  builder.Connect(pelvis_filt->get_output_port(0),
                  high_level_command->get_state_input_port());

  // Create heading traj generator
  auto head_traj_gen = builder.AddSystem<cassie::osc::HeadingTrajGenerator>(
      plant_w_spr, context_w_spr.get());
  builder.Connect(simulator_drift->get_output_port(0),
                  head_traj_gen->get_state_input_port());
  builder.Connect(high_level_command->get_yaw_output_port(),
                  head_traj_gen->get_yaw_input_port());

  // Create finite state machine
  int left_stance_state = 0;
  int right_stance_state = 1;
  int post_left_double_support_state = 3;
  int post_right_double_support_state = 4;
  double left_support_duration = gains.ss_time;
  double right_support_duration = gains.ss_time;
  double double_support_duration = gains.ds_time;
  vector<int> fsm_states;
  vector<double> state_durations;
  if (FLAGS_is_two_phase) {
    fsm_states = {left_stance_state, right_stance_state};
    state_durations = {left_support_duration, right_support_duration};
  } else {
    fsm_states = {left_stance_state, post_left_double_support_state,
                  right_stance_state, post_right_double_support_state};
    state_durations = {left_support_duration, double_support_duration,
                       right_support_duration, double_support_duration};
  }
  auto fsm = builder.AddSystem<systems::TimeBasedFiniteStateMachine>(
      plant_w_spr, fsm_states, state_durations);
  builder.Connect(simulator_drift->get_output_port(0),
                  fsm->get_input_port_state());

  // Create leafsystem that record the switching time of the FSM
  std::vector<int> single_support_states = {left_stance_state,
                                            right_stance_state};
  auto liftoff_event_time =
      builder.AddSystem<systems::FiniteStateMachineEventTime>(
          plant_w_spr, single_support_states);
  liftoff_event_time->set_name("liftoff_time");
  builder.Connect(fsm->get_output_port(0),
                  liftoff_event_time->get_input_port_fsm());
  builder.Connect(simulator_drift->get_output_port(0),
                  liftoff_event_time->get_input_port_state());
  std::vector<int> double_support_states = {post_left_double_support_state,
                                            post_right_double_support_state};
  auto touchdown_event_time =
      builder.AddSystem<systems::FiniteStateMachineEventTime>(
          plant_w_spr, double_support_states);
  touchdown_event_time->set_name("touchdown_time");
  builder.Connect(fsm->get_output_port(0),
                  touchdown_event_time->get_input_port_fsm());
  builder.Connect(simulator_drift->get_output_port(0),
                  touchdown_event_time->get_input_port_state());

  // Create CoM trajectory generator
  // Note that we are tracking COM acceleration instead of position and velocity
  // because we construct the LIPM traj which starts from the current state
  double desired_com_height = gains.lipm_height;
  vector<int> unordered_fsm_states;
  vector<double> unordered_state_durations;
  vector<vector<std::pair<const Vector3d, const Frame<double>&>>>
      contact_points_in_each_state;
  if (FLAGS_is_two_phase) {
    unordered_fsm_states = {left_stance_state, right_stance_state};
    unordered_state_durations = {left_support_duration, right_support_duration};
    contact_points_in_each_state.push_back({left_toe_mid});
    contact_points_in_each_state.push_back({right_toe_mid});
  } else {
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
  }
  auto alip_traj_generator = builder.AddSystem<systems::ALIPTrajGenerator>(
      plant_w_spr, context_w_spr.get(), desired_com_height,
      unordered_fsm_states, unordered_state_durations,
      contact_points_in_each_state, gains.Q_alip_kalman_filter.asDiagonal(),
      gains.R_alip_kalman_filter.asDiagonal());

  builder.Connect(fsm->get_output_port(0),
                  alip_traj_generator->get_input_port_fsm());
  builder.Connect(touchdown_event_time->get_output_port_event_time(),
                  alip_traj_generator->get_input_port_touchdown_time());
  builder.Connect(simulator_drift->get_output_port(0),
                  alip_traj_generator->get_input_port_state());

  // Create swing leg trajectory generator
  // Since the ground is soft in the simulation, we raise the desired final
  // foot height by 1 cm. The controller is sensitive to this number, should
  // tune this every time we change the simulation parameter or when we move
  // to the hardware testing.
  // Additionally, implementing a double support phase might mitigate the
  // instability around state transition.
  vector<int> left_right_support_fsm_states = {left_stance_state,
                                               right_stance_state};
  vector<double> left_right_support_state_durations = {left_support_duration,
                                                       right_support_duration};
  vector<std::pair<const Vector3d, const Frame<double>&>> left_right_foot = {
      left_toe_origin, right_toe_origin};
  auto swing_ft_traj_generator =
      builder.AddSystem<systems::AlipSwingFootTrajGenerator>(
          plant_w_spr, context_w_spr.get(), left_right_support_fsm_states,
          left_right_support_state_durations, left_right_foot, "pelvis",
          double_support_duration, gains.mid_foot_height,
          gains.final_foot_height, gains.final_foot_velocity_z,
          gains.max_CoM_to_footstep_dist, gains.footstep_offset,
          gains.center_line_offset);
  builder.Connect(fsm->get_output_port(0),
                  swing_ft_traj_generator->get_input_port_fsm());
  builder.Connect(liftoff_event_time->get_output_port_event_time_of_interest(),
                  swing_ft_traj_generator->get_input_port_fsm_switch_time());
  builder.Connect(simulator_drift->get_output_port(0),
                  swing_ft_traj_generator->get_input_port_state());
  builder.Connect(alip_traj_generator->get_output_port_alip_state(),
                  swing_ft_traj_generator->get_input_port_alip_state());
  builder.Connect(high_level_command->get_xy_output_port(),
                  swing_ft_traj_generator->get_input_port_vdes());

  // Swing toe joint trajectory
  map<string, int> pos_map = multibody::makeNameToPositionsMap(plant_w_spr);
  vector<std::pair<const Vector3d, const Frame<double>&>> left_foot_points = {
      left_heel, left_toe};
  vector<std::pair<const Vector3d, const Frame<double>&>> right_foot_points = {
      right_heel, right_toe};
  auto left_toe_angle_traj_gen =
      builder.AddSystem<cassie::osc::SwingToeTrajGenerator>(
          plant_w_spr, context_w_spr.get(), pos_map["toe_left"],
          left_foot_points, "left_toe_angle_traj");
  auto right_toe_angle_traj_gen =
      builder.AddSystem<cassie::osc::SwingToeTrajGenerator>(
          plant_w_spr, context_w_spr.get(), pos_map["toe_right"],
          right_foot_points, "right_toe_angle_traj");
  builder.Connect(pelvis_filt->get_output_port(0),
                  left_toe_angle_traj_gen->get_state_input_port());
  builder.Connect(pelvis_filt->get_output_port(0),
                  right_toe_angle_traj_gen->get_state_input_port());

  // Create Operational space control
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      plant_w_spr, plant_w_spr, context_w_spr.get(), context_w_spr.get(), true,
      FLAGS_print_osc, FLAGS_qp_time_limit);

  // Cost
  int n_v = plant_w_spr.num_velocities();
  MatrixXd Q_accel = gains.w_accel * MatrixXd::Identity(n_v, n_v);
  osc->SetAccelerationCostForAllJoints(Q_accel);
  osc->SetInputRegularizationWeight(gains.w_input_reg);

  // Constraints in OSC
  multibody::KinematicEvaluatorSet<double> evaluators(plant_w_spr);
  // 1. fourbar constraint
  auto left_loop = LeftLoopClosureEvaluator(plant_w_spr);
  auto right_loop = RightLoopClosureEvaluator(plant_w_spr);
  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);
  // 2. fixed spring constraint
  // Note that we set the position value to 0, but this is not used in OSC,
  // because OSC constraint only use JdotV and J.
  std::unique_ptr<FixedJointEvaluator<double>> left_fixed_knee_spring;
  std::unique_ptr<FixedJointEvaluator<double>> right_fixed_knee_spring;
  std::unique_ptr<FixedJointEvaluator<double>> left_fixed_ankle_spring;
  std::unique_ptr<FixedJointEvaluator<double>> right_fixed_ankle_spring;
  if (FLAGS_spring_model) {
    auto pos_idx_map = multibody::makeNameToPositionsMap(plant_w_spr);
    auto vel_idx_map = multibody::makeNameToVelocitiesMap(plant_w_spr);
    left_fixed_knee_spring = std::make_unique<FixedJointEvaluator<double>>(
        plant_w_spr, pos_idx_map.at("knee_joint_left"),
        vel_idx_map.at("knee_joint_leftdot"), 0);
    right_fixed_knee_spring = std::make_unique<FixedJointEvaluator<double>>(
        plant_w_spr, pos_idx_map.at("knee_joint_right"),
        vel_idx_map.at("knee_joint_rightdot"), 0);
    left_fixed_ankle_spring = std::make_unique<FixedJointEvaluator<double>>(
        plant_w_spr, pos_idx_map.at("ankle_spring_joint_left"),
        vel_idx_map.at("ankle_spring_joint_leftdot"), 0);
    right_fixed_ankle_spring = std::make_unique<FixedJointEvaluator<double>>(
        plant_w_spr, pos_idx_map.at("ankle_spring_joint_right"),
        vel_idx_map.at("ankle_spring_joint_rightdot"), 0);
    evaluators.add_evaluator(left_fixed_knee_spring.get());
    evaluators.add_evaluator(right_fixed_knee_spring.get());
    evaluators.add_evaluator(left_fixed_ankle_spring.get());
    evaluators.add_evaluator(right_fixed_ankle_spring.get());
  }
  osc->AddKinematicConstraint(&evaluators);

  // Soft constraint
  // w_contact_relax shouldn't be too big, cause we want tracking error to be
  // important
  osc->SetWeightOfSoftContactConstraint(gains.w_soft_constraint);
  // Friction coefficient
  osc->SetContactFriction(gains.mu);
  // Add contact points (The position doesn't matter. It's not used in OSC)
  const auto& pelvis = plant_w_spr.GetBodyByName("pelvis");
  multibody::WorldYawViewFrame view_frame(pelvis);
  auto left_toe_evaluator = multibody::WorldPointEvaluator(
      plant_w_spr, left_toe.first, left_toe.second, view_frame,
      Matrix3d::Identity(), Vector3d::Zero(), {1, 2});
  auto left_heel_evaluator = multibody::WorldPointEvaluator(
      plant_w_spr, left_heel.first, left_heel.second, view_frame,
      Matrix3d::Identity(), Vector3d::Zero(), {0, 1, 2});
  auto right_toe_evaluator = multibody::WorldPointEvaluator(
      plant_w_spr, right_toe.first, right_toe.second, view_frame,
      Matrix3d::Identity(), Vector3d::Zero(), {1, 2});
  auto right_heel_evaluator = multibody::WorldPointEvaluator(
      plant_w_spr, right_heel.first, right_heel.second, view_frame,
      Matrix3d::Identity(), Vector3d::Zero(), {0, 1, 2});
  osc->AddStateAndContactPoint(left_stance_state, &left_toe_evaluator);
  osc->AddStateAndContactPoint(left_stance_state, &left_heel_evaluator);
  osc->AddStateAndContactPoint(right_stance_state, &right_toe_evaluator);
  osc->AddStateAndContactPoint(right_stance_state, &right_heel_evaluator);
  if (!FLAGS_is_two_phase) {
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

  // Swing foot tracking
  std::vector<double> swing_ft_gain_multiplier_breaks{
      0, left_support_duration / 2, left_support_duration};
  std::vector<drake::MatrixX<double>> swing_ft_gain_multiplier_samples(
      3, drake::MatrixX<double>::Identity(3, 3));
  swing_ft_gain_multiplier_samples[2](2, 2) *= 0.3;
  PiecewisePolynomial<double> swing_ft_gain_multiplier_gain_multiplier =
      PiecewisePolynomial<double>::FirstOrderHold(
          swing_ft_gain_multiplier_breaks, swing_ft_gain_multiplier_samples);
  std::vector<double> swing_ft_accel_gain_multiplier_breaks{
      0, left_support_duration / 2, left_support_duration * 3 / 4,
      left_support_duration};
  std::vector<drake::MatrixX<double>> swing_ft_accel_gain_multiplier_samples(
      4, drake::MatrixX<double>::Identity(3, 3));
  swing_ft_accel_gain_multiplier_samples[2](2, 2) *= 0;
  swing_ft_accel_gain_multiplier_samples[3](2, 2) *= 0;
  PiecewisePolynomial<double> swing_ft_accel_gain_multiplier_gain_multiplier =
      PiecewisePolynomial<double>::FirstOrderHold(
          swing_ft_accel_gain_multiplier_breaks,
          swing_ft_accel_gain_multiplier_samples);

  TransTaskSpaceTrackingData swing_foot_data(
      "swing_ft_data", gains.K_p_swing_foot, gains.K_d_swing_foot,
      gains.W_swing_foot, plant_w_spr, plant_w_spr);
  swing_foot_data.AddStateAndPointToTrack(left_stance_state, "toe_right");
  swing_foot_data.AddStateAndPointToTrack(right_stance_state, "toe_left");

  auto vel_map = makeNameToVelocitiesMap<double>(plant_w_spr);
  swing_foot_data.AddJointAndStateToIgnoreInJacobian(
      vel_map["hip_yaw_right"], left_stance_state);
  swing_foot_data.AddJointAndStateToIgnoreInJacobian(
      vel_map["hip_yaw_left"], right_stance_state);

  ComTrackingData com_data("com_data", gains.K_p_swing_foot,
                           gains.K_d_swing_foot, gains.W_swing_foot,
                           plant_w_spr, plant_w_spr);
  com_data.AddFiniteStateToTrack(left_stance_state);
  com_data.AddFiniteStateToTrack(right_stance_state);
  RelativeTranslationTrackingData swing_ft_traj_local(
      "swing_ft_traj", gains.K_p_swing_foot, gains.K_d_swing_foot,
      gains.W_swing_foot, plant_w_spr, plant_w_spr, &swing_foot_data,
      &com_data);
  WorldYawViewFrame pelvis_view_frame(plant_w_spr.GetBodyByName("pelvis"));
  swing_ft_traj_local.SetViewFrame(pelvis_view_frame);

  TransTaskSpaceTrackingData swing_ft_traj_global(
      "swing_ft_traj", gains.K_p_swing_foot, gains.K_d_swing_foot,
      gains.W_swing_foot, plant_w_spr, plant_w_spr);
  swing_ft_traj_global.AddStateAndPointToTrack(left_stance_state, "toe_right");
  swing_ft_traj_global.AddStateAndPointToTrack(right_stance_state, "toe_left");

  swing_ft_traj_local.SetTimeVaryingGains(
        swing_ft_gain_multiplier_gain_multiplier);
  swing_ft_traj_local.SetFeedforwardAccelMultiplier(
        swing_ft_accel_gain_multiplier_gain_multiplier);
  osc->AddTrackingData(&swing_ft_traj_local);

  ComTrackingData center_of_mass_traj("alip_com_traj", gains.K_p_com, gains.K_d_com,
                                      gains.W_com, plant_w_spr, plant_w_spr);
  // FiniteStatesToTrack cannot be empty
  center_of_mass_traj.AddFiniteStateToTrack(-1);
  osc->AddTrackingData(&center_of_mass_traj);

  // Pelvis rotation tracking (pitch and roll)
  RotTaskSpaceTrackingData pelvis_balance_traj(
      "pelvis_balance_traj", gains.K_p_pelvis_balance, gains.K_d_pelvis_balance,
      gains.W_pelvis_balance, plant_w_spr, plant_w_spr);
  pelvis_balance_traj.AddFrameToTrack("pelvis");
  osc->AddTrackingData(&pelvis_balance_traj);
  // Pelvis rotation tracking (yaw)
  RotTaskSpaceTrackingData pelvis_heading_traj(
      "pelvis_heading_traj", gains.K_p_pelvis_heading, gains.K_d_pelvis_heading,
      gains.W_pelvis_heading, plant_w_spr, plant_w_spr);
  pelvis_heading_traj.AddFrameToTrack("pelvis");
  osc->AddTrackingData(&pelvis_heading_traj,
                       gains.period_of_no_heading_control);

  // Swing toe joint tracking
  JointSpaceTrackingData swing_toe_traj_left(
      "left_toe_angle_traj", gains.K_p_swing_toe, gains.K_d_swing_toe,
      gains.W_swing_toe, plant_w_spr, plant_w_spr);
  JointSpaceTrackingData swing_toe_traj_right(
      "right_toe_angle_traj", gains.K_p_swing_toe, gains.K_d_swing_toe,
      gains.W_swing_toe, plant_w_spr, plant_w_spr);
  swing_toe_traj_right.AddStateAndJointToTrack(left_stance_state, "toe_right",
                                               "toe_rightdot");
  swing_toe_traj_left.AddStateAndJointToTrack(right_stance_state, "toe_left",
                                              "toe_leftdot");
  osc->AddTrackingData(&swing_toe_traj_left);
  osc->AddTrackingData(&swing_toe_traj_right);

  // Swing hip yaw joint tracking
  JointSpaceTrackingData swing_hip_yaw_traj(
      "swing_hip_yaw_traj", gains.K_p_hip_yaw, gains.K_d_hip_yaw,
      gains.W_hip_yaw, plant_w_spr, plant_w_spr);
  swing_hip_yaw_traj.AddStateAndJointToTrack(left_stance_state, "hip_yaw_right",
                                             "hip_yaw_rightdot");
  swing_hip_yaw_traj.AddStateAndJointToTrack(right_stance_state, "hip_yaw_left",
                                             "hip_yaw_leftdot");
  osc->AddConstTrackingData(&swing_hip_yaw_traj, VectorXd::Zero(1));

  // Set double support duration for force blending
  osc->SetUpDoubleSupportPhaseBlending(
      double_support_duration, left_stance_state, right_stance_state,
      {post_left_double_support_state, post_right_double_support_state});

  osc->SetOsqpSolverOptionsFromYaml(
      "examples/Cassie/osc/solver_settings/osqp_options_walking.yaml");
  // Build OSC problem

  osc->AddInputCostByJointAndFsmState("toe_left_motor", left_stance_state, 1.0);
  osc->AddInputCostByJointAndFsmState("toe_left_motor", post_right_double_support_state, 1.0);
  osc->AddInputCostByJointAndFsmState("toe_right_motor", right_stance_state, 1.0);
  osc->AddInputCostByJointAndFsmState("toe_right_motor", post_left_double_support_state, 1.0);
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

  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm_local, std::move(owned_diagram), state_receiver, FLAGS_channel_x,
      true);
  loop.Simulate();

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
