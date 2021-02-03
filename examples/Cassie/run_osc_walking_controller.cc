#include <gflags/gflags.h>

#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/osc/heading_traj_generator.h"
#include "examples/Cassie/osc/high_level_command.h"
#include "examples/Cassie/osc/osc_walking_gains.h"
#include "examples/Cassie/osc/pelvis_roll_traj_generator.h"
#include "examples/Cassie/osc/spring_to_no_spring_converter.h"
#include "examples/Cassie/osc/swing_toe_traj_generator.h"
#include "examples/Cassie/osc/walking_speed_control.h"
#include "examples/Cassie/simulator_drift.h"
#include "multibody/kinematic/fixed_joint_evaluator.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/multibody_utils.h"
#include "systems/controllers/fsm_event_time.h"
#include "systems/controllers/lipm_traj_gen.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/controllers/swing_ft_traj_gen.h"
#include "systems/controllers/time_based_fsm.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"

#include "drake/common/yaml/yaml_read_archive.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

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
using drake::systems::lcm::TriggerTypeSet;

using systems::controllers::ComTrackingData;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;

using multibody::FixedJointEvaluator;

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
DEFINE_string(gains_filename, "examples/Cassie/osc/osc_walking_gains.yaml",
              "Filepath containing gains");
DEFINE_bool(publish_osc_data, true,
            "whether to publish lcm messages for OscTrackData");
DEFINE_bool(print_osc, false, "whether to print the osc debug message or not");

DEFINE_bool(is_two_phase, false,
            "true: only right/left single support"
            "false: both double and single support");
DEFINE_int32(
    footstep_option, 1,
    "0 uses the capture point\n"
    "1 uses the neutral point derived from LIPM given the stance duration");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Build Cassie MBP
  drake::multibody::MultibodyPlant<double> plant_w_spr(0.0);
  addCassieMultibody(&plant_w_spr, nullptr, true /*floating base*/,
                     "examples/Cassie/urdf/cassie_v2.urdf",
                     true /*spring model*/, false /*loop closure*/);
  plant_w_spr.Finalize();

  auto context_w_spr = plant_w_spr.CreateDefaultContext();

  // Build the controller diagram
  DiagramBuilder<double> builder;

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");

  OSCWalkingGains gains;
  const YAML::Node& root =
      YAML::LoadFile(FindResourceOrThrow(FLAGS_gains_filename));
  drake::yaml::YamlReadArchive(root).Accept(&gains);

  // Get contact frames and position (doesn't matter whether we use
  // plant_w_spr or plant_wo_spr because the contact frames exit in both
  // plants)
  auto left_toe = LeftToeFront(plant_w_spr);
  auto left_heel = LeftToeRear(plant_w_spr);
  auto right_toe = RightToeFront(plant_w_spr);
  auto right_heel = RightToeRear(plant_w_spr);

  // Get body frames and points
  Vector3d mid_contact_point = (left_toe.first + left_heel.first) / 2;
  auto left_toe_mid = std::pair<const Vector3d, const Frame<double>&>(
      mid_contact_point, plant_w_spr.GetFrameByName("toe_left"));
  auto right_toe_mid = std::pair<const Vector3d, const Frame<double>&>(
      mid_contact_point, plant_w_spr.GetFrameByName("toe_right"));
  auto left_toe_origin = std::pair<const Vector3d, const Frame<double>&>(
      Vector3d::Zero(), plant_w_spr.GetFrameByName("toe_left"));
  auto right_toe_origin = std::pair<const Vector3d, const Frame<double>&>(
      Vector3d::Zero(), plant_w_spr.GetFrameByName("toe_right"));

  // Create state receiver (must use cassie with spring because dispather_out
  // uses it)
  auto state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_w_spr);

  // Create command sender.
  auto command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_channel_u, &lcm_local, TriggerTypeSet({TriggerType::kForced})));
  auto command_sender =
      builder.AddSystem<systems::RobotCommandSender>(plant_w_spr);

  auto cassie_out_receiver =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_cassie_out>(
          FLAGS_cassie_out_channel, &lcm_local));

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
  builder.Connect(state_receiver->get_output_port(0),
                  simulator_drift->get_input_port_state());

  // Create human high-level control
  Eigen::Vector2d global_target_position(0, 0);
  Eigen::Vector2d params_of_no_turning(5, 1);
  // Logistic function 1/(1+5*exp(x-1))
  // The function ouputs 0.0007 when x = 0
  //                     0.5    when x = 1
  //                     0.9993 when x = 2
  cassie::osc::HighLevelCommand* high_level_command;
  if (FLAGS_use_radio) {
    high_level_command = builder.AddSystem<cassie::osc::HighLevelCommand>(
        plant_w_spr, context_w_spr.get(), gains.vel_scale_rot,
        gains.vel_scale_trans_sagital, gains.vel_scale_trans_lateral);
    builder.Connect(cassie_out_receiver->get_output_port(),
                    high_level_command->get_cassie_output_port());
  } else {
    high_level_command = builder.AddSystem<cassie::osc::HighLevelCommand>(
        plant_w_spr, context_w_spr.get(), global_target_position,
        params_of_no_turning, gains.kp_yaw, gains.kd_yaw, gains.vel_max_yaw,
        gains.kp_pos_sagital, gains.kd_pos_sagital, gains.kp_pos_lateral,
        gains.kd_pos_lateral, gains.target_pos_offset);
  }

  builder.Connect(state_receiver->get_output_port(0),
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
  int double_support_state = 2;
  double left_support_duration = gains.ss_time;
  double right_support_duration = gains.ss_time;
  double double_support_duration = gains.ds_time;
  vector<int> fsm_states;
  vector<double> state_durations;
  if (FLAGS_is_two_phase) {
    fsm_states = {left_stance_state, right_stance_state};
    state_durations = {left_support_duration, right_support_duration};
  } else {
    fsm_states = {left_stance_state, double_support_state, right_stance_state,
                  double_support_state};
    state_durations = {left_support_duration, double_support_duration,
                       right_support_duration, double_support_duration};
  }
  auto fsm = builder.AddSystem<systems::TimeBasedFiniteStateMachine>(
      plant_w_spr, fsm_states, state_durations, 0.0, gains.impact_threshold);
  builder.Connect(simulator_drift->get_output_port(0),
                  fsm->get_input_port_state());

  // Create leafsystem that record the switching time of the FSM
  std::vector<int> single_support_states = {left_stance_state,
                                            right_stance_state};
  auto liftoff_event_time =
      builder.AddSystem<systems::FiniteStateMachineEventTime>(
          single_support_states);
  liftoff_event_time->set_name("liftoff_time");
  builder.Connect(fsm->get_output_port_fsm(),
                  liftoff_event_time->get_input_port_fsm());
  auto touchdown_event_time =
      builder.AddSystem<systems::FiniteStateMachineEventTime>(
          std::vector<int>(1, double_support_state));
  touchdown_event_time->set_name("touchdown_time");
  builder.Connect(fsm->get_output_port_fsm(),
                  touchdown_event_time->get_input_port_fsm());

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
                            double_support_state};
    unordered_state_durations = {left_support_duration, right_support_duration,
                                 double_support_duration};
    contact_points_in_each_state.push_back({left_toe_mid});
    contact_points_in_each_state.push_back({right_toe_mid});
    contact_points_in_each_state.push_back({left_toe_mid, right_toe_mid});
  }
  auto lipm_traj_generator = builder.AddSystem<systems::LIPMTrajGenerator>(
      plant_w_spr, context_w_spr.get(), desired_com_height,
      unordered_fsm_states, unordered_state_durations,
      contact_points_in_each_state);
  builder.Connect(fsm->get_output_port_fsm(),
                  lipm_traj_generator->get_input_port_fsm());
  builder.Connect(touchdown_event_time->get_output_port_event_time(),
                  lipm_traj_generator->get_input_port_fsm_switch_time());
  builder.Connect(simulator_drift->get_output_port(0),
                  lipm_traj_generator->get_input_port_state());
  auto pelvis_traj_generator = builder.AddSystem<systems::LIPMTrajGenerator>(
      plant_w_spr, context_w_spr.get(), desired_com_height,
      unordered_fsm_states, unordered_state_durations,
      contact_points_in_each_state, false);
  builder.Connect(fsm->get_output_port_fsm(),
                  pelvis_traj_generator->get_input_port_fsm());
  builder.Connect(touchdown_event_time->get_output_port_event_time(),
                  pelvis_traj_generator->get_input_port_fsm_switch_time());
  builder.Connect(simulator_drift->get_output_port(0),
                  pelvis_traj_generator->get_input_port_state());

  // Create velocity control by foot placement
  bool use_predicted_com_vel = true;
  auto walking_speed_control =
      builder.AddSystem<cassie::osc::WalkingSpeedControl>(
          plant_w_spr, context_w_spr.get(), FLAGS_footstep_option,
          use_predicted_com_vel ? left_support_duration : 0);
  builder.Connect(high_level_command->get_xy_output_port(),
                  walking_speed_control->get_input_port_des_hor_vel());
  builder.Connect(simulator_drift->get_output_port(0),
                  walking_speed_control->get_input_port_state());
  if (use_predicted_com_vel) {
    builder.Connect(lipm_traj_generator->get_output_port_lipm_from_current(),
                    walking_speed_control->get_input_port_com());
    builder.Connect(
        liftoff_event_time->get_output_port_event_time_of_interest(),
        walking_speed_control->get_input_port_fsm_switch_time());
  }

  // Create swing leg trajectory generator (capture point)
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
      builder.AddSystem<systems::SwingFootTrajGenerator>(
          plant_w_spr, context_w_spr.get(), left_right_support_fsm_states,
          left_right_support_state_durations, left_right_foot, "pelvis",
          gains.mid_foot_height, gains.final_foot_height,
          gains.final_foot_velocity_z, gains.max_CoM_to_footstep_dist,
          gains.footstep_offset, gains.center_line_offset, true, true, true,
          FLAGS_footstep_option);
  builder.Connect(fsm->get_output_port_fsm(),
                  swing_ft_traj_generator->get_input_port_fsm());
  builder.Connect(liftoff_event_time->get_output_port_event_time_of_interest(),
                  swing_ft_traj_generator->get_input_port_fsm_switch_time());
  builder.Connect(simulator_drift->get_output_port(0),
                  swing_ft_traj_generator->get_input_port_state());
  builder.Connect(lipm_traj_generator->get_output_port_lipm_from_current(),
                  swing_ft_traj_generator->get_input_port_com());
  builder.Connect(walking_speed_control->get_output_port(0),
                  swing_ft_traj_generator->get_input_port_sc());

  // Create Operational space control
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      plant_w_spr, plant_w_spr, context_w_spr.get(), context_w_spr.get(), true,
      FLAGS_print_osc /*print_tracking_info*/);

  // Cost
  int n_v = plant_w_spr.num_velocities();
  MatrixXd Q_accel = gains.w_accel * MatrixXd::Identity(n_v, n_v);
  osc->SetAccelerationCostForAllJoints(Q_accel);

  // Constraints in OSC
  multibody::KinematicEvaluatorSet<double> evaluators(plant_w_spr);
  // 1. fourbar constraint
  auto left_loop = LeftLoopClosureEvaluator(plant_w_spr);
  auto right_loop = RightLoopClosureEvaluator(plant_w_spr);
  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);
  // 2. fixed spring constriant
  // Note that we set the position value to 0, but this is not used in OSC,
  // because OSC constraint only use JdotV and J.
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

  // Soft constraint
  // w_contact_relax shouldn't be too big, cause we want tracking error to be
  // important
  osc->SetWeightOfSoftContactConstraint(gains.w_soft_constraint);
  // Friction coefficient
  double mu = 0.4;
  osc->SetContactFriction(mu);
  // Add contact points (The position doesn't matter. It's not used in OSC)
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
  osc->AddStateAndContactPoint(left_stance_state, &left_toe_evaluator);
  osc->AddStateAndContactPoint(left_stance_state, &left_heel_evaluator);
  osc->AddStateAndContactPoint(right_stance_state, &right_toe_evaluator);
  osc->AddStateAndContactPoint(right_stance_state, &right_heel_evaluator);
  if (!FLAGS_is_two_phase) {
    osc->AddStateAndContactPoint(double_support_state, &left_toe_evaluator);
    osc->AddStateAndContactPoint(double_support_state, &left_heel_evaluator);
    osc->AddStateAndContactPoint(double_support_state, &right_toe_evaluator);
    osc->AddStateAndContactPoint(double_support_state, &right_heel_evaluator);
  }

  map<string, int> pos_map = multibody::makeNameToPositionsMap(plant_w_spr);
  map<string, int> vel_map = multibody::makeNameToVelocitiesMap(plant_w_spr);
  map<string, int> act_map = multibody::makeNameToActuatorsMap(plant_w_spr);

  // Swing foot tracking
  TransTaskSpaceTrackingData swing_foot_traj(
      "swing_ft_traj", gains.K_p_swing_foot, gains.K_d_swing_foot,
      gains.W_swing_foot, plant_w_spr, plant_w_spr);
  swing_foot_traj.AddStateAndPointToTrack(left_stance_state, "toe_right");
  swing_foot_traj.AddStateAndPointToTrack(right_stance_state, "toe_left");
  osc->AddTrackingData(&swing_foot_traj);
  // Center of mass tracking
  TransTaskSpaceTrackingData center_of_mass_traj("lipm_traj", gains.K_p_com,
                                                 gains.K_d_com, gains.W_com,
                                                 plant_w_spr, plant_w_spr);
  center_of_mass_traj.AddPointToTrack("pelvis");
  osc->AddTrackingData(&center_of_mass_traj);
  // Pelvis rotation tracking (pitch and roll)
  RotTaskSpaceTrackingData pelvis_balance_traj(
      "pelvis_balance_traj", gains.K_p_pelvis_balance, gains.K_d_pelvis_balance,
      gains.W_pelvis_balance, plant_w_spr, plant_w_spr);
  pelvis_balance_traj.AddFrameToTrack("pelvis");
  VectorXd pelvis_desired_quat(4);
  pelvis_desired_quat << 1, 0, 0, 0;
  osc->AddConstTrackingData(&pelvis_balance_traj, pelvis_desired_quat);
  // Pelvis rotation tracking (yaw)
  RotTaskSpaceTrackingData pelvis_heading_traj(
      "pelvis_heading_traj", gains.K_p_pelvis_heading, gains.K_d_pelvis_heading,
      gains.W_pelvis_heading, plant_w_spr, plant_w_spr);
  pelvis_heading_traj.AddFrameToTrack("pelvis");
  osc->AddTrackingData(&pelvis_heading_traj, 0.05);  // 0.05
  // Swing toe joint tracking (Currently use fix position)
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
  MatrixXd W_swing_toe = gains.w_swing_toe * MatrixXd::Identity(1, 1);
  MatrixXd K_p_swing_toe = gains.swing_toe_kp * MatrixXd::Identity(1, 1);
  MatrixXd K_d_swing_toe = gains.swing_toe_kd * MatrixXd::Identity(1, 1);
  JointSpaceTrackingData swing_toe_traj_left(
      "left_toe_angle_traj", K_p_swing_toe, K_d_swing_toe, W_swing_toe,
      plant_w_spr, plant_w_spr);
  JointSpaceTrackingData swing_toe_traj_right(
      "right_toe_angle_traj", K_p_swing_toe, K_d_swing_toe, W_swing_toe,
      plant_w_spr, plant_w_spr);
  swing_toe_traj_right.AddStateAndJointToTrack(left_stance_state, "toe_right",
                                               "toe_rightdot");
  swing_toe_traj_left.AddStateAndJointToTrack(right_stance_state, "toe_left",
                                              "toe_leftdot");
  osc->AddTrackingData(&swing_toe_traj_left);
  osc->AddTrackingData(&swing_toe_traj_right);
  // Swing hip yaw joint tracking
  MatrixXd W_hip_yaw = gains.w_hip_yaw * MatrixXd::Identity(1, 1);
  MatrixXd K_p_hip_yaw = gains.hip_yaw_kp * MatrixXd::Identity(1, 1);
  MatrixXd K_d_hip_yaw = gains.hip_yaw_kd * MatrixXd::Identity(1, 1);
  JointSpaceTrackingData swing_hip_yaw_traj("swing_hip_yaw_traj", K_p_hip_yaw,
                                            K_d_hip_yaw, W_hip_yaw, plant_w_spr,
                                            plant_w_spr);
  swing_hip_yaw_traj.AddStateAndJointToTrack(left_stance_state, "hip_yaw_right",
                                             "hip_yaw_rightdot");
  swing_hip_yaw_traj.AddStateAndJointToTrack(right_stance_state, "hip_yaw_left",
                                             "hip_yaw_leftdot");
  osc->AddConstTrackingData(&swing_hip_yaw_traj, VectorXd::Zero(1));
  MatrixXd W_stance_hip_roll = gains.w_stance_hip_roll * MatrixXd::Identity(1, 1);
  MatrixXd K_p_stance_hip_roll = gains.stance_hip_roll_kp * MatrixXd::Identity(1, 1);
  MatrixXd K_d_stance_hip_roll = gains.stance_hip_roll_kd * MatrixXd::Identity(1, 1);
  JointSpaceTrackingData l_stance_hip_roll_traj("left_stance_hip_roll_traj", K_p_stance_hip_roll,
                                              K_d_stance_hip_roll, W_stance_hip_roll, plant_w_spr,
                                            plant_w_spr);
  JointSpaceTrackingData r_stance_hip_roll_traj("right_stance_hip_roll_traj", K_p_stance_hip_roll,
                                              K_d_stance_hip_roll, W_stance_hip_roll, plant_w_spr,
                                            plant_w_spr);
  l_stance_hip_roll_traj.AddStateAndJointToTrack(left_stance_state, "hip_roll_left",
                                             "hip_roll_leftdot");
  r_stance_hip_roll_traj.AddStateAndJointToTrack(right_stance_state, "hip_roll_right",
                                             "hip_roll_rightdot");
  osc->AddTrackingData(&l_stance_hip_roll_traj);
  osc->AddTrackingData(&r_stance_hip_roll_traj);
  auto left_stance_hip_roll_traj_gen =
      builder.AddSystem<cassie::osc::PelvisRollTrajGenerator>(
          plant_w_spr, context_w_spr.get(), 1, "left_stance_hip_roll_traj");
  auto right_stance_hip_roll_traj_gen =
      builder.AddSystem<cassie::osc::PelvisRollTrajGenerator>(
          plant_w_spr, context_w_spr.get(), 1, "right_stance_hip_roll_traj");


  // Build OSC problem
  osc->Build();

  builder.Connect(left_toe_angle_traj_gen->get_output_port(0),
                  osc->get_tracking_data_input_port("left_toe_angle_traj"));
  builder.Connect(right_toe_angle_traj_gen->get_output_port(0),
                  osc->get_tracking_data_input_port("right_toe_angle_traj"));
  builder.Connect(left_stance_hip_roll_traj_gen->get_output_port(0),
                  osc->get_tracking_data_input_port("left_stance_hip_roll_traj"));
  builder.Connect(right_stance_hip_roll_traj_gen->get_output_port(0),
                  osc->get_tracking_data_input_port("right_stance_hip_roll_traj"));
  builder.Connect(state_receiver->get_output_port(0),
                  left_toe_angle_traj_gen->get_state_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  right_toe_angle_traj_gen->get_state_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  left_stance_hip_roll_traj_gen->get_state_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  right_stance_hip_roll_traj_gen->get_state_input_port());
  // Connect ports
  builder.Connect(simulator_drift->get_output_port(0),
                  osc->get_robot_output_input_port());
  builder.Connect(fsm->get_output_port_fsm(), osc->get_fsm_input_port());
  builder.Connect(pelvis_traj_generator->get_output_port_lipm_from_touchdown(),
                  osc->get_tracking_data_input_port("lipm_traj"));
  builder.Connect(swing_ft_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("swing_ft_traj"));
  builder.Connect(head_traj_gen->get_output_port(0),
                  osc->get_tracking_data_input_port("pelvis_heading_traj"));
  builder.Connect(osc->get_output_port(0), command_sender->get_input_port(0));
  builder.Connect(fsm->get_output_port_impact(),
                  osc->get_near_impact_input_port());
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
