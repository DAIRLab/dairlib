#include <gflags/gflags.h>

#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/osc/heading_traj_generator.h"
#include "examples/Cassie/osc/high_level_command.h"
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

// Currently the controller runs at the rate between 500 Hz and 200 Hz, so the
// publish rate of the robot state needs to be less than 500 Hz. Otherwise, the
// performance seems to degrade due to this. (Recommended publish rate: 200 Hz)
// Maybe we need to update the lcm driven loop to clear the queue of lcm message
// if it's more than one message?

struct OSCWalkingGains {
  int rows;
  int cols;
  double w_accel;
  double w_soft_constraint;
  std::vector<double> CoMW;
  std::vector<double> CoMKp;
  std::vector<double> CoMKd;
  std::vector<double> PelvisHeadingW;
  std::vector<double> PelvisHeadingKp;
  std::vector<double> PelvisHeadingKd;
  std::vector<double> PelvisBalanceW;
  std::vector<double> PelvisBalanceKp;
  std::vector<double> PelvisBalanceKd;
  std::vector<double> SwingFootW;
  std::vector<double> SwingFootKp;
  std::vector<double> SwingFootKd;
  double w_swing_toe;
  double swing_toe_kp;
  double swing_toe_kd;
  double w_hip_yaw;
  double hip_yaw_kp;
  double hip_yaw_kd;
  double center_line_offset;
  double footstep_offset;
  double mid_foot_height;
  double final_foot_height;
  double final_foot_velocity_z;
  double lipm_height;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(rows));
    a->Visit(DRAKE_NVP(cols));
    a->Visit(DRAKE_NVP(w_accel));
    a->Visit(DRAKE_NVP(w_soft_constraint));
    a->Visit(DRAKE_NVP(CoMW));
    a->Visit(DRAKE_NVP(CoMKp));
    a->Visit(DRAKE_NVP(CoMKd));
    a->Visit(DRAKE_NVP(PelvisHeadingW));
    a->Visit(DRAKE_NVP(PelvisHeadingKp));
    a->Visit(DRAKE_NVP(PelvisHeadingKd));
    a->Visit(DRAKE_NVP(PelvisBalanceW));
    a->Visit(DRAKE_NVP(PelvisBalanceKp));
    a->Visit(DRAKE_NVP(PelvisBalanceKd));
    a->Visit(DRAKE_NVP(SwingFootW));
    a->Visit(DRAKE_NVP(SwingFootKp));
    a->Visit(DRAKE_NVP(SwingFootKd));
    a->Visit(DRAKE_NVP(w_swing_toe));
    a->Visit(DRAKE_NVP(swing_toe_kp));
    a->Visit(DRAKE_NVP(swing_toe_kd));
    a->Visit(DRAKE_NVP(w_hip_yaw));
    a->Visit(DRAKE_NVP(hip_yaw_kp));
    a->Visit(DRAKE_NVP(hip_yaw_kd));
    // swing foot heuristics
    a->Visit(DRAKE_NVP(mid_foot_height));
    a->Visit(DRAKE_NVP(center_line_offset));
    a->Visit(DRAKE_NVP(footstep_offset));
    a->Visit(DRAKE_NVP(final_foot_height));
    a->Visit(DRAKE_NVP(final_foot_velocity_z));
    // lipm heursitics
    a->Visit(DRAKE_NVP(lipm_height));
  }
};

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

  MatrixXd W_com = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.CoMW.data(), gains.rows, gains.cols);
  MatrixXd K_p_com = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.CoMKp.data(), gains.rows, gains.cols);
  MatrixXd K_d_com = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.CoMKd.data(), gains.rows, gains.cols);
  MatrixXd W_pelvis_heading = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.PelvisHeadingW.data(), gains.rows, gains.cols);
  MatrixXd K_p_pelvis_heading = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.PelvisHeadingKp.data(), gains.rows, gains.cols);
  MatrixXd K_d_pelvis_heading = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.PelvisHeadingKd.data(), gains.rows, gains.cols);
  MatrixXd W_pelvis_balance = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.PelvisBalanceW.data(), gains.rows, gains.cols);
  MatrixXd K_p_pelvis_balance = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.PelvisBalanceKp.data(), gains.rows, gains.cols);
  MatrixXd K_d_pelvis_balance = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.PelvisBalanceKd.data(), gains.rows, gains.cols);
  MatrixXd W_swing_foot = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.SwingFootW.data(), gains.rows, gains.cols);
  MatrixXd K_p_swing_foot = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.SwingFootKp.data(), gains.rows, gains.cols);
  MatrixXd K_d_swing_foot = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.SwingFootKd.data(), gains.rows, gains.cols);
  std::cout << "w accel: \n" << gains.w_accel << std::endl;
  std::cout << "w soft constraint: \n" << gains.w_soft_constraint << std::endl;
  std::cout << "COM W: \n" << W_com << std::endl;
  std::cout << "COM Kp: \n" << K_p_com << std::endl;
  std::cout << "COM Kd: \n" << K_d_com << std::endl;
  std::cout << "Pelvis Heading W: \n" << W_pelvis_heading << std::endl;
  std::cout << "Pelvis Heading Kp: \n" << K_p_pelvis_heading << std::endl;
  std::cout << "Pelvis Heading Kd: \n" << K_d_pelvis_heading << std::endl;
  std::cout << "Pelvis Balance W: \n" << W_pelvis_balance << std::endl;
  std::cout << "Pelvis Balance Kp: \n" << K_p_pelvis_balance << std::endl;
  std::cout << "Pelvis Balance Kd: \n" << K_d_pelvis_balance << std::endl;
  std::cout << "Swing Foot W: \n" << W_swing_foot << std::endl;
  std::cout << "Swing Foot Kp: \n" << K_p_swing_foot << std::endl;
  std::cout << "Swing Foot Kd: \n" << K_d_swing_foot << std::endl;

  // Get contact frames and position (doesn't matter whether we use
  // plant_w_spr or plant_wospr because the contact frames exit in both
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

  // Create state receiver.
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
    double vel_scale_rot = 0.5;
    double vel_scale_trans = 1.5;
    high_level_command = builder.AddSystem<cassie::osc::HighLevelCommand>(
        plant_w_spr, context_w_spr.get(), vel_scale_rot, vel_scale_trans,
        FLAGS_footstep_option);
    builder.Connect(cassie_out_receiver->get_output_port(),
                    high_level_command->get_cassie_output_port());
  } else {
    high_level_command = builder.AddSystem<cassie::osc::HighLevelCommand>(
        plant_w_spr, context_w_spr.get(), global_target_position,
        params_of_no_turning, FLAGS_footstep_option);
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
  double left_support_duration = 0.35;
  double right_support_duration = 0.35;
  double double_support_duration = 0.02;
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
      plant_w_spr, fsm_states, state_durations);
  builder.Connect(simulator_drift->get_output_port(0),
                  fsm->get_input_port_state());

  // Create leafsystem that record the switching time of the FSM
  std::vector<int> single_support_states = {left_stance_state,
                                            right_stance_state};
  auto liftoff_event_time =
      builder.AddSystem<systems::FiniteStateMachineEventTime>(
          single_support_states);
  liftoff_event_time->set_name("liftoff_time");
  builder.Connect(fsm->get_output_port(0),
                  liftoff_event_time->get_input_port_fsm());
  auto touchdown_event_time =
      builder.AddSystem<systems::FiniteStateMachineEventTime>(
          std::vector<int>(1, double_support_state));
  touchdown_event_time->set_name("touchdown_time");
  builder.Connect(fsm->get_output_port(0),
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
  builder.Connect(fsm->get_output_port(0),
                  lipm_traj_generator->get_input_port_fsm());
  builder.Connect(touchdown_event_time->get_output_port_event_time(),
                  lipm_traj_generator->get_input_port_fsm_switch_time());
  builder.Connect(simulator_drift->get_output_port(0),
                  lipm_traj_generator->get_input_port_state());

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
  double max_CoM_to_footstep_dist = 0.4;

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
          gains.final_foot_velocity_z, max_CoM_to_footstep_dist,
          gains.footstep_offset, gains.center_line_offset, true, true, true,
          FLAGS_footstep_option);
  builder.Connect(fsm->get_output_port(0),
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

  // Swing foot tracking
  TransTaskSpaceTrackingData swing_foot_traj("swing_ft_traj", K_p_swing_foot,
                                             K_d_swing_foot, W_swing_foot,
                                             plant_w_spr, plant_w_spr);
  swing_foot_traj.AddStateAndPointToTrack(left_stance_state, "toe_right");
  swing_foot_traj.AddStateAndPointToTrack(right_stance_state, "toe_left");
  osc->AddTrackingData(&swing_foot_traj);
  // Center of mass tracking
  ComTrackingData center_of_mass_traj("lipm_traj", K_p_com, K_d_com, W_com,
                                      plant_w_spr, plant_w_spr);
  osc->AddTrackingData(&center_of_mass_traj);
  // Pelvis rotation tracking (pitch and roll)
  RotTaskSpaceTrackingData pelvis_balance_traj(
      "pelvis_balance_traj", K_p_pelvis_balance, K_d_pelvis_balance,
      W_pelvis_balance, plant_w_spr, plant_w_spr);
  pelvis_balance_traj.AddFrameToTrack("pelvis");
  VectorXd pelvis_desired_quat(4);
  pelvis_desired_quat << 1, 0, 0, 0;
  osc->AddConstTrackingData(&pelvis_balance_traj, pelvis_desired_quat);
  // Pelvis rotation tracking (yaw)
  RotTaskSpaceTrackingData pelvis_heading_traj(
      "pelvis_heading_traj", K_p_pelvis_heading, K_d_pelvis_heading,
      W_pelvis_heading, plant_w_spr, plant_w_spr);
  pelvis_heading_traj.AddFrameToTrack("pelvis");
  osc->AddTrackingData(&pelvis_heading_traj, 0.1);  // 0.05
  // Swing toe joint tracking (Currently use fix position)
  // The desired position, -1.5, was derived heuristically. It is roughly the
  // toe angle when Cassie stands on the ground.
  MatrixXd W_swing_toe = gains.w_swing_toe * MatrixXd::Identity(1, 1);
  MatrixXd K_p_swing_toe = gains.swing_toe_kp * MatrixXd::Identity(1, 1);
  MatrixXd K_d_swing_toe = gains.swing_toe_kd * MatrixXd::Identity(1, 1);
  JointSpaceTrackingData swing_toe_traj("swing_toe_traj", K_p_swing_toe,
                                        K_d_swing_toe, W_swing_toe, plant_w_spr,
                                        plant_w_spr);
  swing_toe_traj.AddStateAndJointToTrack(left_stance_state, "toe_right",
                                         "toe_rightdot");
  swing_toe_traj.AddStateAndJointToTrack(right_stance_state, "toe_left",
                                         "toe_leftdot");
  osc->AddConstTrackingData(&swing_toe_traj, -1.5 * VectorXd::Ones(1), 0, 0.3);
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
  // Build OSC problem
  osc->Build();
  // Connect ports
  builder.Connect(simulator_drift->get_output_port(0),
                  osc->get_robot_output_input_port());
  builder.Connect(fsm->get_output_port(0), osc->get_fsm_input_port());
  builder.Connect(lipm_traj_generator->get_output_port_lipm_from_touchdown(),
                  osc->get_tracking_data_input_port("lipm_traj"));
  builder.Connect(swing_ft_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("swing_ft_traj"));
  builder.Connect(head_traj_gen->get_output_port(0),
                  osc->get_tracking_data_input_port("pelvis_heading_traj"));
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
