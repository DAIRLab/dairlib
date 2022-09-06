#include <gflags/gflags.h>

// lcmtypes
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_fsm_info.hpp"
#include "dairlib/lcmt_footstep_target.hpp"

// Cassie and multibody
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/osc/high_level_command.h"
#include "examples/Cassie/osc/hip_yaw_traj_gen.h"
#include "examples/Cassie/osc/heading_traj_generator.h"
#include "examples/Cassie/osc/osc_walking_gains_alip.h"
#include "examples/Cassie/osc/swing_toe_traj_generator.h"
#include "examples/Cassie/systems/cassie_out_to_radio.h"
#include "multibody/kinematic/fixed_joint_evaluator.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/multibody_utils.h"

// MPC related
#include "systems/controllers/footstep_planning/alip_minlp_footstep_controller.h"
#include "systems/primitives/fsm_lcm_systems.h"
#include "systems/controllers/footstep_planning/footstep_lcm_systems.h"
#include "systems/controllers/lcm_trajectory_receiver.h"
#include "systems/controllers/swing_foot_target_traj_gen.h"

// OSC
#include "systems/controllers/osc/com_tracking_data.h"
#include "systems/controllers/osc/joint_space_tracking_data.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/controllers/osc/options_tracking_data.h"
#include "systems/controllers/osc/relative_translation_tracking_data.h"
#include "systems/controllers/osc/rot_space_tracking_data.h"
#include "systems/controllers/osc/trans_space_tracking_data.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"

// misc
#include "systems/system_utils.h"

// drake
#include "drake/common/yaml/yaml_io.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_scope_system.h"
#include "drake/systems/primitives/constant_value_source.h"

namespace dairlib {

using std::cout;
using std::endl;
using std::map;
using std::string;
using std::vector;

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using drake::multibody::Frame;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::DiagramBuilder;
using drake::systems::ConstantValueSource;
using drake::systems::lcm::LcmScopeSystem;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

using multibody::WorldYawViewFrame;
using systems::FsmReceiver;
using systems::SwingFootTargetTrajGen;
using systems::controllers::LcmTrajectoryReceiver;
using systems::controllers::TrajectoryType;
using systems::controllers::FootstepReceiver;
using systems::controllers::ComTrackingData;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::RelativeTranslationTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;

using geometry::ConvexFoothold;
using systems::controllers::alip_utils::PointOnFramed;
using systems::controllers::AlipMINLPGains;
using systems::controllers::AlipMINLPFootstepController;
using systems::controllers::FootstepSender;
using systems::FsmSender;

using multibody::FixedJointEvaluator;
using multibody::MakeNameToVelocitiesMap;
using multibody::MakeNameToPositionsMap;

using drake::trajectories::PiecewisePolynomial;

DEFINE_string(channel_x, "CASSIE_STATE_SIMULATION",
              "LCM channel for receiving state. "
              "Use CASSIE_STATE_SIMULATION to get state from simulator, and "
              "use CASSIE_STATE_DISPATCHER to get state from state estimator");

DEFINE_string(channel_u, "OSC_WALKING",
              "The name of the channel which publishes command");

DEFINE_string(channel_foot, "FOOTSTEP_TARGET",
              "LCM channel for footstep target");

DEFINE_string(channel_com, "ALIP_COM_TRAJ",
              "LCM channel for center of mass trajectory");

DEFINE_string(channel_fsm, "FSM", "lcm channel for fsm");

DEFINE_string(cassie_out_channel, "CASSIE_OUTPUT_ECHO",
              "The name of the channel to receive the cassie "
              "out structure from.");

DEFINE_string(gains_filename, "examples/Cassie/osc/osc_walking_gains_alip.yaml",
              "Filepath containing gains");

DEFINE_bool(publish_osc_data, true,
            "whether to publish lcm messages for OscTrackData");

DEFINE_bool(is_two_phase, true,
            "true: only right/left single support"
            "false: both double and single support");

//DEFINE_double(qp_time_limit, 0.002, "maximum qp solve time");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  /* ---  Common setup (MPC and OSC) ---*/
  // Read-in the parameters
  auto gains = drake::yaml::LoadYamlFile<OSCWalkingGainsALIP>(FLAGS_gains_filename);

  // Build Cassie MBP
  drake::multibody::MultibodyPlant<double> plant_w_spr(0.0);
  AddCassieMultibody(&plant_w_spr, nullptr, true /*floating base*/,
                     "examples/Cassie/urdf/cassie_v2.urdf",
                     true /*spring model*/, false /*loop closure*/);
  plant_w_spr.Finalize();
  auto context_w_spr = plant_w_spr.CreateDefaultContext();

  // Build the controller diagram
  DiagramBuilder<double> builder;

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");

  // Get contact frames and position
  auto left_toe = LeftToeFront(plant_w_spr);
  auto left_heel = LeftToeRear(plant_w_spr);
  auto right_toe = RightToeFront(plant_w_spr);
  auto right_heel = RightToeRear(plant_w_spr);

  // Get body frames and points
  Vector3d center_of_pressure = left_heel.first +
      gains.contact_point_pos * (left_toe.first - left_heel.first);
  auto left_toe_mid = std::pair<const Vector3d, const Frame<double>&>(
      center_of_pressure, plant_w_spr.GetFrameByName("toe_left"));
  auto right_toe_mid = std::pair<const Vector3d, const Frame<double>&>(
      center_of_pressure, plant_w_spr.GetFrameByName("toe_right"));
  auto left_toe_origin = std::pair<const Vector3d, const Frame<double>&>(
      Vector3d::Zero(), plant_w_spr.GetFrameByName("toe_left"));
  auto right_toe_origin = std::pair<const Vector3d, const Frame<double>&>(
      Vector3d::Zero(), plant_w_spr.GetFrameByName("toe_right"));

  // Create state receiver.
  auto state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_w_spr);

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

  // radio interface
  auto cassie_out_to_radio =
      builder.AddSystem<systems::CassieOutToRadio>();
  auto cassie_out_receiver =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_cassie_out>(
          FLAGS_cassie_out_channel, &lcm_local));
  auto high_level_command = builder.AddSystem<cassie::osc::HighLevelCommand>(
      plant_w_spr, context_w_spr.get(), gains.vel_scale_rot,
      gains.vel_scale_trans_sagital, gains.vel_scale_trans_lateral, 0.4);
  builder.Connect(*cassie_out_receiver, *cassie_out_to_radio);
  builder.Connect(cassie_out_to_radio->get_output_port(),
                  high_level_command->get_radio_port());

  builder.Connect(state_receiver->get_output_port(0),
                  high_level_command->get_state_input_port());


  /* --- MPC setup --- */
  std::vector<PointOnFramed> left_right_toe = {left_toe_mid, right_toe_mid};
  auto gains_mpc = AlipMINLPGains{
      0.1, 0.85, 0.2, 3, 10,
      5 * Matrix4d::Identity(), 0.1 * MatrixXd::Ones(1,1)};
  auto foot_placement_controller =
      builder.AddSystem<AlipMINLPFootstepController>(
          plant_w_spr, context_w_spr.get(), fsm_states, state_durations,
          left_right_toe, gains_mpc);
  ConvexFoothold big_square;
  big_square.SetContactPlane(Vector3d::UnitZ(), Vector3d::Zero()); // Flat Ground
  big_square.AddFace(Vector3d::UnitX(), 100 * Vector3d::UnitX());
  big_square.AddFace(Vector3d::UnitY(), 100 * Vector3d::UnitY());
  big_square.AddFace(-Vector3d::UnitX(), -100 * Vector3d::UnitX());
  big_square.AddFace(-Vector3d::UnitY(), -100 * Vector3d::UnitY());
  std::vector<ConvexFoothold> footholds = {big_square};
  auto foothold_oracle =
      builder.AddSystem<ConstantValueSource<double>>(
          drake::Value<std::vector<ConvexFoothold>>(footholds));

  auto footstep_sender = builder.AddSystem<FootstepSender>();
  auto footstep_pub_ptr = LcmPublisherSystem::Make<lcmt_footstep_target>(FLAGS_channel_foot, &lcm_local);
  auto footstep_pub = builder.AddSystem(std::move(footstep_pub_ptr));
  auto fsm_sender = builder.AddSystem<FsmSender>(plant_w_spr);
  auto fsm_pub_ptr = LcmPublisherSystem::Make<lcmt_fsm_info>(FLAGS_channel_fsm, &lcm_local);
  auto fsm_pub = builder.AddSystem(std::move(fsm_pub_ptr));
  auto com_traj_pub_ptr = LcmPublisherSystem::Make<lcmt_saved_traj>(FLAGS_channel_com, &lcm_local);
  auto com_traj_pub = builder.AddSystem(std::move(com_traj_pub_ptr));

  // --- Connect the mpc diagram subparts --- //
  // State Reciever connections
  builder.Connect(state_receiver->get_output_port(0),
                  foot_placement_controller->get_input_port_state());
  builder.Connect(state_receiver->get_output_port(0),
                  fsm_sender->get_input_port_state());

  // planner ports
  builder.Connect(high_level_command->get_xy_output_port(),
                  foot_placement_controller->get_input_port_vdes());
  builder.Connect(foothold_oracle->get_output_port(),
                  foot_placement_controller->get_input_port_footholds());

  // planner out ports
  builder.Connect(foot_placement_controller->get_output_port_fsm(),
                  fsm_sender->get_input_port_fsm());
  builder.Connect(foot_placement_controller->get_output_port_prev_impact_time(),
                  fsm_sender->get_input_port_prev_switch_time());
  builder.Connect(foot_placement_controller->get_output_port_next_impact_time(),
                  fsm_sender->get_input_port_next_switch_time());
  builder.Connect(foot_placement_controller->get_output_port_footstep_target(),
                  footstep_sender->get_input_port());
  builder.Connect(foot_placement_controller->get_output_port_com_traj(),
                  com_traj_pub->get_input_port());

  // misc
  builder.Connect(fsm_sender->get_output_port_fsm_info(),
                  fsm_pub->get_input_port());
  builder.Connect(*footstep_sender, *footstep_pub);
  std:: cout << "check1\n";

  /* --- OSC Setup ---*/

  // Create command sender.
  auto command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_channel_u, &lcm_local, TriggerTypeSet({TriggerType::kForced})));
  auto command_sender =
      builder.AddSystem<systems::RobotCommandSender>(plant_w_spr);

  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port());



  std:: cout << "check2\n";

  // Create translate com_traj from lcm
  auto com_traj_receiver = builder.AddSystem<LcmTrajectoryReceiver>(
      "com_traj", TrajectoryType::kCubicShapePreserving);
  builder.Connect(foot_placement_controller->get_output_port_com_traj(),
                  com_traj_receiver->get_input_port());

  // Create heading traj generator
  auto head_traj_gen = builder.AddSystem<cassie::osc::HeadingTrajGenerator>(
      plant_w_spr, context_w_spr.get());
  builder.Connect(state_receiver->get_output_port(),
                  head_traj_gen->get_state_input_port());
  builder.Connect(high_level_command->get_yaw_output_port(),
                  head_traj_gen->get_yaw_input_port());

  std::vector<int> single_support_states = {left_stance_state,
                                            right_stance_state};
  std::vector<int> double_support_states = {post_left_double_support_state,
                                            post_right_double_support_state};

  // Create swing leg trajectory generator
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


  vector<int> left_right_support_fsm_states = {left_stance_state,
                                               right_stance_state};
  vector<double> left_right_support_state_durations = {left_support_duration,
                                                       right_support_duration};
  vector<std::pair<const Vector3d, const Frame<double>&>> left_right_foot = {
      left_toe_origin, right_toe_origin};


  auto swing_ft_traj_generator = builder.AddSystem<SwingFootTargetTrajGen>(
      plant_w_spr, context_w_spr.get(), left_right_support_fsm_states,
      left_right_foot, gains.mid_foot_height, gains.final_foot_height,
      gains.final_foot_velocity_z, false);
  std:: cout << "check3\n";

  builder.Connect(foot_placement_controller->get_output_port_fsm(),
                  swing_ft_traj_generator->get_input_port_fsm());
  builder.Connect(foot_placement_controller->get_output_port_prev_impact_time(),
                  swing_ft_traj_generator->get_input_port_fsm_switch_time());
  builder.Connect(foot_placement_controller->get_output_port_next_impact_time(),
                  swing_ft_traj_generator->get_input_port_next_fsm_switch_time());
  builder.Connect(state_receiver->get_output_port(0),
                  swing_ft_traj_generator->get_input_port_state());
  builder.Connect(foot_placement_controller->get_output_port_footstep_target(),
                  swing_ft_traj_generator->get_input_port_footstep_target());
  std:: cout << "check4\n";

  // Swing toe joint trajectory
  map<string, int> pos_map = multibody::MakeNameToPositionsMap(plant_w_spr);
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
  builder.Connect(state_receiver->get_output_port(0),
                  left_toe_angle_traj_gen->get_state_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  right_toe_angle_traj_gen->get_state_input_port());

  // Create Operational space control
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      plant_w_spr, plant_w_spr, context_w_spr.get(), context_w_spr.get(), true,
      0);

  // Cost
  int n_v = plant_w_spr.num_velocities();
  int n_u = plant_w_spr.num_actuators();
  MatrixXd Q_accel = gains.w_accel * MatrixXd::Identity(n_v, n_v);
  osc->SetAccelerationCostWeights(Q_accel);
  osc->SetInputSmoothingWeights(gains.w_input_reg * MatrixXd::Identity(n_u, n_u));

  // Constraints in OSC
  multibody::KinematicEvaluatorSet<double> evaluators(plant_w_spr);

  // 1. fourbar constraint
  auto left_loop = LeftLoopClosureEvaluator(plant_w_spr);
  auto right_loop = RightLoopClosureEvaluator(plant_w_spr);
  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);

  // 2. fixed spring constraint
  auto pos_idx_map = multibody::MakeNameToPositionsMap(plant_w_spr);
  auto vel_idx_map = multibody::MakeNameToVelocitiesMap(plant_w_spr);
  auto left_fixed_knee_spring = FixedJointEvaluator<double>(
      plant_w_spr, pos_idx_map.at("knee_joint_left"),
      vel_idx_map.at("knee_joint_leftdot"), 0);
  auto right_fixed_knee_spring = FixedJointEvaluator<double>(
      plant_w_spr, pos_idx_map.at("knee_joint_right"),
      vel_idx_map.at("knee_joint_rightdot"), 0);
  auto left_fixed_ankle_spring = FixedJointEvaluator<double>(
      plant_w_spr, pos_idx_map.at("ankle_spring_joint_left"),
      vel_idx_map.at("ankle_spring_joint_leftdot"), 0);
  auto right_fixed_ankle_spring = FixedJointEvaluator<double>(
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
  osc->SetContactSoftConstraintWeight(gains.w_soft_constraint);
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

  osc->SetOsqpSolverOptionsFromYaml(
      "examples/perceptive_locomotion/gains/osqp_options.yaml");

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
      "swing_ft_traj", gains.K_p_swing_foot, gains.K_d_swing_foot,
      gains.W_swing_foot, plant_w_spr, plant_w_spr);
  swing_foot_data.AddStateAndPointToTrack(left_stance_state, "toe_right");
  swing_foot_data.AddStateAndPointToTrack(right_stance_state, "toe_left");

  auto vel_map = MakeNameToVelocitiesMap<double>(plant_w_spr);
  swing_foot_data.AddJointAndStateToIgnoreInJacobian(
      vel_map["hip_yaw_right"], left_stance_state);
  swing_foot_data.AddJointAndStateToIgnoreInJacobian(
      vel_map["hip_yaw_left"], right_stance_state);

  ComTrackingData com_data("com_data", gains.K_p_swing_foot,
                           gains.K_d_swing_foot, gains.W_swing_foot,
                           plant_w_spr, plant_w_spr);
  com_data.AddFiniteStateToTrack(left_stance_state);
  com_data.AddFiniteStateToTrack(right_stance_state);

//  RelativeTranslationTrackingData swing_ft_traj_local(
//      "swing_ft_traj", gains.K_p_swing_foot, gains.K_d_swing_foot,
//      gains.W_swing_foot, plant_w_spr, plant_w_spr, &swing_foot_data,
//      &com_data);
//  WorldYawViewFrame pelvis_view_frame(plant_w_spr.GetBodyByName("pelvis"));
//  swing_ft_traj_local.SetViewFrame(pelvis_view_frame);
//
//  swing_ft_traj_local.SetTimeVaryingGains(
//      swing_ft_gain_multiplier_gain_multiplier);
//  swing_ft_traj_local.SetFeedforwardAccelMultiplier(
//      swing_ft_accel_gain_multiplier_gain_multiplier);
  osc->AddTrackingData(&swing_foot_data);

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


  auto hip_yaw_traj_gen =
      builder.AddSystem<cassie::HipYawTrajGen>(left_stance_state);

  // Swing hip yaw joint tracking
  JointSpaceTrackingData swing_hip_yaw_traj(
      "swing_hip_yaw_traj", gains.K_p_hip_yaw, gains.K_d_hip_yaw,
      gains.W_hip_yaw, plant_w_spr, plant_w_spr);
  swing_hip_yaw_traj.AddStateAndJointToTrack(left_stance_state, "hip_yaw_right",
                                             "hip_yaw_rightdot");
  swing_hip_yaw_traj.AddStateAndJointToTrack(right_stance_state, "hip_yaw_left",
                                             "hip_yaw_leftdot");


  builder.Connect(cassie_out_to_radio->get_output_port(),
                  hip_yaw_traj_gen->get_radio_input_port());
  builder.Connect(foot_placement_controller->get_output_port_fsm(),
                  hip_yaw_traj_gen->get_fsm_input_port());
  osc->AddTrackingData(&swing_hip_yaw_traj);

  // Set double support duration for force blending
  osc->SetUpDoubleSupportPhaseBlending(
      double_support_duration, left_stance_state, right_stance_state,
      {post_left_double_support_state, post_right_double_support_state});

  osc->SetOsqpSolverOptionsFromYaml(
      "examples/perceptive_locomotion/gains/osqp_options.yaml");

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

  std:: cout << "check5\n";
  // Connect ports
  builder.Connect(state_receiver->get_output_port(0),
                  osc->get_robot_output_input_port());
  builder.Connect(foot_placement_controller->get_output_port_fsm(), osc->get_fsm_input_port());
  builder.Connect(com_traj_receiver->get_output_port(),
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

  std:: cout << "check6\n";
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
  owned_diagram->set_name("osc controller for alip_minlp");
  DrawAndSaveDiagramGraph(*owned_diagram, "../alip_minlp_and_osc");

  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm_local, std::move(owned_diagram), state_receiver, FLAGS_channel_x,
      true);
  auto& loop_context = loop.get_diagram_mutable_context();

  loop.Simulate();

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
