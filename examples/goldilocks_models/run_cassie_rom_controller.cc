// (This file is modified from examples/Cassie/run_osc_walking_controller.cc)

// TODO(yminchen): Keep in mind that you need to rotate the planned traj back
//  from local to global frame if the ROM is dependent on x, y or yaw.

#include <string>

#include <dairlib/lcmt_target_standing_height.hpp>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "dairlib/lcmt_dairlib_signal.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "dairlib/lcmt_timestamped_vector.hpp"
#include "dairlib/lcmt_trajectory_block.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/osc/heading_traj_generator.h"
#include "examples/Cassie/osc/high_level_command.h"
#include "examples/Cassie/osc/swing_toe_traj_generator.h"
#include "examples/Cassie/osc/walking_speed_control.h"
#include "examples/Cassie/simulator_drift.h"
#include "examples/goldilocks_models/controller/control_parameters.h"
#include "examples/goldilocks_models/controller/local_lipm_traj_gen.h"
#include "examples/goldilocks_models/controller/osc_rom_walking_gains.h"
#include "examples/goldilocks_models/controller/planned_traj_guard.h"
#include "examples/goldilocks_models/controller/saved_traj_receiver.h"
#include "examples/goldilocks_models/goldilocks_utils.h"
#include "examples/goldilocks_models/reduced_order_models.h"
#include "examples/goldilocks_models/rom_walking_gains.h"
#include "lcm/rom_planner_saved_trajectory.h"
#include "multibody/kinematic/fixed_joint_evaluator.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/multibody_utils.h"
#include "systems/controllers/fsm_event_time.h"
#include "systems/controllers/lipm_traj_gen.h"
#include "systems/controllers/osc/com_tracking_data.h"
#include "systems/controllers/osc/joint_space_tracking_data.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/controllers/osc/optimal_rom_tracking_data.h"
#include "systems/controllers/osc/options_tracking_data.h"
#include "systems/controllers/osc/osc_tracking_data.h"
#include "systems/controllers/osc/relative_translation_tracking_data.h"
#include "systems/controllers/osc/rot_space_tracking_data.h"
#include "systems/controllers/osc/trans_space_tracking_data.h"
#include "systems/controllers/swing_ft_traj_gen.h"
#include "systems/controllers/time_based_fsm.h"
#include "systems/dairlib_signal_lcm_systems.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/framework/output_vector.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/yaml/yaml_read_archive.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/primitives/multiplexer.h"

namespace dairlib::goldilocks_models {

using std::cout;
using std::endl;
using std::to_string;
using std::vector;

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::MatrixX;
using drake::VectorX;
using drake::multibody::Frame;
using drake::multibody::JacobianWrtVariable;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::TriggerTypeSet;
using drake::trajectories::PiecewisePolynomial;

using systems::OutputVector;
using systems::controllers::ComTrackingData;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::OptimalRomTrackingData;
using systems::controllers::RelativeTranslationTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;

using multibody::FixedJointEvaluator;

using multibody::JwrtqdotToJwrtv;

//
DEFINE_bool(broadcast, false,
            "broadcast between controller thread and planner thread");
DEFINE_bool(hardware, false, "");
DEFINE_bool(use_hardware_osc_gains, false, "");

//
DEFINE_bool(close_sim_gap, true,
            "Modify to close the gap between open loop and closed loop sim");

//

DEFINE_int32(iter, -1, "The iteration # of the model that you use");

DEFINE_double(stride_length, -10000, "set constant walking stride length");
DEFINE_double(stride_length_scaling, 1.0, "");
DEFINE_double(turning_rate, -10000, "set constant turning rate");
DEFINE_bool(start_with_left_stance, true, "");

DEFINE_string(init_traj_file_name, "",
              "This won't be necessary if the simulation is run at a much "
              "slower speed than realtime. "
              "An example file name is 0_rom_trajectory");
DEFINE_string(init_traj_file_path, "",
              "This won't be necessary if the simulation is run at a much "
              "slower speed than realtime. ");

//
DEFINE_string(channel_x, "CASSIE_STATE_SIMULATION",
              "LCM channel for receiving state. "
              "Use CASSIE_STATE_SIMULATION to get state from simulator, and "
              "use CASSIE_STATE_DISPATCHER to get state from state estimator");
DEFINE_string(channel_u, "ROM_WALKING",
              "The name of the channel which publishes command");
DEFINE_string(channel_fsm_t, "FSM_T",
              "LCM channel for sending fsm and time of latest liftoff event. ");
DEFINE_string(channel_y, "MPC_OUTPUT",
              "The name of the channel which receives MPC output");
DEFINE_string(channel_ik, "IK_OUTPUT",
              "The name of the channel which receives IK traj");
DEFINE_string(cassie_out_channel, "CASSIE_OUTPUT_ECHO",
              "The name of the channel to receive the cassie out structure "
              "from. (to get radio message)");

DEFINE_bool(publish_osc_data, true,
            "whether to publish lcm messages for OscTrackData");
DEFINE_bool(print_osc, false, "whether to print the osc debug message or not");

DEFINE_bool(is_two_phase, false,
            "true: only right/left single support"
            "false: both double and single support");

DEFINE_bool(use_IK, false, "use the IK approach or not");

DEFINE_bool(get_swing_foot_from_planner, false, "");
DEFINE_bool(get_stance_hip_angles_from_planner, false, "");
DEFINE_bool(get_swing_hip_angle_from_planner, false, "");

// Simulated robot
DEFINE_bool(spring_model, true, "Use a URDF with or without legs springs");

// For testing
DEFINE_double(drift_rate, 0.0, "Drift rate for floating-base state");

// Testing
DEFINE_string(lcm_url_port, "7667", "port number. Should be > 1024");
DEFINE_string(path_wait_identifier, "", "");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_broadcast) {
    DRAKE_DEMAND(FLAGS_lcm_url_port == "7667");
  }

  // Read-in the parameters
  RomWalkingGains gains;
  const YAML::Node& root = YAML::LoadFile(FindResourceOrThrow(GAINS_FILENAME));
  drake::yaml::YamlReadArchive(root).Accept(&gains);
  OSCRomWalkingGains osc_gains;
  std::string osc_gains_filename = "";
  if (FLAGS_hardware || FLAGS_use_hardware_osc_gains) {
    osc_gains_filename =
        "examples/goldilocks_models/controller/"
        "osc_rom_walking_gains_hardware.yaml";
  } else {
    if (FLAGS_close_sim_gap) {
      // TODO: we haven't found the optimal gains (if we replan from previous
      //  solution, then we don't need to have strong gains)
      osc_gains_filename =
          "examples/goldilocks_models/controller/"
          "osc_rom_walking_gains_simulation_for_sim_eval.yaml";
      //      osc_gains_filename =
      //          "examples/goldilocks_models/controller/"
      //          "osc_rom_walking_gains_simulation.yaml";
    } else {
      osc_gains_filename =
          "examples/goldilocks_models/controller/"
          "osc_rom_walking_gains_simulation.yaml";
    }
  }
  cout << "osc_gains_filename = " << osc_gains_filename << "\n\n";
  const YAML::Node& root2 =
      YAML::LoadFile(FindResourceOrThrow(osc_gains_filename));
  drake::yaml::YamlReadArchive(root2).Accept(&osc_gains);

  if (gains.use_virtual_radio) {
    cout << "Set `use_radio` to true because `use_virtual_radio` = true\n\n";
    gains.use_radio = true;
  }

  if (FLAGS_stride_length > -100) {
    gains.set_constant_walking_speed = true;
    gains.constant_step_length_x = FLAGS_stride_length;
  }
  gains.constant_step_length_x *= FLAGS_stride_length_scaling;
  if (FLAGS_turning_rate > -100) {
    gains.set_constant_turning_rate = true;
    gains.constant_turning_rate = FLAGS_turning_rate;
  }

  if (gains.use_radio) {
    gains.set_constant_walking_speed = false;
    gains.set_constant_turning_rate = false;
  }

  if (gains.set_constant_walking_speed && !gains.set_constant_turning_rate) {
    // We require global_target_position_x so that the desired yaw position is
    // towards x axis direction
    DRAKE_DEMAND(gains.global_target_position_x >= 100);
  }

  // TODO: (low priority) Check why it can only turn 180 degree with constant
  //  walking turning rate.
  //  20220103 Update: There were two causes. Fixed in master.
  // DRAKE_DEMAND(!gains.set_constant_turning_rate);

  // Some checks for sim gap heurisitc
  bool close_sim_gap = FLAGS_close_sim_gap;
  if (FLAGS_hardware) {
    cout << "Since we are running on hardware, we set `close_sim_gap` to "
            "false\n";
    close_sim_gap = false;
  }
  if (close_sim_gap) {
    // Currently open loop traj doesn't have double support phase, so I added a
    // check here. The double support phase contributes to the sim gap.
    DRAKE_DEMAND(gains.mid_foot_height == 0.05);
    DRAKE_DEMAND(gains.double_support_duration == 0);
    DRAKE_DEMAND(!gains.constant_rom_vel_during_double_support);
    DRAKE_DEMAND(FLAGS_get_swing_foot_from_planner);
    DRAKE_DEMAND(!gains.relative_swing_ft);  // I guess this doesn't matter
    // TODO: you can add more checks here if you find important factors in the
    //  future
  }
  // More checks for hardware setting
  if (FLAGS_hardware) {
    if (gains.double_support_duration > 0) {
      DRAKE_DEMAND(gains.constant_rom_vel_during_double_support);
    }
  }

  // Build Cassie MBP
  std::string urdf = FLAGS_spring_model
                         ? "examples/Cassie/urdf/cassie_v2.urdf"
                         : "examples/Cassie/urdf/cassie_fixed_springs.urdf";
  drake::multibody::MultibodyPlant<double> plant_w_spr(0.0);
  addCassieMultibody(&plant_w_spr, nullptr, true /*floating base*/, urdf,
                     FLAGS_spring_model, false /*loop closure*/);
  plant_w_spr.Finalize();
  plant_w_spr.set_name("cassie_with_spring");
  // Build fix-spring Cassie MBP
  drake::multibody::MultibodyPlant<double> plant_wo_springs(0.0);
  addCassieMultibody(&plant_wo_springs, nullptr, true,
                     "examples/Cassie/urdf/cassie_fixed_springs.urdf", false,
                     false);
  plant_wo_springs.Finalize();
  plant_wo_springs.set_name("cassie_without_spring");

  auto context_w_spr = plant_w_spr.CreateDefaultContext();
  auto context_wo_spr = plant_wo_springs.CreateDefaultContext();

  // Get contact frames and position (doesn't matter whether we use
  // plant_w_spr or plant_wo_springs because the contact frames exit in both
  // plants)
  auto left_toe = LeftToeFront(plant_wo_springs);
  auto left_heel = LeftToeRear(plant_wo_springs);
  auto right_toe = RightToeFront(plant_wo_springs);
  auto right_heel = RightToeRear(plant_wo_springs);

  // Reduced order model
  std::unique_ptr<ReducedOrderModel> rom =
      CreateRom(gains.rom_option, 1 /*robot_option*/, plant_wo_springs, true);
  ReadModelParameters(rom.get(), gains.dir_model,
                      FLAGS_iter > 0 ? FLAGS_iter : gains.model_iter);

  // Mirrored reduced order model
  int robot_option = 1;
  StateMirror state_mirror(
      MirrorPosIndexMap(plant_wo_springs, robot_option),
      MirrorPosSignChangeSet(plant_wo_springs, robot_option),
      MirrorVelIndexMap(plant_wo_springs, robot_option),
      MirrorVelSignChangeSet(plant_wo_springs, robot_option));
  MirroredReducedOrderModel mirrored_rom(plant_wo_springs, *rom, state_mirror);

  // Build the controller diagram
  DiagramBuilder<double> builder;

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:" + FLAGS_lcm_url_port +
                                 "?ttl=0");
  drake::lcm::DrakeLcm lcm_network("udpm://239.255.76.67:7667?ttl=1");

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

  // Create finite state machine
  int left_stance_state = gains.left_support;
  int right_stance_state = gains.right_support;
  int post_left_double_support_state = gains.post_left_double_support;
  int post_right_double_support_state = gains.post_right_double_support;
  double left_support_duration = gains.left_support_duration;
  double right_support_duration = gains.right_support_duration;
  double double_support_duration = gains.double_support_duration;
  bool is_two_phase = FLAGS_is_two_phase || (double_support_duration == 0);
  if (FLAGS_hardware) {
    // We need double support phase for hardware (at least some sort of command
    // blending)
    DRAKE_DEMAND(!is_two_phase);
  }
  vector<int> fsm_states;
  vector<double> state_durations;
  double stride_period = left_support_duration + double_support_duration;
  if (is_two_phase) {
    if (!FLAGS_start_with_left_stance) {
      fsm_states = {right_stance_state, left_stance_state};
      state_durations = {right_support_duration, left_support_duration};
    } else {
      fsm_states = {left_stance_state, right_stance_state};
      state_durations = {left_support_duration, right_support_duration};
    }
  } else {
    if (!FLAGS_start_with_left_stance) {
      fsm_states = {right_stance_state, post_right_double_support_state,
                    left_stance_state, post_left_double_support_state};
      state_durations = {right_support_duration, double_support_duration,
                         left_support_duration, double_support_duration};
    } else {
      fsm_states = {left_stance_state, post_left_double_support_state,
                    right_stance_state, post_right_double_support_state};
      state_durations = {left_support_duration, double_support_duration,
                         right_support_duration, double_support_duration};
    }
  }
  auto fsm = builder.AddSystem<systems::TimeBasedFiniteStateMachineWithTrigger>(
      plant_w_spr, fsm_states, state_durations, FLAGS_hardware, stride_period);
  builder.Connect(simulator_drift->get_output_port(0),
                  fsm->get_input_port_state());
  if (FLAGS_hardware) {
    // Create Lcm subsriber for lcmt_target_standing_height. This is used to
    // trigger the FSM
    auto trigger_receiver = builder.AddSystem(
        LcmSubscriberSystem::Make<dairlib::lcmt_target_standing_height>(
            "MPC_SWITCH", &lcm_local));
    builder.Connect(trigger_receiver->get_output_port(),
                    fsm->get_input_port_trigger());
  }

  // Create leafsystem that record the switching time of the FSM
  std::vector<int> single_support_states = {left_stance_state,
                                            right_stance_state};
  auto event_time = builder.AddSystem<systems::FiniteStateMachineEventTime>(
      plant_w_spr, single_support_states,
      FLAGS_hardware ? -1 : -std::numeric_limits<int>::infinity(),
      FLAGS_hardware);
  builder.Connect(fsm->get_output_port_fsm(), event_time->get_input_port_fsm());
  builder.Connect(simulator_drift->get_output_port(0),
                  event_time->get_input_port_state());

  // Create human high-level control
  Eigen::Vector2d global_target_position(gains.global_target_position_x,
                                         gains.global_target_position_y);
  cout << "global_target_position = " << global_target_position.transpose()
       << endl;
  Eigen::Vector2d params_of_no_turning(gains.yaw_deadband_blur,
                                       gains.yaw_deadband_radius);
  cassie::osc::HighLevelCommand* high_level_command;
  if (gains.use_radio) {
    high_level_command = builder.AddSystem<cassie::osc::HighLevelCommand>(
        plant_w_spr, context_w_spr.get(), gains.vel_scale_rot,
        gains.vel_scale_trans_sagital, gains.vel_scale_trans_lateral);
    auto cassie_out_receiver =
        builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_cassie_out>(
            gains.use_virtual_radio ? "CASSIE_OUTPUT_ONLY_RADIO"
                                    : FLAGS_cassie_out_channel,
            &lcm_local));
    builder.Connect(cassie_out_receiver->get_output_port(),
                    high_level_command->get_cassie_output_port());
  } else {
    high_level_command = builder.AddSystem<cassie::osc::HighLevelCommand>(
        plant_w_spr, context_w_spr.get(), gains.kp_yaw, gains.kd_yaw,
        gains.vel_max_yaw, gains.kp_pos_sagital, gains.kd_pos_sagital,
        gains.vel_max_sagital, gains.kp_pos_lateral, gains.kd_pos_lateral,
        gains.vel_max_lateral, gains.target_pos_offset, global_target_position,
        params_of_no_turning);
  }
  builder.Connect(state_receiver->get_output_port(0),
                  high_level_command->get_state_input_port());

  // Create a multiplexer which combines current finite state machine state
  // and the latest lift-off event time, and create publisher for this
  // combined vector
  std::vector<int> input_size = {1, 1, 1, 2};
  auto mux = builder.AddSystem<drake::systems::Multiplexer<double>>(input_size);
  builder.Connect(fsm->get_output_port(0), mux->get_input_port(0));
  builder.Connect(event_time->get_output_port_event_time_of_interest(),
                  mux->get_input_port(1));
  builder.Connect(fsm->get_output_port_global_fsm_idx(),
                  mux->get_input_port(2));
  builder.Connect(high_level_command->get_xy_output_port(),
                  mux->get_input_port(3));

  std::vector<std::string> signal_names = {"fsm", "t_lo", "fsm_idx", "des_vx",
                                           "des_vy"};
  auto fsm_and_liftoff_time_sender =
      builder.AddSystem<systems::DairlibSignalSender>(
          signal_names, left_support_duration + double_support_duration);
  builder.Connect(mux->get_output_port(0),
                  fsm_and_liftoff_time_sender->get_input_port(0));
  auto fsm_and_liftoff_time_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_dairlib_signal>(
          FLAGS_channel_fsm_t, FLAGS_broadcast ? &lcm_network : &lcm_local,
          TriggerTypeSet({TriggerType::kForced})));
  builder.Connect(fsm_and_liftoff_time_sender->get_output_port(0),
                  fsm_and_liftoff_time_publisher->get_input_port());

  // Evaluators for OSC constraints
  multibody::KinematicEvaluatorSet<double> evaluators(plant_wo_springs);
  // 1. fourbar constraint
  auto left_loop = LeftLoopClosureEvaluator(plant_wo_springs);
  auto right_loop = RightLoopClosureEvaluator(plant_wo_springs);
  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);
  // Note that we are still using fixed-spring model in OSC, so we don't need
  // the spring constraint below
  // 2. fixed spring constriant
  // Note that we set the position value to 0, but this is not used in OSC,
  // because OSC constraint only use JdotV and J.
  /*auto pos_idx_map = multibody::makeNameToPositionsMap(plant_w_spr);
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
  evaluators.add_evaluator(&right_fixed_ankle_spring);*/
  // 3. Contact points (The position doesn't matter. It's not used in OSC)
  multibody::WorldYawViewFrame view_frame(
      plant_wo_springs.GetBodyByName("pelvis"));
  multibody::WorldYawViewFrame view_frame_w_spring(
      plant_w_spr.GetBodyByName("pelvis"));
  auto left_toe_evaluator = multibody::WorldPointEvaluator(
      plant_wo_springs, left_toe.first, left_toe.second, view_frame,
      Matrix3d::Identity(), Vector3d::Zero(), {1, 2});
  auto left_heel_evaluator = multibody::WorldPointEvaluator(
      plant_wo_springs, left_heel.first, left_heel.second, view_frame,
      Matrix3d::Identity(), Vector3d::Zero(), {0, 1, 2});
  auto right_toe_evaluator = multibody::WorldPointEvaluator(
      plant_wo_springs, right_toe.first, right_toe.second, view_frame,
      Matrix3d::Identity(), Vector3d::Zero(), {1, 2});
  auto right_heel_evaluator = multibody::WorldPointEvaluator(
      plant_wo_springs, right_heel.first, right_heel.second, view_frame,
      Matrix3d::Identity(), Vector3d::Zero(), {0, 1, 2});

  if (!FLAGS_use_IK) {
    ///
    /// Non IK conroller
    ///

    bool wrt_com_in_local_frame = gains.relative_swing_ft;

    // Create Lcm subscriber for MPC's output
    auto planner_output_subscriber = builder.AddSystem(
        LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
            FLAGS_channel_y, &lcm_local));

    // Create a system that translate MPC lcm into trajectory
    vector<std::pair<const Vector3d, const Frame<double>&>> left_right_foot = {
        left_toe_origin, right_toe_origin};
    vector<int> left_right_support_fsm_states = {left_stance_state,
                                                 right_stance_state};
    auto planner_traj_receiver = builder.AddSystem<SavedTrajReceiver>(
        *rom, plant_w_spr, plant_wo_springs, context_w_spr.get(),
        left_right_foot, left_right_support_fsm_states, left_support_duration,
        double_support_duration, gains.mid_foot_height, gains.final_foot_height,
        gains, state_mirror, view_frame_w_spring, view_frame,
        wrt_com_in_local_frame);
    builder.Connect(planner_output_subscriber->get_output_port(),
                    planner_traj_receiver->get_input_port_lcm_traj());
    builder.Connect(fsm->get_output_port(0),
                    planner_traj_receiver->get_input_port_fsm());
    builder.Connect(simulator_drift->get_output_port(0),
                    planner_traj_receiver->get_input_port_state());

    // Create heading traj generator
    auto head_traj_gen = builder.AddSystem<cassie::osc::HeadingTrajGenerator>(
        plant_w_spr, context_w_spr.get());
    builder.Connect(simulator_drift->get_output_port(0),
                    head_traj_gen->get_state_input_port());
    if (!gains.set_constant_turning_rate) {
      builder.Connect(high_level_command->get_yaw_output_port(),
                      head_traj_gen->get_yaw_input_port());
    }

    // Create CoM trajectory generator
    double desired_com_height = gains.lipm_height;
    vector<int> unordered_fsm_states;
    vector<double> unordered_state_durations;
    vector<vector<std::pair<const Vector3d, const Frame<double>&>>>
        contact_points_in_each_state;
    if (is_two_phase) {
      unordered_fsm_states = {left_stance_state, right_stance_state};
      unordered_state_durations = {left_support_duration,
                                   right_support_duration};
      contact_points_in_each_state.push_back({left_toe_mid});
      contact_points_in_each_state.push_back({right_toe_mid});
    } else {
      unordered_fsm_states = {left_stance_state, right_stance_state,
                              post_left_double_support_state,
                              post_right_double_support_state};
      unordered_state_durations = {
          left_support_duration, right_support_duration,
          double_support_duration, double_support_duration};
      contact_points_in_each_state.push_back({left_toe_mid});
      contact_points_in_each_state.push_back({right_toe_mid});
      contact_points_in_each_state.push_back({left_toe_mid, right_toe_mid});
      contact_points_in_each_state.push_back({left_toe_mid, right_toe_mid});
    }
    auto lipm_traj_generator = builder.AddSystem<systems::LIPMTrajGenerator>(
        plant_w_spr, context_w_spr.get(), desired_com_height,
        unordered_fsm_states, unordered_state_durations,
        contact_points_in_each_state);
    builder.Connect(fsm->get_output_port(0),
                    lipm_traj_generator->get_input_port_fsm());
    builder.Connect(event_time->get_output_port_event_time(),
                    lipm_traj_generator->get_input_port_touchdown_time());
    builder.Connect(simulator_drift->get_output_port(0),
                    lipm_traj_generator->get_input_port_state());

    // Create velocity control by foot placement
    auto walking_speed_control =
        builder.AddSystem<cassie::osc::WalkingSpeedControl>(
            plant_w_spr, context_w_spr.get(), gains.k_ff_lateral,
            gains.k_fb_lateral, gains.k_ff_sagittal, gains.k_fb_sagittal,
            left_support_duration, gains.speed_control_offset_sagittal,
            wrt_com_in_local_frame);
    if (!gains.set_constant_walking_speed) {
      builder.Connect(high_level_command->get_xy_output_port(),
                      walking_speed_control->get_input_port_des_hor_vel());
    }
    builder.Connect(simulator_drift->get_output_port(0),
                    walking_speed_control->get_input_port_state());
    builder.Connect(lipm_traj_generator->get_output_port(0),
                    walking_speed_control->get_input_port_com());
    builder.Connect(event_time->get_output_port_event_time_of_interest(),
                    walking_speed_control->get_input_port_fsm_switch_time());

    // Create swing leg trajectory generator
    vector<double> left_right_support_state_durations = {
        left_support_duration, right_support_duration};
    auto swing_ft_traj_generator =
        builder.AddSystem<systems::SwingFootTrajGenerator>(
            plant_w_spr, context_w_spr.get(), left_right_support_fsm_states,
            left_right_support_state_durations, left_right_foot, "pelvis",
            double_support_duration, gains.mid_foot_height,
            gains.final_foot_height, gains.final_foot_velocity_z,
            gains.max_CoM_to_footstep_dist, gains.footstep_offset,
            gains.center_line_offset, wrt_com_in_local_frame);
    builder.Connect(fsm->get_output_port(0),
                    swing_ft_traj_generator->get_input_port_fsm());
    builder.Connect(event_time->get_output_port_event_time_of_interest(),
                    swing_ft_traj_generator->get_input_port_fsm_switch_time());
    builder.Connect(simulator_drift->get_output_port(0),
                    swing_ft_traj_generator->get_input_port_state());
    builder.Connect(lipm_traj_generator->get_output_port(0),
                    swing_ft_traj_generator->get_input_port_com());
    builder.Connect(walking_speed_control->get_output_port(0),
                    swing_ft_traj_generator->get_input_port_sc());

    // Swing toe joint trajectory
    std::map<std::string, int> pos_map =
        multibody::makeNameToPositionsMap(plant_w_spr);
    vector<std::pair<const Vector3d, const Frame<double>&>> left_foot_points = {
        left_heel, left_toe};
    vector<std::pair<const Vector3d, const Frame<double>&>> right_foot_points =
        {right_heel, right_toe};
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

    /*// lipm traj for ROM (com wrt to stance foot)
    vector<bool> flip_in_y;
    if (is_two_phase) {
      flip_in_y = {false, true};
    } else {
      flip_in_y = {false, true, false, true};
    }
    auto local_lipm_traj_generator =
        builder.AddSystem<systems::LocalLIPMTrajGenerator>(
            plant_w_spr, context_w_spr.get(), desired_com_height,
            unordered_fsm_states, unordered_state_durations,
            contact_points_in_each_state, flip_in_y);
    builder.Connect(fsm->get_output_port(0),
                    local_lipm_traj_generator->get_input_port_fsm());
    builder.Connect(event_time->get_output_port_event_time(),
                    local_lipm_traj_generator->get_input_port_fsm_switch_time());
    builder.Connect(simulator_drift->get_output_port(0),
                    local_lipm_traj_generator->get_input_port_state());

    // Create a guard for the planner in case it doesn't finish solving in time
    auto optimal_traj_planner_guard =
        builder.AddSystem<goldilocks_models::PlannedTrajGuard>();
    builder.Connect(
        planner_traj_receiver->get_output_port(0),
        optimal_traj_planner_guard->get_input_port_optimal_rom_traj());
    builder.Connect(local_lipm_traj_generator->get_output_port(0),
                    optimal_traj_planner_guard->get_input_port_lipm_traj());*/

    // Create Operational space control
    auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
        plant_w_spr, plant_wo_springs, context_w_spr.get(),
        context_wo_spr.get(), true, FLAGS_print_osc /*print_tracking_info*/, 0,
        false, close_sim_gap);

    // Scaling weight for testing
    // Didn't help. Might need to use the new osqp solver to see improvement
    double weight_scale = 1;

    // Cost
    int n_v = plant_wo_springs.num_velocities();
    MatrixXd Q_accel =
        weight_scale * osc_gains.w_accel * MatrixXd::Identity(n_v, n_v);
    osc->SetAccelerationCostForAllJoints(Q_accel);
    if (!close_sim_gap) {
      osc->SetInputRegularizationWeight(osc_gains.w_input_reg);
    }

    // Constraints in OSC
    osc->AddKinematicConstraint(&evaluators);

    // Soft constraint
    // w_contact_relax shouldn't be too big, cause we want tracking error to be
    // important
    osc->SetWeightOfSoftContactConstraint(weight_scale *
                                          osc_gains.w_soft_constraint);
    // Friction coefficient
    osc->SetContactFriction(osc_gains.mu);
    // Add contact points
    osc->AddStateAndContactPoint(left_stance_state, &left_toe_evaluator);
    osc->AddStateAndContactPoint(left_stance_state, &left_heel_evaluator);
    osc->AddStateAndContactPoint(right_stance_state, &right_toe_evaluator);
    osc->AddStateAndContactPoint(right_stance_state, &right_heel_evaluator);
    if (!is_two_phase) {
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

    // Gain scheduling
    std::vector<double> swing_ft_gain_multiplier_breaks{
        0, left_support_duration / 2, left_support_duration};
    std::vector<drake::MatrixX<double>> swing_ft_gain_multiplier_samples(
        3, drake::MatrixX<double>::Identity(3, 3));
    swing_ft_gain_multiplier_samples[2](2, 2) *= 0.3;
    PiecewisePolynomial<double> swing_ft_gain_multiplier =
        PiecewisePolynomial<double>::FirstOrderHold(
            swing_ft_gain_multiplier_breaks, swing_ft_gain_multiplier_samples);
    std::vector<double> swing_ft_accel_gain_multiplier_breaks{
        0, left_support_duration / 4, left_support_duration / 2,
        left_support_duration * 3 / 4, left_support_duration};
    std::vector<drake::MatrixX<double>> swing_ft_accel_gain_multiplier_samples(
        5, drake::MatrixX<double>::Identity(3, 3));
    //    swing_ft_accel_gain_multiplier_samples[0](2, 2) *= 0;
    //    swing_ft_accel_gain_multiplier_samples[1](2, 2) *= 0;
    swing_ft_accel_gain_multiplier_samples[3](2, 2) *= 0;
    swing_ft_accel_gain_multiplier_samples[4](2, 2) *= 0;
    PiecewisePolynomial<double> swing_ft_accel_gain_multiplier =
        PiecewisePolynomial<double>::FirstOrderHold(
            swing_ft_accel_gain_multiplier_breaks,
            swing_ft_accel_gain_multiplier_samples);

    // Swing foot tracking
    TransTaskSpaceTrackingData swing_foot_data(
        "swing_ft_data", osc_gains.K_p_swing_foot, osc_gains.K_d_swing_foot,
        osc_gains.W_swing_foot, plant_w_spr, plant_wo_springs);
    swing_foot_data.AddStateAndPointToTrack(left_stance_state, "toe_right");
    swing_foot_data.AddStateAndPointToTrack(right_stance_state, "toe_left");
    ComTrackingData com_data("com_data", osc_gains.K_p_swing_foot,
                             osc_gains.K_d_swing_foot, osc_gains.W_swing_foot,
                             plant_w_spr, plant_wo_springs);
    com_data.AddFiniteStateToTrack(left_stance_state);
    com_data.AddFiniteStateToTrack(right_stance_state);
    RelativeTranslationTrackingData swing_ft_traj_local(
        "swing_ft_traj", osc_gains.K_p_swing_foot, osc_gains.K_d_swing_foot,
        osc_gains.W_swing_foot, plant_w_spr, plant_wo_springs, &swing_foot_data,
        &com_data);
    swing_ft_traj_local.SetViewFrame(view_frame_w_spring);

    TransTaskSpaceTrackingData swing_ft_traj_global(
        "swing_ft_traj", osc_gains.K_p_swing_foot, osc_gains.K_d_swing_foot,
        osc_gains.W_swing_foot, plant_w_spr, plant_wo_springs);
    swing_ft_traj_global.AddStateAndPointToTrack(left_stance_state,
                                                 "toe_right");
    swing_ft_traj_global.AddStateAndPointToTrack(right_stance_state,
                                                 "toe_left");

    if (wrt_com_in_local_frame) {
      swing_ft_traj_local.SetTimeVaryingGains(swing_ft_gain_multiplier);
      swing_ft_traj_local.SetFeedforwardAccelMultiplier(
          swing_ft_accel_gain_multiplier);
      osc->AddTrackingData(&swing_ft_traj_local);
    } else {
      swing_ft_traj_global.SetTimeVaryingGains(swing_ft_gain_multiplier);
      swing_ft_traj_global.SetFeedforwardAccelMultiplier(
          swing_ft_accel_gain_multiplier);
      osc->AddTrackingData(&swing_ft_traj_global);
    }

    // "Center of mass" tracking (Using RomTrackingData with initial ROM being
    // COM)
    OptimalRomTrackingData optimal_rom_traj(
        "optimal_rom_traj", rom->n_y(), osc_gains.K_p_rom, osc_gains.K_d_rom,
        weight_scale * osc_gains.W_rom, plant_w_spr, plant_wo_springs);
    optimal_rom_traj.AddStateAndRom(left_stance_state, *rom);
    optimal_rom_traj.AddStateAndRom(post_left_double_support_state, *rom);
    optimal_rom_traj.AddStateAndRom(right_stance_state, mirrored_rom);
    optimal_rom_traj.AddStateAndRom(post_right_double_support_state,
                                    mirrored_rom);
    std::vector<double> rom_ratio_breaks{0, left_support_duration / 2,
                                         left_support_duration};
    std::vector<MatrixX<double>> rom_ratio_samples(
        3, MatrixX<double>::Identity(3, 3));
    rom_ratio_samples[0] *= 0;
    PiecewisePolynomial<double> rom_gain_ratio =
        PiecewisePolynomial<double>::FirstOrderHold(rom_ratio_breaks,
                                                    rom_ratio_samples);
    optimal_rom_traj.SetTimeVaryingGains(rom_gain_ratio);
    if (is_two_phase) {
      // With high gains, the initial error could throw Cassie into the air
      // If the new stance foot in the beginning of stance phase is not on the
      // ground, the error combining with high gains can throw Cassie into air
      // with optimaized ROM.
      // One solution is to soften the ROM tracking gains in the beginning of
      // stance, and the other solution is to make sure the swing foot is on the
      // ground before switch. (Update: The later solution doesn't fix the issue
      // for the optimaized models, because it's not physically interpreted as
      // height anymore)
      //      osc->AddTrackingData(&optimal_rom_traj, 0.02);
      osc->AddTrackingData(&optimal_rom_traj);
    } else {
      osc->AddTrackingData(&optimal_rom_traj);
    }

    // Stance hip roll, pitch and yaw
    MatrixXd W_stance_hip_rpy = MatrixXd::Identity(3, 3);
    MatrixXd K_p_stance_hip_rpy = (osc_gains.K_p_swing_foot(2, 2) > 400)
                                      ? 500 * MatrixXd::Identity(3, 3)
                                      : 200 * MatrixXd::Identity(3, 3);
    MatrixXd K_d_stance_hip_rpy = 5 * MatrixXd::Identity(3, 3);
    JointSpaceTrackingData stance_hip_rpy_traj(
        "stance_hip_rpy_traj", K_p_stance_hip_rpy, K_d_stance_hip_rpy,
        weight_scale * W_stance_hip_rpy, plant_w_spr, plant_wo_springs);
    stance_hip_rpy_traj.AddStateAndJointsToTrack(
        left_stance_state, {"hip_roll_left", "hip_pitch_left", "hip_yaw_left"},
        {"hip_roll_leftdot", "hip_pitch_leftdot", "hip_yaw_leftdot"});
    stance_hip_rpy_traj.AddStateAndJointsToTrack(
        post_left_double_support_state,
        {"hip_roll_left", "hip_pitch_left", "hip_yaw_left"},
        {"hip_roll_leftdot", "hip_pitch_leftdot", "hip_yaw_leftdot"});
    stance_hip_rpy_traj.AddStateAndJointsToTrack(
        right_stance_state,
        {"hip_roll_right", "hip_pitch_right", "hip_yaw_right"},
        {"hip_roll_rightdot", "hip_pitch_rightdot", "hip_yaw_rightdot"});
    stance_hip_rpy_traj.AddStateAndJointsToTrack(
        post_right_double_support_state,
        {"hip_roll_right", "hip_pitch_right", "hip_yaw_right"},
        {"hip_roll_rightdot", "hip_pitch_rightdot", "hip_yaw_rightdot"});
    if (FLAGS_get_stance_hip_angles_from_planner) {
      osc->AddTrackingData(&stance_hip_rpy_traj);
    }

    // Heuristics -- Pelvis rotation tracking (pitch and roll)
    RotTaskSpaceTrackingData pelvis_balance_traj(
        "pelvis_balance_traj", osc_gains.K_p_pelvis_balance,
        osc_gains.K_d_pelvis_balance, weight_scale * osc_gains.W_pelvis_balance,
        plant_w_spr, plant_wo_springs);
    pelvis_balance_traj.AddFrameToTrack("pelvis");
    std::vector<double> balance_ratio_breaks{0, left_support_duration / 2,
                                             left_support_duration};
    std::vector<MatrixX<double>> balance_ratio_samples(
        3, MatrixX<double>::Identity(3, 3));
    balance_ratio_samples[0] *= 0.5;
    PiecewisePolynomial<double> balance_gain_ratio =
        PiecewisePolynomial<double>::FirstOrderHold(balance_ratio_breaks,
                                                    balance_ratio_samples);
    pelvis_balance_traj.SetTimeVaryingGains(balance_gain_ratio);
    if (!FLAGS_get_stance_hip_angles_from_planner) {
      osc->AddTrackingData(&pelvis_balance_traj);
    }

    // Heuristics -- Pelvis rotation tracking (yaw)
    RotTaskSpaceTrackingData pelvis_heading_traj(
        "pelvis_heading_traj", osc_gains.K_p_pelvis_heading,
        osc_gains.K_d_pelvis_heading, weight_scale * osc_gains.W_pelvis_heading,
        plant_w_spr, plant_wo_springs);
    pelvis_heading_traj.AddFrameToTrack("pelvis");
    /*std::vector<double> heading_breaks{0, left_support_duration / 2,
                                       left_support_duration};
    std::vector<MatrixX<double>> heading_samples(
        3, MatrixX<double>::Identity(3, 3));
    heading_samples[0] *= 0.5;
    PiecewisePolynomial<double> heading_gain_ratio =
        PiecewisePolynomial<double>::FirstOrderHold(heading_breaks,
                                                    heading_samples);
    pelvis_heading_traj.SetTimeVaryingGains(heading_gain_ratio);*/
    if (!FLAGS_get_stance_hip_angles_from_planner) {
      osc->AddTrackingData(&pelvis_heading_traj,
                           osc_gains.period_of_no_heading_control);
    }

    // Swing hip yaw joint tracking
    JointSpaceTrackingData swing_hip_yaw_traj(
        "swing_hip_yaw_traj", osc_gains.K_p_hip_yaw, osc_gains.K_d_hip_yaw,
        weight_scale * osc_gains.W_hip_yaw, plant_w_spr, plant_wo_springs);
    swing_hip_yaw_traj.AddStateAndJointToTrack(
        left_stance_state, "hip_yaw_right", "hip_yaw_rightdot");
    swing_hip_yaw_traj.AddStateAndJointToTrack(
        right_stance_state, "hip_yaw_left", "hip_yaw_leftdot");
    std::vector<double> hip_yaw_ratio_breaks{0, left_support_duration / 2,
                                             left_support_duration};
    std::vector<MatrixX<double>> hip_yaw_ratio_samples(
        3, MatrixX<double>::Identity(1, 1));
    if (!FLAGS_get_swing_hip_angle_from_planner) {
      hip_yaw_ratio_samples[0] *= 0;
    }
    PiecewisePolynomial<double> hip_yaw_gain_ratio =
        PiecewisePolynomial<double>::FirstOrderHold(hip_yaw_ratio_breaks,
                                                    hip_yaw_ratio_samples);
    swing_hip_yaw_traj.SetTimeVaryingGains(hip_yaw_gain_ratio);
    if (!FLAGS_get_swing_hip_angle_from_planner) {
      osc->AddConstTrackingData(&swing_hip_yaw_traj, VectorXd::Zero(1));
    } else {
      osc->AddTrackingData(&swing_hip_yaw_traj);
    }

    // Swing toe joint tracking
    JointSpaceTrackingData swing_toe_traj_left(
        "left_toe_angle_traj", osc_gains.K_p_swing_toe, osc_gains.K_d_swing_toe,
        weight_scale * osc_gains.W_swing_toe, plant_w_spr, plant_wo_springs);
    JointSpaceTrackingData swing_toe_traj_right(
        "right_toe_angle_traj", osc_gains.K_p_swing_toe,
        osc_gains.K_d_swing_toe, weight_scale * osc_gains.W_swing_toe,
        plant_w_spr, plant_wo_springs);
    swing_toe_traj_right.AddStateAndJointToTrack(left_stance_state, "toe_right",
                                                 "toe_rightdot");
    swing_toe_traj_left.AddStateAndJointToTrack(right_stance_state, "toe_left",
                                                "toe_leftdot");
    osc->AddTrackingData(&swing_toe_traj_left);
    osc->AddTrackingData(&swing_toe_traj_right);

    // Set double support duration for force blending
    if (!close_sim_gap) {
      osc->SetUpDoubleSupportPhaseBlending(
          double_support_duration, left_stance_state, right_stance_state,
          {post_left_double_support_state, post_right_double_support_state});
    }

    // Build OSC problem
    osc->Build();
    // Connect ports
    builder.Connect(simulator_drift->get_output_port(0),
                    osc->get_robot_output_input_port());
    builder.Connect(fsm->get_output_port(0), osc->get_fsm_input_port());

    builder.Connect(planner_traj_receiver->get_output_port_rom(),
                    osc->get_tracking_data_input_port("optimal_rom_traj"));
    if (FLAGS_get_swing_foot_from_planner) {
      builder.Connect(planner_traj_receiver->get_output_port_swing_foot(),
                      osc->get_tracking_data_input_port("swing_ft_traj"));
    } else {
      builder.Connect(swing_ft_traj_generator->get_output_port(0),
                      osc->get_tracking_data_input_port("swing_ft_traj"));
    }

    if (FLAGS_get_stance_hip_angles_from_planner) {
      builder.Connect(planner_traj_receiver->get_output_port_hip_rpy(),
                      osc->get_tracking_data_input_port("stance_hip_rpy_traj"));
    } else {
      builder.Connect(head_traj_gen->get_output_port(0),
                      osc->get_tracking_data_input_port("pelvis_balance_traj"));
      builder.Connect(head_traj_gen->get_output_port(0),
                      osc->get_tracking_data_input_port("pelvis_heading_traj"));
    }

    if (FLAGS_get_swing_hip_angle_from_planner) {
      builder.Connect(planner_traj_receiver->get_output_port_swing_hip(),
                      osc->get_tracking_data_input_port("swing_hip_yaw_traj"));
    }

    builder.Connect(left_toe_angle_traj_gen->get_output_port(0),
                    osc->get_tracking_data_input_port("left_toe_angle_traj"));
    builder.Connect(right_toe_angle_traj_gen->get_output_port(0),
                    osc->get_tracking_data_input_port("right_toe_angle_traj"));

    builder.Connect(osc->get_output_port(0), command_sender->get_input_port(0));
    if (FLAGS_publish_osc_data) {
      // Create osc debug sender.
      auto osc_debug_pub =
          builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
              "OSC_DEBUG", &lcm_local, TriggerTypeSet({TriggerType::kForced})));
      builder.Connect(osc->get_osc_debug_port(),
                      osc_debug_pub->get_input_port());
    }

    // Create the diagram
    auto owned_diagram = builder.Build();
    owned_diagram->set_name("rom_walking_controller");

    DrawAndSaveDiagramGraph(*owned_diagram);

    // Run lcm-driven simulation
    systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
        &lcm_local, std::move(owned_diagram), state_receiver, FLAGS_channel_x,
        true);

    // Get init traj from ROM planner result
    dairlib::lcmt_timestamped_saved_traj traj_msg;
    if (!FLAGS_init_traj_file_path.empty()) {
      RomPlannerTrajectory saved_traj(FLAGS_init_traj_file_path, true);
      traj_msg = saved_traj.GenerateLcmObject();
      DRAKE_DEMAND(FLAGS_init_traj_file_name.empty());
    } else if (!FLAGS_init_traj_file_name.empty()) {
      RomPlannerTrajectory saved_traj(
          gains.dir_data + FLAGS_init_traj_file_name, true);
      traj_msg = saved_traj.GenerateLcmObject();
    } else {
      traj_msg.saved_traj.metadata.name = "";
      traj_msg.saved_traj.num_trajectories = 0;
    }

    // Get context and initialize the lcm message of LcmSubsriber for
    // lcmt_timestamped_saved_traj
    auto& diagram_context = loop.get_diagram_mutable_context();
    auto& planner_subscriber_context =
        loop.get_diagram()->GetMutableSubsystemContext(
            *planner_output_subscriber, &diagram_context);
    // Note that currently the LcmSubscriber stores the lcm message in the first
    // state of the leaf system (we hard coded index 0 here)
    auto& mutable_state =
        planner_subscriber_context
            .get_mutable_abstract_state<dairlib::lcmt_timestamped_saved_traj>(
                0);
    mutable_state = traj_msg;

    // Set constant walking speed
    if (gains.set_constant_walking_speed &&
        !FLAGS_get_swing_foot_from_planner) {
      double const_walking_speed_x =
          gains.constant_step_length_x / stride_period;

      auto& walking_speed_control_context =
          loop.get_diagram()->GetMutableSubsystemContext(*walking_speed_control,
                                                         &diagram_context);
      walking_speed_control->get_input_port_des_hor_vel().FixValue(
          &walking_speed_control_context,
          drake::systems::BasicVector<double>({const_walking_speed_x, 0}));
      cout << "Set constant walking speed " << const_walking_speed_x << endl;
    }

    // Set constant turning rate
    if (gains.set_constant_turning_rate) {
      auto& head_traj_gen_context =
          loop.get_diagram()->GetMutableSubsystemContext(*head_traj_gen,
                                                         &diagram_context);
      head_traj_gen->get_yaw_input_port().FixValue(
          &head_traj_gen_context,
          drake::systems::BasicVector<double>({gains.constant_turning_rate}));
      cout << "Set constant turning rate " << gains.constant_turning_rate
           << endl;
    }

    // Create the file to indicate that the planner thread is listening
    if (!FLAGS_path_wait_identifier.empty()) {
      std::system(("touch " + FLAGS_path_wait_identifier).c_str());
      cout << "Created " << FLAGS_path_wait_identifier << endl;
    }

    loop.Simulate();
  } else {
    ///
    /// IK conroller
    ///

    // TODO: think about how I want to construct the spline from IK.
    //  Currently, I'm using CubicWithContinuousSecondDerivatives with 0 vel at
    //  the end points.

    std::vector<std::string> ordered_pos_names = {
        "hip_roll_left", "hip_yaw_left",   "hip_pitch_left", "knee_left",
        "toe_left",      "hip_roll_right", "hip_yaw_right",  "hip_pitch_right",
        "knee_right",    "toe_right"};
    //    std::vector<std::string> ordered_pos_names = {
    //        "hip_roll_left",  "hip_roll_right",  "hip_yaw_left",
    //        "hip_yaw_right", "hip_pitch_left", "hip_pitch_right", "knee_left",
    //        "knee_right", "toe_left",       "toe_right"};
    std::vector<std::string> ordered_vel_names;
    for (const auto& name : ordered_pos_names) {
      ordered_vel_names.push_back(name + "dot");
    }

    // int n_y = ordered_pos_names.size();

    VectorXd Kp_stance(5);
    Kp_stance << gains.kp_hip_roll_stance, gains.kp_hip_yaw_stance,
        gains.kp_hip_pitch_stance, gains.kp_knee_stance, gains.kp_toe_stance;
    VectorXd Kd_stance(5);
    Kd_stance << gains.kd_hip_roll_stance, gains.kd_hip_yaw_stance,
        gains.kd_hip_pitch_stance, gains.kd_knee_stance, gains.kd_toe_stance;
    VectorXd Kp_swing(5);
    Kp_swing << gains.kp_hip_roll_swing, gains.kp_hip_yaw_swing,
        gains.kp_hip_pitch_swing, gains.kp_knee_swing, gains.kp_toe_swing;
    VectorXd Kd_swing(5);
    Kd_swing << gains.kd_hip_roll_swing, gains.kd_hip_yaw_swing,
        gains.kd_hip_pitch_swing, gains.kd_knee_swing, gains.kd_toe_swing;

    // Walking gains
    Eigen::DiagonalMatrix<double, 10> K_p_left_stance;
    K_p_left_stance.diagonal() << Kp_stance, Kp_swing;
    Eigen::DiagonalMatrix<double, 10> K_d_left_stance;
    K_d_left_stance.diagonal() << Kd_stance, Kd_swing;
    Eigen::DiagonalMatrix<double, 10> K_p_right_stance;
    K_p_right_stance.diagonal() << Kp_swing, Kp_stance;
    Eigen::DiagonalMatrix<double, 10> K_d_right_stance;
    K_d_right_stance.diagonal() << Kd_swing, Kd_stance;
    // Cost weights
    Eigen::DiagonalMatrix<double, 10> W_left_stance =
        200 * VectorXd::Ones(10).asDiagonal();
    Eigen::DiagonalMatrix<double, 10> W_right_stance =
        200 * VectorXd::Ones(10).asDiagonal();

    // Create Lcm subscriber for IK output
    auto IK_output_subscriber = builder.AddSystem(
        LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
            FLAGS_channel_ik, &lcm_local));
    // Create a system that translate IK lcm into trajectory
    auto optimal_ik_traj_gen =
        builder.AddSystem<IKTrajReceiver>(plant_wo_springs, ordered_pos_names);
    builder.Connect(IK_output_subscriber->get_output_port(),
                    optimal_ik_traj_gen->get_input_port(0));

    // Create Operational space control
    auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
        plant_w_spr, plant_wo_springs, context_w_spr.get(),
        context_wo_spr.get(), true, FLAGS_print_osc /*print_tracking_info*/, 0,
        false, close_sim_gap);

    // Cost
    int n_v = plant_wo_springs.num_velocities();

    if (osc_gains.w_accel > 0) {
      MatrixXd Q_accel = osc_gains.w_accel * MatrixXd::Identity(n_v, n_v);
      osc->SetAccelerationCostForAllJoints(Q_accel);
    }
    if (osc_gains.w_input_reg > 0) {
      osc->SetInputRegularizationWeight(osc_gains.w_input_reg);
    }

    // Constraints in OSC
    osc->AddKinematicConstraint(&evaluators);

    // Soft constraint
    // w_contact_relax shouldn't be too big, cause we want tracking error to be
    // important
    osc->SetWeightOfSoftContactConstraint(osc_gains.w_soft_constraint);
    // Friction coefficient
    osc->SetContactFriction(osc_gains.mu);
    // Add contact points
    osc->AddStateAndContactPoint(left_stance_state, &left_toe_evaluator);
    osc->AddStateAndContactPoint(left_stance_state, &left_heel_evaluator);
    osc->AddStateAndContactPoint(right_stance_state, &right_toe_evaluator);
    osc->AddStateAndContactPoint(right_stance_state, &right_heel_evaluator);
    if (!is_two_phase) {
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

    // TODO: Add an API to OSC to allow vector weights (for W, Kp and Kd)
    // TrackingData for left support phase
    JointSpaceTrackingData left_support_traj(
        "left_support_traj", K_p_left_stance, K_d_left_stance, W_left_stance,
        plant_w_spr, plant_wo_springs);
    left_support_traj.AddStateAndJointsToTrack(
        left_stance_state, ordered_pos_names, ordered_vel_names);
    left_support_traj.AddStateAndJointsToTrack(
        post_left_double_support_state, ordered_pos_names, ordered_vel_names);
    osc->AddTrackingData(&left_support_traj);

    // TrackingData for right support phase
    JointSpaceTrackingData right_support_traj(
        "right_support_traj", K_p_right_stance, K_d_right_stance,
        W_right_stance, plant_w_spr, plant_wo_springs);
    right_support_traj.AddStateAndJointsToTrack(
        right_stance_state, ordered_pos_names, ordered_vel_names);
    right_support_traj.AddStateAndJointsToTrack(
        post_right_double_support_state, ordered_pos_names, ordered_vel_names);
    osc->AddTrackingData(&right_support_traj);

    // Set double support duration for force blending
    osc->SetUpDoubleSupportPhaseBlending(
        double_support_duration, left_stance_state, right_stance_state,
        {post_left_double_support_state, post_right_double_support_state});

    // Build OSC problem
    osc->Build();
    // Connect ports
    builder.Connect(simulator_drift->get_output_port(0),
                    osc->get_robot_output_input_port());
    builder.Connect(fsm->get_output_port(0), osc->get_fsm_input_port());
    builder.Connect(optimal_ik_traj_gen->get_output_port(0),
                    osc->get_tracking_data_input_port("left_support_traj"));
    builder.Connect(optimal_ik_traj_gen->get_output_port(0),
                    osc->get_tracking_data_input_port("right_support_traj"));
    builder.Connect(osc->get_output_port(0), command_sender->get_input_port(0));
    if (FLAGS_publish_osc_data) {
      // Create osc debug sender.
      auto osc_debug_pub =
          builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
              "OSC_DEBUG", &lcm_local, TriggerTypeSet({TriggerType::kForced})));
      builder.Connect(osc->get_osc_debug_port(),
                      osc_debug_pub->get_input_port());
    }

    // Create the diagram
    auto owned_diagram = builder.Build();
    owned_diagram->set_name("rom_ik_walking_controller");

    DrawAndSaveDiagramGraph(*owned_diagram);

    // Run lcm-driven simulation
    systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
        &lcm_local, std::move(owned_diagram), state_receiver, FLAGS_channel_x,
        true);

    // TODO: modify LCM driven loop so that you don't need to initial it here.
    // Get init traj from ROM planner result
    VectorXd time_at_knots(3);
    time_at_knots << 0, 1, 2;
    MatrixXd pos_at_knots = MatrixXd::Zero(plant_wo_springs.num_positions(), 3);
    // Initial message for the LCM subscriber. In the first timestep, the
    // subscriber might not receive a solution yet
    dairlib::lcmt_trajectory_block traj_msg0;
    traj_msg0.trajectory_name = "";
    traj_msg0.num_points = time_at_knots.size();
    traj_msg0.num_datatypes = plant_wo_springs.num_positions();
    // Reserve space for vectors
    traj_msg0.time_vec.resize(traj_msg0.num_points);
    traj_msg0.datatypes.resize(traj_msg0.num_datatypes);
    traj_msg0.datapoints.clear();
    // Copy Eigentypes to std::vector
    traj_msg0.time_vec = CopyVectorXdToStdVector(time_at_knots);
    traj_msg0.datatypes = vector<std::string>(plant_wo_springs.num_positions());
    for (int i = 0; i < traj_msg0.num_datatypes; ++i) {
      traj_msg0.datapoints.push_back(
          CopyVectorXdToStdVector(pos_at_knots.row(i)));
    }
    dairlib::lcmt_saved_traj traj_msg;
    traj_msg.num_trajectories = 1;
    traj_msg.trajectories.push_back(traj_msg0);
    traj_msg.trajectory_names.push_back("");

    // Get context and initialize the lcm message of LcmSubsriber for
    // lcmt_timestamped_saved_traj
    auto& diagram_context = loop.get_diagram_mutable_context();
    auto& ik_subscriber_context =
        loop.get_diagram()->GetMutableSubsystemContext(*IK_output_subscriber,
                                                       &diagram_context);
    // Note that currently the LcmSubscriber stores the lcm message in the first
    // state of the leaf system (we hard coded index 0 here)
    auto& mutable_state =
        ik_subscriber_context
            .get_mutable_abstract_state<dairlib::lcmt_saved_traj>(0);
    mutable_state = traj_msg;

    loop.Simulate();
  }

  return 0;
}

}  // namespace dairlib::goldilocks_models

int main(int argc, char* argv[]) {
  return dairlib::goldilocks_models::DoMain(argc, argv);
}
