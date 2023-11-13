#include <gflags/gflags.h>
#include <iostream>

// lcmtypes
#include "dairlib/lcmt_alip_mpc_output.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"

// Cassie and multibody
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/osc/high_level_command.h"
#include "examples/Cassie/osc/hip_yaw_traj_gen.h"
#include "examples/Cassie/osc/heading_traj_generator.h"
#include "examples/Cassie/osc/pelvis_pitch_traj_generator.h"
#include "examples/Cassie/osc/osc_walking_gains_alip.h"
#include "examples/Cassie/osc/swing_toe_traj_generator.h"
#include "examples/Cassie/systems/cassie_out_to_radio.h"
#include "multibody/kinematic/fixed_joint_evaluator.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/multibody_utils.h"

// MPC related
#include "examples/perceptive_locomotion/gains/alip_mpfc_gains.h"
#include "examples/perceptive_locomotion/systems/cassie_ankle_torque_receiver.h"
#include "systems/primitives/fsm_lcm_systems.h"
#include "systems/controllers/footstep_planning/alip_mpc_output_reciever.h"
#include "systems/controllers/footstep_planning/alip_mpc_interface_system.h"

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
using drake::multibody::SpatialInertia;
using drake::multibody::RotationalInertia;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::LcmScopeSystem;
using drake::systems::TriggerTypeSet;

using multibody::WorldYawViewFrame;
using systems::FsmReceiver;
using systems::controllers::AlipMPCInterfaceSystem;
using systems::controllers::AlipMpcOutputReceiver;
using systems::controllers::ComTrackingData;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::RelativeTranslationTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;

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

DEFINE_string(cassie_out_channel, "CASSIE_OUTPUT_ECHO",
              "The name of the channel to receive the cassie "
              "out structure from.");

DEFINE_string(gains_filename,
              "examples/perceptive_locomotion/gains/osc_gains.yaml",
              "Filepath containing gains");

DEFINE_string(mpfc_gains_filename,
              "examples/perceptive_locomotion/gains/alip_mpfc_gains.yaml",
              "Filepath to alip mpfc gains");

DEFINE_string(channel_mpc, "ALIP_MPC", "alip mpc output lcm channel");

DEFINE_bool(publish_osc_data, true,
            "whether to publish lcm messages for OscTrackData");

DEFINE_bool(is_two_phase, false,
            "true: only right/left single support"
            "false: both double and single support");

DEFINE_bool(use_spring_model, false,
            "whether to use the cassie spring model in the controller");

DEFINE_bool(add_camera_inertia, true,
            "whether to add the camera assembly to the robot model");

DEFINE_bool(com_height_track_terrain, true,
            "track the terrain with the COM height, use for "
            "piecewise steep terrain");

//DEFINE_double(qp_time_limit, 0.002, "maximum qp solve time");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Read-in the parameters
  auto gains = drake::yaml::LoadYamlFile<OSCWalkingGainsALIP>(FLAGS_gains_filename);
  auto gains_mpc =
      drake::yaml::LoadYamlFile<AlipMpfcGainsImport>(FLAGS_mpfc_gains_filename);

  const std::string urdf_w_spr = "examples/Cassie/urdf/cassie_v2.urdf";
  const std::string urdf_wo_spr = "examples/Cassie/urdf/cassie_fixed_springs.urdf";

  // Build Cassie MBP
  drake::multibody::MultibodyPlant<double> plant_w_spr(0.0);
  drake::multibody::MultibodyPlant<double> plant_wo_spr(0.0);
  plant_w_spr.set_name("plant_w_spr");
  plant_wo_spr.set_name("plant_wo_spr");

  auto instance_w_spr =
      AddCassieMultibody(&plant_w_spr, nullptr, true, urdf_w_spr, true, false);

  drake::multibody::ModelInstanceIndex instance_wo_spr;
  if (FLAGS_use_spring_model) {
    instance_wo_spr = AddCassieMultibody(
        &plant_wo_spr, nullptr, true, urdf_w_spr, true, false);
  } else {
    instance_wo_spr = AddCassieMultibody(
        &plant_wo_spr, nullptr, true, urdf_wo_spr, false, false);
  }
  if (FLAGS_add_camera_inertia) {
    auto camera_inertia_about_com =
        RotationalInertia<double>::MakeFromMomentsAndProductsOfInertia(
            0.04, 0.04, 0.04, 0, 0, 0);
    auto camera_inertia = SpatialInertia<double>::MakeFromCentralInertia(
        1.06, Vector3d(0.07, 0.0, 0.17), camera_inertia_about_com);
    plant_w_spr.AddRigidBody("camera_inertia", instance_w_spr, camera_inertia);
    plant_wo_spr.AddRigidBody("camera_inertia", instance_wo_spr, camera_inertia);
    plant_w_spr.WeldFrames(
        plant_w_spr.GetBodyByName("pelvis").body_frame(),
        plant_w_spr.GetBodyByName("camera_inertia").body_frame()
    );
    plant_wo_spr.WeldFrames(
        plant_wo_spr.GetBodyByName("pelvis").body_frame(),
        plant_wo_spr.GetBodyByName("camera_inertia").body_frame()
    );
  }
  plant_w_spr.Finalize();
  plant_wo_spr.Finalize();

  auto context_w_spr = plant_w_spr.CreateDefaultContext();
  auto context_wo_spr = plant_wo_spr.CreateDefaultContext();

  // Build the controller diagram
  DiagramBuilder<double> builder;

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");

  // Get contact frames and position
  auto left_toe = LeftToeFront(plant_wo_spr);
  auto left_heel = LeftToeRear(plant_wo_spr);
  auto right_toe = RightToeFront(plant_wo_spr);
  auto right_heel = RightToeRear(plant_wo_spr);

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

  // Create the MPC receiver.
  auto mpc_subscriber = builder.AddSystem(
      LcmSubscriberSystem::Make<lcmt_alip_mpc_output>(
          FLAGS_channel_mpc, &lcm_local));
  auto mpc_receiver = builder.AddSystem<AlipMpcOutputReceiver>();
  builder.Connect(*mpc_subscriber, *mpc_receiver);


  // Create command sender.
  auto command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_channel_u, &lcm_local, TriggerTypeSet({TriggerType::kForced})));
  auto command_sender =
      builder.AddSystem<systems::RobotCommandSender>(plant_w_spr);

  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port());

  auto cassie_out_to_radio =
      builder.AddSystem<systems::CassieOutToRadio>();
  auto cassie_out_receiver =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_cassie_out>(
            FLAGS_cassie_out_channel, &lcm_local));
  auto high_level_command = builder.AddSystem<cassie::osc::HighLevelCommand>(
      plant_w_spr,
      context_w_spr.get(),
      gains.vel_scale_rot,
      gains.vel_scale_trans_sagital,
      gains.vel_scale_trans_lateral,
      1.0);


  builder.Connect(*cassie_out_receiver, *cassie_out_to_radio);
  builder.Connect(cassie_out_to_radio->get_output_port(),
                    high_level_command->get_input_port_radio());

  builder.Connect(state_receiver->get_output_port(0),
                  high_level_command->get_input_port_state());

  // Create heading traj generator
  auto head_traj_gen = builder.AddSystem<cassie::osc::HeadingTrajGenerator>(
      plant_w_spr, context_w_spr.get());
  auto pitch_traj_gen = builder.AddSystem
      <cassie::osc::PelvisPitchTrajGenerator>(plant_w_spr, context_w_spr.get());
  builder.Connect(state_receiver->get_output_port(),
                  head_traj_gen->get_input_port_state());
  builder.Connect(high_level_command->get_output_port_yaw(),
                  head_traj_gen->get_input_port_yaw());
  builder.Connect(state_receiver->get_output_port(),
                  pitch_traj_gen->get_state_input_port());
  builder.Connect(mpc_receiver->get_output_port_pitch(),
                  pitch_traj_gen->get_pitch_input_port());

  // Create finite state machine
  int left_stance_state = 0;
  int right_stance_state = 1;
  int post_left_double_support_state = 3;
  int post_right_double_support_state = 4;
  double left_support_duration = gains_mpc.ss_time;
  double right_support_duration = gains_mpc.ss_time;
  double double_support_duration = gains_mpc.ds_time;

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

  auto fsm = builder.AddSystem<FsmReceiver>(plant_w_spr);
  builder.Connect(mpc_receiver->get_output_port_fsm(),
                  fsm->get_input_port_fsm_info());
  builder.Connect(state_receiver->get_output_port(),
                  fsm->get_input_port_state());

  // Create leafsystem that record the switching time of the FSM
  std::vector<int> single_support_states = {left_stance_state,
                                            right_stance_state};
  std::vector<int> double_support_states = {post_left_double_support_state,
                                            post_right_double_support_state};

  // Create swing leg trajectory generator
  vector<int> unordered_fsm_states;
  vector<std::pair<const Vector3d, const Frame<double>&>> contact_points_in_each_state;
  if (FLAGS_is_two_phase) {
    unordered_fsm_states = {left_stance_state, right_stance_state};
    contact_points_in_each_state.push_back({left_toe_mid});
    contact_points_in_each_state.push_back({right_toe_mid});
  } else {
    unordered_fsm_states = {left_stance_state, right_stance_state,
                            post_left_double_support_state,
                            post_right_double_support_state};
    contact_points_in_each_state.push_back(left_toe_mid);
    contact_points_in_each_state.push_back(right_toe_mid);
    contact_points_in_each_state.push_back(left_toe_mid);
    contact_points_in_each_state.push_back(right_toe_mid);
  }


  vector<int> left_right_support_fsm_states = {left_stance_state,
                                               right_stance_state};
  vector<double> left_right_support_state_durations = {left_support_duration,
                                                       right_support_duration};
  vector<std::pair<const Vector3d, const Frame<double>&>> left_right_foot = {
      left_toe_origin, right_toe_origin};

  systems::controllers::SwingFootInterfaceSystemParams swing_params{
    left_right_support_fsm_states,
    left_right_foot,
    {post_left_double_support_state, post_right_double_support_state},
    gains_mpc.h_des,
    gains.mid_foot_height,
    0.05608,
    gains.final_foot_height,
    gains.final_foot_velocity_z,
    gains_mpc.retraction_dist,
    true
  };

  systems::controllers::ComTrajInterfaceParams com_params{
      gains_mpc.h_des,
      gains_mpc.ds_time,
      unordered_fsm_states,
      contact_points_in_each_state,
      FLAGS_com_height_track_terrain
  };

  auto mpc_interface = builder.AddSystem<AlipMPCInterfaceSystem>(
      plant_w_spr, context_w_spr.get(), com_params, swing_params);
  builder.Connect(mpc_receiver->get_output_port_slope_parameters(),
                  mpc_interface->get_input_port_slope_parameters());
  builder.Connect(fsm->get_output_port_fsm(),
                  mpc_interface->get_input_port_fsm());
  builder.Connect(fsm->get_output_port_prev_switch_time(),
                  mpc_interface->get_input_port_fsm_switch_time());
  builder.Connect(fsm->get_output_port_next_switch_time(),
                  mpc_interface->get_input_port_next_fsm_switch_time());
  builder.Connect(state_receiver->get_output_port(0),
                  mpc_interface->get_input_port_state());
  builder.Connect(mpc_receiver->get_output_port_footstep_target(),
                  mpc_interface->get_input_port_footstep_target());

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
                  left_toe_angle_traj_gen->get_input_port_state());
  builder.Connect(state_receiver->get_output_port(0),
                  right_toe_angle_traj_gen->get_input_port_state());
  builder.Connect(mpc_receiver->get_output_port_pitch(),
                  right_toe_angle_traj_gen->get_input_port_toe_angle());
  builder.Connect(mpc_receiver->get_output_port_pitch(),
                  left_toe_angle_traj_gen->get_input_port_toe_angle());


  auto alip_input_receiver =
      builder.AddSystem<perceptive_locomotion::CassieAnkleTorqueReceiver>(
          plant_w_spr,
          left_right_support_fsm_states,
          std::vector<std::string>({"toe_left_motor", "toe_right_motor"}));

  // Create Operational space control
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      plant_w_spr, plant_wo_spr, context_w_spr.get(), context_wo_spr.get(), true);

  // Cost
  int n_v = plant_wo_spr.num_velocities();
  int n_u = plant_wo_spr.num_actuators();
  MatrixXd Q_accel = gains.w_accel * MatrixXd::Identity(n_v, n_v);
  osc->SetAccelerationCostWeights(Q_accel);
  osc->SetInputSmoothingCostWeights(
      gains.w_input_reg * MatrixXd::Identity(n_u, n_u));

  // Constraints in OSC
  multibody::KinematicEvaluatorSet<double> evaluators(plant_wo_spr);

  // 1. fourbar constraint
  auto left_loop = LeftLoopClosureEvaluator(plant_wo_spr);
  auto right_loop = RightLoopClosureEvaluator(plant_wo_spr);
  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);

  // Make fixed spring constraint
  auto pos_idx_map = multibody::MakeNameToPositionsMap(plant_w_spr);
  auto vel_idx_map = multibody::MakeNameToVelocitiesMap(plant_w_spr);
  std::unique_ptr<FixedJointEvaluator<double>> left_fixed_knee_spring;
  std::unique_ptr<FixedJointEvaluator<double>> right_fixed_knee_spring;
  std::unique_ptr<FixedJointEvaluator<double>> left_fixed_ankle_spring;
  std::unique_ptr<FixedJointEvaluator<double>> right_fixed_ankle_spring;

  if (FLAGS_use_spring_model) {
    left_fixed_knee_spring = std::make_unique<FixedJointEvaluator<double>>(
        plant_wo_spr, pos_idx_map.at("knee_joint_left"),
        vel_idx_map.at("knee_joint_leftdot"), 0);
    right_fixed_knee_spring = std::make_unique<FixedJointEvaluator<double>>(
        plant_wo_spr, pos_idx_map.at("knee_joint_right"),
        vel_idx_map.at("knee_joint_rightdot"), 0);
    left_fixed_ankle_spring = std::make_unique<FixedJointEvaluator<double>>(
        plant_wo_spr, pos_idx_map.at("ankle_spring_joint_left"),
        vel_idx_map.at("ankle_spring_joint_leftdot"), 0);
    right_fixed_ankle_spring = std::make_unique<FixedJointEvaluator<double>>(
        plant_wo_spr, pos_idx_map.at("ankle_spring_joint_right"),
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
  osc->SetContactSoftConstraintWeight(gains.w_soft_constraint);
  // Friction coefficient
  osc->SetContactFriction(gains.mu);
  // Add contact points (The position doesn't matter. It's not used in OSC)
  const auto& pelvis = plant_wo_spr.GetBodyByName("pelvis");
  multibody::WorldYawViewFrame view_frame(pelvis);
  auto left_toe_evaluator = multibody::WorldPointEvaluator(
      plant_wo_spr, left_toe.first, left_toe.second, view_frame,
      Matrix3d::Identity(), Vector3d::Zero(), {1, 2});
  auto left_heel_evaluator = multibody::WorldPointEvaluator(
      plant_wo_spr, left_heel.first, left_heel.second, view_frame,
      Matrix3d::Identity(), Vector3d::Zero(), {0, 1, 2});
  auto right_toe_evaluator = multibody::WorldPointEvaluator(
      plant_wo_spr, right_toe.first, right_toe.second, view_frame,
      Matrix3d::Identity(), Vector3d::Zero(), {1, 2});
  auto right_heel_evaluator = multibody::WorldPointEvaluator(
      plant_wo_spr, right_heel.first, right_heel.second, view_frame,
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
      "examples/perceptive_locomotion/gains/osqp_options_osc.yaml");

  // Swing foot tracking
  std::vector<double> swing_ft_gain_schedule_breaks =
      {0, left_support_duration / 2, left_support_duration};
  std::vector<drake::MatrixX<double>> swing_ft_gain_multiplier_samples(
      3, drake::MatrixX<double>::Identity(3, 3));
  swing_ft_gain_multiplier_samples[2](2, 2) *= 0.5;
  //swing_ft_gain_multiplier_samples[0].topLeftCorner<1,1>() *= 0.25;
  auto swing_ft_gain_multiplier_gain_multiplier =
      std::make_shared< PiecewisePolynomial<double>>(
          PiecewisePolynomial<double>::FirstOrderHold(
          swing_ft_gain_schedule_breaks,
          swing_ft_gain_multiplier_samples));
  std::vector<double> swing_ft_accel_gain_multiplier_breaks{
      0, left_support_duration / 2, left_support_duration * 3 / 4,
      gains_mpc.t_max};
  std::vector<drake::MatrixX<double>> swing_ft_accel_gain_multiplier_samples(
      4, drake::MatrixX<double>::Identity(3, 3));
  swing_ft_accel_gain_multiplier_samples[0](2, 2) *= 1.1;
  swing_ft_accel_gain_multiplier_samples[2](2, 2) *= 0.5;
  swing_ft_accel_gain_multiplier_samples[3](2, 2) *= 0;
  swing_ft_accel_gain_multiplier_samples[2](0, 0) *= 0.15;
  swing_ft_accel_gain_multiplier_samples[3](0, 0) *= 0.0;
  auto swing_ft_accel_gain_multiplier_gain_multiplier =
      std::make_shared<PiecewisePolynomial<double>>(
          PiecewisePolynomial<double>::FirstOrderHold(
          swing_ft_accel_gain_multiplier_breaks,
          swing_ft_accel_gain_multiplier_samples));

  auto swing_foot_data = std::make_unique<TransTaskSpaceTrackingData>(
      "swing_ft_traj", gains.K_p_swing_foot, gains.K_d_swing_foot,
      gains.W_swing_foot, plant_w_spr, plant_wo_spr);
  swing_foot_data->AddStateAndPointToTrack(left_stance_state, "toe_right");
  swing_foot_data->AddStateAndPointToTrack(right_stance_state, "toe_left");

  auto stance_foot_data = std::make_unique<TransTaskSpaceTrackingData>(
      "com_data", gains.K_p_swing_foot,
      gains.K_d_swing_foot, gains.W_swing_foot,
      plant_w_spr, plant_wo_spr);
  stance_foot_data->AddStateAndPointToTrack(left_stance_state, "toe_left");
  stance_foot_data->AddStateAndPointToTrack(right_stance_state, "toe_right");

  auto swing_ft_traj_local = std::make_unique<RelativeTranslationTrackingData>(
      "swing_ft_traj", gains.K_p_swing_foot, gains.K_d_swing_foot,
      gains.W_swing_foot, plant_w_spr, plant_wo_spr, swing_foot_data.get(),
      stance_foot_data.get());
  swing_ft_traj_local->SetSpringsInKinematicCalculation(FLAGS_use_spring_model);

  auto pelvis_view_frame = std::make_shared<WorldYawViewFrame<double>>(
      plant_w_spr.GetBodyByName("pelvis"));
  swing_ft_traj_local->SetViewFrame(pelvis_view_frame);

  swing_ft_traj_local->SetTimeVaryingPDGainMultiplier(
      swing_ft_gain_multiplier_gain_multiplier);
  swing_ft_traj_local->SetTimerVaryingFeedForwardAccelMultiplier(
      swing_ft_accel_gain_multiplier_gain_multiplier);
  osc->AddTrackingData(std::move(swing_ft_traj_local));

  auto center_of_mass_traj = std::make_unique<ComTrackingData>(
      "alip_com_traj", gains.K_p_com, gains.K_d_com,
      gains.W_com, plant_w_spr, plant_wo_spr);
  center_of_mass_traj->SetViewFrame(pelvis_view_frame);
  
  // FiniteStatesToTrack cannot be empty
  center_of_mass_traj->AddFiniteStateToTrack(-1);
  osc->AddTrackingData(std::move(center_of_mass_traj));

  // Pelvis rotation tracking (pitch and roll)
  auto pelvis_balance_traj = std::make_unique<RotTaskSpaceTrackingData>(
      "pelvis_balance_traj", gains.K_p_pelvis_balance, gains.K_d_pelvis_balance,
      gains.W_pelvis_balance, plant_w_spr, plant_wo_spr);
  pelvis_balance_traj->AddFrameToTrack("pelvis");
  osc->AddTrackingData(std::move(pelvis_balance_traj));
  // Pelvis rotation tracking (yaw)
 auto pelvis_heading_traj = std::make_unique<RotTaskSpaceTrackingData>(
      "pelvis_heading_traj", gains.K_p_pelvis_heading, gains.K_d_pelvis_heading,
      gains.W_pelvis_heading, plant_w_spr, plant_wo_spr);
  pelvis_heading_traj->AddFrameToTrack("pelvis");
  osc->AddTrackingData(std::move(pelvis_heading_traj),
                       gains.period_of_no_heading_control);

  // Swing toe joint tracking
  auto swing_toe_traj_left = std::make_unique<JointSpaceTrackingData>(
      "left_toe_angle_traj", gains.K_p_swing_toe, gains.K_d_swing_toe,
      gains.W_swing_toe, plant_w_spr, plant_wo_spr);
  auto swing_toe_traj_right = std::make_unique<JointSpaceTrackingData>(
      "right_toe_angle_traj", gains.K_p_swing_toe, gains.K_d_swing_toe,
      gains.W_swing_toe, plant_w_spr, plant_wo_spr);
  swing_toe_traj_right->AddStateAndJointToTrack(left_stance_state, "toe_right",
                                               "toe_rightdot");
  swing_toe_traj_left->AddStateAndJointToTrack(right_stance_state, "toe_left",
                                              "toe_leftdot");
  osc->AddTrackingData(std::move(swing_toe_traj_left));
  osc->AddTrackingData(std::move(swing_toe_traj_right));


  auto hip_yaw_traj_gen =
      builder.AddSystem<cassie::HipYawTrajGen>(left_stance_state);

  // Swing hip yaw joint tracking
  auto swing_hip_yaw_traj = std::make_unique<JointSpaceTrackingData>(
      "swing_hip_yaw_traj", gains.K_p_hip_yaw, gains.K_d_hip_yaw,
      gains.W_hip_yaw, plant_w_spr, plant_wo_spr);
  swing_hip_yaw_traj->AddStateAndJointToTrack(left_stance_state, "hip_yaw_right",
                                             "hip_yaw_rightdot");
  swing_hip_yaw_traj->AddStateAndJointToTrack(right_stance_state, "hip_yaw_left",
                                             "hip_yaw_leftdot");


  builder.Connect(cassie_out_to_radio->get_output_port(),
                    hip_yaw_traj_gen->get_radio_input_port());
  builder.Connect(fsm->get_output_port_fsm(),
                    hip_yaw_traj_gen->get_fsm_input_port());
  osc->AddTrackingData(std::move(swing_hip_yaw_traj));

  // Set double support duration for force blending
  osc->SetUpDoubleSupportPhaseBlending(
      double_support_duration, left_stance_state, right_stance_state,
      {post_left_double_support_state, post_right_double_support_state});

  if (gains.W_com(0,0) == 0){
    osc->SetInputCostForJointAndFsmStateWeight(
        "toe_left_motor", left_stance_state, 0.01);
    osc->SetInputCostForJointAndFsmStateWeight(
        "toe_left_motor", post_right_double_support_state, 0.01);
    osc->SetInputCostForJointAndFsmStateWeight(
        "toe_right_motor", right_stance_state, 0.01);
    osc->SetInputCostForJointAndFsmStateWeight(
        "toe_right_motor", post_left_double_support_state, 0.01);
  }
  osc->Build();

  // Connect ports
  builder.Connect(state_receiver->get_output_port(0),
                  osc->get_input_port_robot_output());
  builder.Connect(fsm->get_output_port_fsm(), osc->get_input_port_fsm());
  builder.Connect(mpc_interface->get_output_port_com_traj(),
                  osc->get_input_port_tracking_data("alip_com_traj"));
  builder.Connect(mpc_interface->get_output_port_swing_foot_traj(),
                  osc->get_input_port_tracking_data("swing_ft_traj"));
  builder.Connect(head_traj_gen->get_output_port(0),
                  osc->get_input_port_tracking_data("pelvis_heading_traj"));
  builder.Connect(pitch_traj_gen->get_output_port(0),
                  osc->get_input_port_tracking_data("pelvis_balance_traj"));
  builder.Connect(left_toe_angle_traj_gen->get_output_port(0),
                  osc->get_input_port_tracking_data("left_toe_angle_traj"));
  builder.Connect(right_toe_angle_traj_gen->get_output_port(0),
                  osc->get_input_port_tracking_data("right_toe_angle_traj"));

  builder.Connect(hip_yaw_traj_gen->get_hip_yaw_output_port(),
                    osc->get_input_port_tracking_data("swing_hip_yaw_traj"));
  builder.Connect(fsm->get_output_port_fsm(),
                  alip_input_receiver->get_input_port_fsm());
  builder.Connect(mpc_receiver->get_output_port_ankle_torque(),
                  alip_input_receiver->get_input_port_u());
  builder.Connect(alip_input_receiver->get_output_port(),
                  osc->get_feedforward_input_port());

  builder.Connect(osc->get_output_port(0), command_sender->get_input_port(0));
  if (FLAGS_publish_osc_data) {
    // Create osc debug sender.
    auto osc_debug_pub =
        builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
            "OSC_DEBUG_WALKING", &lcm_local,
            TriggerTypeSet({TriggerType::kForced})));
    builder.Connect(osc->get_output_port_osc_debug(), osc_debug_pub->get_input_port());
  }

  // Create the diagram
  auto owned_diagram = builder.Build();
  owned_diagram->set_name("osc controller for alip_mpfc");
  DrawAndSaveDiagramGraph(*owned_diagram, "../osc_for_alip_mpfc");

  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm_local, std::move(owned_diagram), state_receiver, FLAGS_channel_x,
      true);
  auto& loop_context = loop.get_diagram_mutable_context();

  LcmHandleSubscriptionsUntil(&lcm_local, [&]() {
    return mpc_subscriber->GetInternalMessageCount() > 1; });
  mpc_subscriber->ForcedPublish(loop.get_diagram()->
      GetMutableSubsystemContext(*mpc_subscriber, &loop_context));


  loop.Simulate();

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
