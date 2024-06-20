#include <gflags/gflags.h>
#include <iostream>

// dairlib
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

#include "cassie_acom_tracking_data.h"
#include "systems/controllers/osc/com_tracking_data.h"
#include "systems/controllers/osc/joint_space_tracking_data.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/controllers/osc/options_tracking_data.h"
#include "systems/controllers/osc/relative_translation_tracking_data.h"
#include "systems/controllers/osc/rot_space_tracking_data.h"
#include "systems/controllers/osc/trans_space_tracking_data.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"

#include "systems/system_utils.h"
#include "systems/primitives/fsm_lcm_systems.h"
#include "systems/controllers/footstep_planning/cf_mpfc_output_receiver.h"
#include "systems/controllers/footstep_planning/swing_foot_trajectory_generator.h"

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
using drake::systems::TriggerTypeSet;

using multibody::WorldYawViewFrame;
using systems::FsmReceiver;
using systems::controllers::CFMPFCOutputReceiver;
using systems::controllers::ComTrackingData;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::RelativeTranslationTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;
using systems::controllers::SwingFootTrajectoryGenerator;

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
              "examples/cf_mpfc/gains/osc_gains.yaml",
              "Filepath containing osc gains");

DEFINE_string(channel_mpc, "ALIP_MPC", "alip mpc output lcm channel");

int DoMain(int argc, char** argv) {

  gflags::ParseCommandLineFlags(&argc, &argv, true);

  const std::string urdf = "examples/Cassie/urdf/cassie_v2.urdf";

  // Build Cassie MBP
  drake::multibody::MultibodyPlant<double> plant(0.0);
  plant.set_name("plant");

  auto instance =
      AddCassieMultibody(&plant, nullptr, true, urdf, true, false);

  // Add camera mass/inertia to Cassie model
  auto camera_inertia_about_com =
      RotationalInertia<double>::MakeFromMomentsAndProductsOfInertia(
          0.04, 0.04, 0.04, 0, 0, 0);
  auto camera_inertia = SpatialInertia<double>::MakeFromCentralInertia(
      1.06, Vector3d(0.07, 0.0, 0.17), camera_inertia_about_com);
  plant.AddRigidBody("camera_inertia", instance, camera_inertia);
  plant.WeldFrames(
      plant.GetBodyByName("pelvis").body_frame(),
      plant.GetBodyByName("camera_inertia").body_frame());

  plant.Finalize();
  auto context = plant.CreateDefaultContext();

  auto gains = drake::yaml::LoadYamlFile<OSCWalkingGainsALIP>(FLAGS_gains_filename);

  // Build the controller diagram
  DiagramBuilder<double> builder;

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");

  // Get contact frames and position
  auto left_toe = LeftToeFront(plant);
  auto left_heel = LeftToeRear(plant);
  auto right_toe = RightToeFront(plant);
  auto right_heel = RightToeRear(plant);

  // Get body frames and points
  Vector3d center_of_pressure = left_heel.first +
      gains.contact_point_pos * (left_toe.first - left_heel.first);
  auto left_toe_mid = std::pair<const Vector3d, const Frame<double>&>(
      center_of_pressure, plant.GetFrameByName("toe_left"));
  auto right_toe_mid = std::pair<const Vector3d, const Frame<double>&>(
      center_of_pressure, plant.GetFrameByName("toe_right"));
  auto left_toe_origin = std::pair<const Vector3d, const Frame<double>&>(
      Vector3d::Zero(), plant.GetFrameByName("toe_left"));
  auto right_toe_origin = std::pair<const Vector3d, const Frame<double>&>(
      Vector3d::Zero(), plant.GetFrameByName("toe_right"));

  // Create state receiver.
  auto state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant);

  // Create the MPC receiver.
  auto mpc_subscriber = builder.AddSystem(
      LcmSubscriberSystem::Make<lcmt_cf_mpfc_output>(
          FLAGS_channel_mpc, &lcm_local));
  std::vector<std::string> left_contact_names{"left_heel", "left_toe"};
  std::vector<std::string> right_contact_names{"right_heel", "right_toe"};
  auto mpc_receiver = builder.AddSystem<CFMPFCOutputReceiver>(
      left_contact_names, right_contact_names, plant);
  builder.Connect(mpc_subscriber->get_output_port(),
                  mpc_receiver->get_input_port_mpfc_output());
  builder.Connect(state_receiver->get_output_port(),
                  mpc_receiver->get_input_port_state());


  // Create command sender.
  auto command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_channel_u, &lcm_local, TriggerTypeSet({TriggerType::kForced})));
  auto command_sender =
      builder.AddSystem<systems::RobotCommandSender>(plant);

  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port());

  auto cassie_out_to_radio =
      builder.AddSystem<systems::CassieOutToRadio>();
  auto cassie_out_receiver =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_cassie_out>(
          FLAGS_cassie_out_channel, &lcm_local));
  auto high_level_command = builder.AddSystem<cassie::osc::HighLevelCommand>(
      plant,
      context.get(),
      gains.vel_scale_rot,
      gains.vel_scale_trans_sagital,
      gains.vel_scale_trans_lateral,
      1.0);

  builder.Connect(*cassie_out_receiver, *cassie_out_to_radio);
  builder.Connect(cassie_out_to_radio->get_output_port(),
                  high_level_command->get_input_port_radio());
  builder.Connect(state_receiver->get_output_port(0),
                  high_level_command->get_input_port_state());

  // Create finite state machine
  int left_stance_state = 0;
  int right_stance_state = 1;
  int post_left_double_support_state = 3;
  int post_right_double_support_state = 4;

  // TODO (@Brian-Acosta) write MPC gains files
  double left_support_duration = 0.3;
  double right_support_duration = 0.3;
  double double_support_duration = 0.1;

  auto fsm = builder.AddSystem<FsmReceiver>(plant);
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

  unordered_fsm_states = {left_stance_state, right_stance_state,
                          post_left_double_support_state,
                          post_right_double_support_state};
  contact_points_in_each_state.push_back(left_toe_mid);
  contact_points_in_each_state.push_back(right_toe_mid);
  contact_points_in_each_state.push_back(left_toe_mid);
  contact_points_in_each_state.push_back(right_toe_mid);



  vector<int> left_right_support_fsm_states = {left_stance_state,
                                               right_stance_state};
  vector<double> left_right_support_state_durations = {left_support_duration,
                                                       right_support_duration};
  vector<std::pair<const Vector3d, const Frame<double>&>> left_right_foot = {
      left_toe_origin, right_toe_origin};

  systems::controllers::SwingFootTrajectoryGeneratorParams swing_params{
      left_right_support_fsm_states,
      left_right_foot,
      {post_left_double_support_state, post_right_double_support_state},
      gains.mid_foot_height,
      gains.final_foot_height,
      gains.final_foot_velocity_z
  };

  auto swing_traj_gen = builder.AddSystem<SwingFootTrajectoryGenerator>(
      plant, context.get(), swing_params);

  builder.Connect(state_receiver->get_output_port(),
                  swing_traj_gen->get_input_port_state());
  builder.Connect(mpc_receiver->get_output_port_footstep_target(),
                  swing_traj_gen->get_input_port_footstep_target());
  builder.Connect(fsm->get_output_port_fsm(),
                  swing_traj_gen->get_input_port_fsm());
  builder.Connect(fsm->get_output_port_next_switch_time(),
                  swing_traj_gen->get_input_port_next_fsm_switch_time());
  builder.Connect(fsm->get_output_port_prev_switch_time(),
                  swing_traj_gen->get_input_port_fsm_switch_time());

  // Swing toe joint trajectory
  map<string, int> pos_map = multibody::MakeNameToPositionsMap(plant);
  vector<std::pair<const Vector3d, const Frame<double>&>> left_foot_points = {
      left_heel, left_toe};
  vector<std::pair<const Vector3d, const Frame<double>&>> right_foot_points = {
      right_heel, right_toe};
  auto left_toe_angle_traj_gen =
      builder.AddSystem<cassie::osc::SwingToeTrajGenerator>(
          plant, context.get(), pos_map["toe_left"],
          left_foot_points, "left_toe_angle_traj");
  auto right_toe_angle_traj_gen =
      builder.AddSystem<cassie::osc::SwingToeTrajGenerator>(
          plant, context.get(), pos_map["toe_right"],
          right_foot_points, "right_toe_angle_traj");
  builder.Connect(state_receiver->get_output_port(0),
                  left_toe_angle_traj_gen->get_input_port_state());
  builder.Connect(state_receiver->get_output_port(0),
                  right_toe_angle_traj_gen->get_input_port_state());
  builder.Connect(mpc_receiver->get_output_port_pitch(),
                  right_toe_angle_traj_gen->get_input_port_toe_angle());
  builder.Connect(mpc_receiver->get_output_port_pitch(),
                  left_toe_angle_traj_gen->get_input_port_toe_angle());

  // Create Operational space control
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      plant, context.get(), true,
      systems::controllers::OscSolverChoice::kFCCQP);

  int n_v = plant.num_velocities();
  int n_u = plant.num_actuators();
  osc->SetAccelerationCostWeights(
      gains.w_accel * MatrixXd::Identity(n_v, n_v));
  osc->SetInputSmoothingCostWeights(
      gains.w_input_reg * MatrixXd::Identity(n_u, n_u));

  // Constraints in OSC
  multibody::KinematicEvaluatorSet<double> evaluators(plant);

  // 1. fourbar constraint
  auto left_loop = LeftLoopClosureEvaluator(plant);
  auto right_loop = RightLoopClosureEvaluator(plant);
  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);

  // Make fixed spring constraint
  auto pos_idx_map = multibody::MakeNameToPositionsMap(plant);
  auto vel_idx_map = multibody::MakeNameToVelocitiesMap(plant);
  std::unique_ptr<FixedJointEvaluator<double>> left_fixed_knee_spring;
  std::unique_ptr<FixedJointEvaluator<double>> right_fixed_knee_spring;
  std::unique_ptr<FixedJointEvaluator<double>> left_fixed_ankle_spring;
  std::unique_ptr<FixedJointEvaluator<double>> right_fixed_ankle_spring;

  left_fixed_knee_spring = std::make_unique<FixedJointEvaluator<double>>(
      plant, pos_idx_map.at("knee_joint_left"),
      vel_idx_map.at("knee_joint_leftdot"), 0);
  right_fixed_knee_spring = std::make_unique<FixedJointEvaluator<double>>(
      plant, pos_idx_map.at("knee_joint_right"),
      vel_idx_map.at("knee_joint_rightdot"), 0);
  left_fixed_ankle_spring = std::make_unique<FixedJointEvaluator<double>>(
      plant, pos_idx_map.at("ankle_spring_joint_left"),
      vel_idx_map.at("ankle_spring_joint_leftdot"), 0);
  right_fixed_ankle_spring = std::make_unique<FixedJointEvaluator<double>>(
      plant, pos_idx_map.at("ankle_spring_joint_right"),
      vel_idx_map.at("ankle_spring_joint_rightdot"), 0);
  evaluators.add_evaluator(left_fixed_knee_spring.get());
  evaluators.add_evaluator(right_fixed_knee_spring.get());
  evaluators.add_evaluator(left_fixed_ankle_spring.get());
  evaluators.add_evaluator(right_fixed_ankle_spring.get());

  osc->AddKinematicConstraint(
      std::unique_ptr<multibody::KinematicEvaluatorSet<double>>(&evaluators));

  osc->SetContactSoftConstraintWeight(gains.w_soft_constraint);
  osc->SetContactFriction(gains.mu);
  osc->SetLambdaContactRegularizationWeight(
      gains.w_lambda * gains.W_lambda_c_regularization);

  // Add contact points (The position doesn't matter. It's not used in OSC)
  const auto& pelvis = plant.GetBodyByName("pelvis");
  multibody::WorldYawViewFrame view_frame(pelvis);
  auto left_toe_evaluator = multibody::WorldPointEvaluator(
      plant, left_toe.first, left_toe.second, view_frame,
      Matrix3d::Identity(), Vector3d::Zero(), {1, 2});
  auto left_heel_evaluator = multibody::WorldPointEvaluator(
      plant, left_heel.first, left_heel.second, view_frame,
      Matrix3d::Identity(), Vector3d::Zero(), {0, 1, 2});
  auto right_toe_evaluator = multibody::WorldPointEvaluator(
      plant, right_toe.first, right_toe.second, view_frame,
      Matrix3d::Identity(), Vector3d::Zero(), {1, 2});
  auto right_heel_evaluator = multibody::WorldPointEvaluator(
      plant, right_heel.first, right_heel.second, view_frame,
      Matrix3d::Identity(), Vector3d::Zero(), {0, 1, 2});
  osc->AddContactPoint(
      "left_toe",
      std::unique_ptr<multibody::WorldPointEvaluator<double>>(&left_toe_evaluator),
      {left_stance_state, post_left_double_support_state, post_right_double_support_state});
  osc->AddContactPoint(
      "left_heel",
      std::unique_ptr<multibody::WorldPointEvaluator<double>>(&left_heel_evaluator),
      {left_stance_state, post_left_double_support_state, post_right_double_support_state});
  osc->AddContactPoint(
      "right_toe",
      std::unique_ptr<multibody::WorldPointEvaluator<double>>(&right_toe_evaluator),
      {right_stance_state, post_left_double_support_state, post_right_double_support_state});
  osc->AddContactPoint(
      "right_heel",
      std::unique_ptr<multibody::WorldPointEvaluator<double>>(&right_heel_evaluator),
      {right_stance_state, post_left_double_support_state, post_right_double_support_state});

  auto swing_foot_data = std::make_unique<TransTaskSpaceTrackingData>(
      "swing_ft_traj", gains.K_p_swing_foot, gains.K_d_swing_foot,
      gains.W_swing_foot, plant);
  swing_foot_data->AddStateAndPointToTrack(left_stance_state, "toe_right");
  swing_foot_data->AddStateAndPointToTrack(right_stance_state, "toe_left");

  auto stance_foot_data = std::make_unique<TransTaskSpaceTrackingData>(
      "stance_foot_data", gains.K_p_swing_foot,
      gains.K_d_swing_foot, gains.W_swing_foot, plant);
  stance_foot_data->AddStateAndPointToTrack(left_stance_state, "toe_left");
  stance_foot_data->AddStateAndPointToTrack(right_stance_state, "toe_right");

  auto swing_ft_traj_local = std::make_unique<RelativeTranslationTrackingData>(
      "swing_ft_traj", gains.K_p_swing_foot, gains.K_d_swing_foot,
      gains.W_swing_foot, plant, swing_foot_data.get(), stance_foot_data.get());

  auto pelvis_view_frame = std::make_shared<WorldYawViewFrame<double>>(
      plant.GetBodyByName("pelvis"));
  swing_ft_traj_local->SetViewFrame(pelvis_view_frame);

  auto osc_debug_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
          "OSC_DEBUG_WALKING", &lcm_local,
          TriggerTypeSet({TriggerType::kForced})));
  builder.Connect(osc->get_output_port_osc_debug(),
                  osc_debug_pub->get_input_port());

  // Create the diagram
  auto owned_diagram = builder.Build();
  owned_diagram->set_name("osc controller for alip_mpfc");
  DrawAndSaveDiagramGraph(*owned_diagram, "../osc_for_cf_mpfc");

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

}

int main(int argc, char** argv) {
  return dairlib::DoMain(argc, argv);
}