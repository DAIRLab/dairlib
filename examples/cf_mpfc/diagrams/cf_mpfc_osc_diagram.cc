#include "cf_mpfc_osc_diagram.h"

// lcmtypes
#include "dairlib/lcmt_robot_input.hpp"

// MPC related
#include "systems/primitives/fsm_lcm_systems.h"
#include "systems/controllers/footstep_planning//cf_mpfc_system.h"
#include "examples/perceptive_locomotion/systems/cassie_ankle_torque_receiver.h"
#include "systems/controllers/footstep_planning/cf_mpfc_output_receiver.h"

// misc
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

// drake
#include "drake/common/yaml/yaml_io.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/framework/diagram_builder.h"


namespace dairlib::perceptive_locomotion {

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
using drake::systems::lcm::LcmScopeSystem;
using drake::systems::TriggerTypeSet;
using drake::trajectories::PiecewisePolynomial;
using drake::yaml::LoadYamlFile;

using multibody::WorldYawViewFrame;
using systems::FsmReceiver;
using systems::controllers::CFMPFCOutputReceiver;
using systems::controllers::ComTrackingData;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::RelativeTranslationTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;
using systems::controllers::SwingFootTrajectoryGenerator;
using systems::controllers::DistanceTrackingData;

using multibody::FixedJointEvaluator;
using multibody::WorldPointEvaluator;
using multibody::MakeNameToVelocitiesMap;
using multibody::MakeNameToPositionsMap;


CfMpfcOscDiagram::CfMpfcOscDiagram(
    const drake::multibody::MultibodyPlant<double>& plant,
    const string& osc_gains_filename, const string& mpc_gains_filename,
    const string& osqp_settings_filename) :
    plant_(&plant),
    left_toe(LeftToeFront(plant)),
    left_heel(LeftToeRear(plant)),
    right_toe(RightToeFront(plant)),
    right_heel(RightToeRear(plant)),
    left_toe_mid({(left_toe.first + left_heel.first) / 2,
                  plant.GetFrameByName("toe_left")}),
    right_toe_mid({(right_toe.first + right_heel.first) / 2,
                   plant.GetFrameByName("toe_right")}),
    left_toe_origin({Vector3d::Zero(), plant.GetFrameByName("toe_left")}),
    right_toe_origin({Vector3d::Zero(), plant.GetFrameByName("toe_right")}),
    left_right_foot({left_toe_origin, right_toe_origin}),
    pos_map(MakeNameToPositionsMap(plant)),
    vel_map(MakeNameToVelocitiesMap(plant)),
    left_foot_points({left_heel, left_toe}),
    right_foot_points({right_heel, right_toe}),
    left_loop(LeftLoopClosureEvaluator(plant)),
    right_loop(RightLoopClosureEvaluator(plant)),
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
                            vel_map.at("ankle_spring_joint_rightdot"), 0)),
    view_frame(WorldYawViewFrame<double>(plant.GetBodyByName("pelvis"))) {

  // Build the controller diagram
  DiagramBuilder<double> builder;
  plant_context = plant.CreateDefaultContext();

  // Read-in the parameters
  auto gains = LoadYamlFile<OSCWalkingGainsALIP>(osc_gains_filename);
  auto gains_mpc = systems::controllers::cf_mpfc_params_io::get_params_from_yaml(
      mpc_gains_filename,
      "examples/perceptive_locomotion/gains/gurobi_options_planner.yaml",
      plant, *plant_context
  );

  // Create state receiver.
  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);
  auto input_sender = builder.AddSystem<systems::RobotCommandSender>(plant);

  auto high_level_command = builder.AddSystem<cassie::osc::HighLevelCommand>(
      plant, plant_context.get(), gains.vel_scale_rot,
      gains.vel_scale_trans_sagital, gains.vel_scale_trans_lateral);

  auto mpc_receiver = builder.AddSystem<CFMPFCOutputReceiver>(plant);
  auto fsm = builder.AddSystem<FsmReceiver>(plant);

  builder.Connect(mpc_receiver->get_output_port_fsm(),
                  fsm->get_input_port_fsm_info());
  input_port_cf_mpc_output_ = builder.ExportInput(
        mpc_receiver->get_input_port_mpfc_output(), "lcmt_alip_mpc_output");

  builder.Connect(state_receiver->get_output_port(0),
                  high_level_command->get_input_port_state());
  builder.Connect(state_receiver->get_output_port(),
                  mpc_receiver->get_input_port_state());

  // Create heading traj generator
  auto head_traj_gen = builder.AddSystem<cassie::osc::HeadingTrajGenerator>(
      plant, plant_context.get());
  auto pitch_traj_gen = builder.AddSystem
      <cassie::osc::PelvisPitchTrajGenerator>(plant, plant_context.get());
  builder.Connect(state_receiver->get_output_port(),
                  head_traj_gen->get_input_port_state());
  builder.Connect(high_level_command->get_output_port_yaw(),
                  head_traj_gen->get_input_port_yaw());
  builder.Connect(state_receiver->get_output_port(),
                  pitch_traj_gen->get_state_input_port());
  builder.Connect(mpc_receiver->get_output_port_pitch(),
                  pitch_traj_gen->get_pitch_input_port());

  // Create finite state machine
  left_stance_state = 0;
  right_stance_state = 1;
  post_left_double_support_state = 3;
  post_right_double_support_state = 4;
  left_support_duration = gains_mpc.gait_params.single_stance_duration;
  right_support_duration = gains_mpc.gait_params.single_stance_duration;
  double_support_duration = gains_mpc.gait_params.double_stance_duration;


  builder.Connect(state_receiver->get_output_port(),
                  fsm->get_input_port_state());

  // Create leafsystem that record the switching time of the FSM
  single_support_states = {left_stance_state, right_stance_state};
  double_support_states = {post_left_double_support_state,
                           post_right_double_support_state};

  unordered_fsm_states = {left_stance_state, right_stance_state,
                          post_left_double_support_state,
                          post_right_double_support_state};
  contact_points_in_each_state.push_back(left_toe_mid);
  contact_points_in_each_state.push_back(right_toe_mid);
  contact_points_in_each_state.push_back(left_toe_mid);
  contact_points_in_each_state.push_back(right_toe_mid);

  left_right_support_fsm_states = {left_stance_state, right_stance_state};
  left_right_support_state_durations = {left_support_duration,
                                        right_support_duration};

  swing_params = {
      left_right_support_fsm_states,
      left_right_foot,
      {post_left_double_support_state, post_right_double_support_state},
      gains.mid_foot_height,
      gains.final_foot_height,
      gains.final_foot_velocity_z,
      0.0, // retraction dist is unused
      true // make the swing foot generator driven by a standalone simulator
  };

  auto swing_traj_gen = builder.AddSystem<SwingFootTrajectoryGenerator>(
      plant, plant_context.get(), swing_params);

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

  vector<int> post_left_post_right_ds_states =
      {post_left_double_support_state, post_right_double_support_state};
  vector<systems::controllers::alip_utils::PointOnFramed> left_right_contacts(
      {left_toe_mid, right_toe_mid}
  );

  auto left_toe_angle_traj_gen =
      builder.AddSystem<cassie::osc::SwingToeTrajGenerator>(
          plant, plant_context.get(), pos_map["toe_left"],
          left_foot_points, "left_toe_angle_traj");
  auto right_toe_angle_traj_gen =
      builder.AddSystem<cassie::osc::SwingToeTrajGenerator>(
          plant, plant_context.get(), pos_map["toe_right"],
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
          plant,
          left_right_support_fsm_states,
          vector<string>({"toe_left_motor", "toe_right_motor"}));

  // Create Operational space control
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      plant, plant_context.get(), true,
      systems::controllers::OscSolverChoice::kFCCQP
  );

  // Cost
  int n_v = plant.num_velocities();
  int n_u = plant.num_actuators();
  MatrixXd Q_accel = gains.w_accel * MatrixXd::Identity(n_v, n_v);
  osc->SetAccelerationCostWeights(Q_accel);
  if (gains.w_input_reg > 0) {
    osc->SetInputSmoothingCostWeights(
        gains.w_input_reg * MatrixXd::Identity(n_u, n_u)
    );
  }

  // Constraints in OSC
  auto evaluators = std::make_unique<KinematicEvaluatorSet<double>>(plant);

  std::vector<int> yz_active = {1, 2};
  std::vector<int> xyz_active = {0, 1, 2};
  auto left_toe_evaluator = std::make_unique<WorldPointEvaluator<double>>(
      plant, left_toe.first, left_toe.second, view_frame,
      Matrix3d::Identity(), Vector3d::Zero(), yz_active);
  auto left_heel_evaluator = std::make_unique<WorldPointEvaluator<double>>(
      plant, left_heel.first, left_heel.second, view_frame,
      Matrix3d::Identity(), Vector3d::Zero(), xyz_active);
  auto right_toe_evaluator = std::make_unique<WorldPointEvaluator<double>>(
      plant, right_toe.first, right_toe.second, view_frame,
      Matrix3d::Identity(), Vector3d::Zero(), yz_active);
  auto right_heel_evaluator = std::make_unique<WorldPointEvaluator<double>>(
      plant, right_heel.first, right_heel.second, view_frame,
      Matrix3d::Identity(), Vector3d::Zero(), xyz_active);

  evaluators->add_evaluator(&left_loop);
  evaluators->add_evaluator(&right_loop);
  evaluators->add_evaluator(&left_fixed_knee_spring);
  evaluators->add_evaluator(&right_fixed_knee_spring);
  evaluators->add_evaluator(&left_fixed_ankle_spring);
  evaluators->add_evaluator(&right_fixed_ankle_spring);

  osc->AddKinematicConstraint(std::move(evaluators));

  osc->SetContactSoftConstraintWeight(gains.w_soft_constraint);
  osc->SetContactFriction(gains.mu);

  osc->AddContactPoint(
      "left_toe", std::move(left_toe_evaluator),
      {left_stance_state, post_left_double_support_state, post_right_double_support_state});
  osc->AddContactPoint(
      "left_heel", std::move(left_heel_evaluator),
      {left_stance_state, post_left_double_support_state, post_right_double_support_state});
  osc->AddContactPoint(
      "right_toe", std::move(right_toe_evaluator),
      {right_stance_state, post_left_double_support_state, post_right_double_support_state});
  osc->AddContactPoint(
      "right_heel", std::move(right_heel_evaluator),
      {right_stance_state, post_left_double_support_state, post_right_double_support_state});

  osc->SetSolverOptionsFromYaml(osqp_settings_filename);

  // Swing foot tracking
  std::vector<double> swing_ft_gain_schedule_breaks =
      {0, left_support_duration / 2, left_support_duration};
  std::vector<drake::MatrixX < double>>
  swing_ft_gain_multiplier_samples(
      3, drake::MatrixX<double>::Identity(3, 3));
  swing_ft_gain_multiplier_samples[2](2, 2) *= 0.5;
  swing_ft_gain_multiplier_gain_multiplier =
      std::make_shared<PiecewisePolynomial < double>>
  (
      PiecewisePolynomial<double>::FirstOrderHold(
          swing_ft_gain_schedule_breaks,
          swing_ft_gain_multiplier_samples));
  std::vector<double> swing_ft_accel_gain_multiplier_breaks{
      0, left_support_duration / 2, left_support_duration * 3 / 4,
      left_support_duration};
  std::vector<drake::MatrixX < double>>
  swing_ft_accel_gain_multiplier_samples(
      4, drake::MatrixX<double>::Identity(3, 3));
  swing_ft_accel_gain_multiplier_samples[0](2, 2) *= 1.1;
  swing_ft_accel_gain_multiplier_samples[2](2, 2) *= 0.5;
  swing_ft_accel_gain_multiplier_samples[3](2, 2) *= 0;
  swing_ft_accel_gain_multiplier_samples[2](0, 0) *= 0.15;
  swing_ft_accel_gain_multiplier_samples[3](0, 0) *= 0.0;
  swing_ft_accel_gain_multiplier_gain_multiplier =
      std::make_shared<PiecewisePolynomial < double>>
  (
      PiecewisePolynomial<double>::FirstOrderHold(
          swing_ft_accel_gain_multiplier_breaks,
          swing_ft_accel_gain_multiplier_samples));

  swing_foot_data = std::make_unique<TransTaskSpaceTrackingData>(
      "swing_ft_traj", gains.K_p_swing_foot, gains.K_d_swing_foot,
      gains.W_swing_foot, plant);
  swing_foot_data->AddStateAndPointToTrack(left_stance_state, "toe_right");
  swing_foot_data->AddStateAndPointToTrack(right_stance_state, "toe_left");

  stance_foot_data = std::make_unique<TransTaskSpaceTrackingData>(
      "com_data", gains.K_p_swing_foot, gains.K_d_swing_foot,
      gains.W_swing_foot, plant);
  stance_foot_data->AddStateAndPointToTrack(left_stance_state, "toe_left");
  stance_foot_data->AddStateAndPointToTrack(right_stance_state, "toe_right");

  swing_ft_data_local = std::make_unique<RelativeTranslationTrackingData>(
      "swing_ft_traj", gains.K_p_swing_foot, gains.K_d_swing_foot,
      gains.W_swing_foot, plant, swing_foot_data.get(),
      stance_foot_data.get());

  pelvis_view_frame = std::make_shared<WorldYawViewFrame<double>>(
      plant.GetBodyByName("pelvis"));
  swing_ft_data_local->SetViewFrame(pelvis_view_frame);

  swing_ft_data_local->SetTimeVaryingPDGainMultiplier(
      swing_ft_gain_multiplier_gain_multiplier);
  swing_ft_data_local->SetTimerVaryingFeedForwardAccelMultiplier(
      swing_ft_accel_gain_multiplier_gain_multiplier);

  stance_foot_data_for_leg_len = std::make_unique<TransTaskSpaceTrackingData>(
      "stance_foot_data_for_leg_len", gains.K_p_swing_foot,
      gains.K_d_swing_foot, gains.W_swing_foot, plant);
  stance_foot_data_for_leg_len->AddStateAndPointToTrack(left_stance_state, "toe_left", left_toe_mid.first);
  stance_foot_data_for_leg_len->AddStateAndPointToTrack(post_right_double_support_state, "toe_left", left_toe_mid.first);
  stance_foot_data_for_leg_len->AddStateAndPointToTrack(right_stance_state, "toe_right", right_toe_mid.first);
  stance_foot_data_for_leg_len->AddStateAndPointToTrack(post_left_double_support_state, "toe_right", right_toe_mid.first);

  center_of_mass_data = std::make_unique<ComTrackingData>(
      "alip_com_traj", gains.K_p_com, gains.K_d_com,
      gains.W_com, plant);
  center_of_mass_data->AddFiniteStateToTrack(left_stance_state);
  center_of_mass_data->AddFiniteStateToTrack(post_left_double_support_state);
  center_of_mass_data->AddFiniteStateToTrack(right_stance_state);
  center_of_mass_data->AddFiniteStateToTrack(post_right_double_support_state);

  kpll = gains.K_p_com.bottomRightCorner<1,1>();
  kdll = gains.K_d_com.bottomRightCorner<1,1>();
  wll = gains.W_com.bottomRightCorner<1,1>();
  leg_length_data = std::make_unique<DistanceTrackingData>(
      "leg_length", kpll, kdll, wll, plant, stance_foot_data_for_leg_len.get(),
      center_of_mass_data.get());


  // Pelvis rotation tracking (pitch and roll)
  pelvis_balance_data = std::make_unique<RotTaskSpaceTrackingData>(
      "pelvis_balance_traj", gains.K_p_pelvis_balance, gains.K_d_pelvis_balance,
      gains.W_pelvis_balance, plant);
  pelvis_balance_data->AddFrameToTrack("pelvis");

  // Pelvis rotation tracking (yaw)
  pelvis_heading_data = std::make_unique<RotTaskSpaceTrackingData>(
      "pelvis_heading_traj", gains.K_p_pelvis_heading, gains.K_d_pelvis_heading,
      gains.W_pelvis_heading, plant);
  pelvis_heading_data->AddFrameToTrack("pelvis");

  // Swing toe joint tracking
  swing_toe_data_left = std::make_unique<JointSpaceTrackingData>(
      "left_toe_angle_traj", gains.K_p_swing_toe, gains.K_d_swing_toe,
      gains.W_swing_toe, plant);
  swing_toe_data_right = std::make_unique<JointSpaceTrackingData>(
      "right_toe_angle_traj", gains.K_p_swing_toe, gains.K_d_swing_toe,
      gains.W_swing_toe, plant);
  swing_toe_data_right->AddStateAndJointToTrack(left_stance_state, "toe_right",
                                                "toe_rightdot");
  swing_toe_data_left->AddStateAndJointToTrack(right_stance_state, "toe_left",
                                               "toe_leftdot");

  swing_hip_yaw_data = std::make_unique<JointSpaceTrackingData>(
      "swing_hip_yaw_traj", gains.K_p_hip_yaw, gains.K_d_hip_yaw,
      gains.W_hip_yaw, plant);
  swing_hip_yaw_data->AddStateAndJointToTrack(
      left_stance_state, "hip_yaw_right", "hip_yaw_rightdot");
  swing_hip_yaw_data->AddStateAndJointToTrack(
      right_stance_state, "hip_yaw_left", "hip_yaw_leftdot");

  auto hip_yaw_traj_gen =
      builder.AddSystem<cassie::HipYawTrajGen>(left_stance_state);

  builder.Connect(fsm->get_output_port_fsm(),
                  hip_yaw_traj_gen->get_fsm_input_port());

  osc->AddTrackingData(std::move(swing_ft_data_local));
  osc->AddTrackingData(std::move(pelvis_balance_data));
  osc->AddTrackingData(std::move(swing_toe_data_left));
  osc->AddTrackingData(std::move(swing_toe_data_right));
  osc->AddTrackingData(std::move(swing_hip_yaw_data));
  osc->AddTrackingData(std::move(leg_length_data));
  osc->AddTrackingData(std::move(pelvis_heading_data),
                       gains.period_of_no_heading_control);

  // Set double support duration for force blending
  osc->SetUpDoubleSupportPhaseBlending(
      double_support_duration, left_stance_state, right_stance_state,
      {post_left_double_support_state, post_right_double_support_state});

  if (gains.W_com(0, 0) == 0) {
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
  builder.Connect(mpc_receiver->get_output_port_r_traj(),
                  osc->get_input_port_tracking_data("leg_length"));
  builder.Connect(swing_traj_gen->get_output_port_swing_foot_traj(),
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
                  alip_input_receiver->get_input_port_u_vec());
  builder.Connect(alip_input_receiver->get_output_port(),
                  osc->get_feedforward_input_port());
  builder.Connect(osc->get_output_port_osc_command(),
                  input_sender->get_input_port());

  output_port_u_cmd_ = builder.ExportOutput(
      osc->get_output_port_osc_command(), "u, t"
  );
  output_port_u_lcm_ = builder.ExportOutput(
      input_sender->get_output_port(), "lcmt_robot_input"
  );
  output_port_osc_debug_ = builder.ExportOutput(
      osc->get_output_port_osc_debug(), "lcmt_osc_debug"
  );
  input_port_state_ = builder.ExportInput(
      state_receiver->get_input_port(), "lcm_robot_output"
  );
  input_port_radio_ = builder.ExportInput(
      high_level_command->get_input_port_radio(), "lcmt_radio_out"
  );
  builder.ConnectInput(
      "lcmt_radio_out", hip_yaw_traj_gen->get_radio_input_port()
  );
  // Create the diagram
  builder.BuildInto(this);
  this->set_name("osc_controller_for_cf_mpfc");
  DrawAndSaveDiagramGraph(*this);
}


void CfMpfcOscDiagram::SetSwingFootPositionAtLiftoff(
    drake::systems::Context<double>* context,
    const Vector3d& init_swing_pos) const {
  const auto& swing_traj_sys = GetSubsystemByName(
    "swing_ft_traj_interface_system");
  const auto swing_traj_sys_casted =
      dynamic_cast<const systems::controllers::SwingFootTrajectoryGenerator*>(
          &swing_traj_sys
      );
  auto& swing_traj_ctx = swing_traj_sys_casted->GetMyMutableContextFromRoot(context);
  swing_traj_ctx.SetDiscreteState(
      swing_traj_sys_casted->liftoff_swing_foot_pos_state_idx(),
      init_swing_pos
  );
}

}  // namespace dairlib