#include "mpfc_osc_diagram.h"

// lcmtypes
#include "dairlib/lcmt_robot_input.hpp"

// MPC related
#include "examples/perceptive_locomotion/gains/alip_mpfc_gains.h"
#include "examples/perceptive_locomotion/systems/cassie_ankle_torque_receiver.h"
#include "examples/perceptive_locomotion/diagrams/mpfc_output_from_footstep.h"
#include "systems/primitives/fsm_lcm_systems.h"
#include "systems/controllers/footstep_planning/alip_mpc_output_reciever.h"
#include "systems/controllers/footstep_planning/alip_mpc_interface_system.h"
#include "systems/controllers/footstep_planning/alip_state_calculator_system.h"

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
using systems::controllers::AlipMPCInterfaceSystem;
using systems::controllers::AlipMpcOutputReceiver;
using systems::controllers::ComTrackingData;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::RelativeTranslationTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;
using systems::controllers::alip_utils::AlipStateCalculator;

using multibody::FixedJointEvaluator;
using multibody::WorldPointEvaluator;
using multibody::MakeNameToVelocitiesMap;
using multibody::MakeNameToPositionsMap;


MpfcOscDiagram::MpfcOscDiagram(
    drake::multibody::MultibodyPlant<double>& plant,
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
    evaluators(multibody::KinematicEvaluatorSet<double>(plant)),
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
    view_frame(WorldYawViewFrame<double>(plant.GetBodyByName("pelvis"))),
    left_toe_evaluator(WorldPointEvaluator(
        plant, left_toe.first, left_toe.second, view_frame,
        Matrix3d::Identity(), Vector3d::Zero(), {1, 2})),
    left_heel_evaluator(WorldPointEvaluator(
        plant, left_heel.first, left_heel.second, view_frame,
        Matrix3d::Identity(), Vector3d::Zero(), {0, 1, 2})),
    right_toe_evaluator(WorldPointEvaluator(
        plant, right_toe.first, right_toe.second, view_frame,
        Matrix3d::Identity(), Vector3d::Zero(), {1, 2})),
    right_heel_evaluator(WorldPointEvaluator(
        plant, right_heel.first, right_heel.second, view_frame,
        Matrix3d::Identity(), Vector3d::Zero(), {0, 1, 2})) {

  // Read-in the parameters
  auto gains = LoadYamlFile<OSCWalkingGainsALIP>(osc_gains_filename);
  auto gains_mpc = LoadYamlFile<AlipMpfcGainsImport>(mpc_gains_filename);

  // Build the controller diagram
  DiagramBuilder<double> builder;
  plant_context = plant.CreateDefaultContext();

  // Create state receiver.
  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);
  auto high_level_command = builder.AddSystem<cassie::osc::HighLevelCommand>(
      plant, plant_context.get(), gains.vel_scale_rot,
      gains.vel_scale_trans_sagital, gains.vel_scale_trans_lateral);

  auto mpc_receiver_fsm = builder.AddSystem<AlipMpcOutputReceiver>();
  auto mpc_receiver = builder.AddSystem<AlipMpcOutputReceiver>();
  auto footstep_passthrough = builder.AddSystem<MpfcOutputFromFootstep>(
      gains_mpc.ss_time, gains_mpc.ds_time, plant);
  auto fsm_passthrough = builder.AddSystem<MpfcOutputFromFootstep>(
      gains_mpc.ss_time, gains_mpc.ds_time, plant);
  builder.Connect(state_receiver->get_output_port(0),
                  footstep_passthrough->get_input_port_state());
  builder.Connect(*footstep_passthrough, *mpc_receiver);
  builder.Connect(state_receiver->get_output_port(0),
                  fsm_passthrough->get_input_port_state());
  builder.Connect(*fsm_passthrough, *mpc_receiver_fsm);
  auto dummy_foothold_source =
      builder.AddSystem<drake::systems::ConstantVectorSource<double>>(Vector3d::Zero());
  builder.Connect(dummy_foothold_source->get_output_port(),
                  fsm_passthrough->get_input_port_footstep());

  builder.Connect(state_receiver->get_output_port(0),
                  high_level_command->get_input_port_state());

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
  left_support_duration = gains_mpc.ss_time;
  right_support_duration = gains_mpc.ss_time;
  double_support_duration = gains_mpc.ds_time;

  auto fsm = builder.AddSystem<FsmReceiver>(plant);
  builder.Connect(mpc_receiver_fsm->get_output_port_fsm(),
                  fsm->get_input_port_fsm_info());
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
      gains_mpc.h_des,
      gains.mid_foot_height,
      0.05608,
      gains.final_foot_height,
      gains.final_foot_velocity_z,
      gains_mpc.retraction_dist,
      true
  };

  com_params = {
      gains_mpc.h_des,
      gains_mpc.ds_time,
      unordered_fsm_states,
      contact_points_in_each_state,
      false
  };

  vector<int> post_left_post_right_ds_states =
      {post_left_double_support_state, post_right_double_support_state};
  vector<systems::controllers::alip_utils::PointOnFramed> left_right_contacts(
      {left_toe_mid, right_toe_mid}
  );
  auto alip_calc = builder.AddSystem<AlipStateCalculator>(
      plant,
      plant_context.get(),
      left_right_support_fsm_states,
      post_left_post_right_ds_states,
      left_right_contacts,
      "pelvis"
  );
  builder.Connect(state_receiver->get_output_port(),
                  alip_calc->get_input_port_state());
  builder.Connect(fsm->get_output_port_fsm(),
                  alip_calc->get_input_port_fsm());

  auto mpc_interface = builder.AddSystem<AlipMPCInterfaceSystem>(
      plant, plant_context.get(), com_params, swing_params);
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
      plant, plant, plant_context.get(), plant_context.get(), true);

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


  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);
  evaluators.add_evaluator(&left_fixed_knee_spring);
  evaluators.add_evaluator(&right_fixed_knee_spring);
  evaluators.add_evaluator(&left_fixed_ankle_spring);
  evaluators.add_evaluator(&right_fixed_ankle_spring);
  osc->AddKinematicConstraint(&evaluators);

  osc->SetContactSoftConstraintWeight(gains.w_soft_constraint);
  osc->SetContactFriction(gains.mu);
  osc->AddStateAndContactPoint(left_stance_state, &left_toe_evaluator);
  osc->AddStateAndContactPoint(left_stance_state, &left_heel_evaluator);
  osc->AddStateAndContactPoint(right_stance_state, &right_toe_evaluator);
  osc->AddStateAndContactPoint(right_stance_state, &right_heel_evaluator);
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

  osc->SetOsqpSolverOptionsFromYaml(osqp_settings_filename);

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
      gains_mpc.t_max};
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
      gains.W_swing_foot, plant, plant);
  swing_foot_data->AddStateAndPointToTrack(left_stance_state, "toe_right");
  swing_foot_data->AddStateAndPointToTrack(right_stance_state, "toe_left");

  stance_foot_data = std::make_unique<TransTaskSpaceTrackingData>(
      "com_data", gains.K_p_swing_foot, gains.K_d_swing_foot,
      gains.W_swing_foot, plant, plant);
  stance_foot_data->AddStateAndPointToTrack(left_stance_state, "toe_left");
  stance_foot_data->AddStateAndPointToTrack(right_stance_state, "toe_right");

  swing_ft_data_local = std::make_unique<RelativeTranslationTrackingData>(
      "swing_ft_traj", gains.K_p_swing_foot, gains.K_d_swing_foot,
      gains.W_swing_foot, plant, plant, swing_foot_data.get(),
      stance_foot_data.get());

  swing_ft_data_local->SetSpringsInKinematicCalculation(true);

  pelvis_view_frame = std::make_shared<WorldYawViewFrame<double>>(
      plant.GetBodyByName("pelvis"));
  swing_ft_data_local->SetViewFrame(pelvis_view_frame);

  swing_ft_data_local->SetTimeVaryingPDGainMultiplier(
      swing_ft_gain_multiplier_gain_multiplier);
  swing_ft_data_local->SetTimerVaryingFeedForwardAccelMultiplier(
      swing_ft_accel_gain_multiplier_gain_multiplier);

  center_of_mass_data = std::make_unique<ComTrackingData>(
      "alip_com_traj", gains.K_p_com, gains.K_d_com,
      gains.W_com, plant, plant);
  center_of_mass_data->SetViewFrame(pelvis_view_frame);
  center_of_mass_data->AddFiniteStateToTrack(-1);


  // Pelvis rotation tracking (pitch and roll)
  pelvis_balance_data = std::make_unique<RotTaskSpaceTrackingData>(
      "pelvis_balance_traj", gains.K_p_pelvis_balance, gains.K_d_pelvis_balance,
      gains.W_pelvis_balance, plant, plant);
  pelvis_balance_data->AddFrameToTrack("pelvis");

  // Pelvis rotation tracking (yaw)
  pelvis_heading_data = std::make_unique<RotTaskSpaceTrackingData>(
      "pelvis_heading_traj", gains.K_p_pelvis_heading, gains.K_d_pelvis_heading,
      gains.W_pelvis_heading, plant, plant);
  pelvis_heading_data->AddFrameToTrack("pelvis");

  // Swing toe joint tracking
  swing_toe_data_left = std::make_unique<JointSpaceTrackingData>(
      "left_toe_angle_traj", gains.K_p_swing_toe, gains.K_d_swing_toe,
      gains.W_swing_toe, plant, plant);
  swing_toe_data_right = std::make_unique<JointSpaceTrackingData>(
      "right_toe_angle_traj", gains.K_p_swing_toe, gains.K_d_swing_toe,
      gains.W_swing_toe, plant, plant);
  swing_toe_data_right->AddStateAndJointToTrack(left_stance_state, "toe_right",
                                                "toe_rightdot");
  swing_toe_data_left->AddStateAndJointToTrack(right_stance_state, "toe_left",
                                               "toe_leftdot");

  swing_hip_yaw_data = std::make_unique<JointSpaceTrackingData>(
      "swing_hip_yaw_traj", gains.K_p_hip_yaw, gains.K_d_hip_yaw,
      gains.W_hip_yaw, plant, plant);
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
  osc->AddTrackingData(std::move(center_of_mass_data));
  osc->AddTrackingData(std::move(swing_toe_data_left));
  osc->AddTrackingData(std::move(swing_toe_data_right));
  osc->AddTrackingData(std::move(swing_hip_yaw_data));
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


  output_port_u_cmd_ = builder.ExportOutput(
      osc->get_output_port_osc_command(), "u, t"
  );
  output_port_fsm_ = builder.ExportOutput(
      fsm->get_output_port_fsm(), "fsm"
  );
  output_port_alip_ = builder.ExportOutput(
      alip_calc->get_output_port(), "alip"
  );
  output_port_switching_time_ = builder.ExportOutput(
      fsm->get_output_port_time_until_switch(), "time_until_switch"
  );
  input_port_state_ = builder.ExportInput(
      state_receiver->get_input_port(), "lcm_robot_output"
  );
  input_port_footstep_command_ = builder.ExportInput(
      footstep_passthrough->get_input_port_footstep(), "footstep"
  );
  input_port_radio_ = builder.ExportInput(
      high_level_command->get_input_port_radio(), "lcmt_radio_out"
  );
  builder.ConnectInput(
      "lcmt_radio_out", hip_yaw_traj_gen->get_radio_input_port()
  );
  // Create the diagram
  builder.BuildInto(this);
  this->set_name("osc_controller_for_alip_mpfc");
}

void MpfcOscDiagram::SetSwingFootPositionAtLiftoff(
    drake::systems::Context<double>* context,
    const Vector3d& init_swing_pos) const {
  const auto& swing_traj_sys = dynamic_cast<const drake::systems::Diagram<double>&>(
      GetSubsystemByName(
      "alip_mpc_interface_system"
  )).GetSubsystemByName(
      "swing_ft_traj_interface_system"
  );
  const auto swing_traj_sys_casted =
      dynamic_cast<const systems::controllers::SwingFootInterfaceSystem*>(
          &swing_traj_sys
      );
  auto& swing_traj_ctx = swing_traj_sys_casted->GetMyMutableContextFromRoot(context);
  swing_traj_ctx.SetDiscreteState(
      swing_traj_sys_casted->liftoff_swing_foot_pos_state_idx(),
      init_swing_pos
  );
}

}  // namespace dairlib