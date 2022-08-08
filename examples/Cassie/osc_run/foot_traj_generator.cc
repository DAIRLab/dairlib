#include "foot_traj_generator.h"

#include <iostream>

#include <drake/math/saturate.h>

#include "dairlib/lcmt_radio_out.hpp"
#include "examples/Cassie/contact_scheduler/contact_scheduler.h"
#include "multibody/multibody_utils.h"

using Eigen::Map;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using std::string;

using dairlib::systems::OutputVector;
using drake::multibody::BodyFrame;
using drake::multibody::Frame;
using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

namespace dairlib::examples::osc_run {

FootTrajGenerator::FootTrajGenerator(const MultibodyPlant<double>& plant,
                                     Context<double>* context,
                                     const string& foot_name,
                                     const string& hip_name, bool relative_feet,
                                     const int stance_state,
                                     std::vector<double> state_durations)
    : plant_(plant),
      context_(context),
      world_(plant.world_frame()),
      foot_frame_(plant.GetFrameByName(foot_name)),
      hip_frame_(plant.GetFrameByName(hip_name)),
      relative_feet_(relative_feet),
      stance_state_(stance_state),
      state_durations_(state_durations) {
  PiecewisePolynomial<double> empty_pp_traj(VectorXd(0));
  Trajectory<double>& traj_inst = empty_pp_traj;

  if (foot_name == "toe_left") {
    is_left_foot_ = true;
    this->set_name("left_ft_traj");
    this->DeclareAbstractOutputPort("left_ft_traj", traj_inst,
                                    &FootTrajGenerator::CalcTraj);
  } else {
    is_left_foot_ = false;
    this->set_name("right_ft_traj");
    this->DeclareAbstractOutputPort("right_ft_traj", traj_inst,
                                    &FootTrajGenerator::CalcTraj);
  }

  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(
                        "x", OutputVector<double>(plant_.num_positions(),
                                                  plant_.num_velocities(),
                                                  plant_.num_actuators()))
                    .get_index();
  target_vel_port_ = this->DeclareVectorInputPort(
                             "v_des", BasicVector<double>(VectorXd::Zero(2)))
                         .get_index();
  fsm_port_ = this->DeclareVectorInputPort(
                      "fsm", BasicVector<double>(VectorXd::Zero(1)))
                  .get_index();
  clock_port_ = this->DeclareVectorInputPort(
                        "clock", BasicVector<double>(VectorXd::Zero(1)))
                    .get_index();
  radio_port_ =
      this->DeclareAbstractInputPort("lcmt_radio_out",
                                     drake::Value<dairlib::lcmt_radio_out>{})
          .get_index();
  contact_scheduler_port_ =
      this->DeclareVectorInputPort("t_mode", BasicVector<double>(6))
          .get_index();

  // The swing foot position in the beginning of the swing phase
  initial_foot_pos_idx_ = this->DeclareDiscreteState(3);
  initial_hip_pos_idx_ = this->DeclareDiscreteState(3);
  pelvis_yaw_idx_ = this->DeclareDiscreteState(1);
  pelvis_vel_est_idx_ = this->DeclareDiscreteState(3);
  last_stance_timestamp_idx_ = this->DeclareDiscreteState(1);

  // State variables inside this controller block
  DeclarePerStepDiscreteUpdateEvent(&FootTrajGenerator::DiscreteVariableUpdate);

  m_ = plant_.CalcTotalMass(*context_);
}

EventStatus FootTrajGenerator::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  // Read in current state
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  // Read in finite state machine
  VectorXd fsm_state = this->EvalVectorInput(context, fsm_port_)->get_value();

  VectorXd q = robot_output->GetPositions();
  VectorXd v = robot_output->GetVelocities();
  multibody::SetPositionsIfNew<double>(plant_, q, context_);

  Vector3d pelvis_heading_vec =
      plant_.EvalBodyPoseInWorld(*context_, plant_.GetBodyByName("pelvis"))
          .rotation()
          .col(0);
  double approx_pelvis_yaw =
      atan2(pelvis_heading_vec(1), pelvis_heading_vec(0));
  discrete_state->get_mutable_vector(pelvis_yaw_idx_).get_mutable_value()(0) =
      approx_pelvis_yaw;

  Eigen::Matrix3d rot;
  rot << cos(approx_pelvis_yaw), -sin(approx_pelvis_yaw), 0,
      sin(approx_pelvis_yaw), cos(approx_pelvis_yaw), 0, 0, 0, 1;

  // Only update the current foot position when in stance (
  if (fsm_state(0) == stance_state_) {
    auto foot_pos = discrete_state->get_mutable_vector(initial_foot_pos_idx_)
                        .get_mutable_value();
    plant_.CalcPointsPositions(*context_, foot_frame_, Vector3d::Zero(), world_,
                               &foot_pos);
    auto hip_pos = discrete_state->get_mutable_vector(initial_hip_pos_idx_)
                       .get_mutable_value();
    plant_.CalcPointsPositions(*context_, hip_frame_, Vector3d::Zero(), world_,
                               &hip_pos);
    foot_pos = rot.transpose() * foot_pos;
    hip_pos = rot.transpose() * hip_pos;
    auto pelvis_vel = discrete_state->get_mutable_vector(pelvis_vel_est_idx_)
                          .get_mutable_value();
    pelvis_vel = 0.99 * v.segment(3, 3) + 0.01 * pelvis_vel;
    auto last_stance_time =
        discrete_state->get_mutable_vector(pelvis_vel_est_idx_)
            .get_mutable_value();
    last_stance_time[0] = robot_output->get_timestamp();
    //    pelvis_vel = v.segment(3, 3);
    //    std::cout << "stance state: " << stance_state_ << std::endl;
    //    pelvis_vel = Vector3d::Zero();
  }
  //  if (fsm_state(0) != stance_state_) {
  //  }

  return EventStatus::Succeeded();
}

PiecewisePolynomial<double> FootTrajGenerator::GenerateFlightTraj(
    const drake::systems::Context<double>& context) const {
  const auto robot_output =
      this->template EvalVectorInput<OutputVector>(context, state_port_);
  const auto desired_pelvis_vel_xy =
      this->EvalVectorInput(context, target_vel_port_)->get_value();
  double clock = this->EvalVectorInput(context, clock_port_)->get_value()(0);
  const auto& mode_lengths =
      this->EvalVectorInput(context, contact_scheduler_port_)->get_value();

  // Offset between 0 and 2
  double lateral_radio_tuning = 1.0;
  double sagital_radio_tuning = 1.0;
  if (this->get_input_port(radio_port_).HasValue(context)) {
    const auto& radio_out =
        this->EvalInputValue<dairlib::lcmt_radio_out>(context, radio_port_);
    lateral_radio_tuning += radio_out->channel[4];
    sagital_radio_tuning += radio_out->channel[5];
  }

  VectorXd q = robot_output->GetPositions();
  VectorXd v = robot_output->GetVelocities();
  multibody::SetPositionsAndVelocitiesIfNew<double>(
      plant_, robot_output->GetState(), context_);

  Vector3d pelvis_heading_vec =
      plant_.EvalBodyPoseInWorld(*context_, plant_.GetBodyByName("pelvis"))
          .rotation()
          .col(0);
  double approx_pelvis_yaw =
      atan2(pelvis_heading_vec(1), pelvis_heading_vec(0));
  Eigen::Matrix3d rot;
  rot << cos(approx_pelvis_yaw), -sin(approx_pelvis_yaw), 0,
      sin(approx_pelvis_yaw), cos(approx_pelvis_yaw), 0, 0, 0, 1;

  // TODO(yangwill): should not use estimated pelvis velocity - from discussion
  // with OSU DRL

  Vector3d desired_pelvis_vel;
  desired_pelvis_vel << desired_pelvis_vel_xy, 0;
  /// ALIP
  // x refers to sagital plane
  // y refers to the lateral plane
  auto foot_pos = context.get_discrete_state(initial_foot_pos_idx_).get_value();
  Vector3d pelvis_pos;
  plant_.CalcPointsPositions(*context_, hip_frame_, Vector3d::Zero(), world_,
                             &pelvis_pos);

  VectorXd pelvis_vel = v.segment(3, 3);
  VectorXd pelvis_vel_err = rot.transpose() * pelvis_vel - desired_pelvis_vel;
  VectorXd foot_end_pos_des = Kd_ * (pelvis_vel_err);

  if (is_left_foot_) {
    foot_end_pos_des(1) += lateral_radio_tuning * lateral_offset_;
  } else {
    foot_end_pos_des(1) -= lateral_radio_tuning * lateral_offset_;
  }
  foot_end_pos_des(0) += sagital_radio_tuning * sagital_offset_;
  foot_end_pos_des(2) = 0;

  double pelvis_t0 = mode_lengths[0];
  double pelvis_tf = mode_lengths[1];
  double left_t0 = mode_lengths[2];
  double left_tf = mode_lengths[3];
  double right_t0 = mode_lengths[4];
  double right_tf = mode_lengths[5];

  std::vector<double> T_waypoints;
  std::vector<double> T_waypoints_0;
  std::vector<double> T_waypoints_1;
  std::vector<double> T_waypoints_2;

  // Storing old method for record keeping
  T_waypoints = {
      state_durations_[1],
      state_durations_[1] + 0.5 * (state_durations_[4] - state_durations_[1]),
      state_durations_[4]};

  //  T_waypoints_0 = {
  //      state_durations_[0],
  //      (state_durations_[3] - state_durations_[4]) +
  //          0.5 * (0.1 + state_durations_[2] - state_durations_[0]),
  //      state_durations_[2]};
  //  T_waypoints_1 = {state_durations_[2], state_durations_[3]};
  //  T_waypoints_2 = {state_durations_[3], state_durations_[4]};
  //
  //

  if (is_left_foot_) {
    //    std::cout << "is left foot: " << is_left_foot_ << std::endl;
    T_waypoints = {left_t0, left_t0 + 0.6 * (left_tf - left_t0), left_tf};
    //    std::cout << T_waypoints.back() - T_waypoints.front() << std::endl;
    //    std::cout << left_t0 << std::endl;
    //    std::cout << left_t0 + 0.6 * (left_tf - left_t0) << std::endl;
    //    std::cout << left_tf << std::endl;
  } else {
    //    std::cout << "is left foot: " << is_left_foot_ << std::endl;
    T_waypoints = {right_t0, right_t0 + 0.6 * (right_tf - right_t0), right_tf};
    //    std::cout << T_waypoints.back() - T_waypoints.front() << std::endl;
    //    std::cout << right_t0 << std::endl;
    //    std::cout << right_t0 + 0.6 * (right_tf - right_t0) << std::endl;
    //    std::cout << right_tf << std::endl;
  }
//  std::cout << T_waypoints.back() - T_waypoints.front() << std::endl;
  //  T_waypoints_0 = {
  //      pelvis_t0,
  //      (state_durations_[3] - state_durations_[4]) +
  //          1.5 / 3.0 * (0.1 + state_durations_[2] - pelvis_t0),
  //      state_durations_[2]};
  //  T_waypoints_1 = {state_durations_[2], state_durations_[3]};
  //  T_waypoints_2 = {state_durations_[3], state_durations_[4]};

  //  auto foot_pos =
  //  context.get_discrete_state(initial_foot_pos_idx_).get_value();
  auto hip_pos = context.get_discrete_state(initial_hip_pos_idx_).get_value();
  std::vector<MatrixXd> Y(T_waypoints.size(), VectorXd::Zero(3));
  VectorXd start_pos = foot_pos - hip_pos;
  Y[0] = start_pos;
  if (start_pos(2) == 0) {
    Y[0](2) = -rest_length_;
  }
  Y[1] = start_pos + 0.85 * foot_end_pos_des;
  Y[1](2) = -rest_length_ + mid_foot_height_;
  Y[2] = foot_end_pos_des;
  Y[2](2) = -rest_length_;

  // corrections
  if (is_left_foot_) {
    Y[1](1) -= 0.25 * lateral_offset_;
    Y[1](1) = drake::math::saturate(Y[1](1), lateral_offset_, 0.2);
    Y[2](1) = drake::math::saturate(Y[2](1), lateral_offset_, 0.2);
  } else {
    Y[1](1) += 0.25 * lateral_offset_;
    Y[1](1) = drake::math::saturate(Y[1](1), -0.2, -lateral_offset_);
    Y[2](1) = drake::math::saturate(Y[2](1), -0.2, -lateral_offset_);
  }

  //  MatrixXd Y_dot_start = MatrixXd::Zero(3, 1);
  //  MatrixXd Y_dot_end = MatrixXd::Zero(3, 1);
  //  Y_dot_end(2) = -0.1;

  //  PiecewisePolynomial<double> swing_foot_spline =
  //      PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
  //          T_waypoints, Y, Y_dot_start, Y_dot_end);
//  std::cout << "is left foot: " << is_left_foot_ << std::endl;
//  for (auto& t :T_waypoints){
//    std::cout << "t: " << t << std::endl;
//  }
  PiecewisePolynomial<double> swing_foot_spline =
      PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
          T_waypoints, Y, false);
  return swing_foot_spline;

  if (is_left_foot_) {
    return swing_foot_spline;
  } else {
    //    MatrixXd Y0 = MatrixXd::Zero(3, 3);
    //    MatrixXd Ydot0 = MatrixXd::Zero(3, 3);
    //    MatrixXd Y1 = MatrixXd::Zero(2, 3);
    //    MatrixXd Ydot1 = MatrixXd::Zero(2, 3);
    std::vector<MatrixXd> Y0(T_waypoints_0.size(), VectorXd::Zero(3));
    std::vector<MatrixXd> Ydot0(T_waypoints_0.size(), VectorXd::Zero(3));
    std::vector<MatrixXd> Y2(T_waypoints_1.size(), VectorXd::Zero(3));
    std::vector<MatrixXd> Ydot2(T_waypoints_1.size(), VectorXd::Zero(3));
    Y0[0] = swing_foot_spline.value(state_durations_[2]);
    Y0[1] = swing_foot_spline.value(T_waypoints[1]);
    Y0[2] = swing_foot_spline.value(T_waypoints[2]);
    Ydot0[0] = swing_foot_spline.EvalDerivative(state_durations_[2], 1);
    Ydot0[1] = swing_foot_spline.EvalDerivative(T_waypoints[1], 1);
    Ydot0[2] = swing_foot_spline.EvalDerivative(T_waypoints[2], 1);
    Y2[0] = swing_foot_spline.value(state_durations_[1]);
    Y2[1] = swing_foot_spline.value(state_durations_[2]);
    Ydot2[0] = swing_foot_spline.EvalDerivative(state_durations_[1], 1);
    Ydot2[1] = swing_foot_spline.EvalDerivative(state_durations_[2], 1);
    PiecewisePolynomial<double> offset_swing_foot_spline =
        PiecewisePolynomial<double>::CubicHermite(T_waypoints_0, Y0, Ydot0);
    offset_swing_foot_spline.ConcatenateInTime(
        PiecewisePolynomial<double>::ZeroOrderHold(T_waypoints_1, Y2));
    offset_swing_foot_spline.ConcatenateInTime(
        PiecewisePolynomial<double>::CubicHermite(T_waypoints_2, Y2, Ydot2));
    //    for (auto t: offset_swing_foot_spline.get_segment_times()) {
    //      std::cout << t << std::endl;
    //    }
    return offset_swing_foot_spline;
  }
  //  return swing_foot_spline;
}

void FootTrajGenerator::CalcTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Read in current state
  //  const auto robot_output =
  //      this->template EvalVectorInput<OutputVector>(context, state_port_);
  //  VectorXd x = robot_output->GetState();
  //  double timestamp = robot_output->get_timestamp();
  //
  //  // Read in finite state machine
  const auto fsm_state = this->EvalVectorInput(context, fsm_port_)->get_value()[0];

  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  if (is_left_foot_ ) {
    if(fsm_state != LEFT_STANCE){
//      *casted_traj =  PiecewisePolynomial<double>(Vector3d{0, 0, rest_length_});
      *casted_traj = GenerateFlightTraj(context);
    }
//    else{
//      *casted_traj =  PiecewisePolynomial<double>(Vector3d{0, 0, rest_length_});
//    }

  }
  else{
    if(fsm_state != RIGHT_STANCE){
      *casted_traj = GenerateFlightTraj(context);
//      *casted_traj =  PiecewisePolynomial<double>(Vector3d{0, 0, rest_length_});
    }
//    else{
//      *casted_traj =  PiecewisePolynomial<double>(Vector3d{0, 0, rest_length_});
//    }
  }
}

}  // namespace dairlib::examples::osc_run
