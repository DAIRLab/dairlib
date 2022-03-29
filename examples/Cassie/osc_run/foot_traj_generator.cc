#include "foot_traj_generator.h"

#include <drake/math/saturate.h>

#include "multibody/multibody_utils.h"

using Eigen::Map;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
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

  // The swing foot position in the beginning of the swing phase
  initial_foot_pos_idx_ = this->DeclareDiscreteState(3);
  initial_hip_pos_idx_ = this->DeclareDiscreteState(3);
  pelvis_yaw_idx_ = this->DeclareDiscreteState(1);
  pelvis_vel_est_idx_ = this->DeclareDiscreteState(3);

  // State variables inside this controller block
  DeclarePerStepDiscreteUpdateEvent(&FootTrajGenerator::DiscreteVariableUpdate);
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
  VectorXd pelvis_vel = v.segment(3, 3);
  pelvis_vel(0) = context.get_discrete_state(pelvis_vel_est_idx_).GetAtIndex(0);
//  pelvis_vel(1) = context.get_discrete_state(pelvis_vel_est_idx_).GetAtIndex(1);
  VectorXd pelvis_vel_err = rot.transpose() * pelvis_vel - desired_pelvis_vel;
  VectorXd footstep_correction = Kd_ * (pelvis_vel_err);
  if (is_left_foot_) {
    footstep_correction(1) += center_line_offset_;
  } else {
    footstep_correction(1) -= center_line_offset_;
  }
  footstep_correction(0) += footstep_offset_;
  footstep_correction(2) = 0;

  std::vector<double> T_waypoints;
  std::vector<double> T_waypoints_0;
  std::vector<double> T_waypoints_1;
  std::vector<double> T_waypoints_2;
  T_waypoints = {state_durations_[1],
                 state_durations_[1] +
                     1.5 / 3.0 * (state_durations_[4] - state_durations_[1]),
                 state_durations_[4]};
  T_waypoints_0 = {
      state_durations_[0],
      (state_durations_[3] - state_durations_[4]) +
          1.5 / 3.0 * (0.1 + state_durations_[2] - state_durations_[0]),
      state_durations_[2]};
  T_waypoints_1 = {state_durations_[2], state_durations_[3]};
  T_waypoints_2 = {state_durations_[3], state_durations_[4]};

  auto foot_pos = context.get_discrete_state(initial_foot_pos_idx_).get_value();
  auto hip_pos = context.get_discrete_state(initial_hip_pos_idx_).get_value();
  std::vector<MatrixXd> Y(T_waypoints.size(), VectorXd::Zero(3));
  VectorXd start_pos = foot_pos - hip_pos;
  Y[0] = start_pos;
  Y[0](2) = -rest_length_;
  Y[1] = start_pos + 0.85 * footstep_correction;
  Y[1](2) = -rest_length_ + mid_foot_height_;
  Y[2] = footstep_correction;
  Y[2](2) = -rest_length_ + mid_foot_height_ / 2;

  // corrections
  if (is_left_foot_) {
    Y[1](1) -= 0.25 * center_line_offset_;
    //    Y[0](1) = drake::math::saturate(Y[2](1), 0.05, 0.2);
    Y[1](1) = drake::math::saturate(Y[1](1), center_line_offset_, 0.2);
    Y[2](1) = drake::math::saturate(Y[2](1), center_line_offset_, 0.2);
  } else {
    Y[1](1) += 0.25 * center_line_offset_;
    //    Y[0](1) = drake::math::saturate(Y[2](1), -0.2, -0.05);
    Y[1](1) = drake::math::saturate(Y[1](1), -0.2, -center_line_offset_);
    Y[2](1) = drake::math::saturate(Y[2](1), -0.2, -center_line_offset_);
  }

  MatrixXd Y_dot_start = MatrixXd::Zero(3, 1);
  MatrixXd Y_dot_end = MatrixXd::Zero(3, 1);
  Y_dot_end(2) = -0.1;

  PiecewisePolynomial<double> swing_foot_spline =
      PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
          T_waypoints, Y, Y_dot_start, Y_dot_end);

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
    //    for (auto t : offset_swing_foot_spline.get_segment_times()){
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
  //  const auto fsm_state = this->EvalVectorInput(context,
  //  fsm_port_)->get_value();

  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  //  if (fsm_state[0] == FLIGHT) {
  *casted_traj = GenerateFlightTraj(context);
  //  }
}

}  // namespace dairlib::examples::osc_run
