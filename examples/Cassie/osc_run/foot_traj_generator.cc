#include "foot_traj_generator.h"

#include <iostream>

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
                                     const string& hip_name,
                                     const int stance_state)
    : plant_(plant),
      context_(context),
      world_(plant.world_frame()),
      foot_frame_(plant.GetFrameByName(foot_name)),
      hip_frame_(plant.GetFrameByName(hip_name)),
      stance_state_(stance_state) {
  PiecewisePolynomial<double> empty_pp_traj(VectorXd(0));
  Trajectory<double>& traj_inst = empty_pp_traj;
  target_vel_filter_ = std::make_unique<FirstOrderLowPassFilter>(1.0, 2);

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
  }

  return EventStatus::Succeeded();
}

PiecewisePolynomial<double> FootTrajGenerator::GenerateFlightTraj(
    const drake::systems::Context<double>& context) const {
  const auto robot_output =
      this->template EvalVectorInput<OutputVector>(context, state_port_);
  const auto desired_pelvis_vel_xy =
      this->EvalVectorInput(context, target_vel_port_)->get_value();
  const auto& mode_lengths =
      this->EvalVectorInput(context, contact_scheduler_port_)->get_value();

  double pelvis_t0 = mode_lengths[0];
  double pelvis_tf = mode_lengths[1];
  double left_t0 = mode_lengths[2];
  double left_tf = mode_lengths[3];
  double right_t0 = mode_lengths[4];
  double right_tf = mode_lengths[5];

  // Offset between 0 and 2
  double lateral_radio_tuning = 1.0;
  double sagital_radio_tuning = 1.0;
  if (this->get_input_port(radio_port_).HasValue(context)) {
    const auto& radio_out =
        this->EvalInputValue<dairlib::lcmt_radio_out>(context, radio_port_);
    lateral_radio_tuning += radio_out->channel[4];
    sagital_radio_tuning += radio_out->channel[5];
  }

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
  target_vel_filter_->Update(desired_pelvis_vel_xy);
  desired_pelvis_vel << target_vel_filter_->Value(), 0;

  auto foot_pos = context.get_discrete_state(initial_foot_pos_idx_).get_value();
  Vector3d pelvis_pos;
  plant_.CalcPointsPositions(*context_, hip_frame_, Vector3d::Zero(), world_,
                             &pelvis_pos);

  VectorXd pelvis_vel = v.segment(3, 3);
  VectorXd pelvis_vel_err = rot.transpose() * pelvis_vel - desired_pelvis_vel;

  std::vector<double> T_waypoints;

  double x_mid_point_ratio = 0.8;
  double t_mid_point_ratio = 0.6;

  if (is_left_foot_) {
    T_waypoints = {left_t0, left_t0 + t_mid_point_ratio * (left_tf - left_t0),
                   left_tf};
  } else {
    T_waypoints = {right_t0,
                   right_t0 + t_mid_point_ratio * (right_tf - right_t0),
                   right_tf};
  }

  // TODO(yangwill): Footsteps are planned with constant stance duration T_s of
  // 0.3s
  VectorXd foot_end_pos_des =
      0.5 * (0.3) * rot.transpose() * pelvis_vel + Kd_ * (pelvis_vel_err);

  if (is_left_foot_) {
    foot_end_pos_des(1) += lateral_radio_tuning * lateral_offset_;
  } else {
    foot_end_pos_des(1) -= lateral_radio_tuning * lateral_offset_;
  }
  foot_end_pos_des(0) += sagital_radio_tuning * sagital_offset_;
  foot_end_pos_des(2) = -rest_length_ - rest_length_offset_;

  auto hip_pos = context.get_discrete_state(initial_hip_pos_idx_).get_value();
  std::vector<MatrixXd> Y(T_waypoints.size(), VectorXd::Zero(3));
  VectorXd start_pos = foot_pos - hip_pos;
  Y[0] = start_pos;
  if (start_pos(2) == 0) {
    Y[0](2) = -rest_length_;
  }
  Y[1] = start_pos + x_mid_point_ratio * (foot_end_pos_des - start_pos);
  double foot_height_scale = std::clamp(pelvis_vel[0] / 2.5, 1.0, 1.4);
  Y[1](2) += foot_height_scale * mid_foot_height_;
  Y[2] = foot_end_pos_des;

  // prevent foot collisions
  if (is_left_foot_) {
    Y[1](1) = std::clamp(Y[1](1), lateral_offset_, 0.2);
    Y[2](1) = std::clamp(Y[2](1), lateral_offset_, 0.2);

  } else {
    Y[1](1) = std::clamp(Y[1](1), -0.2, -lateral_offset_);
    Y[2](1) = std::clamp(Y[2](1), -0.2, -lateral_offset_);
  }
  Y[1](0) = std::clamp(Y[1](0), -0.4, 0.4);
  Y[2](0) = std::clamp(Y[2](0), -0.4, 0.4);

  PiecewisePolynomial<double> swing_foot_spline =
      PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
          T_waypoints, Y, false);
  return swing_foot_spline;
}

void FootTrajGenerator::CalcTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  //  // Read in finite state machine
  const auto fsm_state =
      this->EvalVectorInput(context, fsm_port_)->get_value()[0];

  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  if (is_left_foot_) {
    if (fsm_state != kLeftStance) {
      *casted_traj = GenerateFlightTraj(context);
    }

  } else {
    if (fsm_state != kRightStance) {
      *casted_traj = GenerateFlightTraj(context);
    }
  }
}

}  // namespace dairlib::examples::osc_run
