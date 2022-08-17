#include "systems/controllers/swing_ft_traj_gen.h"

#include <math.h>

#include <algorithm>
#include <fstream>
#include <string>

#include <drake/math/saturate.h>
#include "systems/controllers/control_utils.h"

using std::string;

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::multibody::Frame;
using drake::multibody::JacobianWrtVariable;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;

using drake::trajectories::ExponentialPlusPiecewisePolynomial;
using drake::trajectories::PiecewisePolynomial;

namespace dairlib {
namespace systems {

SwingFootTrajGenerator::SwingFootTrajGenerator(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    std::vector<int> left_right_support_fsm_states,
    std::vector<double> left_right_support_durations,
    std::vector<std::pair<const Vector3d, const Frame<double>&>>
        left_right_foot,
    std::string floating_base_body_name, double double_support_duration,
    double mid_foot_height, double desired_final_foot_height,
    double desired_final_vertical_foot_velocity,
    double max_com_to_footstep_dist, double footstep_offset,
    double center_line_offset, bool wrt_com_in_local_frame)
    : plant_(plant),
      context_(context),
      world_(plant_.world_frame()),
      pelvis_(plant_.GetBodyByName(floating_base_body_name)),
      left_right_support_fsm_states_(left_right_support_fsm_states),
      mid_foot_height_(mid_foot_height),
      desired_final_foot_height_(desired_final_foot_height),
      desired_final_vertical_foot_velocity_(
          desired_final_vertical_foot_velocity),
      max_com_to_footstep_dist_(max_com_to_footstep_dist),
      footstep_offset_(footstep_offset),
      center_line_offset_(center_line_offset),
      double_support_duration_(double_support_duration),
      wrt_com_in_local_frame_(wrt_com_in_local_frame),
      left_right_foot_(left_right_foot) {
  this->set_name("swing_ft_traj");

  DRAKE_DEMAND(left_right_support_fsm_states_.size() == 2);
  DRAKE_DEMAND(left_right_support_durations.size() == 2);
  DRAKE_DEMAND(left_right_foot.size() == 2);

  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(
                        "x, u, t", OutputVector<double>(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
                    .get_index();
  fsm_port_ =
      this->DeclareVectorInputPort("fsm", BasicVector<double>(1)).get_index();
  liftoff_time_port_ =
      this->DeclareVectorInputPort("t_liftoff", BasicVector<double>(1))
          .get_index();

  PiecewisePolynomial<double> pp(VectorXd::Zero(0));
  com_port_ = this->DeclareAbstractInputPort(
                      "com_xyz",
                      drake::Value<drake::trajectories::Trajectory<double>>(pp))
                  .get_index();
  footstep_adjustment_port_ =
      this->DeclareVectorInputPort("foot_adjustment_xy", BasicVector<double>(2))
          .get_index();
  // Provide an instance to allocate the memory first (for the output)
  drake::trajectories::Trajectory<double>& traj_instance = pp;
  this->DeclareAbstractOutputPort("swing_foot_xyz", traj_instance,
                                  &SwingFootTrajGenerator::CalcTrajs);

  // State variables inside this controller block
  DeclarePerStepDiscreteUpdateEvent(
      &SwingFootTrajGenerator::DiscreteVariableUpdate);
  // The swing foot position in the beginning of the swing phase
  liftoff_swing_foot_pos_idx_ = this->DeclareDiscreteState(3);
  // The last state of FSM
  prev_fsm_state_idx_ = this->DeclareDiscreteState(
      -std::numeric_limits<double>::infinity() * VectorXd::Ones(1));

  // Construct maps
  duration_map_.insert({left_right_support_fsm_states.at(0),
                        left_right_support_durations.at(0)});
  duration_map_.insert({left_right_support_fsm_states.at(1),
                        left_right_support_durations.at(1)});
  stance_foot_map_.insert(
      {left_right_support_fsm_states.at(0), left_right_foot.at(0)});
  stance_foot_map_.insert(
      {left_right_support_fsm_states.at(1), left_right_foot.at(1)});
  swing_foot_map_.insert(
      {left_right_support_fsm_states.at(0), left_right_foot.at(1)});
  swing_foot_map_.insert(
      {left_right_support_fsm_states.at(1), left_right_foot.at(0)});
}

EventStatus SwingFootTrajGenerator::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  // Read in finite state machine
  VectorXd fsm_state = this->EvalVectorInput(context, fsm_port_)->get_value();

  auto prev_fsm_state = discrete_state->get_mutable_vector(prev_fsm_state_idx_)
                            .get_mutable_value();

  // Find fsm_state in left_right_support_fsm_states
  auto it = find(left_right_support_fsm_states_.begin(),
                 left_right_support_fsm_states_.end(), int(fsm_state(0)));
  // swing phase if current state is in left_right_support_fsm_states_
  bool is_single_support_phase = it != left_right_support_fsm_states_.end();

  // when entering a new state which is in left_right_support_fsm_states
  if ((fsm_state(0) != prev_fsm_state(0)) && is_single_support_phase) {
    prev_fsm_state(0) = fsm_state(0);

    auto swing_foot_pos_at_liftoff =
        discrete_state->get_mutable_vector(liftoff_swing_foot_pos_idx_)
            .get_mutable_value();

    // Read in current state
    const OutputVector<double>* robot_output =
        (OutputVector<double>*)this->EvalVectorInput(context, state_port_);

    VectorXd q = robot_output->GetPositions();
    multibody::SetPositionsIfNew<double>(plant_, q, context_);

    // Swing foot position (Forward Kinematics) at touchdown
    auto swing_foot = swing_foot_map_.at(int(fsm_state(0)));
    plant_.CalcPointsPositions(*context_, swing_foot.second, swing_foot.first,
                               world_, &swing_foot_pos_at_liftoff);

    if (wrt_com_in_local_frame_) {
      // Get approximated heading angle of pelvis and rotational matrix
      swing_foot_pos_at_liftoff =
          multibody::ReExpressWorldVector3InBodyYawFrame(
              plant_, *context_, "pelvis",
              swing_foot_pos_at_liftoff -
                  plant_.CalcCenterOfMassPositionInWorld(*context_));

      // Testing
      // Heuristic ratio for LIPM dyanmics (because the centroidal angular
      // momentum is not constant
      // TODO(yminchen): use either this or the one in lipm_traj_gen
      Vector3d toe_left_origin_position;
      plant_.CalcPointsPositions(*context_, left_right_foot_.at(0).second,
                                 Vector3d::Zero(), pelvis_.body_frame(),
                                 &toe_left_origin_position);
      Vector3d toe_right_origin_position;
      plant_.CalcPointsPositions(*context_, left_right_foot_.at(1).second,
                                 Vector3d::Zero(), pelvis_.body_frame(),
                                 &toe_right_origin_position);
      double dist = toe_left_origin_position(1) - toe_right_origin_position(1);
      // <foot_spread_lb_ meter: ratio 1
      // >foot_spread_ub_ meter: ratio ratio_lb_
      // Linear interpolate in between
      heuristic_ratio_ = 1;
      if (dist > foot_spread_ub_) {
        heuristic_ratio_ = ratio_lb_;
      } else if (dist > foot_spread_lb_) {
        heuristic_ratio_ = 1 + (ratio_lb_ - 1) /
                                   (foot_spread_ub_ - foot_spread_lb_) *
                                   (dist - foot_spread_lb_);
      }
    }
  }

  return EventStatus::Succeeded();
}

void SwingFootTrajGenerator::CalcFootStepAndStanceFootHeight(
    const Context<double>& context, const OutputVector<double>* robot_output,
    const double end_time_of_this_interval, Vector2d* x_fs,
    double* stance_foot_height) const {
  // Read in finite state machine
  auto fsm_output =
      (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
  VectorXd fsm_state = fsm_output->get_value();

  VectorXd q = robot_output->GetPositions();
  multibody::SetPositionsIfNew<double>(plant_, q, context_);

  // Stance foot position
  auto stance_foot = stance_foot_map_.at(int(fsm_state(0)));
  Vector3d stance_foot_pos;
  plant_.CalcPointsPositions(*context_, stance_foot.second, stance_foot.first,
                             world_, &stance_foot_pos);

  // Get approximated heading angle of pelvis and rotational matrix
  Vector3d pelvis_heading_vec =
      plant_.EvalBodyPoseInWorld(*context_, pelvis_).rotation().col(0);
  double approx_pelvis_yaw =
      atan2(pelvis_heading_vec(1), pelvis_heading_vec(0));
  Eigen::MatrixXd rot(2, 2);
  rot << cos(approx_pelvis_yaw), -sin(approx_pelvis_yaw),
      sin(approx_pelvis_yaw), cos(approx_pelvis_yaw);

  // Get CoM_pos
  Vector3d CoM_curr = plant_.CalcCenterOfMassPositionInWorld(*context_);

  // Get predicted CoM_pos and CoM_vel
  Vector3d CoM_pos_pred;
  Vector3d CoM_vel_pred;
  // CoM_pos_pred and CoM_vel_pred at the end of the step (predicted)
  const drake::AbstractValue* com_traj_output =
      this->EvalAbstractInput(context, com_port_);
  DRAKE_ASSERT(com_traj_output != nullptr);
  const auto& com_traj =
      com_traj_output->get_value<drake::trajectories::Trajectory<double>>();
  CoM_pos_pred = com_traj.value(end_time_of_this_interval);
  CoM_vel_pred = com_traj.MakeDerivative(1)->value(end_time_of_this_interval);

  // Filter the CoM_vel_pred
  if (robot_output->get_timestamp() != last_timestamp_) {
    double dt = robot_output->get_timestamp() - last_timestamp_;
    // last_timestamp_ = robot_output->get_timestamp();
    double alpha =
        2 * M_PI * dt * cutoff_freq_ / (2 * M_PI * dt * cutoff_freq_ + 1);
    filtered_com_vel_ = alpha * CoM_vel_pred + (1 - alpha) * filtered_com_vel_;
  }
  CoM_vel_pred = filtered_com_vel_;

  bool is_right_support = fsm_state(0) == left_right_support_fsm_states_[1];

  // Compute footstep location (use LIPM to derive neutral point)
  double omega = sqrt(9.81 / (CoM_pos_pred(2) - stance_foot_pos(2)));
  //  double T = duration_map_.at(int(fsm_state(0)));  // This was the old code for ROM MPC
  double T = duration_map_.at(int(fsm_state(0))) + double_support_duration_;
  if (wrt_com_in_local_frame_) {
    // v_i is CoM_vel_pred_local_start_of_next_stride
    Vector2d v_i = rot.transpose() * CoM_vel_pred.head<2>();
    // v_f is CoM_vel_pred_local_end_of_next_stride
    Vector2d v_f;
    double desired_y_vel_end_of_stride = 0.25;
    //    double desired_y_vel_end_of_stride = 0.25 * heuristic_ratio_;
    if (is_right_support) {
      v_f << v_i(0), -desired_y_vel_end_of_stride;
    } else {
      v_f << v_i(0), desired_y_vel_end_of_stride;
    }
    Vector2d com_wrt_foot =
        ((v_f * exp(omega * T) - v_i) / (exp(2 * omega * T) - 1) -
         (v_f * exp(-omega * T) - v_i) / (exp(-2 * omega * T) - 1)) /
        omega;
    *x_fs = (-com_wrt_foot);
  } else {
    Vector2d com_wrt_foot =
        CoM_vel_pred.head<2>() *
        ((exp(omega * T) - 1) / (exp(2 * omega * T) - 1) -
         (exp(-omega * T) - 1) / (exp(-2 * omega * T) - 1)) /
        omega;
    *x_fs = CoM_pos_pred.head<2>() - com_wrt_foot;
  }

  // Walking position control
  // Read in speed regularization term
  auto speed_control_output = (BasicVector<double>*)this->EvalVectorInput(
      context, footstep_adjustment_port_);
  *x_fs += speed_control_output->get_value();

  /// Imposing guards
  if (wrt_com_in_local_frame_) {
    // Shift footstep laterally away from sagittal plane
    // so that the foot placement position at steady state is right below the
    // hip joint
    if (is_right_support) {
      (*x_fs)(1) += footstep_offset_;
    } else {
      (*x_fs)(1) -= footstep_offset_;
    }

    // Impose half-plane guard
    Vector2d stance_foot_wrt_com_in_local_frame =
        rot.transpose() * (stance_foot_pos - CoM_curr).head<2>();
    if (is_right_support) {
      double line_pos =
          std::max(center_line_offset_, stance_foot_wrt_com_in_local_frame(1));
      (*x_fs)(1) = std::max(line_pos, (*x_fs)(1));
    } else {
      double line_pos =
          std::min(-center_line_offset_, stance_foot_wrt_com_in_local_frame(1));
      (*x_fs)(1) = std::min(line_pos, (*x_fs)(1));
    }

    // Cap by the step length
    double dist = x_fs->norm();
    if (dist > max_com_to_footstep_dist_) {
      (*x_fs) = (*x_fs) * max_com_to_footstep_dist_ / dist;
    }
  } else {
    // Shift footstep laterally away from sagittal plane
    // so that the foot placement position at steady state is right below the
    // hip joint
    Vector2d shift_foothold_dir;
    if (is_right_support) {
      shift_foothold_dir << cos(approx_pelvis_yaw + M_PI * 1 / 2),
          sin(approx_pelvis_yaw + M_PI * 1 / 2);
    } else {
      shift_foothold_dir << cos(approx_pelvis_yaw + M_PI * 3 / 2),
          sin(approx_pelvis_yaw + M_PI * 3 / 2);
    }
    *x_fs += shift_foothold_dir * footstep_offset_;

    *x_fs = ImposeHalfplaneGuard(*x_fs, !is_right_support, approx_pelvis_yaw,
                                 CoM_pos_pred.head<2>(), stance_foot_pos.head<2>(),
                                 center_line_offset_);

    // Cap by the step length
    *x_fs = ImposeStepLengthGuard(*x_fs, CoM_pos_pred.head<2>(),
                                  max_com_to_footstep_dist_);
  }

  /// Assignment for stance foot height
  if (wrt_com_in_local_frame_) {
    // stance foot height wrt COM
    *stance_foot_height = stance_foot_pos(2) - CoM_curr(2);
  } else {
    // absolute stance foot height
    *stance_foot_height = stance_foot_pos(2);
  }

  last_timestamp_ = robot_output->get_timestamp();
}

PiecewisePolynomial<double> SwingFootTrajGenerator::CreateSplineForSwingFoot(
    const double start_time_of_this_interval,
    const double end_time_of_this_interval, const double stance_duration,
    const Vector3d& init_swing_foot_pos, const Vector2d& x_fs,
    double stance_foot_height) const {
  // Two segment of cubic polynomial with velocity constraints
  std::vector<double> T_waypoint = {
      start_time_of_this_interval,
      (start_time_of_this_interval + end_time_of_this_interval) / 2,
      end_time_of_this_interval};

  std::vector<MatrixXd> Y(T_waypoint.size(), MatrixXd::Zero(3, 1));
  // x
  Y[0](0, 0) = init_swing_foot_pos(0);
  Y[1](0, 0) = (init_swing_foot_pos(0) + x_fs(0)) / 2;
  Y[2](0, 0) = x_fs(0);
  // y
  Y[0](1, 0) = init_swing_foot_pos(1);
  Y[1](1, 0) = (init_swing_foot_pos(1) + x_fs(1)) / 2;
  Y[2](1, 0) = x_fs(1);
  // z
  /// We added stance_foot_height because we want the desired trajectory to be
  /// relative to the stance foot in case the floating base state estimation
  /// drifts.
  Y[0](2, 0) = init_swing_foot_pos(2);
  Y[1](2, 0) = mid_foot_height_ + stance_foot_height;
  Y[2](2, 0) = desired_final_foot_height_ + stance_foot_height;

  MatrixXd Y_dot_start = MatrixXd::Zero(3, 1);
  MatrixXd Y_dot_end = MatrixXd::Zero(3, 1);
  Y_dot_end(2) = desired_final_vertical_foot_velocity_;

  // Use CubicWithContinuousSecondDerivatives instead of CubicHermite to make
  // the traj smooth at the mid point
  PiecewisePolynomial<double> swing_foot_spline =
      PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
          T_waypoint, Y, Y_dot_start, Y_dot_end);

  return swing_foot_spline;
}

void SwingFootTrajGenerator::CalcTrajs(
    const Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Cast traj for polymorphism
  auto* pp_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);

  // Get discrete states
  const auto swing_foot_pos_at_liftoff =
      context.get_discrete_state(liftoff_swing_foot_pos_idx_).get_value();
  // Read in finite state machine switch time
  VectorXd liftoff_time =
      this->EvalVectorInput(context, liftoff_time_port_)->get_value();

  // Read in finite state machine
  auto fsm_output =
      (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
  VectorXd fsm_state = fsm_output->get_value();

  // Find fsm_state in left_right_support_fsm_states
  auto it = find(left_right_support_fsm_states_.begin(),
                 left_right_support_fsm_states_.end(), int(fsm_state(0)));

  // swing phase if current state is in left_right_support_fsm_states_
  bool is_single_support_phase = it != left_right_support_fsm_states_.end();

  // Generate trajectory if it's currently in swing phase.
  // Otherwise, generate a constant trajectory
  if (is_single_support_phase) {
    // Read in current robot state
    const OutputVector<double>* robot_output =
        (OutputVector<double>*)this->EvalVectorInput(context, state_port_);

    // Get current time
    double timestamp = robot_output->get_timestamp();
    auto current_time = static_cast<double>(timestamp);

    // Get the start time and the end time of the current stance phase
    double start_time_of_this_interval = liftoff_time(0);
    double end_time_of_this_interval =
        liftoff_time(0) + duration_map_.at(int(fsm_state(0)));

    // Ensure current_time < end_time_of_this_interval to avoid error in
    // creating trajectory.
    // Ensure "current_time < end_time" to avoid error in
    // creating trajectory.
    start_time_of_this_interval = drake::math::saturate(
        start_time_of_this_interval, -std::numeric_limits<double>::infinity(),
        end_time_of_this_interval - 0.001);

    // Get Capture Point
    double stance_foot_height;
    Vector2d x_fs(0, 0);
    CalcFootStepAndStanceFootHeight(context, robot_output,
                                    end_time_of_this_interval, &x_fs,
                                    &stance_foot_height);

    // Swing foot position at touchdown
    Vector3d init_swing_foot_pos = swing_foot_pos_at_liftoff;

    // Assign traj
    *pp_traj = CreateSplineForSwingFoot(
        start_time_of_this_interval, end_time_of_this_interval,
        duration_map_.at(int(fsm_state(0))), init_swing_foot_pos, x_fs,
        stance_foot_height);

  } else {
    // Assign a constant traj
    *pp_traj = PiecewisePolynomial<double>(Vector3d::Zero());
  }
}
}  // namespace systems
}  // namespace dairlib
