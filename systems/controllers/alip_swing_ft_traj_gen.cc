#include "systems/controllers/alip_swing_ft_traj_gen.h"

#include <cmath>
#include <algorithm>
#include <fstream>
#include <string>

#include "drake/math/roll_pitch_yaw.h"
#include "systems/controllers/control_utils.h"
#include "multibody/multibody_utils.h"

using std::string;

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using drake::multibody::Frame;
using drake::multibody::JacobianWrtVariable;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::math::RollPitchYaw;
using drake::trajectories::ExponentialPlusPiecewisePolynomial;
using drake::trajectories::PiecewisePolynomial;

namespace dairlib {
namespace systems {

AlipSwingFootTrajGenerator::AlipSwingFootTrajGenerator(
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
    double center_line_offset)
    : plant_(plant),
      context_(context),
      world_(plant_.world_frame()),
      left_right_support_fsm_states_(left_right_support_fsm_states),
      mid_foot_height_(mid_foot_height),
      desired_final_foot_height_(desired_final_foot_height),
      desired_final_vertical_foot_velocity_(
          desired_final_vertical_foot_velocity),
      max_com_to_footstep_dist_(max_com_to_footstep_dist),
      footstep_offset_(footstep_offset),
      center_line_offset_(center_line_offset),
      double_support_duration_(double_support_duration) {
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
  alip_state_port_ = this->DeclareAbstractInputPort(
          "alip x, y, Lx, Ly",
          drake::Value<drake::trajectories::Trajectory<double>>(pp))
      .get_index();
  vdes_port_ =
      this->DeclareVectorInputPort("desired horizontal walking speed",
                                   BasicVector<double>(2))
          .get_index();
  // Provide an instance to allocate the memory first (for the output)
  drake::trajectories::Trajectory<double>& traj_instance = pp;
  this->DeclareAbstractOutputPort("swing_foot_xyz", traj_instance,
                                  &AlipSwingFootTrajGenerator::CalcTrajs);

  // State variables inside this controller block
  DeclarePerStepDiscreteUpdateEvent(
      &AlipSwingFootTrajGenerator::DiscreteVariableUpdate);
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

  m_ = plant_.CalcTotalMass(*context_);
}

EventStatus AlipSwingFootTrajGenerator::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  // Read from ports
  int fsm_state = this->EvalVectorInput(context, fsm_port_)->get_value()(0);
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);

  auto prev_fsm_state = discrete_state->get_mutable_vector(prev_fsm_state_idx_)
      .get_mutable_value();

  // Find fsm_state in left_right_support_fsm_states
  bool is_single_support_phase = (find(left_right_support_fsm_states_.begin(),
                                      left_right_support_fsm_states_.end(),
                                      fsm_state)
                              != left_right_support_fsm_states_.end());

  // when entering a new state which is in left_right_support_fsm_states
  if (fsm_state != prev_fsm_state(0) && is_single_support_phase) {
    prev_fsm_state(0) = fsm_state;

    VectorXd q = robot_output->GetPositions();
    multibody::SetPositionsIfNew<double>(plant_, q, context_);
    auto swing_foot_pos_at_liftoff = discrete_state->get_mutable_vector(
        liftoff_swing_foot_pos_idx_).get_mutable_value();

    auto swing_foot = swing_foot_map_.at(fsm_state);
    plant_.CalcPointsPositions(*context_, swing_foot.second, swing_foot.first,
                               world_, &swing_foot_pos_at_liftoff);

    swing_foot_pos_at_liftoff = multibody::ReExpressWorldVector3InBodyYawFrame(
        plant_, *context_, "pelvis",
        swing_foot_pos_at_liftoff -
        plant_.CalcCenterOfMassPositionInWorld(*context_));
  }

  return EventStatus::Succeeded();
}

void AlipSwingFootTrajGenerator::CalcFootStepAndStanceFootHeight(
    const Context<double>& context, const OutputVector<double>* robot_output,
    const double end_time_of_this_interval, Vector2d* x_fs,
    double* stance_foot_height) const {

  int fsm_state =
      this->EvalVectorInput(context, fsm_port_)->get_value()(0);

  VectorXd q = robot_output->GetPositions();
  multibody::SetPositionsIfNew<double>(plant_, q, context_);

  // Kinematic Calculations (Full robot)
  auto stance_foot = stance_foot_map_.at(fsm_state);
  Vector3d stance_foot_pos;
  plant_.CalcPointsPositions(*context_, stance_foot.second, stance_foot.first,
                             world_, &stance_foot_pos);

  Vector3d CoM_curr = plant_.CalcCenterOfMassPositionInWorld(*context_);
  double H = CoM_curr(2) - stance_foot_pos(2);
  double omega = sqrt(9.81 / H);
  double T = duration_map_.at(fsm_state) + double_support_duration_;
  double L_x_n = m_ * H * footstep_offset_ *
      (omega * sinh(omega * T) / (1 + cosh(omega * T)));

  Vector2d vdes_xy = this->EvalVectorInput(context, vdes_port_)->get_value();
  double L_y_des = vdes_xy(0) * H * m_;
  double L_x_offset = -vdes_xy(1) * H * m_;

  // com and angular momentum prediction
  const drake::AbstractValue* alip_traj_p =
      this->EvalAbstractInput(context, alip_state_port_);
  DRAKE_DEMAND(alip_traj_p != nullptr);
  const auto& alip_traj =
      alip_traj_p->get_value<drake::trajectories::Trajectory<double>>();
  Vector4d alip_pred = alip_traj.value(end_time_of_this_interval);

  bool is_right_support = (fsm_state == left_right_support_fsm_states_[1]);

  Vector2d L_i = multibody::ReExpressWorldVector2InBodyYawFrame<double>(
      plant_, *context_,"pelvis", alip_pred.tail<2>());
  Vector2d L_f = is_right_support ?
      Vector2d(L_x_offset + L_x_n, L_y_des) :
      Vector2d(L_x_offset - L_x_n, L_y_des);
  double p_x_ft_to_com = ( L_f(1) - cosh(omega*T) * L_i(1) ) /
                         (m_ * H * omega * sinh(omega*T));
  double p_y_ft_to_com = -( L_f(0) - cosh(omega*T) * L_i(0) ) /
                         (m_ * H * omega * sinh(omega*T));
  *x_fs = Vector2d(-p_x_ft_to_com, -p_y_ft_to_com);

  /// Imposing guards
  Vector2d stance_foot_wrt_com_in_local_frame =
      multibody::ReExpressWorldVector2InBodyYawFrame<double>(
          plant_, *context_,"pelvis",  (stance_foot_pos - CoM_curr).head<2>());
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

  *stance_foot_height = -H;
}

PiecewisePolynomial<double> AlipSwingFootTrajGenerator::CreateSplineForSwingFoot(
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

void AlipSwingFootTrajGenerator::CalcTrajs(
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
  int fsm_state =
      this->EvalVectorInput(context, fsm_port_)->get_value()(0);

  // Find fsm_state in left_right_support_fsm_states
  auto it = find(left_right_support_fsm_states_.begin(),
                 left_right_support_fsm_states_.end(), fsm_state);

  // swing phase if current state is in left_right_support_fsm_states_
  bool is_single_support_phase = it != left_right_support_fsm_states_.end();

  // Generate trajectory if it's currently in swing phase.
  // Otherwise, generate a constant trajectory
  if (is_single_support_phase) {
    // Read in current robot state
    const OutputVector<double>* robot_output =
        (OutputVector<double>*)this->EvalVectorInput(context, state_port_);

    // Get the start time and the end time of the current stance phase
    double start_time_of_this_interval = liftoff_time(0);
    double end_time_of_this_interval =
        liftoff_time(0) + duration_map_.at(fsm_state);

    // Ensure current_time < end_time_of_this_interval to avoid error in
    // creating trajectory.
    // Ensure "current_time < end_time" to avoid error in
    // creating trajectory.
    start_time_of_this_interval = std::clamp(
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
        duration_map_.at(fsm_state), init_swing_foot_pos, x_fs,
        stance_foot_height);
  } else {
    // Assign a constant traj
    *pp_traj = PiecewisePolynomial<double>(Vector3d::Zero());
  }
}
}  // namespace systems
}  // namespace dairlib
