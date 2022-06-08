//
// Created by brian on 6/1/22.
//

#include "swing_foot_target_traj_gen.h"

#include <cmath>
#include <algorithm>
#include <fstream>
#include <string>

#include "drake/math/saturate.h"
#include "drake/math/roll_pitch_yaw.h"
#include "systems/controllers/control_utils.h"
#include "multibody/multibody_utils.h"

using std::cout;
using std::endl;
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

SwingFootTargetTrajGen::SwingFootTargetTrajGen(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    std::vector<int> left_right_support_fsm_states,
    std::vector<double> left_right_support_durations,
    std::vector<std::pair<const Vector3d, const Frame<double>&>>
    left_right_foot,
    double mid_foot_height, double desired_final_foot_height,
    double desired_final_vertical_foot_velocity, bool learn_params)
    : plant_(plant),
      context_(context),
      world_(plant_.world_frame()),
      left_right_support_fsm_states_(left_right_support_fsm_states),
      mid_foot_height_(mid_foot_height),
      desired_final_foot_height_(desired_final_foot_height),
      desired_final_vertical_foot_velocity_(
          desired_final_vertical_foot_velocity),
      learn_params_(learn_params) {
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

  footstep_target_port_ =
      this->DeclareVectorInputPort("desired footstep target",
                                   BasicVector<double>(3))
          .get_index();

  // Provide an instance to allocate the memory first (for the output)
  PiecewisePolynomial<double> pp(VectorXd::Zero(0));
  drake::trajectories::Trajectory<double>& traj_instance = pp;
  this->DeclareAbstractOutputPort("swing_foot_xyz", traj_instance,
                                  &SwingFootTargetTrajGen::CalcTrajs);

  // State variables inside this controller block
  DeclarePerStepDiscreteUpdateEvent(
      &SwingFootTargetTrajGen::DiscreteVariableUpdate);

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

  // Make default spline parameters
  default_spline_params_.n_knot = 5;
  default_spline_params_.knot_xyz = std::vector<std::vector<double>>(
      default_spline_params_.n_knot, std::vector<double>(3));
  for (int i = 0; i < default_spline_params_.n_knot; i++) {
    double t =  (double) i / (default_spline_params_.n_knot - 1);
    std::vector<double>& xyz = default_spline_params_.knot_xyz.at(i);
    xyz[0] = 0.5 * (sin(M_PI * (t - 0.5)) + 1);
    xyz[1] = 0.5 * (sin(M_PI * (t - 0.5)) + 1);
    xyz[2] = (mid_foot_height / 0.58) * cos(M_PI * (t - 0.5))*t;
  }
  for (int i = 0; i < 3; i++) {
    default_spline_params_.swing_foot_vel_initial[i] = 0;
    default_spline_params_.swing_foot_vel_final[i] = 0;
  }
  default_spline_params_.swing_foot_vel_final[2] =
      desired_final_vertical_foot_velocity;

  if (learn_params_) {
    swing_params_port_ =
        this->DeclareAbstractInputPort(
            "swing foot parameters",
            drake::Value<lcmt_swing_foot_spline_params>(default_spline_params_)).get_index();
  }
}

EventStatus SwingFootTargetTrajGen::DiscreteVariableUpdate(
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

PiecewisePolynomial<double> SwingFootTargetTrajGen::CreateSplineForSwingFoot(
    const double start_time_of_this_interval,
    const double end_time_of_this_interval, const double stance_duration,
    const Vector3d& init_swing_foot_pos, const Vector3d& final_swing_foot_pos,
    lcmt_swing_foot_spline_params& params)
    const {

  // Two segment of cubic polynomial with velocity constraints
  std::vector<double> T_waypoint(params.n_knot);
  std::vector<MatrixXd> Y(params.n_knot, MatrixXd::Zero(3, 1));

  for (int i = 0; i < params.n_knot; i++) {
    T_waypoint[i] = start_time_of_this_interval +
        ((double) i) / (params.n_knot - 1)*
            (end_time_of_this_interval - start_time_of_this_interval);
    Vector3d knot = Eigen::Map<Eigen::Matrix<double, 3, 1>>(
        params.knot_xyz[i].data());
    knot.head<2>() = knot.head<2>().cwiseProduct(
        final_swing_foot_pos.head<2>() - init_swing_foot_pos.head<2>())
            + init_swing_foot_pos.head<2>();
    knot(2) += init_swing_foot_pos(2);
    Y[i] = knot;
  }

  Vector3d Y_dot_start = Eigen::Map<Vector3d>(params.swing_foot_vel_initial);
  Vector3d Y_dot_end = Eigen::Map<Vector3d>(params.swing_foot_vel_final);

  // the traj smooth at the mid point
  PiecewisePolynomial<double> swing_foot_spline =
      PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
          T_waypoint, Y, Y_dot_start, Y_dot_end);

  return swing_foot_spline;
}

void SwingFootTargetTrajGen::CalcTrajs(
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
    lcmt_swing_foot_spline_params spline_params = learn_params_ ?
          this->EvalAbstractInput(context, swing_params_port_)->
              get_value<lcmt_swing_foot_spline_params>() : default_spline_params_;

    // Get the start time and the end time of the current stance phase
    double start_time_of_this_interval = liftoff_time(0);
    double end_time_of_this_interval =
        liftoff_time(0) + duration_map_.at(fsm_state);

    // Ensure current_time < end_time_of_this_interval to avoid error in
    // creating trajectory.
    // Ensure "current_time < end_time" to avoid error in
    // creating trajectory.
    start_time_of_this_interval = drake::math::saturate(
        start_time_of_this_interval, -std::numeric_limits<double>::infinity(),
        end_time_of_this_interval - 0.001);

    // Swing foot position at touchdown
    Vector3d footstep_target =
        this->EvalVectorInput(context, footstep_target_port_)->get_value();

    // Assign traj
    *pp_traj = CreateSplineForSwingFoot(
        start_time_of_this_interval, end_time_of_this_interval,
        duration_map_.at(fsm_state),swing_foot_pos_at_liftoff,
        footstep_target, spline_params);
  } else {
    // Assign a constant traj
    *pp_traj = PiecewisePolynomial<double>(Vector3d::Zero());
  }
}
}  // namespace systems
}  // namespace dairlib
