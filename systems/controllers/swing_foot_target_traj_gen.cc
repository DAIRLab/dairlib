#include "swing_foot_target_traj_gen.h"

#include <cmath>
#include <algorithm>
#include <fstream>
#include <string>
#include <iostream>

#include "multibody/multibody_utils.h"
#include "systems/controllers/control_utils.h"

#include "drake/math/saturate.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/bspline_trajectory.h"
#include "drake/common/trajectories/path_parameterized_trajectory.h"


using std::endl;
using std::string;

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using drake::Vector1d;


using drake::multibody::Frame;
using drake::multibody::JacobianWrtVariable;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::BsplineTrajectory;
using drake::trajectories::PathParameterizedTrajectory;
using drake::trajectories::Trajectory;


namespace dairlib {
namespace systems {

SwingFootTargetTrajGen::SwingFootTargetTrajGen(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    std::vector<int> left_right_support_fsm_states,
    std::vector<std::pair<const Vector3d, const Frame<double>&>>
    left_right_foot,
    double mid_foot_height, double desired_final_foot_height,
    double desired_final_vertical_foot_velocity, bool relative_to_com)
    : plant_(plant),
      context_(context),
      world_(plant_.world_frame()),
      left_right_support_fsm_states_(left_right_support_fsm_states),
      mid_foot_height_(mid_foot_height),
      desired_final_foot_height_(desired_final_foot_height),
      desired_final_vertical_foot_velocity_(
          desired_final_vertical_foot_velocity),
      relative_to_com_(relative_to_com) {

  this->set_name("swing_ft_traj");
  DRAKE_DEMAND(left_right_support_fsm_states_.size() == 2);
  DRAKE_DEMAND(left_right_foot.size() == 2);

  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(
          "x, u, t", OutputVector<double>(plant.num_positions(),
                                          plant.num_velocities(),
                                          plant.num_actuators()))
      .get_index();
  fsm_port_ = this->DeclareVectorInputPort("fsm", 1).get_index();
  liftoff_time_port_ =
      this->DeclareVectorInputPort("t_liftoff", 1).get_index();
  touchdown_time_port_ =
      this->DeclareVectorInputPort("t_touchdown", 1).get_index();
  footstep_target_port_ =
      this->DeclareVectorInputPort("desired footstep target",3).get_index();

  // Provide an instance to allocate the memory first (for the output)
  PathParameterizedTrajectory<double> pp(
      PiecewisePolynomial<double>(VectorXd::Zero(1)),
      PiecewisePolynomial<double>(VectorXd::Zero(1))
      );
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
  stance_foot_map_.insert(
      {left_right_support_fsm_states.at(0), left_right_foot.at(0)});
  stance_foot_map_.insert(
      {left_right_support_fsm_states.at(1), left_right_foot.at(1)});
  swing_foot_map_.insert(
      {left_right_support_fsm_states.at(0), left_right_foot.at(1)});
  swing_foot_map_.insert(
      {left_right_support_fsm_states.at(1), left_right_foot.at(0)});
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

    if (relative_to_com_) {
      swing_foot_pos_at_liftoff =
          multibody::ReExpressWorldVector3InBodyYawFrame(
          plant_, *context_, "pelvis",
          swing_foot_pos_at_liftoff -
              plant_.CalcCenterOfMassPositionInWorld(*context_));
    }
  }
  return EventStatus::Succeeded();
}

drake::trajectories::PathParameterizedTrajectory<double>
SwingFootTargetTrajGen::CreateSplineForSwingFoot(
    double start_time, double end_time, const Vector3d& init_pos,
    const Vector3d& final_pos) const {

  const std::vector<double> time_scaling_breaks = {
      start_time,
      0.5 * (start_time + end_time),
      end_time
  };

  std::vector<MatrixXd> time_scaling_knots(
      time_scaling_breaks.size(), drake::Vector1<double>::Zero());

  for (int i = 0; i < time_scaling_breaks.size(); i++) {
    time_scaling_knots.at(i)(0) =
        static_cast<double>(i) / (time_scaling_breaks.size() - 1);
  }

  auto time_scaling_trajectory =
      PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
          time_scaling_breaks, time_scaling_knots, Vector1d::Zero(),
          Vector1d::Zero());


  std::vector<double> nominal_heights = {0, 0.66, 1.0, 1.0, 0.75, 0};
  auto basis = drake::math::BsplineBasis<double>(
      nominal_heights.size(), nominal_heights.size());

  std::vector<MatrixXd> control_points =
      std::vector<MatrixXd>(nominal_heights.size(), Vector3d::Zero());

  for (int i = 0; i < control_points.size(); i++) {
    control_points.at(i).block(0, 0, 2, 1) =
        init_pos.head<2>() + i *
        (final_pos.head<2>() - init_pos.head<2>()) /
        (control_points.size() - 1);
    control_points.at(i)(2) =
        mid_foot_height_ * nominal_heights.at(i) +
        std::max(init_pos(2), final_pos(2));
  }

  control_points.front()(2) = init_pos(2);
  control_points.back()(2) = final_pos(2) + desired_final_foot_height_;

  auto swing_foot_path = BsplineTrajectory<double>(basis, control_points);
  auto swing_foot_spline = PathParameterizedTrajectory<double>(
      swing_foot_path,time_scaling_trajectory);

  return swing_foot_spline;
}

void SwingFootTargetTrajGen::CalcTrajs(
    const Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {

  // Get discrete states
  const auto swing_foot_pos_at_liftoff =
      context.get_discrete_state(liftoff_swing_foot_pos_idx_).get_value();
  // Read in finite state machine switch time
  double liftoff_time =
      this->EvalVectorInput(context, liftoff_time_port_)->get_value()(0);
  double touchdown_time =
      this->EvalVectorInput(context, touchdown_time_port_)->get_value()(0);

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
    // Ensure current_time < end_time_of_this_interval to avoid error in
    // creating trajectory.
    double start_time_of_this_interval = std::clamp(
        liftoff_time, -std::numeric_limits<double>::infinity(),
        touchdown_time - 0.001);

    // SOME DEBUGGING
//    const OutputVector<double>* robot_output =
//        (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
//    std::cout << "t, ts, te: " << robot_output->get_timestamp() << ", "
//                               << liftoff_time << ", "
//                               << touchdown_time << std::endl;

    // Swing foot position at touchdown
    Vector3d footstep_target =
        this->EvalVectorInput(context, footstep_target_port_)->get_value();

    // Assign traj
    auto pp_traj = dynamic_cast<PathParameterizedTrajectory<double>*>(traj);
    *pp_traj = CreateSplineForSwingFoot(
        start_time_of_this_interval, touchdown_time,
        swing_foot_pos_at_liftoff, footstep_target);
  } else {
    // Assign a constant traj
    auto pp_traj = dynamic_cast<PathParameterizedTrajectory<double>*>(traj);
    *pp_traj = PathParameterizedTrajectory<double>(
        PiecewisePolynomial<double>(Vector3d::Zero()),
        PiecewisePolynomial<double>(Vector1d::Ones())
    );
  }
}
}  // namespace systems
}  // namespace dairlib
