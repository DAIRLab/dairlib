#include "swing_foot_trajectory_generator.h"

#include <cmath>
#include <algorithm>

#include "multibody/multibody_utils.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/path_parameterized_trajectory.h"
#include "drake/systems/primitives/pass_through.h"

using dairlib::systems::controllers::alip_utils::PointOnFramed;

using std::endl;
using std::string;

using Eigen::MatrixXd;
using Eigen::Matrix3d;
using drake::Vector1d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::systems::Context;

using drake::systems::State;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;

using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::PathParameterizedTrajectory;
using drake::trajectories::Trajectory;


namespace dairlib {
namespace systems {
namespace controllers {

SwingFootTrajectoryGenerator::SwingFootTrajectoryGenerator(
    const drake::multibody::MultibodyPlant<double> &plant,
    drake::systems::Context<double> *context,
    const SwingFootTrajectoryGeneratorParams& params)
    : plant_(plant),
      plant_context_(context),
      world_(plant_.world_frame()),
      left_right_support_fsm_states_(params.left_right_support_fsm_states),
      retraction_dist_(params.retraction_dist),
      mid_foot_height_(params.mid_foot_height),
      desired_final_foot_height_(params.desired_final_foot_height),
      desired_final_vertical_foot_velocity_(
          params.desired_final_vertical_foot_velocity) {

  this->set_name("swing_ft_traj_interface_system");
  DRAKE_DEMAND(left_right_support_fsm_states_.size() == 2);
  DRAKE_DEMAND(params.left_right_foot.size() == 2);

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
      this->DeclareVectorInputPort("footstep_target_in_stance_frame", 3).get_index();

  PathParameterizedTrajectory<double> pp(
      PiecewisePolynomial<double>(VectorXd::Zero(3)),
      PiecewisePolynomial<double>(VectorXd::Zero(1))
  );
  drake::trajectories::Trajectory<double>& traj_instance = pp;
  swing_foot_traj_output_port_ = this->DeclareAbstractOutputPort(
          "swing_foot_xyz_in_stance_frame", traj_instance,
          &SwingFootTrajectoryGenerator::CalcSwingTraj
  ).get_index();


  DeclareForcedUnrestrictedUpdateEvent(
      &SwingFootTrajectoryGenerator::UnrestrictedUpdate);

  // The swing foot position in the beginning of the swing phase
  liftoff_swing_foot_pos_idx_ = this->DeclareDiscreteState(3);

  // The last state of FSM
  prev_fsm_state_idx_ = this->DeclareDiscreteState(
      -std::numeric_limits<double>::infinity() * VectorXd::Ones(1));

  // previous time
  prev_time_idx_ = this->DeclareDiscreteState(1);

  // previous spline solution
  prev_spline_idx_ = this->DeclareAbstractState(
      drake::Value<PathParameterizedTrajectory<double>>(pp));

  // Construct maps
  stance_foot_map_.insert(
      {params.left_right_support_fsm_states.at(0), params.left_right_foot.at(0)});
  stance_foot_map_.insert(
      {params.left_right_support_fsm_states.at(1), params.left_right_foot.at(1)});
  stance_foot_map_.insert(
      {params.post_left_post_right_fsm_states.at(0), params.left_right_foot.at(0)});
  stance_foot_map_.insert(
      {params.post_left_post_right_fsm_states.at(1), params.left_right_foot.at(1)});
  swing_foot_map_.insert(
      {params.left_right_support_fsm_states.at(0), params.left_right_foot.at(1)});
  swing_foot_map_.insert(
      {params.left_right_support_fsm_states.at(1), params.left_right_foot.at(0)});
}

EventStatus SwingFootTrajectoryGenerator::UnrestrictedUpdate(
    const Context<double> &context, State<double> *state) const {

  // Get FSM
  int fsm_state = EvalVectorInput(context, fsm_port_)->get_value()(0);

  // If we're in double support, no need to do anything
  if (not is_single_support(fsm_state)) {
    auto pp_traj =
        state->get_mutable_abstract_state<PathParameterizedTrajectory<double>>(
            prev_spline_idx_);
    pp_traj = PathParameterizedTrajectory<double>(
        PiecewisePolynomial<double>(Vector3d::Zero()),
        PiecewisePolynomial<double>(Vector1d::Ones()));
    return EventStatus::Succeeded();
  }

  // Read the rest of the ports
  const auto robot_output = dynamic_cast<const OutputVector<double>*>(
      EvalVectorInput(context, state_port_));
  double liftoff_time =
      EvalVectorInput(context, liftoff_time_port_)->get_value()(0);
  double touchdown_time =
      EvalVectorInput(context, touchdown_time_port_)->get_value()(0);
  Vector3d footstep_target_in_stance_frame =
      EvalVectorInput(context, footstep_target_port_)->get_value();

  // get relevant discrete states
  auto swing_foot_pos_at_liftoff =
      context.get_discrete_state(liftoff_swing_foot_pos_idx_).CopyToVector();
  auto prev_fsm_state =
      state->get_discrete_state(prev_fsm_state_idx_).CopyToVector();
  auto prev_time =
      state->get_discrete_state(prev_time_idx_).CopyToVector();
  auto prev_spline =
      state->get_abstract_state<PathParameterizedTrajectory<double>>(
          prev_spline_idx_);

  // when entering a new state which is in left_right_support_fsm_states
  if (fsm_state != prev_fsm_state(0) && is_single_support(fsm_state)) {

    // update prev fsm state
    prev_fsm_state(0) = fsm_state;

    // calculate swing foot pos at liftoff
    VectorXd q = robot_output->GetPositions();
    multibody::SetPositionsIfNew<double>(plant_, q, plant_context_);
    swing_foot_pos_at_liftoff = state->get_mutable_discrete_state(
        liftoff_swing_foot_pos_idx_).get_mutable_value();
    Vector3d stance_pos;

    auto swing_foot = swing_foot_map_.at(fsm_state);
    auto stance_foot = stance_foot_map_.at(fsm_state);
    plant_.CalcPointsPositions(*plant_context_, swing_foot.second, swing_foot.first,
                               world_, &swing_foot_pos_at_liftoff);
    plant_.CalcPointsPositions(*plant_context_, stance_foot.second, stance_foot.first,
                               world_, &stance_pos);
    swing_foot_pos_at_liftoff =
        multibody::ReExpressWorldVector3InBodyYawFrame(
            plant_, *plant_context_, "pelvis",
            swing_foot_pos_at_liftoff - stance_pos);

    prev_spline = PathParameterizedTrajectory<double>(
        PiecewisePolynomial<double>(swing_foot_pos_at_liftoff),
        PiecewisePolynomial<double>(VectorXd::Zero(1))
    );

    state->get_mutable_discrete_state(
        liftoff_swing_foot_pos_idx_).SetFromVector(swing_foot_pos_at_liftoff);
    prev_time(0) = liftoff_time;
  }

  prev_spline = foot_traj_solver_.AdaptSwingFootTraj(
      prev_spline, prev_time(0), liftoff_time, touchdown_time, mid_foot_height_,
      -desired_final_vertical_foot_velocity_, desired_final_foot_height_,
      swing_foot_pos_at_liftoff, footstep_target_in_stance_frame);

  // update prev_fsm_state, prev_time, and prev_spline
  state->get_mutable_discrete_state(prev_fsm_state_idx_).set_value(prev_fsm_state);
  state->get_mutable_discrete_state(prev_time_idx_).set_value(prev_time);
  state->get_mutable_abstract_state<PathParameterizedTrajectory<double>>(
      prev_spline_idx_) = prev_spline;
  return EventStatus::Succeeded();
}

bool SwingFootTrajectoryGenerator::is_single_support(int fsm_state) const {
  // Find fsm_state in left_right_support_fsm_states
  auto it = find(left_right_support_fsm_states_.begin(),
                 left_right_support_fsm_states_.end(), fsm_state);

  // swing phase if current state is in left_right_support_fsm_states_
  bool is_single_support_phase = it != left_right_support_fsm_states_.end();
  return is_single_support_phase;
}

void SwingFootTrajectoryGenerator::CalcSwingTraj(
    const Context<double> &context,
    drake::trajectories::Trajectory<double> *traj) const {
  auto pp = dynamic_cast<PathParameterizedTrajectory<double>*>(traj);
  *pp = context.get_abstract_state<PathParameterizedTrajectory<double>>(
      prev_spline_idx_);
}

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
