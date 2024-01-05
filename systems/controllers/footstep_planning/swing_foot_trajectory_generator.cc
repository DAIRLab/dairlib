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

  // Provide an instance to allocate the memory first (for the output)
  PiecewisePolynomial<double> pp(Vector1d::Zero());
  Trajectory<double> &traj_instance = pp;
  swing_foot_traj_output_port_ = this->DeclareAbstractOutputPort(
          "swing_foot_xyz_in_stance_frame", traj_instance,
          &SwingFootTrajectoryGenerator::CalcSwingTraj
  ).get_index();

  DeclarePerStepDiscreteUpdateEvent(
      &SwingFootTrajectoryGenerator::DiscreteVariableUpdate);

  // The swing foot position in the beginning of the swing phase
  liftoff_swing_foot_pos_idx_ = this->DeclareDiscreteState(3);

  // The last state of FSM
  prev_fsm_state_idx_ = this->DeclareDiscreteState(
      -std::numeric_limits<double>::infinity() * VectorXd::Ones(1));

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

EventStatus SwingFootTrajectoryGenerator::DiscreteVariableUpdate(
    const Context<double> &context,
    DiscreteValues<double> *discrete_state) const {
  // Read from ports
  int fsm_state = EvalVectorInput(context, fsm_port_)->get_value()(0);
  const auto robot_output = dynamic_cast<const OutputVector<double>*>(
      EvalVectorInput(context, state_port_));

  auto prev_fsm_state = discrete_state->get_mutable_value(prev_fsm_state_idx_);

  // when entering a new state which is in left_right_support_fsm_states
  if (fsm_state != prev_fsm_state(0) && is_single_support(fsm_state)) {
    prev_fsm_state(0) = fsm_state;

    VectorXd q = robot_output->GetPositions();
    multibody::SetPositionsIfNew<double>(plant_, q, plant_context_);
    auto swing_foot_pos_at_liftoff = discrete_state->get_mutable_vector(
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
  }
  return EventStatus::Succeeded();
}

drake::trajectories::PiecewisePolynomial<double>
SwingFootTrajectoryGenerator::CreateSplineForSwingFoot(
    double start_time, double end_time, const Vector3d &init_pos,
    const Vector3d &final_pos) const {

  Vector3d final = final_pos;
  final(2) += desired_final_foot_height_;

  const Vector3d time_breaks(start_time, (start_time + end_time) / 2, end_time);

  Eigen::Matrix3d control_points = Matrix3d::Zero();
  control_points.col(0) = init_pos;
  control_points.col(2) = final;

  enum StepType { kFlat, kUp, kDown };
  StepType step_type = kFlat;

  Vector3d swing_foot_disp = final - init_pos;
  if (abs(swing_foot_disp(2)) < 0.025){
    swing_foot_disp(2) = 0;
  } else if (swing_foot_disp(2) > 0) {
    step_type = kUp;
  } else {
    step_type = kDown;
  }

  // set midpoint similarly to https://arxiv.org/pdf/2206.14049.pdf
  // (Section V/E)
  double disp_yaw = atan2(swing_foot_disp(1), swing_foot_disp(0));
  Vector3d n_planar(cos(disp_yaw - M_PI_2), sin(disp_yaw - M_PI_2), 0);
  Vector3d n = n_planar.cross(swing_foot_disp).normalized();
  control_points.col(1) = 0.5 * (init_pos + final) + mid_foot_height_ * n;

  Vector3d final_vel = -desired_final_vertical_foot_velocity_ * (end_time - start_time) * Vector3d::UnitZ();

  // Swing foot retraction heuristic to overshoot the predicted momentum when
  // stepping up or down
  if (step_type != kFlat) {
    control_points.col(1) += step_type == kUp ?
                             0.4 * mid_foot_height_ * n : 0.2 * mid_foot_height_ * n;
    Vector3d retract_vel = -swing_foot_disp;
    retract_vel(2) = 0;
    retract_vel = 0.25 * (end_time - start_time) * retract_vel.normalized();
    final_vel += retract_vel;
    Vector3d retract_delta = retraction_dist_ * retract_vel.normalized();
    control_points.col(2) += retract_delta;
  }

  auto swing_foot_spline =
      PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
          time_breaks, control_points, Vector3d::Zero(), final_vel);

  return swing_foot_spline;
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

  int fsm_state = EvalVectorInput(context, fsm_port_)->get_value()(0);

  if (not is_single_support(fsm_state)) {
    auto pp_traj = dynamic_cast<PathParameterizedTrajectory<double> *>(traj);
    *pp_traj = PathParameterizedTrajectory<double>(
        PiecewisePolynomial<double>(Vector3d::Zero()),
        PiecewisePolynomial<double>(Vector1d::Ones()));
    return;
  }

  auto robot_output = dynamic_cast<const OutputVector<double>*>(
      EvalVectorInput(context, state_port_));

  multibody::SetPositionsAndVelocitiesIfNew<double>(
      plant_, robot_output->GetState(), plant_context_);

  auto swing_foot_pos_at_liftoff =
      context.get_discrete_state(liftoff_swing_foot_pos_idx_).CopyToVector();
  double liftoff_time =
      EvalVectorInput(context, liftoff_time_port_)->get_value()(0);
  double touchdown_time =
      EvalVectorInput(context, touchdown_time_port_)->get_value()(0);
  Vector3d footstep_target_in_stance_frame =
      EvalVectorInput(context, footstep_target_port_)->get_value();

  double start_time_of_this_interval = std::clamp(
      liftoff_time, -std::numeric_limits<double>::infinity(),
      touchdown_time - 0.001);

  // Assign traj
  auto pp_traj = dynamic_cast<PiecewisePolynomial<double> *>(traj);
  *pp_traj = CreateSplineForSwingFoot(
      start_time_of_this_interval,
      touchdown_time,
      swing_foot_pos_at_liftoff,
      footstep_target_in_stance_frame
  );
}

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
