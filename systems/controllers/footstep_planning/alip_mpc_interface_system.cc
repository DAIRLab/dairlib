#include "alip_mpc_interface_system.h"

#include <cmath>
#include <algorithm>
#include <string>
#include <iostream>

#include "common/polynomial_utils.h"
#include "multibody/multibody_utils.h"
#include "systems/controllers/minimum_snap_trajectory_generation.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/bspline_trajectory.h"
#include "drake/common/trajectories/path_parameterized_trajectory.h"
#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_vector_source.h"

using dairlib::systems::controllers::alip_utils::PointOnFramed;

using std::endl;
using std::string;

using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using drake::Vector1d;

using drake::multibody::Frame;
using drake::multibody::JacobianWrtVariable;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::ConstantVectorSource;
using drake::systems::PassThrough;
using drake::systems::Adder;
using drake::systems::State;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::trajectories::BsplineTrajectory;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::PathParameterizedTrajectory;
using drake::trajectories::ExponentialPlusPiecewisePolynomial;
using drake::trajectories::Trajectory;


namespace dairlib {
namespace systems {
namespace controllers {

SwingFootInterfaceSystem::SwingFootInterfaceSystem(
    const drake::multibody::MultibodyPlant<double> &plant,
    drake::systems::Context<double> *context,
    const SwingFootInterfaceSystemParams& params)
    : plant_(plant),
      plant_context_(context),
      world_(plant_.world_frame()),
      left_right_support_fsm_states_(params.left_right_support_fsm_states),
      retraction_dist_(params.retraction_dist),
      com_height_(params.com_height_),
      mid_foot_height_(params.mid_foot_height),
      desired_final_foot_height_(params.desired_final_foot_height),
      desired_final_vertical_foot_velocity_(
          params.desired_final_vertical_foot_velocity),
      relative_to_com_(params.relative_to_com) {

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
      this->DeclareVectorInputPort("desired footstep target", 3).get_index();
  ExponentialPlusPiecewisePolynomial<double> exp;
  com_traj_input_port_ = this->DeclareAbstractInputPort(
      "com_xyz", drake::Value<Trajectory<double>>(exp)).get_index();


  // Provide an instance to allocate the memory first (for the output)
  PathParameterizedTrajectory<double> pp(
      PiecewisePolynomial<double>(VectorXd::Zero(1)),
      PiecewisePolynomial<double>(VectorXd::Zero(1))
  );
  Trajectory<double> &traj_instance = pp;
  swing_foot_traj_output_port_ = this->DeclareAbstractOutputPort(
     "swing_foot_xyz", traj_instance, &SwingFootInterfaceSystem::CalcSwingTraj)
     .get_index();

  DeclarePerStepDiscreteUpdateEvent(
      &SwingFootInterfaceSystem::DiscreteVariableUpdate);

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

EventStatus SwingFootInterfaceSystem::DiscreteVariableUpdate(
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

drake::trajectories::PathParameterizedTrajectory<double>
SwingFootInterfaceSystem::CreateSplineForSwingFoot(
    double start_time, double end_time, const Vector3d &init_pos,
    const Vector3d &final_pos) const {

  Vector3d final = final_pos;
  final(2) += desired_final_foot_height_; // What is this value

  const Vector2d time_scaling_breaks(start_time, end_time);
  auto time_scaling_trajectory = PiecewisePolynomial<double>::FirstOrderHold(
      time_scaling_breaks, Vector2d(0, 1).transpose());

  std::vector<double> path_breaks = {0, 0.5, 1.0};
  Eigen::Matrix3d control_points = Matrix3d::Zero();
  control_points.col(0) = init_pos;
  control_points.col(2) = final;

  enum StepType { kFlat, kUp, kDown };
  StepType step_type = kFlat;

  // set midpoint similarly to https://arxiv.org/pdf/2206.14049.pdf
  // (Section V/E)
  Vector3d swing_foot_disp = final - init_pos;
  if (abs(swing_foot_disp(2)) < 0.025){
    swing_foot_disp(2) = 0;
  } else if (swing_foot_disp(2) > 0) {
    step_type = kUp;
  } else {
    step_type = kDown;
  }

  double disp_yaw = atan2(swing_foot_disp(1), swing_foot_disp(0));
  Vector3d n_planar(cos(disp_yaw - M_PI_2), sin(disp_yaw - M_PI_2), 0);
  Vector3d n = n_planar.cross(swing_foot_disp).normalized();
  control_points.col(1) = 0.5 * (init_pos + final) + mid_foot_height_ * n;

  Vector3d final_vel = -desired_final_vertical_foot_velocity_ * (end_time - start_time) * Vector3d::UnitZ();

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
  auto swing_foot_path = minsnap::MakeMinSnapTrajFromWaypoints(
      control_points, path_breaks, Vector3d::Zero(), final_vel);

  auto swing_foot_spline = PathParameterizedTrajectory<double>(
      swing_foot_path, time_scaling_trajectory);

  return swing_foot_spline;
}

bool SwingFootInterfaceSystem::is_single_support(int fsm_state) const {
  // Find fsm_state in left_right_support_fsm_states
  auto it = find(left_right_support_fsm_states_.begin(),
                 left_right_support_fsm_states_.end(), fsm_state);

  // swing phase if current state is in left_right_support_fsm_states_
  bool is_single_support_phase = it != left_right_support_fsm_states_.end();
  return is_single_support_phase;
}

void SwingFootInterfaceSystem::CalcSwingTraj(
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
    auto pp_traj = dynamic_cast<PathParameterizedTrajectory<double> *>(traj);
    *pp_traj = CreateSplineForSwingFoot(
        start_time_of_this_interval,
        touchdown_time,
        swing_foot_pos_at_liftoff,
        footstep_target_in_stance_frame
    );
}

ComTrajInterfaceSystem::ComTrajInterfaceSystem(
    const drake::multibody::MultibodyPlant<double> &plant,
    drake::systems::Context<double> *context,
    double desired_com_height,
    double t_ds,
    const std::vector<int> &unordered_fsm_states,
    const std::vector<PointOnFramed> contact_point_in_each_state) :
    plant_(plant),
    context_(context),
    desired_com_height_(desired_com_height),
    tds_(t_ds),
    fsm_states_(unordered_fsm_states),
    contact_point_in_each_state_(contact_point_in_each_state) {

  DRAKE_DEMAND(unordered_fsm_states.size() == contact_point_in_each_state.size());

  this->set_name("ALIP_com_traj");

  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(
          "x, u, t", OutputVector<double>(plant.num_positions(),
                                          plant.num_velocities(),
                                          plant.num_actuators()))
      .get_index();
  fsm_port_ = DeclareVectorInputPort("fsm", 1).get_index();
  next_touchdown_time_port_ = DeclareVectorInputPort("t1", 1).get_index();
  prev_liftoff_time_port_ = DeclareVectorInputPort("t0", 1).get_index();
  slope_params_port_ = DeclareVectorInputPort("slope_params", 2).get_index();

  // Provide an instance to allocate the memory first (for the output)
  ExponentialPlusPiecewisePolynomial<double> exp;
  drake::trajectories::Trajectory<double>& traj_inst = exp;
  output_port_com_ = DeclareAbstractOutputPort(
      "alip_com_prediction", traj_inst,
      &ComTrajInterfaceSystem::CalcComTrajFromCurrent).get_index();

  prev_slope_idx_ = DeclareDiscreteState(2);
  m_ = plant_.CalcTotalMass(*context);

  DeclarePerStepUnrestrictedUpdateEvent(
      &ComTrajInterfaceSystem::UnrestrictedUpdate);
}

EventStatus ComTrajInterfaceSystem::UnrestrictedUpdate(
    const Context<double> &context, State<double> *state) const {
  auto fsm_state = static_cast<int>(
      this->EvalVectorInput(context, fsm_port_)->value()(0));
  bool is_ss = fsm_state <= 1;
  if (!is_ss) {
    return EventStatus::Succeeded();
  }
  state->get_mutable_discrete_state(prev_slope_idx_).get_mutable_value() =
      EvalVectorInput(context, slope_params_port_)->get_value();
  return EventStatus::Succeeded();
}

ExponentialPlusPiecewisePolynomial<double>
ComTrajInterfaceSystem::ConstructAlipComTraj(
    const Vector3d& stance_foot_pos, const Vector4d& x_alip,
    const Vector2d& kx_ky, double start_time, double end_time) const {

  // create a 3D one-segment polynomial for ExponentialPlusPiecewisePolynomial
  Vector2d T_waypoint_com {start_time, end_time};
  MatrixXd Y = MatrixXd::Zero(3, 2);
  Y.col(0).head(2) = stance_foot_pos.head(2);
  Y.col(1).head(2) = stance_foot_pos.head(2);

  // We add stance_foot_pos(2) to desired COM height to account for state
  // drifting
  double height = stance_foot_pos(2) + desired_com_height_;
  Y(2, 0) = height;
  Y(2, 1) = height;

  PiecewisePolynomial<double> pp_part =
      PiecewisePolynomial<double>::FirstOrderHold(T_waypoint_com, Y);

  MatrixXd K = MatrixXd::Zero(3,4);
  K.topLeftCorner<2,2>() = Eigen::Matrix2d::Identity();
  K.bottomLeftCorner<1, 2>() = kx_ky.transpose();
  auto A = CalcA(desired_com_height_);
  return {K, A, x_alip, pp_part};
}

void ComTrajInterfaceSystem::CalcAlipState(
    const Eigen::VectorXd &x, int mode_index,
    const drake::EigenPtr<Eigen::Vector3d>& CoM_p,
    const drake::EigenPtr<Eigen::Vector3d>& L_p,
    const drake::EigenPtr<Eigen::Vector3d>& stance_pos_p) const {
  controllers::alip_utils::CalcAlipState(
      plant_, context_, x,
      {contact_point_in_each_state_[mode_index]},
      {1.0}, CoM_p, L_p, stance_pos_p);
}

void ComTrajInterfaceSystem::CalcComTrajFromCurrent(
    const drake::systems::Context<double> &context,
    drake::trajectories::Trajectory<double> *traj) const {

  // Read in current state
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);

  // Read in finite state machine
  auto fsm_state = static_cast<int>(
      this->EvalVectorInput(context, fsm_port_)->value()(0));

  // Read in finite state machine switch time
  double end_time =
      EvalVectorInput(context, next_touchdown_time_port_)->get_value()(0);
  double start_time =
      EvalVectorInput(context, prev_liftoff_time_port_)->get_value()(0);
  double timestamp = robot_output->get_timestamp();


  bool is_ss = fsm_state <= 1;
  double start_time_offset = is_ss ? start_time : start_time - 0.3;
  double end_time_offset = is_ss ? end_time + tds_ : start_time + tds_;
  double t = std::clamp<double>(timestamp,
                                -std::numeric_limits<double>::infinity(),
                                end_time_offset - 0.001);

  Vector3d CoM, L, stance_foot_pos;
  CalcAlipState(robot_output->GetState(), GetModeIdx(fsm_state),
                &CoM, &L, &stance_foot_pos);

  Vector4d x_alip = Vector4d::Zero();
  x_alip.head(2) = multibody::ReExpressWorldVector2InBodyYawFrame(
      plant_, *context_, "pelvis", CoM.head(2) - stance_foot_pos.head(2));
  x_alip.tail(2) = multibody::ReExpressWorldVector2InBodyYawFrame(
      plant_, *context_, "pelvis", L.head(2));
  stance_foot_pos = multibody::ReExpressWorldVector3InBodyYawFrame(
      plant_, *context_, "pelvis", stance_foot_pos);

  // Assign traj
  auto exp_pp_traj =
      dynamic_cast<ExponentialPlusPiecewisePolynomial<double>*>(traj);

  // read in slope parameters
  Vector2d kx_ky = is_ss ?
      EvalVectorInput(context, slope_params_port_)->get_value() :
      context.get_discrete_state(prev_slope_idx_).get_value();

  *exp_pp_traj = ConstructAlipComTraj(
      stance_foot_pos, x_alip, kx_ky, t, end_time_offset);
}


AlipMPCInterfaceSystem::AlipMPCInterfaceSystem(
    const drake::multibody::MultibodyPlant<double> &plant,
    drake::systems::Context<double> *context,
    ComTrajInterfaceParams com_params,
    SwingFootInterfaceSystemParams swing_params) {
  set_name("alip_mpc_interface_system");
  drake::systems::DiagramBuilder<double> builder;
  auto swing_interface =
      builder.AddSystem<SwingFootInterfaceSystem>(plant, context, swing_params);
  auto com_interface =
      builder.AddSystem<ComTrajInterfaceSystem>(plant, context, com_params);

  builder.Connect(com_interface->get_output_port_com(),
                  swing_interface->get_input_port_com_traj());

  // Export ports
  state_port_ = ExportSharedInput(
      builder,
      swing_interface->get_input_port_state(),
      com_interface->get_input_port_state(),
      "x, u, t");

  fsm_port_ = ExportSharedInput(
      builder,
      swing_interface->get_input_port_fsm(),
      com_interface->get_input_port_fsm(),
      "fsm");

  next_touchdown_time_port_ = ExportSharedInput(
      builder,
      swing_interface->get_input_port_next_fsm_switch_time(),
      com_interface->get_input_port_next_fsm_switch_time(),
      "tnext");

  prev_liftoff_time_port_ = ExportSharedInput(
      builder,
      swing_interface->get_input_port_fsm_switch_time(),
      com_interface->get_input_port_fsm_switch_time(),
      "tprev");

  footstep_target_port_ =
      builder.ExportInput(swing_interface->get_input_port_footstep_target());
  slope_parameter_input_port_ =
      builder.ExportInput(com_interface->get_input_port_slope_params());
  com_traj_port_ = builder.ExportOutput(com_interface->get_output_port_com());
  swing_traj_port_ =
      builder.ExportOutput(swing_interface->get_output_port_swing_foot_traj());

  builder.BuildInto(this);

}

drake::systems::InputPortIndex AlipMPCInterfaceSystem::ExportSharedInput(
    drake::systems::DiagramBuilder<double>& builder,
    const drake::systems::InputPort<double> &p1,
    const drake::systems::InputPort<double> &p2, std::string name) {

  const drake::systems::InputPortIndex idx = builder.ExportInput(p1, name);
  builder.ConnectInput(name, p2);
  return idx;
}

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
