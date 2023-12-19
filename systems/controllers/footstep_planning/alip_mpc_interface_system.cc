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
    SwingFootTrajectoryGeneratorParams swing_params) {
  set_name("alip_mpc_interface_system");
  drake::systems::DiagramBuilder<double> builder;
  auto swing_interface =
      builder.AddSystem<SwingFootTrajectoryGenerator>(plant, context, swing_params);
  auto com_interface =
      builder.AddSystem<ComTrajInterfaceSystem>(plant, context, com_params);


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
