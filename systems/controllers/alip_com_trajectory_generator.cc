#include "alip_com_trajectory_generator.h"

#include <cmath>
#include <algorithm>
#include <iostream>

#include "multibody/multibody_utils.h"


using dairlib::systems::controllers::alip_utils::PointOnFramed;

using std::endl;
using std::string;

using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using drake::multibody::JacobianWrtVariable;

using drake::systems::Context;
using drake::systems::State;
using drake::systems::EventStatus;

using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::ExponentialPlusPiecewisePolynomial;
using drake::trajectories::Trajectory;


namespace dairlib {
namespace systems {
namespace controllers {

AlipComTrajectoryGenerator::AlipComTrajectoryGenerator(
    const drake::multibody::MultibodyPlant<double> &plant,
    drake::systems::Context<double> *context,
    double desired_com_height,
    const std::vector<int> &unordered_fsm_states,
    const std::vector<PointOnFramed> contact_point_in_each_state) :
    plant_(plant),
    context_(context),
    desired_com_height_(desired_com_height),
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
      &AlipComTrajectoryGenerator::CalcComTrajFromCurrent).get_index();

  prev_slope_idx_ = DeclareDiscreteState(2);
  m_ = plant_.CalcTotalMass(*context);

  DeclarePerStepUnrestrictedUpdateEvent(
      &AlipComTrajectoryGenerator::UnrestrictedUpdate);
}

EventStatus AlipComTrajectoryGenerator::UnrestrictedUpdate(
    const Context<double> &context, State<double> *state) const {
  auto fsm_state = static_cast<int>(
      this->EvalVectorInput(context, fsm_port_)->value()(0));

  // in single stance, save the current slope parameters to use during double
  // stance
  if (fsm_state <= 1) {
    state->get_mutable_discrete_state(prev_slope_idx_).get_mutable_value() =
        EvalVectorInput(context, slope_params_port_)->get_value();
  }

  return EventStatus::Succeeded();
}

ExponentialPlusPiecewisePolynomial<double>
AlipComTrajectoryGenerator::ConstructAlipComTraj(
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

void AlipComTrajectoryGenerator::CalcAlipState(
    const Eigen::VectorXd &x, int mode_index,
    const drake::EigenPtr<Eigen::Vector3d>& CoM_p,
    const drake::EigenPtr<Eigen::Vector3d>& L_p,
    const drake::EigenPtr<Eigen::Vector3d>& stance_pos_p) const {
  alip_utils::CalcAlipState(
      plant_, context_, x,
      {contact_point_in_each_state_[mode_index]},
      {1.0}, CoM_p, L_p, stance_pos_p);
}

void AlipComTrajectoryGenerator::CalcComTrajFromCurrent(
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

  // TODO (@Brian-Acosta standardize walking fsm states with an enum)
  bool is_ss = fsm_state <= 1;
  double t = std::clamp<double>(timestamp,
                                -std::numeric_limits<double>::infinity(),
                                end_time - 0.001);

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
      stance_foot_pos, x_alip, kx_ky, t, end_time);
}


}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
