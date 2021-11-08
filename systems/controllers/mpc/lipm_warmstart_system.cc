#include "systems/controllers/mpc/lipm_warmstart_system.h"
#include "systems/controllers/mpc/lipm_mpc_qp.h"
#include "drake/solvers/solve.h"
#include "drake/solvers/mathematical_program_result.h"

#include <math.h>

#include <fstream>
#include <string>

#include <drake/math/saturate.h>

using std::cout;
using std::endl;
using std::string;
using std::vector;

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;

using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::trajectories::ExponentialPlusPiecewisePolynomial;
using drake::trajectories::PiecewisePolynomial;

namespace dairlib {
namespace systems {

LipmWarmStartSystem::LipmWarmStartSystem(
    const multibody::SingleRigidBodyPlant &plant,
    double desired_com_height,
    double stance_duration,
    const std::vector<int> &unordered_fsm_states,
    const std::vector<BipedStance> &unordered_state_stances)
    : plant_(plant),
      desired_com_height_(desired_com_height),
      stance_duration_(stance_duration),
      unordered_fsm_states_(unordered_fsm_states),
      unordered_state_stances_(unordered_state_stances) {

  DRAKE_DEMAND(unordered_fsm_states.size() == unordered_state_stances.size());

  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(
          "x, u, t", OutputVector<double>(plant.nq(), plant.nv(), plant.nu()))
      .get_index();
  fsm_port_ =
      this->DeclareVectorInputPort(
          "fsm", BasicVector<double>(1)).get_index();

  touchdown_time_port_ =
      this->DeclareVectorInputPort(
          "t_touchdown", BasicVector<double>(1))
          .get_index();

  x_des_input_port_ = this->DeclareVectorInputPort(
      "srbd x desired",
      BasicVector<double>(12))
      .get_index();

  // Provide an instance to allocate the memory first (for the output)
  ExponentialPlusPiecewisePolynomial<double> exp;
  drake::trajectories::Trajectory<double>& traj_inst = exp;
  output_port_lipm_from_current_ =
      this->DeclareAbstractOutputPort("lipm_xyz_from_current", traj_inst,
                                      &LipmWarmStartSystem::CalcTrajFromCurrent)
          .get_index();

  // State variables inside this controller block
  DeclarePerStepDiscreteUpdateEvent(&LipmWarmStartSystem::DiscreteVariableUpdate);
}

EventStatus LipmWarmStartSystem::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {

  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  double fsm_state =
      this->EvalVectorInput(context, fsm_port_)->get_value()(0);
  double prev_event_time = // fsm switch timep
      this->EvalVectorInput(context, touchdown_time_port_)->get_value()(0);
  VectorXd des_srbd_state =
      this->EvalVectorInput(context, x_des_input_port_)->get_value();

  VectorXd x = robot_output->GetState();

  // Find fsm_state in unordered_fsm_states_
  auto it = find(
      unordered_fsm_states_.begin(),
      unordered_fsm_states_.end(), int(fsm_state));
  int mode_index = std::distance(unordered_fsm_states_.begin(), it);

  // Get time
  double timestamp = robot_output->get_timestamp();
  double start_time = timestamp;
  double end_time = prev_event_time + stance_duration_;
  double first_mode_duration = end_time - start_time;

      start_time = drake::math::saturate(start_time, -1,end_time - 0.001);

  VectorXd srbd_state = plant_.CalcSRBStateFromPlantState(x);
  // Get center of mass position and velocity
  Vector3d CoM = srbd_state.segment<3>(0);
  Vector3d dCoM = srbd_state.segment<3>(6);


  Vector3d stance_foot_pos = plant_.CalcFootPosition(
      x, unordered_state_stances_[mode_index]);

  std::vector<std::vector<Eigen::Vector2d>> xy_dxy = MakeDesXYVel(
      3, first_mode_duration, des_srbd_state, srbd_state);

  double Q = 10.0;
  std::vector<double> height_vector = {CoM(2), CoM(2), CoM(2)};
  LipmMpc mpc(
      xy_dxy.front(),
      xy_dxy.back(),
      Q, Q,
      CoM.head(2),
      dCoM.head(2),
      stance_foot_pos.head(2), 3,
      first_mode_duration, stance_duration_,
      height_vector,
      0.55,
      0.55,
      0.05,
      (unordered_state_stances_[mode_index] == BipedStance::kLeft));

  drake::solvers::MathematicalProgramResult result = drake::solvers::Solve(mpc);
  discrete_state->get_mutable_value(mpc_input_sol_idx_) = result.GetSolution(
      mpc.u_lipm_vars());
  discrete_state->get_mutable_value(mpc_state_sol_idx_) = result.GetSolution(
      mpc.u_lipm_vars());

  return EventStatus::Succeeded();
}

std::vector<std::vector<Eigen::Vector2d>> LipmWarmStartSystem::MakeDesXYVel(
    int n_step, double first_mode_duration, VectorXd xdes, VectorXd x) const{
  std::vector<Vector2d> xy;
  std::vector<Vector2d> dxy;
  std::vector<std::vector<Vector2d>> ret;
  for (int i = 0; i < n_step; i++) {
    double t = (i == 0) ? first_mode_duration : stance_duration_;
    xy.emplace_back(x.segment<2>(0) + xdes.segment<2>(6) * t);
    dxy.emplace_back(xdes.segment<2>(6));
  }
  ret.push_back(xy);
  ret.push_back(dxy);
  return ret;
}


ExponentialPlusPiecewisePolynomial<double> LipmWarmStartSystem::ConstructLipmTraj(
    const VectorXd& CoM, const VectorXd& dCoM, const VectorXd& stance_foot_pos,
    double start_time, double end_time_of_this_fsm_state) const {
  // Get CoM_wrt_foot for LIPM
  double CoM_wrt_foot_x = CoM(0) - stance_foot_pos(0);
  double CoM_wrt_foot_y = CoM(1) - stance_foot_pos(1);
  double CoM_wrt_foot_z = (CoM(2) - stance_foot_pos(2));
  double dCoM_wrt_foot_x = dCoM(0);
  double dCoM_wrt_foot_y = dCoM(1);
  DRAKE_DEMAND(CoM_wrt_foot_z > 0);

  // create a 3D one-segment polynomial for ExponentialPlusPiecewisePolynomial
  // Note that the start time in T_waypoint_com is also used by
  // ExponentialPlusPiecewisePolynomial.
  vector<double> T_waypoint_com = {start_time, end_time_of_this_fsm_state};

  vector<MatrixXd> Y(T_waypoint_com.size(), MatrixXd::Zero(3, 1));
  Y[0](0, 0) = stance_foot_pos(0);
  Y[1](0, 0) = stance_foot_pos(0);
  Y[0](1, 0) = stance_foot_pos(1);
  Y[1](1, 0) = stance_foot_pos(1);
  Y[0](2, 0) = CoM(2);
  Y[1](2, 0) = CoM(2);

  MatrixXd Y_dot_start = MatrixXd::Zero(3, 1);
  MatrixXd Y_dot_end = MatrixXd::Zero(3, 1);

  PiecewisePolynomial<double> pp_part =
      PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
          T_waypoint_com, Y, Y_dot_start, Y_dot_end);



  // Dynamics of LIPM
  // ddy = 9.81/CoM_wrt_foot_z*y, which has an analytical solution.
  // Let omega^2 = 9.81/CoM_wrt_foot_z.
  // Let y0 and dy0 be the intial position and velocity. Then the solution is
  //   y = k_1 * exp(w*t) + k_2 * exp(-w*t)
  // where k_1 = (y0 + dy0/w)/2
  //       k_2 = (y0 - dy0/w)/2.
  // double omega = sqrt(9.81 / (final_height - stance_foot_pos(2)));
  double omega = sqrt(9.81 / CoM_wrt_foot_z);
  double k1x = 0.5 * (CoM_wrt_foot_x + dCoM_wrt_foot_x / omega);
  double k2x = 0.5 * (CoM_wrt_foot_x - dCoM_wrt_foot_x / omega);
  double k1y = 0.5 * (CoM_wrt_foot_y + dCoM_wrt_foot_y / omega);
  double k2y = 0.5 * (CoM_wrt_foot_y - dCoM_wrt_foot_y / omega);

  //  cout << "omega = " << omega << endl;

  // Sum of two exponential + one-segment 3D polynomial
  MatrixXd K = MatrixXd::Zero(3, 4);
  MatrixXd A = MatrixXd::Zero(4, 4);
  MatrixXd alpha = MatrixXd::Ones(4, 1);
  K(0, 0) = k1x;
  K(0, 1) = k2x;
  K(1, 2) = k1y;
  K(1, 3) = k2y;
  A(0, 0) = omega;
  A(1, 1) = -omega;
  A(2, 2) = omega;
  A(3, 3) = -omega;

  return ExponentialPlusPiecewisePolynomial<double>(K, A, alpha, pp_part);
}

void LipmWarmStartSystem::CalcTrajFromStep(
    const Context<double>& context,
    drake::trajectories::Trajectory<double>* traj,
    int idx) const {

  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);

  double prev_event_time = // fsm switch time
      this->EvalVectorInput(context, touchdown_time_port_)->get_value()(0);

  double timestamp = robot_output->get_timestamp();
  double start_time = timestamp + idx * stance_duration_;
  double end_time = prev_event_time + stance_duration_ * (idx + 1);

  Vector3d CoM = Vector3d::Zero();
  Vector3d dCoM = Vector3d::Zero();
  Vector3d stance_foot_pos = Vector3d::Zero();

  Vector4d state_sol = LipmMpc::GetStateSolutionByIndex(
      idx, context.get_discrete_state(mpc_state_sol_idx_).get_value());

  CoM.head<2>() = state_sol.head<2>();
  CoM(2) = desired_com_height_;
  dCoM.head<2>() = state_sol.tail<2>();
  stance_foot_pos.head<2>() = LipmMpc::GetInputSolutionByIndex(
      idx, context.get_discrete_state(mpc_input_sol_idx_).get_value());

  // Assign traj
  auto exp_pp_traj = (ExponentialPlusPiecewisePolynomial<double>*)dynamic_cast<
      ExponentialPlusPiecewisePolynomial<double>*>(traj);

  *exp_pp_traj =
      ConstructLipmTraj(CoM, dCoM, stance_foot_pos, start_time, end_time);
}


}  // namespace systems
}  // namespace dairlib
