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
    double mpc_dt,
    const std::vector<int> &unordered_fsm_states,
    const std::vector<BipedStance> &unordered_state_stances)
    : plant_(plant),
      desired_com_height_(desired_com_height),
      stance_duration_(stance_duration),
      mpc_dt_(mpc_dt),
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

  PiecewisePolynomial<double> pp;
  drake::trajectories::Trajectory<double>& traj_inst = pp;

  output_port_lipm_from_current_ =
      this->DeclareAbstractOutputPort("srbd_x_from_current", traj_inst,
                                      &LipmWarmStartSystem::CalcTrajFromCurrent)
          .get_index();

  output_port_foot_target_ =
      this->DeclareVectorOutputPort("foot_target_out",
          BasicVector<double>(6), &LipmWarmStartSystem::CalcFootTarget)
          .get_index();

  // 4 xy state variables per 3 steps
  BasicVector<double> model_vec(18);
  warmstart_sol_ = &DeclareCacheEntry(
      "warmstart qp solution",
      model_vec,
      &LipmWarmStartSystem::CalcWarmstartSolution,
      {all_input_ports_ticket()});
}

void LipmWarmStartSystem::CalcWarmstartSolution(
    const drake::systems::Context<double> &context,
    drake::systems::BasicVector<double> *solvec) const {

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
      4, first_mode_duration, des_srbd_state, srbd_state);

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
      0.00,
      (unordered_state_stances_[mode_index] == BipedStance::kLeft));

  drake::solvers::MathematicalProgramResult result = drake::solvers::Solve(mpc);

  DRAKE_DEMAND(result.is_success());

  solvec->get_mutable_value().head(12) = result.GetSolution(mpc.x_lipm_vars());
  solvec->get_mutable_value().tail(6) = result.GetSolution(mpc.u_lipm_vars());
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

void LipmWarmStartSystem::CalcTrajFromCurrent(
    const Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {

  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);

  double prev_event_time = // fsm switch time
      this->EvalVectorInput(context, touchdown_time_port_)->get_value()(0);
  double timestamp = robot_output->get_timestamp();

  std::vector< drake::trajectories::ExponentialPlusPiecewisePolynomial<double>>
      step_trajs;

  const Eigen::VectorXd& mpc_sol =
      warmstart_sol_->Eval<BasicVector<double>>(context).get_value();

  for (int idx = 0; idx < 3; idx++) {
    double start_time = timestamp + idx * stance_duration_;
    double end_time = prev_event_time + stance_duration_ * (idx + 1);

    Vector3d CoM = Vector3d::Zero();
    Vector3d dCoM = Vector3d::Zero();
    Vector3d stance_foot_pos = Vector3d::Zero();

    Vector4d state_sol = LipmMpc::GetStateSolutionByIndex(
        idx, mpc_sol.head(12));

    CoM.head<2>() = state_sol.head<2>();
    CoM(2) = desired_com_height_;
    dCoM.head<2>() = state_sol.tail<2>();
    stance_foot_pos.head<2>() = LipmMpc::GetInputSolutionByIndex(
        idx, mpc_sol.tail(6));
    step_trajs.push_back(
        ConstructLipmTraj(CoM, dCoM, stance_foot_pos, start_time, end_time));
  }
  MakeCubicSrbdApproximationFromExponentials(step_trajs, traj, prev_event_time);
}

void LipmWarmStartSystem::MakeCubicSrbdApproximationFromExponentials(
    std::vector<
        drake::trajectories::ExponentialPlusPiecewisePolynomial<double>> exps,
    drake::trajectories::Trajectory<double> *output_traj,
    double prev_touchdown_time) const {

  std::vector<
      std::unique_ptr<
          drake::trajectories::Trajectory<double>>> deriv;

  for (auto & exp : exps) {
    deriv.push_back(exp.MakeDerivative(1));
  }

  int n = std::floor(
      (exps.back().end_time() - exps.front().start_time()) / mpc_dt_);

  MatrixXd state_knots = MatrixXd::Zero(6, n);
  MatrixXd state_derivs = MatrixXd::Zero(6, n);
  VectorXd breaks = VectorXd::Zero(n);

  for (int i = 0; i < n; i++) {
    double t = exps.front().start_time() + i * mpc_dt_;
    int idx = std::floor((t - prev_touchdown_time) / stance_duration_);
    state_knots.block(0, i, 3, 1) = exps.at(idx).value(t);
    state_derivs.block(0, i, 3, 1) = deriv.at(idx)->value(t);
    breaks(i) = t;
  }

  // Assign traj
  auto pp_traj = (PiecewisePolynomial<double>*)dynamic_cast<
      PiecewisePolynomial<double>*>(output_traj);

  *pp_traj = PiecewisePolynomial<double>::CubicHermite(
      breaks, state_knots, state_derivs);

}

void LipmWarmStartSystem::CalcFootTarget(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {

  const Eigen::VectorXd& input_sol =
      warmstart_sol_->Eval<BasicVector<double>>(context).get_value().tail(6);

  Vector2d stance1xy = LipmMpc::GetInputSolutionByIndex(1, input_sol);
  Vector2d stance2xy = LipmMpc::GetInputSolutionByIndex(2, input_sol);

  output->get_mutable_value().segment<2>(0) = stance1xy;
  output->get_mutable_value().segment<2>(3) = stance2xy;
  output->get_mutable_value()(2) = 0;
  output->get_mutable_value()(5) = 0;
}
}  // namespace systems
}  // namespace dairlib
