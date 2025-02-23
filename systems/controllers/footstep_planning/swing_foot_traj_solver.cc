#include "swing_foot_traj_solver.h"
#include "common/eigen_utils.h"
#include "common/legendre.h"
#include "common/legendre_polynomial_trajectory.h"

namespace dairlib::systems::controllers {

using drake::solvers::MathematicalProgram;
using drake::solvers::VectorXDecisionVariable;
using drake::trajectories::PathParameterizedTrajectory;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::LegendrePolynomialTrajectory;

using drake::Polynomial;

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::RowVectorXd;

using solvers::LegendreCoefficientVariables;

SwingFootTrajSolver::SwingFootTrajSolver() {

  splines_.push_back(LegendreCoefficientVariables(prog_, kPolyDegXY, "cx"));
  splines_.push_back(LegendreCoefficientVariables(prog_, kPolyDegXY, "cy"));
  splines_.push_back(LegendreCoefficientVariables(prog_, kPolyDegZ, "cz"));

  cont_slack_ = prog_.NewContinuousVariables(9, "e");

  Eigen::MatrixXd cont_cost = 100 * Eigen::MatrixXd::Identity(9,9);
  cont_cost.block<3,3>(0,0) *= 1000;
  cont_cost.block<3,3>(3,3) *= 10;

  cont_slack_cost_ = prog_.AddQuadraticCost(
      cont_cost, VectorXd::Zero(9), cont_slack_).evaluator();

  for (int dim = 0; dim < 3; ++dim) {
    for (int deriv = 0; deriv < 3; ++deriv) {
      splines_[dim].AddDerivativeConstraint(prog_, "end", 1, deriv, 0);
      splines_[dim].AddDerivativeSoftConstraint(
          prog_, "continuity", -1, deriv,
          0, cont_slack_.segment<1>(3*deriv +  dim));
    }

    splines_[dim].AddDerivativeConstraint(prog_, "midpoint", 0, 0, 0);
    splines_[dim].AddRunningCostOnDerivativeSquared(prog_, "min_accel", 2);
  }
  solver_.InitializeSolver(prog_, drake::solvers::SolverOptions(), 0, 0, {});
}

drake::trajectories::PathParameterizedTrajectory<double>
SwingFootTrajSolver::AdaptSwingFootTraj(
    const drake::trajectories::PathParameterizedTrajectory<double> &prev_traj,
    double prev_time,
    double t_start,
    double t_end,
    double clearance,
    double z_vel_final,
    double z_pos_final_offset,
    const Eigen::Vector3d &initial_pos,
    const Eigen::Vector3d &target) {

  // we are using the polynomial from the kth control cycle to
  // derive coefficients for the k+1th control cycle
  double tk = -1 + 2 * (prev_time - t_start) / (t_end - t_start);
  std::vector<double> knots = {-1, tk, 1};

  double z_vel_final_scaled = 0.5 * (t_end - t_start)  * z_vel_final;
  
  // update constraints
  std::array<Vector3d, 3> end_vals = {
      target + Vector3d(0, 0, z_pos_final_offset),
      Vector3d(0, 0, z_vel_final_scaled),
      Vector3d::Zero()
  };
  for (int deriv = 0; deriv < 3; ++deriv) {
    double scale = pow(2 / (t_end - t_start), -deriv);
    Vector3d prev_vals = scale * prev_traj.EvalDerivative(prev_time, deriv);
    for (int dim = 0; dim < 3; ++dim) {
      splines_[dim].UpdateDerivativeSoftConstraint(
          "continuity", tk, deriv, prev_vals(dim));
      splines_[dim].UpdateDerivativeConstraint(
          "end" , 1, deriv, end_vals.at(deriv)(dim));
    }
  }

  Vector3d des_mid_point = CalcDesiredMidpoint(initial_pos, target, clearance);
  for (int dim = 0; dim < 3; ++dim) {
    splines_[dim].UpdateDerivativeConstraint("midpoint", 0, 0, des_mid_point(dim));
  }

  auto result = solver_.Solve(prog_);
  return ConvertSolutionToTrajectory(result, t_start, t_end);
}

drake::trajectories::PathParameterizedTrajectory<double>
SwingFootTrajSolver::ConvertSolutionToTrajectory(
    const drake::solvers::MathematicalProgramResult& result,
    double t_start, double t_end) const {

  MatrixXd coeffs = Eigen::MatrixXd::Zero(3, kPolyDegZ+1);

  for (int i = 0; i < 3; ++i) {
    const auto sol = splines_[i].GetSolution(result);
    coeffs.row(i).leftCols(sol.rows()) = sol.transpose();
  }

  auto time_scaling = PiecewisePolynomial<double>::FirstOrderHold(
      Eigen::Vector2d(t_start, t_end),
      Eigen::RowVector2d(-1, 1)
  );

  return PathParameterizedTrajectory<double>(
      LegendrePolynomialTrajectory(coeffs), time_scaling);
}

Eigen::Vector3d SwingFootTrajSolver::CalcDesiredMidpoint(
    const Vector3d &initial_pos, const Vector3d &target, double clearance) {
  Vector3d pos_T = target - initial_pos;
  double disp_yaw = atan2(pos_T(1), pos_T(0));
  Vector3d n_planar(cos(disp_yaw - M_PI_2), sin(disp_yaw - M_PI_2), 0);
  Vector3d n = n_planar.cross(pos_T).normalized();

  // bias toward the normal being straight up for short footsteps to avoid
  // weird scaling issues
  double blend = pos_T.norm() - 0.1 / 0.1;
  blend = std::clamp(blend, 0.0, 1.0);
  n = blend * n + (1.0 - blend) * Vector3d::UnitZ();
  n.normalize();

  clearance += 0.8 * std::min(fabs(pos_T(2)), clearance);
  Vector3d des_mid_point = initial_pos + 0.5 * pos_T + clearance * n;
  return des_mid_point;
}

}