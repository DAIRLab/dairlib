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

using polynomials::EvalLegendreBasis;
using polynomials::EvalLegendreBasisDerivative;
using polynomials::MakeChangeOfBasisOperatorFromLegendreToMonomials;
using polynomials::MakeCostMatrixForMinimizingPathDerivativeSquaredWithLegendreBasis;

SwingFootTrajSolver::SwingFootTrajSolver() {

  cx_ = prog_.NewContinuousVariables(kPolyDegXY, "cx");
  cy_ = prog_.NewContinuousVariables(kPolyDegXY, "cy");
  cz_ = prog_.NewContinuousVariables(kPolyDegZ, "cz");
  cont_slack_ = prog_.NewContinuousVariables(9, "e");

  Eigen::MatrixXd cont_cost = 10 * Eigen::MatrixXd::Identity(9,9);
  cont_cost.block<3,3>(0,0) *= 1000;
  cont_cost.block<3,3>(3,3) *= 10;

  cont_slack_cost_ = prog_.AddQuadraticCost(
      cont_cost, VectorXd::Zero(9), cont_slack_).evaluator();

  dim_var_map_ = {cx_, cy_, cz_};

  knot_deriv_multipliers_ = std::vector<std::vector<RowVectorXd>>(
      3, std::vector<RowVectorXd>(3, RowVectorXd::Zero(kPolyDegZ))
  );
  knot_deriv_rhs_ = std::vector<std::vector<Vector3d>>(
      3, std::vector<Vector3d>(3, Vector3d::Zero()));

  for (int deriv = 0; deriv < 3; ++deriv) {
    x_end_constraints_.push_back(
        prog_.AddLinearEqualityConstraint(
            RowVectorXd::Zero(kPolyDegXY), VectorXd::Zero(1), cx_
            ).evaluator().get());
    y_end_constraints_.push_back(
        prog_.AddLinearEqualityConstraint(
            RowVectorXd::Zero(kPolyDegXY), VectorXd::Zero(1), cy_
        ).evaluator().get());
    z_end_constraints_.push_back(
        prog_.AddLinearEqualityConstraint(
            RowVectorXd::Zero(kPolyDegZ), VectorXd::Zero(1), cz_
        ).evaluator().get());
    x_knot_constraints_.push_back(
        prog_.AddLinearEqualityConstraint(
            RowVectorXd::Zero(kPolyDegXY + 1), VectorXd::Zero(1), {cx_, cont_slack_.segment<1>(3*deriv)}
        ).evaluator().get());
    y_knot_constraints_.push_back(
        prog_.AddLinearEqualityConstraint(
            RowVectorXd::Zero(kPolyDegXY + 1), VectorXd::Zero(1), {cy_, cont_slack_.segment<1>(3*deriv + 1)}
        ).evaluator().get());
    z_knot_constraints_.push_back(
        prog_.AddLinearEqualityConstraint(
            RowVectorXd::Zero(kPolyDegZ + 1), VectorXd::Zero(1), {cz_, cont_slack_.segment<1>(3*deriv + 2)}
        ).evaluator().get());
  }


  min_acc_Q_ =
      MakeCostMatrixForMinimizingPathDerivativeSquaredWithLegendreBasis(
          kPolyDegZ - 1 , 2);

  int n = 2 * kPolyDegXY + kPolyDegZ;
  midpoint_target_cost_ = prog_.AddQuadraticCost(
      MatrixXd::Identity(n, n), VectorXd::Zero(n), {cx_, cy_, cz_}
  ).evaluator();

  x_min_accel_cost = prog_.AddQuadraticCost(
      min_acc_Q_.topLeftCorner<kPolyDegXY, kPolyDegXY>(),
      VectorXd::Zero(kPolyDegXY),
      cx_,
      true
  ).evaluator();

  y_min_accel_cost = prog_.AddQuadraticCost(
      min_acc_Q_.topLeftCorner<kPolyDegXY, kPolyDegXY>(),
      VectorXd::Zero(kPolyDegXY),
      cy_,
      true
  ).evaluator();

  z_min_accel_cost = prog_.AddQuadraticCost(
      0.01 * min_acc_Q_,
      VectorXd::Zero(kPolyDegZ),
      cz_,
      true
  ).evaluator();

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

  for (int k = 0; k < 3; ++k) {
    knot_deriv_multipliers_.at(k).at(0) =
        EvalLegendreBasis(kPolyDegZ - 1, knots.at(k)).transpose();
    knot_deriv_multipliers_.at(k).at(1) =
        EvalLegendreBasisDerivative(kPolyDegZ - 1, 1, knots.at(k)).transpose();
    knot_deriv_multipliers_.at(k).at(2) =
        EvalLegendreBasisDerivative(kPolyDegZ - 1, 2, knots.at(k)).transpose();
  }

  for (int i = 0; i < 3; ++i) {
    double scale = pow(2 / (t_end - t_start), -i);
    knot_deriv_rhs_.at(1).at(i) = scale * prev_traj.EvalDerivative(prev_time, i);
  }

  knot_deriv_rhs_.front().front() = initial_pos;

  double z_vel_final_scaled = (t_end - t_start) * (t_end - t_start) * z_vel_final / 4.0;
  knot_deriv_rhs_.at(2).at(0) = target + Vector3d(0, 0, z_pos_final_offset);
  knot_deriv_rhs_.at(2).at(1) = Vector3d(0, 0, z_vel_final_scaled);

  Vector3d des_mid_point = CalcDesiredMidpoint(initial_pos, target, clearance);

  UpdateContinuityConstraintsWithSlack();

  for (int deriv = 0; deriv < 3; ++ deriv) {
    int knot = 2;
    x_end_constraints_.at(deriv)->UpdateCoefficients(
        knot_deriv_multipliers_.at(knot).at(deriv).leftCols<kPolyDegXY>(),
        knot_deriv_rhs_.at(knot).at(deriv).segment<1>(0));
    y_end_constraints_.at(deriv)->UpdateCoefficients(
        knot_deriv_multipliers_.at(knot).at(deriv).leftCols<kPolyDegXY>(),
        knot_deriv_rhs_.at(knot).at(deriv).segment<1>(1));
    z_end_constraints_.at(deriv)->UpdateCoefficients(
        knot_deriv_multipliers_.at(knot).at(deriv),
        knot_deriv_rhs_.at(knot).at(deriv).segment<1>(2));
  }


  RowVectorXd mid_mult = EvalLegendreBasis(kPolyDegZ - 1, 0).transpose();
  MatrixXd mid_mult_broad = MatrixXd::Zero(3, 2 * kPolyDegXY + kPolyDegZ);
  mid_mult_broad.topLeftCorner<2, 2*kPolyDegXY>() =
      BlockDiagonalRepeat<double>(mid_mult.leftCols<kPolyDegXY>(), 2);
  mid_mult_broad.bottomRightCorner<1, kPolyDegZ>() = mid_mult;

  MatrixXd Q = mid_mult_broad.transpose() * mid_mult_broad;
  VectorXd b = mid_mult_broad.transpose() * des_mid_point;

  midpoint_target_cost_->UpdateCoefficients(
      2 * Q, -2 * b, mid_mult_broad.squaredNorm(), true);

  auto result = solver_.Solve(prog_);

  return ConvertSolutionToTrajectory(result, t_start, t_end);
}

drake::trajectories::PathParameterizedTrajectory<double>
SwingFootTrajSolver::ConvertSolutionToTrajectory(
    const drake::solvers::MathematicalProgramResult& result,
    double t_start, double t_end) const {

  MatrixXd coeffs = Eigen::MatrixXd::Zero(3, kPolyDegZ);

  for (int i = 0; i < 3; ++i) {
    const auto& sol = result.GetSolution(dim_var_map_.at(i));
    coeffs.row(i).leftCols(sol.rows()) = sol.transpose();
  }



  auto time_scaling = PiecewisePolynomial<double>::FirstOrderHold(
      Eigen::Vector2d(t_start, t_end),
      Eigen::RowVector2d(-1, 1)
  );

  return PathParameterizedTrajectory<double>(
      LegendrePolynomialTrajectory(coeffs), time_scaling);
}

void SwingFootTrajSolver::UpdateContinuityConstraintsWithSlack() {
  int knot = 1;
  for (int deriv = 0; deriv < 3; ++deriv) {
    Eigen::MatrixXd A_xy = Eigen::MatrixXd::Ones(1, kPolyDegXY + 1);
    A_xy.leftCols(kPolyDegXY) = knot_deriv_multipliers_.at(knot).at(deriv).leftCols<kPolyDegXY>();
    Eigen::MatrixXd A_z = Eigen::MatrixXd::Ones(1, kPolyDegZ + 1);
    A_z.leftCols(kPolyDegZ) = knot_deriv_multipliers_.at(knot).at(deriv);

    x_knot_constraints_.at(deriv)->UpdateCoefficients(
        A_xy, knot_deriv_rhs_.at(knot).at(deriv).segment<1>(0));
    y_knot_constraints_.at(deriv)->UpdateCoefficients(
        A_xy,
        knot_deriv_rhs_.at(knot).at(deriv).segment<1>(1));
    z_knot_constraints_.at(deriv)->UpdateCoefficients(
        A_z,
        knot_deriv_rhs_.at(knot).at(deriv).segment<1>(2));
  }
};

Eigen::Vector3d SwingFootTrajSolver::CalcDesiredMidpoint(
    const Vector3d &initial_pos, const Vector3d &target, double clearance) {
  Vector3d pos_T = target - initial_pos;
  double disp_yaw = atan2(pos_T(1), pos_T(0));
  Vector3d n_planar(cos(disp_yaw - M_PI_2), sin(disp_yaw - M_PI_2), 0);
  Vector3d n = n_planar.cross(pos_T).normalized();
  Vector3d des_mid_point = initial_pos + 0.5 * pos_T + clearance * n;
  return des_mid_point;
}

}