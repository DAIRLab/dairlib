#include "swing_foot_traj_solver.h"
#include "common/eigen_utils.h"
#include "common/legendre.h"

namespace dairlib::systems::controllers {

using drake::solvers::MathematicalProgram;
using drake::solvers::VectorXDecisionVariable;
using drake::trajectories::PathParameterizedTrajectory;
using drake::trajectories::PiecewisePolynomial;
using drake::Polynomial;

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::RowVectorXd;


namespace {

long factorial(long i) {
  long f = 1;
  for (int j = 1; j <= i; ++j) {
    f *= j;
  }
  return f;
}

}

SwingFootTrajSolver::SwingFootTrajSolver() {

  cx_ = prog_.NewContinuousVariables(kPolyDegXY, "cx");
  cy_ = prog_.NewContinuousVariables(kPolyDegXY, "cy");
  cz_ = prog_.NewContinuousVariables(kPolyDegZ, "cz");

  dim_var_map_ = {cx_, cy_, cz_};
  dim_degree_map_ = {kPolyDegXY, kPolyDegXY, kPolyDegZ};

  knot_deriv_multipliers_ = std::vector<std::vector<RowVectorXd>>(
      3, std::vector<RowVectorXd>(3, RowVectorXd::Zero(kPolyDegZ))
  );
  knot_deriv_rhs_ = std::vector<std::vector<Vector3d>>(
      3, std::vector<Vector3d>(3, Vector3d::Zero()));

  for (int knot = 0; knot < 3; ++knot) {
    std::vector<drake::solvers::LinearEqualityConstraint*> xtmp;
    std::vector<drake::solvers::LinearEqualityConstraint*> ytmp;
    std::vector<drake::solvers::LinearEqualityConstraint*> ztmp;
    for (int deriv = 0; deriv < 3; ++deriv) {
      xtmp.push_back(
          prog_.AddLinearEqualityConstraint(
              RowVectorXd::Zero(kPolyDegXY), VectorXd::Zero(1), cx_
              ).evaluator().get());
      ytmp.push_back(
          prog_.AddLinearEqualityConstraint(
              RowVectorXd::Zero(kPolyDegXY), VectorXd::Zero(1), cy_
          ).evaluator().get());
      ztmp.push_back(
          prog_.AddLinearEqualityConstraint(
              RowVectorXd::Zero(kPolyDegZ), VectorXd::Zero(1), cz_
          ).evaluator().get());
    }
    x_knot_constraints_.push_back(xtmp);
    y_knot_constraints_.push_back(ytmp);
    z_knot_constraints_.push_back(ztmp);

  }

  min_accel_Q_ = 0.1 * polynomials::MakeCostMatrixForMinimizingPathDerivativeSquaredWithLegendreBasis(
      kPolyDegZ - 1 , 2) + 0.001 * MatrixXd::Identity(kPolyDegZ, kPolyDegZ);

  int n = 2 * kPolyDegXY + kPolyDegZ;
  midpoint_target_cost_ = prog_.AddQuadraticCost(
      MatrixXd::Identity(n, n), VectorXd::Zero(n), {cx_, cy_, cz_}
  ).evaluator();

  x_min_accel_cost = prog_.AddQuadraticCost(
      min_accel_Q_.topLeftCorner<kPolyDegXY, kPolyDegXY>(), VectorXd::Zero(kPolyDegXY), cx_, true).evaluator();
  y_min_accel_cost = prog_.AddQuadraticCost(
      min_accel_Q_.topLeftCorner<kPolyDegXY, kPolyDegXY>(), VectorXd::Zero(kPolyDegXY),  cy_, true).evaluator();
  z_min_accel_cost = prog_.AddQuadraticCost(
      min_accel_Q_, VectorXd::Zero(kPolyDegZ), cz_, true).evaluator();

  solver_.InitializeSolver(prog_, drake::solvers::SolverOptions(), 0, 0, {});

}

drake::trajectories::PathParameterizedTrajectory<double>
SwingFootTrajSolver::AdaptSwingFootTraj(
    const drake::trajectories::PathParameterizedTrajectory<double> &prev_traj,
    double prev_time,
    double t_start,
    double t_end,
    double swing_foot_clearance,
    double z_vel_final,
    double z_pos_final_offset,
    const Eigen::Vector3d &initial_pos,
    const Eigen::Vector3d &footstep_target) {

  // we are using the polynomial from the kth control cycle to
  // derive coefficients for the k+1th control cycle
  double tk = -1 + 2 * (prev_time - t_start) / (t_end - t_start);

  std::vector<double> knots = {-1, tk, 1};

  double tmid = 0;

  RowVectorXd mid_mult = polynomials::EvalLegendreBasis(kPolyDegZ - 1, tmid).transpose();

  for (int k = 0; k < 3; ++k) {
    knot_deriv_multipliers_.at(k).at(0) = polynomials::EvalLegendreBasis(kPolyDegZ - 1, knots.at(k)).transpose();
    knot_deriv_multipliers_.at(k).at(1) = polynomials::EvalLegendreBasisDerivative(kPolyDegZ - 1, 1, knots.at(k)).transpose();
    knot_deriv_multipliers_.at(k).at(2) = polynomials::EvalLegendreBasisDerivative(kPolyDegZ - 1, 2, knots.at(k)).transpose();
  }

  for (int i = 0; i < 3; ++i) {
    double scale = pow(2 / (t_end - t_start), -i);
    knot_deriv_rhs_.at(1).at(i) = scale * prev_traj.EvalDerivative(prev_time, i);
  }

  knot_deriv_rhs_.front().front() = initial_pos;

  double z_vel_final_scaled = (t_end - t_start) * (t_end - t_start) * z_vel_final / 4.0;
  knot_deriv_rhs_.at(2).at(0) = footstep_target + Vector3d(0, 0, z_pos_final_offset);
  knot_deriv_rhs_.at(2).at(1) = Vector3d(0, 0, z_vel_final_scaled);

  Vector3d pos_T = footstep_target - initial_pos;
  double disp_yaw = atan2(pos_T(1), pos_T(0));
  Vector3d n_planar(cos(disp_yaw - M_PI_2), sin(disp_yaw - M_PI_2), 0);
  Vector3d n = n_planar.cross(pos_T).normalized();
  Vector3d des_mid_point = initial_pos + 0.5 * pos_T + swing_foot_clearance * n;

  for (int knot = 0; knot < 3; ++knot) {
    for (int deriv = 0; deriv < 3; ++ deriv) {
      x_knot_constraints_.at(knot).at(deriv)->UpdateCoefficients(
          knot_deriv_multipliers_.at(knot).at(deriv).leftCols<kPolyDegXY>(),
          knot_deriv_rhs_.at(knot).at(deriv).segment<1>(0));
      y_knot_constraints_.at(knot).at(deriv)->UpdateCoefficients(
          knot_deriv_multipliers_.at(knot).at(deriv).leftCols<kPolyDegXY>(),
          knot_deriv_rhs_.at(knot).at(deriv).segment<1>(1));
      z_knot_constraints_.at(knot).at(deriv)->UpdateCoefficients(
          knot_deriv_multipliers_.at(knot).at(deriv),
          knot_deriv_rhs_.at(knot).at(deriv).segment<1>(2));
    }
  }

  MatrixXd mid_mult_broad = MatrixXd::Zero(3, 2 * kPolyDegXY + kPolyDegZ);
  mid_mult_broad.topLeftCorner<2, 2*kPolyDegXY>() =
      BlockDiagonalRepeat<double>(mid_mult.leftCols<kPolyDegXY>(), 2);
  mid_mult_broad.bottomRightCorner<1, kPolyDegZ>() = mid_mult;

  MatrixXd Q = 1000 * mid_mult_broad.transpose() * mid_mult_broad;
  VectorXd b = 1000 * mid_mult_broad.transpose() * des_mid_point;

  midpoint_target_cost_->UpdateCoefficients(
      2 * Q, -2 * b, mid_mult_broad.squaredNorm(), true);

  auto result = solver_.Solve(prog_);

  drake::Vector3<Polynomial<double>> polymat;
  const MatrixXd bchange = polynomials::MakeChangeOfBasisOperatorFromLegendreToMonomials(kPolyDegZ - 1);

  Polynomial<double> t_shift(Eigen::Vector2d(-1, 1));

  for ( int i = 0; i < 3; ++i) {
    Eigen::VectorXd coeffs = result.GetSolution(dim_var_map_.at(i));

    const MatrixXd& B = bchange.topLeftCorner(coeffs.rows(), coeffs.rows());
    coeffs = B * coeffs;

    for (double t = -1; t < 1; t+=.05) {
      double sum = 0;
      for (int i = 0; i < coeffs.rows(); ++i) {
        sum += coeffs(i) * pow(t, i);
      }
    }

    polymat(i) = Polynomial<double>(coeffs);
    polymat(i) = polymat(i).Substitute(*(polymat(i).GetVariables().begin()), t_shift);
  }

  std::vector<drake::MatrixX<Polynomial<double>>> polys = {polymat};
  std::vector<double> breaks = {0, 2};
  PiecewisePolynomial<double> pp(polys, breaks);

  auto time_scaling = PiecewisePolynomial<double>::FirstOrderHold(
      Eigen::Vector2d(t_start, t_end),
      Eigen::RowVector2d(0, 2) // hack for pp offset
  );

  PathParameterizedTrajectory<double> path(pp, time_scaling);

  return path;
}

}