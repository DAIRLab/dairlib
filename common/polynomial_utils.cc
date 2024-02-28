#include "polynomial_utils.h"

#include "eigen_utils.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/equality_constrained_qp_solver.h"
#include "drake/solvers/clarabel_solver.h"

#include "solvers/fcc_qp_solver.h"

#include <iostream>

namespace dairlib::polynomials {

using drake::solvers::MathematicalProgram;
using drake::solvers::VectorXDecisionVariable;
using drake::trajectories::PathParameterizedTrajectory;
using drake::trajectories::PiecewisePolynomial;
using drake::Polynomial;

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

static constexpr int kSwingPolyDeg = 10;

using RowVectorkSwingPolyDegd = drake::RowVector<double, kSwingPolyDeg>;

PathParameterizedTrajectory<double> AdaptSwingFootTraj(
    const PathParameterizedTrajectory<double>& prev_traj,
    double prev_time, double curr_time, double t_start, double t_end,
    double swing_foot_clearance, double z_vel_final, double z_pos_final_offset,
    const Vector3d& initial_pos, const Vector3d& footstep_target) {

  MathematicalProgram prog;
  auto cx = prog.NewContinuousVariables(kSwingPolyDeg, "cx");
  auto cy = prog.NewContinuousVariables(kSwingPolyDeg, "cy");
  auto cz = prog.NewContinuousVariables(kSwingPolyDeg, "cz");

  std::unordered_map<int, const VectorXDecisionVariable&> dim_var_map = {
      {0, cx}, {1, cy}, {2, cz}
  };

  // we are using the polynomial from the kth control cycle to
  // derive coefficients for the k+1th control cycle
  double tk = prev_time - t_start;
  double T = t_end - t_start;

  // solve for the coefficients that minimize distance of x(T/2) from the
  // desired midpoint, subject to continuous state, acceleration, and input
  // constraints
  //
  // coeff order from t^0 to t^9

  auto knot_deriv_multipliers = std::vector<std::vector<RowVectorkSwingPolyDegd>>(
      3, std::vector<RowVectorkSwingPolyDegd>(3, RowVectorkSwingPolyDegd::Zero()));


  double conditioning_offset = 0.25;

  std::vector<double> knots = {
      conditioning_offset,
      conditioning_offset + tk,
      conditioning_offset + T
  };

  double tmid = conditioning_offset + T / 2;

  RowVectorkSwingPolyDegd mid_mult;
  for (int k = 0; k < 3; ++k) {
    for (int i = 0; i < kSwingPolyDeg; ++i) {
      mid_mult(i) = pow(tmid, i);
      knot_deriv_multipliers.at(k).at(0)(i) = pow(knots.at(k), i);
      knot_deriv_multipliers.at(k).at(1)(i) = i * pow(knots.at(k), i - 1);
      knot_deriv_multipliers.at(k).at(2)(i) = i * (i - 1) * pow(knots.at(k), i - 2);
    }
  }

  auto knot_deriv_rhs = std::vector<std::vector<Vector3d>>(
          3, std::vector<Vector3d>(3, Vector3d::Zero()));

  for (int i = 0; i < 3; ++i) {
    knot_deriv_rhs.at(1).at(i) = prev_traj.EvalDerivative(prev_time, i);
  }
  knot_deriv_rhs.front().front() = initial_pos;
  knot_deriv_rhs.at(2).at(0) =
      footstep_target + Vector3d(0, 0, z_pos_final_offset);
  knot_deriv_rhs.at(2).at(1) = Vector3d(0, 0, z_vel_final);

  Vector3d pos_T = footstep_target - initial_pos;
  double disp_yaw = atan2(pos_T(1), pos_T(0));
  Vector3d n_planar(cos(disp_yaw - M_PI_2), sin(disp_yaw - M_PI_2), 0);
  Vector3d n = n_planar.cross(pos_T).normalized();
  Vector3d des_mid_point = initial_pos + 0.5 * pos_T + swing_foot_clearance * n;

  for (int dim = 0; dim < 3; ++dim) {
    for (int knot = 0; knot < 3; ++ knot) {
      for (int deriv = 0; deriv < 3; ++deriv) {
        // TODO (@Brian-Acosta) We probably don't need to add the initial
        //  position constraints on x-y, can instead regularize another term

//        if (dim < 2 and knot == 0) {
//          continue;
//        }

        auto binding = prog.AddLinearEqualityConstraint(
            knot_deriv_multipliers.at(knot).at(deriv),
            knot_deriv_rhs.at(knot).at(deriv).segment<1>(dim),
            dim_var_map.at(dim)
        );
      }
    }
  }
  MatrixXd mid_mult_broad = BlockDiagonalRepeat<double>(mid_mult, 3);
  MatrixXd Q = mid_mult_broad.transpose() * mid_mult_broad;
  VectorXd b = mid_mult_broad.transpose() * des_mid_point;

  prog.AddQuadraticCost(2 * Q, -2 * b, {cx, cy, cz}, true);

  auto solver = solvers::FCCQPSolver();
  solver.InitializeSolver(prog, drake::solvers::SolverOptions(), 0, 0, {});
  auto result = solver.Solve(prog);

  drake::Vector3<Polynomial<double>> polymat;
  for ( int i = 0; i < 3; ++i) {
    Eigen::VectorXd coeffs = result.GetSolution(dim_var_map.at(i));
    polymat(i) = Polynomial<double>(coeffs);
  }

  std::vector<drake::MatrixX<Polynomial<double>>> polys = {polymat};
  std::vector<double> breaks = {0, knots.back()};
  PiecewisePolynomial<double> pp(polys, breaks);

  auto time_scaling = PiecewisePolynomial<double>::FirstOrderHold(
      Eigen::Vector2d(t_start, t_end),
      Eigen::RowVector2d(knots.front(), knots.back())
  );

  PathParameterizedTrajectory<double> path(pp, time_scaling);

  return path;
}

}