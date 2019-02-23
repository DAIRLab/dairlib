// This is an example code for PiecewisePolynomial

#include <iostream>

#include "systems/framework/output_vector.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Vector4d;

using drake::trajectories::PiecewisePolynomial;

typedef Polynomial<double> PolynomialType;

int main() {
  // Create a piecewise polynomial
  std::vector<double> T_breakpoint = { -3, -2, -1, 0, 1, 2, 3};
  std::vector<MatrixXd> Y(T_breakpoint.size(), MatrixXd::Zero(1, 1));
  Y[0](0, 0) = -1;
  Y[1](0, 0) = -1;
  Y[2](0, 0) = -1;
  Y[3](0, 0) = 0;
  Y[4](0, 0) = 1;
  Y[5](0, 0) = 1;
  Y[6](0, 0) = 1;

  PiecewisePolynomial<double> spline1 =
    PiecewisePolynomial<double>::Pchip(T_breakpoint, Y);

  // This is how you evaluate the polynomial
  std::cout << "\nEvaluating polynomial at different time point:\n";
  double t_evaluationpoint = 0.5;
  std::cout << spline1.value(t_evaluationpoint) << std::endl;

  for (double d = 0; d <= 1; d += 0.1)
    std::cout << spline1.value(d) << " ";
  std::cout << std::endl;
  for (double d = 2; d <= 3; d += 0.1)
    std::cout << spline1.value(d) << " ";
  std::cout << std::endl;

  std::cout << spline1.value(3.1) << std::endl;

  // Create one segment of cubic polynomial
  std::cout << "\nOne segment of cubic polynomial:\n";
  std::vector<double> T2_breakpoint = {0, 1};
  std::vector<MatrixXd> Y2(T2_breakpoint.size(), MatrixXd::Zero(1, 1));
  Y2[0](0, 0) = 1;
  Y2[1](0, 0) = 2;
  PiecewisePolynomial<double> spline2 =
    PiecewisePolynomial<double>::Pchip(T2_breakpoint, Y2, true);
  for (double d = 0; d <= 1; d += 0.1)
    std::cout << spline2.value(d) << " ";
  std::cout << std::endl;

  // If you want to impose a velocity constraints on every knot, you can do the following
  // The code in the class:
  // /**
  //  * Constructs a third order PiecewisePolynomial from `breaks`, `knots` and
  //  * `knots`dot.
  //  * Each segment is fully specified by @knots and @knot_dot at both ends.
  //  * Second derivatives are not continuous.
  //  *
  //  * @throws std::runtime_error if
  //  *    `breaks` and `knots` have different length,
  //  *    `breaks` is not strictly increasing,
  //  *    `breaks` and `knots`dot have different length,
  //  *    `knots` has inconsistent dimensions,
  //  *    `knots_dot` and `knots` have inconsistent dimensions,
  //  *    `breaks` has length smaller than 2.
  //  */
  // static PiecewisePolynomial<T> Cubic(
  //     const std::vector<double>& breaks,
  //     const std::vector<CoefficientMatrix>& knots,
  //     const std::vector<CoefficientMatrix>& knots_dot);
  std::cout << "\nTwo segment of cubic polynomial with velocity constraints:\n";
  std::vector<double> T3_breakpoint = {0, 1, 2};
  std::vector<MatrixXd> Y3(T3_breakpoint.size(), MatrixXd::Zero(1, 1));
  Y3[0](0, 0) = 1;
  Y3[1](0, 0) = 2;
  Y3[2](0, 0) = 3;
  std::vector<MatrixXd> Y3_dot(T3_breakpoint.size(), MatrixXd::Zero(1, 1));
  Y3_dot[0](0, 0) = 0;
  Y3_dot[1](0, 0) = 2;
  Y3_dot[2](0, 0) = 0;
  PiecewisePolynomial<double> spline3 =
    PiecewisePolynomial<double>::Cubic(T3_breakpoint, Y3, Y3_dot);
  for (double d = 0; d <= 2; d += 0.1)
    std::cout << spline3.value(d) << " ";
  std::cout << std::endl;
  // /**
  //  * Constructs a third order PiecewisePolynomial from `breaks` and `knots`.
  //  * The PiecewisePolynomial is constructed such that the interior segments
  //  * have the same value, first and second derivatives at `breaks`.
  //  * `knot_dot_at_start` and `knot_dot_at_end` are used for the first and
  //  * last first derivatives.
  //  *
  //  * @throws std::runtime_error if
  //  *    `breaks` and `knots` have different length,
  //  *    `breaks` is not strictly increasing,
  //  *    `knots` has inconsistent dimensions,
  //  *    `knots_dot_at_start` or `knot_dot_at_end` and `knots` have
  //  *    inconsistent dimensions,
  //  *    `breaks` has length smaller than 2.
  //  */
  // static PiecewisePolynomial<T> Cubic(
  //     const std::vector<double>& breaks,
  //     const std::vector<CoefficientMatrix>& knots,
  //     const CoefficientMatrix& knot_dot_start,
  //     const CoefficientMatrix& knot_dot_end);
  std::cout << "\nTwo segment of cubic polynomial with velocity constraints "
            "only at the start and end:\n";
  Y3[0](0, 0) = 1;
  Y3[1](0, 0) = 2;
  Y3[2](0, 0) = 3;
  MatrixXd Y3_dot_start = MatrixXd::Zero(1, 1);
  MatrixXd Y3_dot_end = MatrixXd::Zero(1, 1);
  Y3_dot_start(0, 0) = 0;
  Y3_dot_end(0, 0) = 0;
  PiecewisePolynomial<double> spline3_se =
    PiecewisePolynomial<double>::Cubic(T3_breakpoint, Y3, Y3_dot_start,
                                       Y3_dot_end);
  for (double d = 0; d <= 2; d += 0.1)
    std::cout << spline3_se.value(d) << " ";
  std::cout << std::endl;

  // You can get derivative from the polynomial
  std::cout << "\nDerivative of the previous polynomial:\n";
  int order = 1;
  PiecewisePolynomial<double> spline3_dot = spline3.derivative(order);
  for (double d = 0; d <= 2; d += 0.1)
    std::cout << spline3_dot.value(d) << " ";
  std::cout << std::endl;

  // You can get the integral of your polynoial
  // 341   PiecewisePolynomial<T> integral(
  // 342       const CoefficientMatrixRef& value_at_start_time) const;
  std::cout << "\nIntegral of the previous polynomial:\n";
  PiecewisePolynomial<double> spline3_int = spline3.integral(0);
  for (double d = 0; d <= 2; d += 0.1)
    std::cout << spline3_int.value(d) << " ";
  std::cout << std::endl;

  // Create 2D two-segment cubic pollynomial
  std::cout << "\nTwo segment of cubic polynomial with velocity constraints "
            "only at the start and end:\n";
  std::vector<MatrixXd> Y3_2D(T3_breakpoint.size(), MatrixXd::Zero(2, 1));
  Y3_2D[0](0, 0) = 1;
  Y3_2D[1](0, 0) = 2;
  Y3_2D[2](0, 0) = 3;
  Y3_2D[0](1, 0) = 2;
  Y3_2D[1](1, 0) = 3;
  Y3_2D[2](1, 0) = 4;
  MatrixXd Y3_2D_dot_start = MatrixXd::Zero(2, 1);
  MatrixXd Y3_2D_dot_end = MatrixXd::Zero(2, 1);
  Y3_2D_dot_start(0, 0) = 0;
  Y3_2D_dot_end(0, 0) = 0;
  Y3_2D_dot_start(1, 0) = 0;
  Y3_2D_dot_end(1, 0) = 0;
  PiecewisePolynomial<double> spline3_2D =
    PiecewisePolynomial<double>::Cubic(T3_breakpoint, Y3_2D, Y3_2D_dot_start,
                                       Y3_2D_dot_end);
  for (double d = 0; d <= 2; d += 0.1)
    std::cout << spline3_2D.value(d)(0, 0) << " ";
  std::cout << std::endl;
  for (double d = 0; d <= 2; d += 0.1)
    std::cout << spline3_2D.value(d)(1, 0) << " ";
  std::cout << std::endl;

  // Integrate a 2D two-segment polynomial
  std::cout << "\nIntegral of the previous polynomial:\n";
  PiecewisePolynomial<double> spline3_2D_int = spline3_2D.integral();
  // PiecewisePolynomial<double> spline3_2D_int = spline3_2D.integral(0);
  // You can use either the above line or no argument
  for (double d = 0; d <= 2; d += 0.1)
    std::cout << spline3_2D_int.value(d)(0, 0) << " ";
  std::cout << std::endl;
  for (double d = 0; d <= 2; d += 0.1)
    std::cout << spline3_2D_int.value(d)(1, 0) << " ";
  std::cout << std::endl;

  // There is a method in piecewise polynomial to get the coefficients,
  // which is ComputeCubicSplineCoeffs(timeSpan, y0, y1, dy0, dy1),
  // but it's an private function.
  // Well, I just found how.
  // First you need to extract a polynomial out of piecewisepolynomial,
  // then use the polynomial's method, GetCoefficients ().


  // This is the ground truth for spline1
  std::cout << "\nCompare spline 1 to ground truth:\n";

  std::vector<Vector4d> coeffs_groundtruth_spline1(T_breakpoint.size());
  coeffs_groundtruth_spline1[0] << -1, 0, 0, 0;
  coeffs_groundtruth_spline1[1] << -1, 0, 0, 0;
  coeffs_groundtruth_spline1[2] << -1, 0, 2, -1;
  coeffs_groundtruth_spline1[3] << 0, 1, 1, -1;
  coeffs_groundtruth_spline1[4] << 1, 0, 0, 0;
  coeffs_groundtruth_spline1[5] << 1, 0, 0, 0;

  Eigen::Matrix<double, 4, 1> coeffs;
  coeffs << 0, 1, 1, -1;  // Test the fourth segment of spline1

  Polynomial<double> polynomial = PolynomialType(coeffs);
  for (double d = 0; d <= 1; d += 0.1)
    std::cout << polynomial.EvaluateUnivariate(d) << " ";
  std::cout << std::endl;
  for (double d = 0; d <= 1; d += 0.1)
    std::cout << spline1.value(d) << " ";
  std::cout << std::endl;



  return 0;
}