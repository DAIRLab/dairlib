#include <iostream>
#include <string>
#include "math.h"
#include <Eigen/Dense>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"
#include "solvers/optimization_utils.h"

#include "drake/common/trajectories/piecewise_polynomial.h"

using std::cout;
using std::endl;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using drake::solvers::MathematicalProgram;
using drake::trajectories::PiecewisePolynomial;

using drake::MatrixX;
using drake::VectorX;
using drake::symbolic::Expression;

int main() {

  // Create variables
  MathematicalProgram quadprog;
  auto w = quadprog.NewContinuousVariables(7, "w");

  // Create a piecewise polynomial
  std::vector<double> T_breakpoint = { -3, -2, -1, 0, 1, 2, 3};
  std::vector<MatrixX<Expression>> Y(T_breakpoint.size(), MatrixXd::Zero(1, 1));
  Y[0](0, 0) = w(0);
  Y[1](0, 0) = w(1);
  Y[2](0, 0) = w(2);
  Y[3](0, 0) = w(3);
  Y[4](0, 0) = w(4);
  Y[5](0, 0) = w(5);
  Y[6](0, 0) = w(6);

  // PiecewisePolynomial<Expression> spline =
  //   PiecewisePolynomial<Expression>::Pchip(T_breakpoint, Y);

  std::vector<MatrixX<Expression>> Y_dot(T_breakpoint.size(), MatrixXd::Zero(1, 1));
  Y_dot[0](0, 0) = 0;
  Y_dot[1](0, 0) = 0;
  Y_dot[2](0, 0) = 0;
  // PiecewisePolynomial<Expression> spline =
  //   PiecewisePolynomial<Expression>::Cubic(T_breakpoint, Y, Y_dot);

  // The above line cannnot be compiled.
  // The compiler complained that 'drake::symbolic::Expression' is not literal
  // because it has a non-trivial destructor.
  // So it seems that we cannot create symbolic trajectory.

  return 0;
}
