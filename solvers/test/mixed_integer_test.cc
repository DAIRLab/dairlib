#include <iostream>
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/branch_and_bound.h"
#include "solvers/optimization_utils.h"

namespace dairlib {
namespace solvers {

using drake::solvers::MathematicalProgram;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::RowVector2d;
using drake::Vector1d;

void test_mip_equality() {
  RowVector2d a1(1, 0);
  RowVector2d a2(0, 1);
  Vector1d b(0);
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(2);
  auto z = prog.NewBinaryVariables(2);
  auto junk1 = LinearBigMEqualityConstraint(prog, a1, b, 5.0, x, z(0));
  auto junk2 = LinearBigMEqualityConstraint(prog, a2, b, 5.0, x, z(1));
  prog.AddLinearEqualityConstraint(RowVector2d::Ones(), Vector1d::Ones(), z);
  prog.AddQuadraticErrorCost(Eigen::Matrix2d::Identity(), Vector2d(0.5, 0.75), x);

  auto bnb = drake::solvers::MixedIntegerBranchAndBound(
      prog, drake::solvers::OsqpSolver::id());
  auto result = bnb.Solve();
  auto x_result = bnb.GetSolution(x);
  auto z_result = bnb.GetSolution(z);
  std::cout << "Finished with code " << result << "\n";
  std::cout << "x solution " << x_result.transpose() << "\n";
  std::cout << "z solution " << z_result.transpose() << "\n";
}

void test_mip_inequality() {
  RowVector2d a1(1, 0);
  RowVector2d a2(0, 1);
  RowVector2d a3 = -a1;
  RowVector2d a4 = -a2;

  MatrixXd A = MatrixXd::Zero(4,2);
  A.row(0) = a1;
  A.row(1) = a2;
  A.row(2) = a3;
  A.row(3) = a4;

  VectorXd b0 = VectorXd::Ones(4);
  b0.tail<2>() *= -1;

  VectorXd b1 = VectorXd::Zero(4);
  Vector2d o(-4, 0);
  for (int i = 0; i < 4; i++) {
    b1(i) = A.row(i).dot(o);
  }

  Vector1d b(0);
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(2);
  auto z = prog.NewBinaryVariables(4);
  auto junk1 = LinearBigMEqualityConstraint(prog, a1, b, 5.0, x, z(0));
  auto junk2 = LinearBigMEqualityConstraint(prog, a2, b, 5.0, x, z(1));
  auto junk3 = LinearBigMConstraint(prog, A, b0, 20, x, z(2));
  auto junk4 = LinearBigMConstraint(prog, A, b1, 20, x, z(3));

  prog.AddLinearEqualityConstraint(RowVector2d::Ones(), Vector1d::Ones(), z.head<2>());
  prog.AddLinearEqualityConstraint(RowVector2d::Ones(), Vector1d::Ones(), z.tail<2>());
  prog.AddQuadraticErrorCost(Eigen::Matrix2d::Identity(), Vector2d(0.5, 0.75), x);

  auto bnb = drake::solvers::MixedIntegerBranchAndBound(
      prog, drake::solvers::OsqpSolver::id());
  auto result = bnb.Solve();
  auto x_result = bnb.GetSolution(x);
  auto z_result = bnb.GetSolution(z);
  std::cout << "Finished with code " << result << "\n";
  std::cout << "x solution " << x_result.transpose() << "\n";
  std::cout << "z solution " << z_result.transpose() << "\n";
}

void test_mip_main() {
  test_mip_equality();
  test_mip_inequality();
}
}
}

int main(int argc, char* argv[]) {
  dairlib::solvers::test_mip_main();
  return 0;
}
