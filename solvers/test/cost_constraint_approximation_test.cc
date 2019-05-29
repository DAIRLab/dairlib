#include <Eigen/Dense>
#include <memory>
#include <utility>
#include <gtest/gtest.h>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "solvers/optimization_utils.h"

namespace dairlib {
namespace solvers {
namespace {

using drake::solvers::MathematicalProgram;
using drake::CompareMatrices;
using std::cout;
using std::endl;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix;

typedef Matrix< double, 1, 1 >  Vector1d;

class CostConstraintApproximationTest : public ::testing::Test {
};

TEST_F(CostConstraintApproximationTest, QPTest) {
  // Initialize the matrix size
  MatrixXd H_o = MatrixXd::Zero(2, 2);
  VectorXd b_o = VectorXd::Zero(2);
  VectorXd c_o = VectorXd::Zero(1);
  MatrixXd A_o = MatrixXd::Zero(2, 2);
  VectorXd lb_o = VectorXd::Zero(2);
  VectorXd ub_o = VectorXd::Zero(2);

  VectorXd w_sol;

  MatrixXd H_a;
  VectorXd b_a;
  Vector1d c_a;
  MatrixXd A_a;
  VectorXd lb_a;
  VectorXd ub_a;
  VectorXd y_a;

  // An example
  H_o <<  1.36075,  0.354964,
          0.354964, 1.19376;
  b_o = VectorXd::Ones(2);
  c_o << 0;
  A_o <<  0.411647, -0.164777,
         -0.302449, 0.26823;
  lb_o = 0.5 * VectorXd::Ones(2);
  ub_o = VectorXd::Ones(2);

  // solve optimization problem
  MathematicalProgram quadprog;
  auto w = quadprog.NewContinuousVariables(2, "w");
  // intentionally split the following constraint into two constraint bindings
  quadprog.AddLinearConstraint(A_o.row(0),
                               lb_o(0),
                               ub_o(0),
                               w);
  quadprog.AddLinearConstraint(A_o.row(1),
                               lb_o(1),
                               ub_o(1),
                               w);
  // Adds a cost term of the form 0.5*x'*H_o*x + b_o'x + c
  // intentionally split the following cost into two cost bindings
  quadprog.AddQuadraticCost(H_o * 1 / 3, b_o * 1 / 3, w);
  quadprog.AddQuadraticCost(H_o * 2 / 3, b_o * 2 / 3, w);
  const auto result = Solve(quadprog);
  w_sol = result.GetSolution(quadprog.decision_variables());

  // Approximation
  LinearizeConstraints(quadprog, w_sol, &y_a, &A_a, &lb_a, &ub_a);
  double c_doub = SecondOrderCost(quadprog, w_sol, &H_a, &b_a);
  c_a << c_doub;

  // Test the cost function
  std::vector<VectorXd> w_sample;
  w_sample.push_back(VectorXd::Random(2));
  w_sample.push_back(VectorXd::Random(2));
  w_sample.push_back(10 * VectorXd::Random(2));

  VectorXd original_cost_value;
  VectorXd approx_cost_value;
  for (int i = 0; i < 3; i++) {
    original_cost_value = 0.5 * w_sample[i].transpose() * H_o * w_sample[i] +
                          b_o.transpose() * w_sample[i] + c_o;
    approx_cost_value =
        0.5 * (w_sample[i] - w_sol).transpose() * H_a * (w_sample[i] - w_sol)
        + b_a.transpose() * (w_sample[i] - w_sol) + c_a;
    EXPECT_TRUE(CompareMatrices(original_cost_value, approx_cost_value, 1e-4));
  }

  // Test A, lb, and ub
  EXPECT_EQ(A_o, A_a);
  EXPECT_EQ(lb_o, lb_a);
  EXPECT_EQ(ub_o, ub_a);
}

}  // namespace
}  // namespace solvers
}  // namespace dairlib



int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
