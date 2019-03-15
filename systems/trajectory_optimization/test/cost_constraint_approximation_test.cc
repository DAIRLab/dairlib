#include <Eigen/Dense>
#include <memory>
#include <utility>
#include <gtest/gtest.h>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"
#include "systems/trajectory_optimization/dircon_util.h"

namespace dairlib {
namespace systems {
namespace {

using drake::solvers::MathematicalProgram;
using std::cout;
using std::endl;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

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

  VectorXd w_sol = VectorXd::Zero(2);

  MatrixXd H_a = MatrixXd::Zero(2, 2);
  VectorXd b_a = VectorXd::Zero(2);
  VectorXd c_a = VectorXd::Zero(1);
  MatrixXd A_a = MatrixXd::Zero(2, 2);
  VectorXd lb_a = VectorXd::Zero(2);
  VectorXd ub_a = VectorXd::Zero(2);
  VectorXd y_a = VectorXd::Zero(2);

  // An example
  H_o <<  1.36075, 0.354964,
       0.354964,  1.19376;
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
  dairlib::systems::trajectory_optimization::linearizeConstraints(
      &quadprog, w_sol, y_a, A_a, lb_a, ub_a);
  double c_doub = dairlib::systems::trajectory_optimization::secondOrderCost(
                    &quadprog, w_sol, H_a, b_a);
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
    ASSERT_TRUE( (original_cost_value - approx_cost_value).norm() < 1e-4 );
  }

  // Test A, lb, and ub
  ASSERT_EQ(A_o, A_a);
  ASSERT_EQ(lb_o, lb_a);
  ASSERT_EQ(ub_o, ub_a);

}

}  // namespace
}  // namespace systems
}  // namespace dairlib



int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
