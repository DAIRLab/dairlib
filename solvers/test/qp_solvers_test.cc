#include "solvers/qp_data.h"
#include "solvers/qpalm_solver.h"

#include "drake/solvers/osqp_solver.h"

#include <iostream>

namespace dairlib::solvers {

int DoMain() {
  drake::solvers::MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(2);

  prog.AddLinearEqualityConstraint(x(0) + x(1) == 0);
  prog.AddLinearConstraint(x(0) - x(1) >= 2);
  prog.AddQuadraticCost(x(0) * x(0) + x(1) * x(1));

  auto qpdata = QPData::ToQPData(prog);

  drake::solvers::OsqpSolver osqp_solver;
  QPALMSolver qpalm_solver(qpdata);

  auto osqp_result = osqp_solver.Solve(prog);
  Eigen::VectorXd qpalm_solution = qpalm_solver.Solve(qpdata);

  std::cout << "OSQP: " << osqp_result.GetSolution(x).transpose()
            << "\nQPALM: " << qpalm_solution.transpose() << std::endl;

  return 0;
}

}

int main(int argc, char*argv[]) {
  return dairlib::solvers::DoMain();
}