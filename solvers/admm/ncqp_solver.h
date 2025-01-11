#pragma once

#include "drake/solvers/osqp_solver.h"
#include "nonconvex_constraint.h"

namespace dairlib::solvers {


struct ADMMParams {
  double rho{1.0};           // Penalty parameter
  int max_iterations{1000};  // Maximum number of iterations
  double tolerance{1e-4};    // Convergence tolerance
};

struct NCQPSolution {
  Eigen::VectorXd x;         // Primal variables
  Eigen::VectorXd z;         // Auxiliary variables
  Eigen::VectorXd w;         // Dual variables
  bool is_solved{false};     // Solution status

  // Constructor to initialize vectors of the right size
  explicit NCQPSolution(int size)
      : x(Eigen::VectorXd::Zero(size)),
        z(Eigen::VectorXd::Zero(size)),
        w(Eigen::VectorXd::Zero(size)) {}
};



/*!
 * A generic solver for optimization problems of the form
 *      minimize   f(x) + Ic(x) + Inc(x)
 *        x
 *
 *  where f is quadratic and positive semidefinite, Ic(x) is the 0-inf
 *  indicator function for linear constraints, and Inc(x) is the 0-inf
 *  indicator function for the general nonconvex constraints.
 *
 *  We use the consensus formulation
 *
 *      minimize   f(x) + Ic(x) + Inc(z)
 *        x, z
 *      subject to x = z
 *
 *  to solve the problem via ADMM
 *
 */
class NCQPSolver {
 public:
  NCQPSolution Solve(
      const drake::solvers::MathematicalProgram& qp,
      const std::vector<drake::solvers::Binding<drake::solvers::Constraint>>&
      feasibility_constraints) const;

 private:
  void DualUpdate();
  bool CheckConvergence();

  ADMMParams params_;

};

}
