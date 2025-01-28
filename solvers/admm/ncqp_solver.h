#pragma once

#include "drake/solvers/mathematical_program.h"
#include "solvers/qp_data.h"
#include "solvers/osqp_wrapper.h"
#include "set_membership_constraint.h"

namespace dairlib::solvers {

struct ADMMParams {
  double rho{0.1};         // Penalty parameter
  int max_iterations{8};  // Maximum number of iterations
  double tolerance{1e-3};    // Convergence tolerance
  bool verbose = false;
};

struct NCQPSolution {
  Eigen::VectorXd x;         // Primal variables
  Eigen::VectorXd z;         // Auxiliary variables
  Eigen::VectorXd w;         // Dual variables
  Eigen::VectorXd slack_res;
  bool is_solved{false};     // Solution status
  double slack_res_norm;
  double total_solve_time;
  double projection_time;
  double primal_solve_time;
  double slack_cost =  std::numeric_limits<double>::infinity();
  bool fallback;
  int n_iter;

  // Constructor to initialize vectors of the right size
  explicit NCQPSolution(int size)
      : x(Eigen::VectorXd::Zero(size)),
        z(Eigen::VectorXd::Zero(size)),
        w(Eigen::VectorXd::Zero(size)) {}

  friend std::ostream& operator<<(std::ostream& os, const NCQPSolution& sol) {
    os << "NCQPSolution:\n"
       << "  x (Primal variables): " << sol.x.transpose() << "\n"
       << "  z (Auxiliary variables): " << sol.z.transpose() << "\n"
       << "  w (Dual variables): " << sol.w.transpose() << "\n"
       << "  slack_res (Slack residual): " << sol.slack_res.transpose() << "\n"
       << "  Slack cost: " << sol.slack_cost << "\n"
       << "  Solved: " << (sol.is_solved ? "Yes" : "No") << "\n"
       << "  Slack Residual Norm: " << sol.slack_res_norm << "\n"
       << "  Total Solve Time: " << sol.total_solve_time << " s\n"
       << "  Projection Time: " << sol.projection_time << " s\n"
       << "  Primal Solve Time: " << sol.primal_solve_time << " s\n"
       << "  Iterations: " << sol.n_iter;
    return os;
  }
};

/*!
 * A generic solver for optimization problems of the form
 *      minimize   f(x) + Ilin(x) + Inc(x)
 *        x
 *
 *  where f is quadratic and positive semidefinite, Ilin(x) is the 0-inf
 *  indicator function for linear constraints, and Inc(x) is the 0-inf
 *  indicator function for the general nonconvex constraints.
 *
 *  We use the consensus formulation
 *
 *      minimize   f(x) + Ic(x) + Inc(z)
 *        x, z
 *      subject to x = z
 *
 *  to solve the problem via ADMM.
 *
 */
class NCQPSolver {
 public:
  explicit NCQPSolver();

  /*!
   * Solve the optimization problem
   * @param qp the convex qp part of the problem
   * @param feasibility_constraints the general constraint part of the problem
   * @return the solution
   *
   * TODO (@Brian-Acosta) make another signature which accepts a solution as
   * a warm-start
   *
   */
  QPResult Solve(
      const drake::solvers::MathematicalProgram& qp,
      const std::vector<drake::solvers::Binding<drake::solvers::Constraint>>&
      feasibility_constraints) const;

 private:

  Eigen::VectorXd DoProjectionStep(
      const Eigen::VectorXd& d,
      const drake::solvers::MathematicalProgram& qp,
      const std::vector<drake::solvers::Binding<drake::solvers::Constraint>>&
      feasibility_constraints) const;

  QPResult Polish(const NCQPSolution& sol,
                  const QPData& original_qp,
                  const QPResult& most_recent_result,
                  const drake::solvers::MathematicalProgram& qp,
                  const std::vector<drake::solvers::Binding<drake::solvers::Constraint>>&
                         feasibility_constraints) const;

  class Timer {
   public:
    void tick() {
      start = std::chrono::high_resolution_clock::now();
    }
    double tock() const {
      auto end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed = end - start;
      return elapsed.count();
    };

   private:
    std::chrono::high_resolution_clock::time_point start;
  };

  ADMMParams params_;
  mutable OsqpWrapper qp_solver_;

};

}
