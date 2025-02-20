#pragma once

#include "drake/solvers/mathematical_program.h"
#include "solvers/qp_data.h"
#include "solvers/osqp_wrapper.h"
#include "solvers/qpalm_wrapper.h"
#include "set_membership_constraint.h"

namespace dairlib::solvers {

enum PolishType {
  kProject = 0,
  kConvexRestriction
};

struct ADMMParams {
  double rho{0.1};         // Penalty parameter
  int max_iterations{5};  // Maximum number of outer iterations
  double tolerance{1e-3};    // Convergence tolerance
  bool verbose = false;
  PolishType polish_type = kConvexRestriction;
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
 *      minimize   f(x) + Ilin(x) + Ism(x)
 *        x
 *
 *  where f is quadratic and positive semidefinite, Ilin(x) is the 0-inf
 *  indicator function for linear constraints, and Ism(x) is the 0-inf
 *  indicator function for the general set membership constraints.
 *
 *  We use the consensus formulation
 *
 *      minimize   f(x) + Ic(x) + Inc(z)
 *        x, z
 *      subject to x = z
 *
 *  to solve the problem via ADMM.
 *
 *  NB: Setting the polish type to kConvexRestriction assumes that for any
 *  possible convex restriction, each set membership constraint is always
 *  converted into a set of linear inequalities with a consistent number of
 *  rows. This allows the osqp workspace for the polishing step to be
 *  warm-started and re-used.
 *
 */
class NCQPSolver {
 public:

  typedef std::pair<std::vector<std::vector<int>>,
                    std::vector<SetMembershipConstraint*>>
                    SetMembershipConstraints;

  explicit NCQPSolver(const drake::solvers::SolverOptions& inner_qp_options,
                      const drake::solvers::SolverOptions& polish_qp_options);

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

  void Solve(const QPData &qp, QPResult &result,
             const SetMembershipConstraints& constraints) const;

  static SetMembershipConstraints ExtractSetMembershipConstraints(
      const drake::solvers::MathematicalProgram& prog,
      const std::vector<drake::solvers::Binding<drake::solvers::Constraint>>&
      set_membership_bindings);

 private:

  std::pair<QPData, QPData> InitializeQPData(const QPData& qp) const;

  void SolveALQP(const QPData& cvx_qp, QPData& al_qp,
                 QPResult* al_result, NCQPSolution* sol, int iter) const;

  Eigen::VectorXd DoProjectionStep(
      const Eigen::VectorXd& d,
      const SetMembershipConstraints& nc_constraints) const;

  QPResult QPPolish(
      const NCQPSolution& sol, const QPData& cvx_qp,
      const QPResult& most_recent_result,
      const SetMembershipConstraints& nc_constraints) const;

  QPResult ProjectionPolish(
      const NCQPSolution& sol,
      const SetMembershipConstraints& nc_constraints) const;

  class Timer {
   public:
    Timer() { this->tick(); }
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
  mutable OsqpWrapper polish_solver_;

  drake::solvers::SolverOptions inner_qp_options_{};
  drake::solvers::SolverOptions polish_qp_options_{};

};

}
