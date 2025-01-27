#include <iostream>
#include "ncqp_solver.h"
#include "set_membership_constraint.h"

namespace dairlib::solvers {

using Eigen::VectorXd;

using drake::solvers::Binding;
using drake::solvers::Constraint;
using drake::solvers::MathematicalProgram;

constexpr double kInf = std::numeric_limits<double>::infinity();

namespace {
void AppendRowsToSparse(Eigen::SparseMatrix<double>& A_sparse,
                      const Eigen::MatrixXd& A,
                      const std::vector<int>& col_indices) {
  // Store original number of rows
  int original_rows = A_sparse.rows();

  // Resize the sparse matrix to accommodate new rows
  A_sparse.conservativeResize(original_rows + A.rows(), A_sparse.cols());

  // Insert new elements
  for (int i = 0; i < A.rows(); i++) {
    for (int j = 0; j < col_indices.size(); j++) {
      A_sparse.insert(original_rows + i, col_indices[j]) = A(i, j);
    }
  }
}
}

NCQPSolver::NCQPSolver(){}

// TODO (@Brian-Acosta) warmstart the duals
NCQPSolution NCQPSolver::Solve(
    const MathematicalProgram &qp,
    const std::vector<Binding<Constraint>>& constraints) const {

  Timer global_timer;
  global_timer.tick();

  NCQPSolution sol(qp.num_vars());
  sol.x = qp.initial_guess().hasNaN() ?
      VectorXd::Zero(qp.num_vars()) : qp.initial_guess();
  sol.z = sol.x;
  sol.w = sol.x - sol.z;
  sol.primal_solve_time = 0;
  sol.projection_time = 0;
  sol.total_solve_time = 0;
  sol.is_solved = false;
  sol.slack_res = VectorXd::Zero(qp.num_vars());
  sol.slack_res_norm = kInf;
  sol.n_iter = 0;

  Timer timer;

  std::vector<std::unique_ptr<SetMembershipConstraint>> convex_restrictions;

  // Convert prog into a QPData object
  QPData original_qp = QPData::ToQPData(qp);

  // Add the Augmented lagrangian cost
  QPData primal_step_qp = original_qp;
  for (int i = 0; i < primal_step_qp.num_vars; ++i) {
    primal_step_qp.H.coeffRef(i, i) += params_.rho;
  }

  if (not qp_solver_.IsInitialized()) {
    qp_solver_.InitializeSolver(original_qp, qp.solver_options());
  }

  timer.tick();
  QPResult initial_result;
  qp_solver_.Solve(original_qp, initial_result);
  sol.x = initial_result.x;
  QPResult primal_result;
  sol.primal_solve_time += timer.tock();
  
  // ADMM Iterations
  for (int iter = 0; iter < params_.max_iterations; ++iter) {
    // Primal update - solve the convex QP
    if (iter > 0) {
      VectorXd g_al = -params_.rho * (sol.z - sol.w);
      primal_step_qp.g = original_qp.g + g_al;
      timer.tick();
      qp_solver_.Solve(primal_step_qp, primal_result);
      sol.x = primal_result.x;
      sol.primal_solve_time += timer.tock();
    }

    // Slack update - project to the feasible set
    timer.tick();
    VectorXd d = sol.x + sol.w;
    d = DoProjectionStep(d, qp, constraints);
    double slack_cost = 0.5 * d.dot(original_qp.H * d) + d.dot(original_qp.g) +
          original_qp.c;
    if (slack_cost > sol.slack_cost) {
      sol.fallback = true;
      auto polish_result = Polish(sol.z, original_qp, qp, constraints);
      if (polish_result.success) {
        sol.is_solved = true;
        sol.x = polish_result.x;
      }
      sol.n_iter = iter;
      break;
    } else {
      sol.z = d;
      sol.slack_cost = slack_cost;
    }
    sol.projection_time += timer.tock();

    // Calculate residual between primal and slack variables
    sol.slack_res = sol.x - sol.z;
    sol.slack_res_norm = sol.slack_res.norm();

    // dual update
    sol.w += sol.slack_res;

    ++sol.n_iter;

    // check for convergence
    if (sol.slack_res_norm < params_.tolerance) {
      sol.is_solved = true;
    }
    if (params_.verbose) {
      std::cout << "----- iteration " << iter << " -----\n"
                << sol << "\n" << std::endl;
    }

    if (sol.is_solved) {
      break;
    }
  }

  sol.total_solve_time = global_timer.tock();
  return sol;
}

VectorXd NCQPSolver::DoProjectionStep(
    const Eigen::VectorXd &d,
    const MathematicalProgram& qp,
    const std::vector<Binding<Constraint>> &constraints) const {

  VectorXd out = d;
  for (const auto& binding: constraints) {
    // TODO (@Brian-Acosta) make sure there are no repeated variables here
    const auto& variables = binding.variables();
    const auto& indices = qp.FindDecisionVariableIndices(variables);
    VectorXd y = VectorXd::Zero(variables.size());

    for (int i = 0; i < variables.size(); ++i) {
      y(i) = d(indices[i]);
    }

    VectorXd y_proj = VectorXd::Zero(y.rows());
    auto evaluator = dynamic_cast<SetMembershipConstraint*>(
        binding.evaluator().get());
    DRAKE_DEMAND(evaluator != nullptr);
    evaluator->ProjectToFeasibleSet(y, &y_proj);

    for (int i = 0; i < variables.size(); ++i) {
      out(indices[i]) = y_proj(i);
    }
  }
  return out;
}

QPResult NCQPSolver::Polish(
    const VectorXd &x,
    const QPData &original_qp,
    const MathematicalProgram &qp,
    const std::vector<Binding<Constraint>> &feasibility_constraints) const {

  QPData copy = original_qp;

  for (const auto& binding : feasibility_constraints) {
    const auto& variables = binding.variables();
    const auto& indices = qp.FindDecisionVariableIndices(variables);
    VectorXd y = VectorXd::Zero(variables.size());

    for (int i = 0; i < variables.size(); ++i) {
      y(i) = x(indices[i]);
    }
    const auto [A, lb, ub] = dynamic_cast<SetMembershipConstraint*>(
        binding.evaluator().get())->CalcClosestConvexRestrictionToQP(y);
    copy.num_ineq += A.rows();
    copy.lb.conservativeResize(copy.num_ineq);
    copy.ub.conservativeResize(copy.num_ineq);
    copy.lb.tail(lb.rows()) = lb;
    copy.ub.tail(ub.rows()) = ub;
    AppendRowsToSparse(copy.A, A, indices);
  }
  copy.A.makeCompressed();

  if (params_.verbose) {
    std::cout << "Polish QP:\n" << copy << std::endl;
  }

  OsqpWrapper tmp_solver;
  tmp_solver.InitializeSolver(copy, qp.solver_options());
  QPResult out;
  tmp_solver.Solve(copy, out);

  if (params_.verbose) {
    std::cout << "Polish result:\n" << out << std::endl;
  }
  return out;
}

}