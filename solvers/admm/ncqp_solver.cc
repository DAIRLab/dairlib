#include <iostream>
#include "ncqp_solver.h"
#include "set_membership_constraint.h"
#include "drake/solvers/osqp_solver.h"

namespace dairlib::solvers {

using std::cout;
using std::endl;

using Eigen::VectorXd;

using drake::solvers::Binding;
using drake::solvers::Constraint;
using drake::solvers::OsqpSolver;
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

inline double eval_cost(const Eigen::VectorXd& v, const QPData& qp) {
  return 0.5 * v.dot(qp.H * v) + v.dot(qp.g) + qp.c;
}

NCQPSolution initialize_sol(const MathematicalProgram& qp_prog) {
  NCQPSolution sol(qp_prog.num_vars());
  sol.x = qp_prog.initial_guess().hasNaN() ?
          VectorXd::Zero(qp_prog.num_vars()) : qp_prog.initial_guess();
  sol.z = sol.x;
  sol.w = sol.x - sol.z;
  sol.primal_solve_time = 0;
  sol.projection_time = 0;
  sol.total_solve_time = 0;
  sol.is_solved = false;
  sol.slack_res = VectorXd::Zero(qp_prog.num_vars());
  sol.slack_res_norm = kInf;
  sol.n_iter = 0;
  return sol;
}

NCQPSolution initialize_sol(const QPData& qp) {
  NCQPSolution sol(qp.num_vars);
  sol.x = VectorXd::Zero(qp.num_vars);
  sol.z = sol.x;
  sol.w = sol.x - sol.z;
  sol.primal_solve_time = 0;
  sol.projection_time = 0;
  sol.total_solve_time = 0;
  sol.is_solved = false;
  sol.slack_res = VectorXd::Zero(qp.num_vars);
  sol.slack_res_norm = kInf;
  sol.n_iter = 0;
  return sol;
}

}

NCQPSolver::NCQPSolver(){}

std::pair<QPData, QPData> NCQPSolver::InitializeQPData(const QPData& qp) const {
  QPData cvx_qp = qp;
  QPData al_qp = cvx_qp;

  // Add the Augmented lagrangian cost
  // We also add zeros to the original QP to keep the same sparsity pattern
  for (int i = 0; i < al_qp.num_vars; ++i) {
    al_qp.H.coeffRef(i, i) += params_.rho;
    cvx_qp.H.coeffRef(i, i) += 0.0;
    al_qp.H.makeCompressed();
    cvx_qp.H.makeCompressed();
  }

  // TODO (@Brian-Acosta) set solver options somehow
  if (not qp_solver_.IsInitialized()) {
    qp_solver_.InitializeSolver(cvx_qp, drake::solvers::SolverOptions());
  }
  return {cvx_qp, al_qp};
}

void NCQPSolver::SolveALQP(
    const QPData &cvx_qp, QPData &al_qp,
    QPResult* al_result, NCQPSolution *sol, int iter) const {
  VectorXd g_al = -params_.rho * (sol->z - sol->w);
  al_qp.g = cvx_qp.g + g_al;

  qp_solver_.Solve(al_qp, *al_result, iter <= 1);
  sol->x = al_result->x;

  if (params_.verbose) {
    cout << "\nprimal qp result:\n" << al_result << endl;
  }
}

NCQPSolver::SetMembershipConstraints
NCQPSolver::ExtractSetMembershipConstraints(
    const drake::solvers::MathematicalProgram &prog,
    const std::vector<Binding<Constraint>> &set_membership_bindings) {

  SetMembershipConstraints out{};
  for (const auto& binding: set_membership_bindings) {
    const auto& variables = binding.variables();
    out.first.push_back(prog.FindDecisionVariableIndices(variables));
    out.second.push_back(
        dynamic_cast<SetMembershipConstraint*>(binding.evaluator().get()));
  }
  return out;
}

QPResult NCQPSolver::Solve(const MathematicalProgram &qp,
                           const std::vector<Binding<Constraint>> &feasibility_constraints) const {
  QPResult result;
  SetMembershipConstraints constraints = ExtractSetMembershipConstraints(
      qp, feasibility_constraints);
  Solve(QPData::ToQPData(qp), result, constraints);
  return result;
}

// TODO (@Brian-Acosta) warmstart the duals
// TODO (@Brian-Acosta) result a mathematicalprogram result with appropriate
//  solution details
void NCQPSolver::Solve(const QPData &qp, QPResult &result,
                       const SetMembershipConstraints& constraints) const {

  Timer global_timer;

  NCQPSolution sol = initialize_sol(qp);
  auto [cvx_qp, al_qp] = InitializeQPData(qp);

  QPResult init_result;
  QPResult primal_result;
  Timer timer;

  qp_solver_.Solve(cvx_qp, init_result);
  sol.x = init_result.x;
  sol.primal_solve_time += timer.tock();
  
  // ADMM Iterations
  for (int iter = 0; iter < params_.max_iterations; ++iter) {
    // Primal update - solve the convex QP
    if (iter > 0) {
      timer.tick();
      SolveALQP(cvx_qp, al_qp, &primal_result, &sol, iter);
      sol.primal_solve_time += timer.tock();
    }

    // Slack update - project to the feasible set
    timer.tick();
    VectorXd d = sol.x + sol.w;
    d = DoProjectionStep(d, constraints);
    double slack_cost = eval_cost(d, cvx_qp);

    if (slack_cost > sol.slack_cost) {
      QPResult out;
      switch (params_.polish_type) {
        case kProject:
          out = ProjectionPolish(sol, constraints);
          out.solution_result = drake::solvers::kSolutionFound;
          break;
        case kConvexRestriction:
          out = QPPolish(
              sol, cvx_qp, primal_result, constraints);
      }
      out.run_time = global_timer.tock();
      result = out;
      return;
    } else {
      sol.z = d;
      sol.slack_cost = slack_cost;
    }
    sol.projection_time += timer.tock();

    // Dual update and increment sol iteration
    sol.slack_res = sol.x - sol.z;
    sol.slack_res_norm = sol.slack_res.norm();
    sol.w += sol.slack_res;
    sol.n_iter = iter;

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

  QPResult out;
  switch (params_.polish_type) {
    case kProject:
      out = ProjectionPolish(sol, constraints);
      out.solution_result = drake::solvers::kSolutionFound;
      break;
    case kConvexRestriction:
      const QPResult& polish_warmstarter = sol.n_iter > 1 ? primal_result : init_result;
      out = QPPolish(sol, cvx_qp, polish_warmstarter, constraints);
  }
  out.run_time = global_timer.tock();
  result = out;
}

VectorXd NCQPSolver::DoProjectionStep(
    const Eigen::VectorXd &d,
    const SetMembershipConstraints& constraints) const {

  const auto& constraint_indices = constraints.first;
  const auto& evaluators = constraints.second;

  VectorXd out = d;
  for (size_t i = 0; i < constraint_indices.size(); ++i) {

    DRAKE_DEMAND(evaluators[i] != nullptr);

    // TODO (@Brian-Acosta) make sure there are no repeated variables here
    const auto& indices = constraint_indices.at(i);
    VectorXd y = VectorXd::Zero(indices.size());

    for (int j = 0; j < indices.size(); ++j) {
      y(j) = d(indices[j]);
    }

    VectorXd y_proj = VectorXd::Zero(y.rows());
    evaluators[i]->ProjectToFeasibleSet(y, &y_proj);

    for (int j = 0; j < indices.size(); ++j) {
      out(indices[j]) = y_proj(j);
    }
  }
  return out;
}

QPResult NCQPSolver::ProjectionPolish(
    const NCQPSolution& sol,
    const NCQPSolver::SetMembershipConstraints& nc_constraints) const {
  QPResult out;
  out.x = DoProjectionStep(sol.x, nc_constraints);
  return out;
}

QPResult NCQPSolver::QPPolish(
    const NCQPSolution& sol,
    const QPData &cvx_qp,
    const QPResult& most_recent_result,
    const NCQPSolver::SetMembershipConstraints& nc_constraints) const {

  const VectorXd& warm_start_primal = sol.x;
  VectorXd warm_start_dual = most_recent_result.y;

  QPData copy = cvx_qp;

  const auto& indices = nc_constraints.first;
  const auto& evaluators = nc_constraints.second;

  for (size_t i = 0; i < indices.size(); ++i) {
    DRAKE_DEMAND(evaluators.at(i) != nullptr);
    VectorXd y = VectorXd::Zero(indices.at(i).size());
    for (size_t j = 0; j < indices.at(i).size(); ++j) {
      y(j) = sol.x(indices.at(i).at(j));
    }
    const auto [A, lb, ub] = evaluators[i]->CalcClosestConvexRestrictionToQP(y);
    copy.num_ineq += A.rows();
    copy.lb.conservativeResize(copy.num_ineq);
    copy.ub.conservativeResize(copy.num_ineq);
    warm_start_dual.conservativeResize(copy.num_ineq);
    copy.lb.tail(lb.rows()) = lb;
    copy.ub.tail(ub.rows()) = ub;
    warm_start_dual.tail(A.rows()) = VectorXd::Zero(A.rows());
    AppendRowsToSparse(copy.A, A, indices.at(i));
  }
  copy.A.makeCompressed();

  OsqpWrapper tmp_solver;
  tmp_solver.InitializeSolver(copy, drake::solvers::SolverOptions());
  tmp_solver.WarmStart(warm_start_primal, warm_start_dual);
  QPResult out;
  tmp_solver.Solve(copy, out);

  if (params_.verbose) {
    std::cout << "Polish result:\n" << out << std::endl;
  }
  return out;
}

}