#include <iostream>
#include "ncqp_solver.h"
#include "set_membership_constraint.h"

namespace dairlib::solvers {

using Eigen::VectorXd;

using drake::solvers::Binding;
using drake::solvers::Constraint;
using drake::solvers::MathematicalProgram;

constexpr double kInf = std::numeric_limits<double>::infinity();

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
  sol.n_iter = params_.max_iterations;

  Timer timer;

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
    if (not sol.fallback) {
      d = DoProjectionStep(d, qp, constraints);
      double dual_cost = d.dot(original_qp.H * d) + d.dot(original_qp.g) +
          original_qp.c;
      if (dual_cost > sol.dual_cost) {
        sol.fallback = true;
      } else {
        sol.z = d;
        sol.dual_cost = dual_cost;
      }
    }
    if (sol.fallback) {
      break;
    }
    sol.projection_time += timer.tock();


    // Calculate residual between primal and slack variables
    sol.slack_res = sol.x - sol.z;
    sol.slack_res_norm = sol.slack_res.norm();

    // dual update
    sol.w += sol.slack_res;

    // check for convergence
    if (sol.slack_res_norm < params_.tolerance) {
      sol.n_iter = iter;
      sol.is_solved = true;
      break;
    }

    if (params_.verbose) {
      std::cout << "----- iteration " << iter << " -----\n"
                << sol << "\n" << std::endl;
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

}