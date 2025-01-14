#include "ncqp_solver.h"

namespace dairlib::solvers {

using Eigen::VectorXd;

using drake::solvers::Binding;
using drake::solvers::Constraint;
using drake::solvers::MathematicalProgram;

NCQPSolver::NCQPSolver(int num_vars, int num_lin_constraints) :
    qp_solver_(num_vars, num_lin_constraints) {

}

// TODO (@Brian-Acosta) warmstart the duals and warmstart QPALM
NCQPSolution NCQPSolver::Solve(
    const MathematicalProgram &qp,
    const std::vector<Binding<Constraint>>& constraints) const {

  NCQPSolution sol(qp.num_vars());
  sol.x = qp.initial_guess();
  sol.z = qp.initial_guess();

  // Convert prog into a QPData object
  QPData original_qp = QPData::ToQPData(qp);

  // Add the Augmented lagrangian cost
  QPData primal_step_qp = original_qp;
  for (int i = 0; i < primal_step_qp.num_vars; ++i) {
    primal_step_qp.H.coeffRef(i,i) += params_.rho;
  }

  // ADMM Iterations
  for (int iter = 0; iter < params_.max_iterations; ++iter) {
    // Primal update - solve the convex QP
    VectorXd g_al = -params_.rho * (sol.z - sol.w);
    primal_step_qp.g = original_qp.g + g_al;
    sol.x = qp_solver_.Solve(primal_step_qp);

    // Slack update - project to the feasible set
    for (const auto& binding: constraints) {
      const auto& variables = binding.variables();
      const auto& indices = qp.FindDecisionVariableIndices(variables);


    }

  }


}


}