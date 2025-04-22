#include <chrono>
#include <iostream>
#include "solvers/sqp/nc_sqp_solver.h"


namespace dairlib::solvers::sqp {

using Eigen::VectorXd;

using SetMembershipConstraints = solvers::NCQPSolver::SetMembershipConstraints;
using drake::solvers::SolverOptions;

NCSQPSolver::NCSQPSolver(
    std::function<void (const VectorXd&, solvers::QPData*)> make_qp,
    std::function<double (const VectorXd&)> eval_constraint_viol,
    std::function<double (const VectorXd&)> eval_cost,
    std::function<void (Eigen::VectorXd*)> proj_to_cspace,
    std::function<SetMembershipConstraints (const Eigen::VectorXd&)> sm_const,
    const std::string& ncqp_solver_options_yaml) :
    make_qp_(make_qp), eval_constraint_viol_(eval_constraint_viol),
    eval_cost_(eval_cost), proj_to_config_space_(proj_to_cspace),
    get_sm_constraints_(sm_const), ncqp_solver_(NCQPSolver::Make(ncqp_solver_options_yaml)) {}

void NCSQPSolver::DoSQPStep(const Eigen::VectorXd &x,
                            SQPIterate *sol) {
  DRAKE_DEMAND(sol != nullptr);

  auto start = std::chrono::high_resolution_clock::now();
  sol->x_init = x;
  make_qp_(x, &qp_);

  auto set_membership_constraints = get_sm_constraints_(x);
  auto post_setup = std::chrono::high_resolution_clock::now();

  solvers::QPResult result;
  ncqp_solver_.Solve(qp_, result, set_membership_constraints);

  if (result.success or result.solution_result ==
      drake::solvers::kIterationLimit) {
    sol->dx = result.x;
    DoLineSearch(eval_constraint_viol_, eval_cost_, proj_to_config_space_, qp_,
                 lsparams_, sol);
  } else {
    std::cout << "SQP qp solve failed with status: " << result.solution_result << "\n";
    sol->step_size = 0.0;
    sol->line_search_time = 0.0;
    sol->accepted = false;
  }
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> setup_time = post_setup - start;
  std::chrono::duration<double> total_time = end - start;
  sol->setup_time = setup_time.count();
  sol->solve_time = result.run_time;
  sol->total_step_time = total_time.count();
}

}