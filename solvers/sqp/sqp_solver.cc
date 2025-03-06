#include <iostream>

#include "solvers/sqp/sqp_solver.h"

namespace dairlib::solvers::sqp {

using Eigen::VectorXd;

SQPSolver::SQPSolver(
    std::function<void (const VectorXd&, solvers::QPData*)> make_qp,
    std::function<double (const VectorXd&)> eval_constraint_viol,
    std::function<double (const VectorXd&)> eval_cost,
    std::function<void (Eigen::VectorXd*)> proj_to_cspace) :
    make_qp_(make_qp), eval_constraint_viol_(eval_constraint_viol),
    eval_cost_(eval_cost), proj_to_config_space_(proj_to_cspace){}

void SQPSolver::DoSQPStep(const VectorXd &x, SQPIterate *sol) {
  DRAKE_DEMAND(sol != nullptr);
  sol->x_init = x;
  make_qp_(x, &qp_);

  // TODO (@Brian-Acosta) add solver options constructor arg
  if (not qp_solver_.IsInitialized()) {
    qp_solver_.InitializeSolver(qp_, drake_solver_options_);
  }
  solvers::QPResult result;
  qp_solver_.Solve(qp_, result);

  if (result.success) {
    sol->dx = result.x;
    DoLineSearch(eval_constraint_viol_, eval_cost_, proj_to_config_space_, qp_,
                 lsparams_, sol);
  } else {
    std::cout << "SQP qp solve failed with status: " << result.solution_result;
  }
}

}