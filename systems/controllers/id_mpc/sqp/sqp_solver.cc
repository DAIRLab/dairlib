#include "sqp_solver.h"
#include <iostream>

namespace dairlib::systems::controllers::id_mpc {

using Eigen::VectorXd;

SQPSolver::SQPSolver(
    int n, int m,
    std::function<void (const VectorXd&, solvers::QPData*)> make_qp,
    std::function<double (const VectorXd&)> eval_constraint_viol,
    std::function<double (const VectorXd&)> eval_cost,
    std::function<void (Eigen::VectorXd*)> proj_to_cspace) :
    n_(n), m_(m),
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
    LineSearch(*sol);
  } else {
    std::cout << "SQP qp solve failed with status: " << result.solution_result;
  }
}

void SQPSolver::LineSearch(SQPIterate &sol) {
  DoLineSearch(eval_constraint_viol_, eval_cost_, proj_to_config_space_, qp_,
               lsparams_, &sol);
}

SQPIterate SQPSolver::AllocateIterate() const {
  SQPIterate ret;
  ret.x_init = VectorXd::Zero(n_);
  ret.dx = VectorXd::Zero(n_);
  ret.x_sol = VectorXd::Zero(n_);
  ret.constraint_viol = 0;
  ret.cost = 0;
  return ret;
}

}