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
  sol->x_init = x;
  make_qp_(x, &qp_);

  solvers::QPResult result;

  auto set_membership_constraints = get_sm_constraints_(x);
  ncqp_solver_.Solve(qp_, result, set_membership_constraints);

  if (result.success) {
    sol->dx = result.x;
    DoLineSearch(eval_constraint_viol_, eval_cost_, proj_to_config_space_, qp_,
                 lsparams_, sol);
  } else {
    std::cout << "SQP qp solve failed with status: " << result.solution_result;
  }
}

}