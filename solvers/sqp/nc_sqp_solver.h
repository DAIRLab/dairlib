#pragma once

#include "solvers/sqp/sqp_utils.h"
#include "solvers/admm/ncqp_solver.h"

namespace dairlib::solvers::sqp {

class NCSQPSolver {

 public:
  NCSQPSolver(
      std::function<void (const Eigen::VectorXd&, solvers::QPData*)> make_qp,
      std::function<double (const Eigen::VectorXd&)> eval_constraint_viol,
      std::function<double (const Eigen::VectorXd&)> eval_cost,
      std::function<void (Eigen::VectorXd*)> proj_to_cspace,
      std::function<solvers::NCQPSolver::SetMembershipConstraints(
          const Eigen::VectorXd&)> get_sm_constraints,
      const std::string& ncqp_solver_options_yaml);

  void DoSQPStep(const Eigen::VectorXd& x, SQPIterate* sol);

 private:

  drake::solvers::SolverOptions drake_solver_options_;

  std::function<void (const Eigen::VectorXd&, solvers::QPData*)> make_qp_;
  std::function<double (const Eigen::VectorXd&)> eval_constraint_viol_;
  std::function<double (const Eigen::VectorXd&)> eval_cost_;
  std::function<void (Eigen::VectorXd*)> proj_to_config_space_;
  std::function<solvers::NCQPSolver::SetMembershipConstraints(
      const Eigen::VectorXd&)> get_sm_constraints_;

  solvers::NCQPSolver ncqp_solver_;
  solvers::QPData qp_;

  LineSearchParams lsparams_;

};

}

