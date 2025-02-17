#pragma once

#include "sqp_utils.h"
#include "solvers/admm/ncqp_solver.h"

namespace dairlib::systems::controllers::id_mpc {

class NCSQPSolver {

 public:
  NCSQPSolver(
      std::function<void (const Eigen::VectorXd&, solvers::QPData*)> make_qp,
      std::function<double (const Eigen::VectorXd&)> eval_constraint_viol,
      std::function<double (const Eigen::VectorXd&)> eval_cost,
      std::function<void (Eigen::VectorXd*)> proj_to_cspace,
      std::function<solvers::NCQPSolver::SetMembershipConstraints(
          const Eigen::VectorXd&)> get_sm_constraints);

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

