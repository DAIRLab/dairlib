#include "sqp_solver.h"
#include <iostream>

namespace dairlib::systems::controllers::id_mpc {

using Eigen::VectorXd;

SQPSolver::SQPSolver(
    int n, int m,
    std::function<void (const VectorXd&, solvers::QPData&)> make_qp,
    std::function<double (const VectorXd&)> eval_constraint_viol,
    std::function<double (const VectorXd&)> eval_cost,
    std::function<void (Eigen::VectorXd&)> proj_to_cspace) :
    n_(n), m_(m),
    make_qp_(make_qp), eval_constraint_viol_(eval_constraint_viol),
    eval_cost_(eval_cost), proj_to_config_space_(proj_to_cspace){}

void SQPSolver::DoSQPStep(const VectorXd &x, SQPIterate &sol) {
  sol.x_init = x;
  make_qp_(x, qp_);

  // TODO (@Brian-Acosta) add solver options constructor arg
  if (not qp_solver_.IsInitialized()) {
    qp_solver_.InitializeSolver(qp_, drake_solver_options_);
  }
  solvers::QPResult result;
  qp_solver_.Solve(qp_, result);

  if (result.success) {
    sol.dx = result.x;
    LineSearch(sol);
  } else {
    std::cout << "SQP qp solve failed with status: " << result.solution_result;
  }
}

void SQPSolver::LineSearch(SQPIterate &sol) {
  double alpha = 1.0;
  double theta_k = eval_constraint_viol_(sol.x_init);
  double phi_k = eval_cost_(sol.x_init);
  const VectorXd& grad_phi_k = qp_.g;
  bool accepted = false;
  double theta_k_p1 = 0;
  double phi_k_p1 = 0;
  VectorXd candidate = sol.x_init;
  while (not accepted and alpha >= lsparams_.alpha_min) {
    candidate = sol.x_init + alpha * sol.dx;
    proj_to_config_space_(candidate);
    theta_k_p1 = eval_constraint_viol_(candidate);
    phi_k_p1 = eval_cost_(candidate);

    if (theta_k_p1 > lsparams_.theta_max) {
      if (theta_k_p1 < (1.0 - lsparams_.gamma_theta) * theta_k) {
        accepted = true;
      }
    } else if (std::max(theta_k, theta_k_p1) < lsparams_.theta_min and
               grad_phi_k.dot(sol.dx) < 0) {
      if (phi_k_p1 < phi_k + lsparams_.eta * alpha * grad_phi_k.dot(sol.dx)) {
        accepted = true;
      }
    } else {
      if (phi_k_p1 < phi_k * (1.0 - lsparams_.gamma_phi) or
          theta_k_p1 < theta_k * (1.0 - lsparams_.gamma_theta)) {
        accepted = true;
      }
    }
    if (not accepted) {
      alpha *= lsparams_.gamma_alpha;
    }
  }
  sol.accepted = accepted;
  sol.x_sol = accepted ? candidate : sol.x_init;
  sol.constraint_viol = accepted ? theta_k_p1 : theta_k;
  sol.cost = accepted ? phi_k_p1 : phi_k;

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