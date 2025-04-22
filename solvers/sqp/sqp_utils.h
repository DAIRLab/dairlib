#pragma once

#include "solvers/qp_data.h"

namespace dairlib::solvers::sqp {

struct SQPIterate {
  Eigen::VectorXd x_init;
  Eigen::VectorXd dx;
  Eigen::VectorXd x_sol;
  bool accepted;
  double constraint_viol;
  double cost;
  double step_size = 0;
  double setup_time = 0;
  double solve_time = 0;
  double line_search_time = 0;
  double total_step_time = 0;
};

struct LineSearchParams {
  double alpha_min = 1e-4;
  double theta_max = 3e-2;
  double theta_min = 1e-6;
  double eta = 1e-4;
  double gamma_phi = 1e-6;
  double gamma_theta = 1e-2;
  double gamma_alpha = 0.5;
};

void ParseCostsToSQP(const Eigen::VectorXd& x,
                     const drake::solvers::MathematicalProgram& prog,
                     solvers::QPData* qp);

void ParseConstraintsToSQP(const Eigen::VectorXd& x,
                           const drake::solvers::MathematicalProgram& prog,
                           solvers::QPData* qp);

/*!
 * line search assuming x_init and dx have already been properly set,
 * where x_init is the initial solution/iterate value, and dx is the
 * descent direction.
 */
void DoLineSearch(
    std::function<double (const Eigen::VectorXd&)> eval_constraint_viol,
    std::function<double (const Eigen::VectorXd&)> eval_cost,
    std::function<void (Eigen::VectorXd*)> proj_to_cspace,
    const solvers::QPData& qp, const LineSearchParams& params,
    SQPIterate* sol);

SQPIterate AllocateSQPIterate(int n);

}

