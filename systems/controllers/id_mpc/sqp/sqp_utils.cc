#include "sqp_utils.h"
#include "solvers/qp_data.h"
#include "systems/controllers/id_mpc/costs/nonlinear_least_squares_cost.h"
#include "solvers/admm/set_membership_constraint.h"

#include "drake/math/autodiff_gradient.h"

namespace dairlib::systems::controllers::id_mpc {

using Eigen::MatrixXd;
using Eigen::VectorXd;

using drake::AutoDiffXd;
using drake::AutoDiffVecXd;
using drake::math::ExtractValue;
using drake::math::ExtractGradient;
using drake::math::InitializeAutoDiff;
using drake::solvers::MathematicalProgram;

using solvers::QPData;

void ParseCostsToSQP(const VectorXd& x, const MathematicalProgram& prog,
                     QPData* qp) {

  qp->g = VectorXd::Zero(prog.num_vars());

  std::vector<Eigen::Triplet<double>> cost_triplets;

  for (const auto& binding : prog.GetAllCosts()) {
    const auto& v = binding.variables();
    const auto& indices = prog.FindDecisionVariableIndices(v);

    VectorXd xval = VectorXd::Zero(v.rows());
    for (int i = 0; i < v.rows(); ++i) {
      xval(i) = x(indices[i]);
    }
    GaussNewtonApproximation cost_data;
    if (dynamic_cast<NonlinearLeastSquaresCost<AutoDiffXd>*>(binding.evaluator().get())) {
      cost_data = dynamic_cast<NonlinearLeastSquaresCost<AutoDiffXd>*>(
          binding.evaluator().get()
      )->CalcGaussNewtonApproximation(xval);
    } else if (dynamic_cast<NonlinearLeastSquaresCost<double>*>(binding.evaluator().get())) {
      cost_data = dynamic_cast<NonlinearLeastSquaresCost<double>*>(
          binding.evaluator().get()
      )->CalcGaussNewtonApproximation(xval);
    } else {
      throw std::runtime_error("IDMPC only supports NonlinearLeastSquares costs "
                               "To ensure that the SQP gauss newton "
                               "approximation is properly implemented");
    }
    solvers::AppendQuadraticCost(
        indices, cost_data.H, cost_data.g, cost_data.c, cost_triplets, qp->g,
        &qp->c, cost_data.diagonal_hessian);
  }
  qp->H.resize(prog.num_vars(), prog.num_vars());
  qp->H.setFromTriplets(cost_triplets.begin(), cost_triplets.end());
}

// TODO (@Brian-Acosta)
//  Can maybe make this more efficient by handling constraints differently
//   Depending on their type. Also likely want to denote true equality vs
//   inequality constraints
void ParseConstraintsToSQP(const VectorXd& x, const MathematicalProgram& prog,
                           QPData* qp) {
  qp->num_eq = 0;
  qp->num_ineq = 0;

  std::vector<Eigen::Triplet<double>> inequality_triplets;

  for (const auto& binding : prog.GetAllConstraints()) {
    // skip set membership constraints when constructing QP
    if (dynamic_cast<solvers::SetMembershipConstraint*>(
        binding.evaluator().get()) != nullptr) {
      continue;
    }
    const auto& v = binding.variables();
    const auto& indices = prog.FindDecisionVariableIndices(v);
    VectorXd xval = VectorXd::Zero(v.rows());

    for (int i = 0; i < v.rows(); ++i) {
      xval(i) = x(indices[i]);
    }

    AutoDiffVecXd xval_ad = InitializeAutoDiff(xval);
    AutoDiffVecXd y_ad;
    binding.evaluator()->Eval(xval_ad, &y_ad);
    MatrixXd dydx = ExtractGradient(y_ad);
    VectorXd yval = ExtractValue(y_ad);
    // SQP constraint: lb <= y(x) <= ub -->  lb <= dydx * dx + y* <= ub
    // subtract y* from the above equation
    VectorXd lb = binding.evaluator()->lower_bound() - yval;
    VectorXd ub = binding.evaluator()->upper_bound() - yval;
    solvers::AppendLinearConstraint(
        indices, qp->num_ineq, dydx, lb, ub, inequality_triplets, qp->lb, qp->ub);
    qp->num_ineq += binding.evaluator()->num_constraints();
  }
  qp->A.resize(qp->num_ineq, x.rows());
  qp->A.setFromTriplets(inequality_triplets.begin(), inequality_triplets.end());
}

void DoLineSearch(std::function<double (const Eigen::VectorXd&)> eval_constraint_viol,
                  std::function<double (const Eigen::VectorXd&)> eval_cost,
                  std::function<void (Eigen::VectorXd*)> proj_to_cspace,
                  const QPData& qp, const LineSearchParams& params,
                  SQPIterate* sol) {
  double alpha = 1.0;
  double theta_k = eval_constraint_viol(sol->x_init);
  double phi_k = eval_cost(sol->x_init);
  const VectorXd& grad_phi_k = qp.H * sol->x_init + qp.g;
  bool accepted = false;
  double theta_k_p1 = 0;
  double phi_k_p1 = 0;
  VectorXd candidate = sol->x_init;
  while (not accepted and alpha >= params.alpha_min) {
    candidate = sol->x_init + alpha * sol->dx;
    proj_to_cspace(&candidate);
    theta_k_p1 = eval_constraint_viol(candidate);
    phi_k_p1 = eval_cost(candidate);

    if (theta_k_p1 > params.theta_max) {
      if (theta_k_p1 < (1.0 - params.gamma_theta) * theta_k) {
        accepted = true;
      }
    } else if (std::max(theta_k, theta_k_p1) < params.theta_min and
        grad_phi_k.dot(sol->dx) < 0) {
      if (phi_k_p1 < phi_k + params.eta * alpha * grad_phi_k.dot(sol->dx)) {
        accepted = true;
      }
    } else {
      if (phi_k_p1 < phi_k * (1.0 - params.gamma_phi) or
          theta_k_p1 < theta_k * (1.0 - params.gamma_theta)) {
        accepted = true;
      }
    }
    if (not accepted) {
      alpha *= params.gamma_alpha;
    }
  }
  sol->accepted = accepted;
  sol->x_sol = accepted ? candidate : sol->x_init;
  sol->constraint_viol = accepted ? theta_k_p1 : theta_k;
  sol->cost = accepted ? phi_k_p1 : phi_k;
}

SQPIterate AllocateSQPIterate(int n) {
  SQPIterate ret;
  ret.x_init = VectorXd::Zero(n);
  ret.dx = VectorXd::Zero(n);
  ret.x_sol = VectorXd::Zero(n);
  ret.constraint_viol = 0;
  ret.cost = 0;
  return ret;
}


}