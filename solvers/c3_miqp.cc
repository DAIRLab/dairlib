#include "solvers/c3_miqp.h"

#include "gurobi_c++.h"

namespace dairlib {
namespace solvers {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

C3MIQP::C3MIQP(const LCS& LCS, const CostMatrices& costs,
               const vector<VectorXd>& xdesired, const C3Options& options)
    : C3(LCS, costs, xdesired, options) {
  //  env.set("LogToConsole", "0");
  // Create an environment
  env.set("LogToConsole", "0");
  env.set("OutputFlag", "1");
  env.set("Threads", "0");
  env.start();
}

VectorXd C3MIQP::SolveSingleProjection(const MatrixXd& U,
                                       const VectorXd& delta_c,
                                       const MatrixXd& E, const MatrixXd& F,
                                       const MatrixXd& H, const VectorXd& c,
                                       const int admm_iteration,
                                       const int& warm_start_index) {

  // set up linear term in cost
  VectorXd cost_lin = -2 * delta_c.transpose() * U;

  // set up for constraints (Ex + F \lambda + Hu + c >= 0)
  MatrixXd Mcons1(n_lambda_, n_x_ + n_lambda_ + n_u_);
  Mcons1 << E, F, H;

  // set up for constraints (\lambda >= 0)
  MatrixXd MM1 = MatrixXd::Zero(n_lambda_, n_x_);
  MatrixXd MM2 = MatrixXd::Identity(n_lambda_, n_lambda_);
  MatrixXd MM3 = MatrixXd::Zero(n_lambda_, n_u_);
  MatrixXd Mcons2(n_lambda_, n_x_ + n_lambda_ + n_u_);
  Mcons2 << MM1, MM2, MM3;

  GRBModel model = GRBModel(env);

  GRBVar delta_k[n_x_ + n_lambda_ + n_u_];
  GRBVar binary[n_lambda_];

  for (int i = 0; i < n_lambda_; i++) {
    binary[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
    if (warm_start_index != -1) {
      binary[i].set(GRB_DoubleAttr_Start,
                    warm_start_binary_[admm_iteration][warm_start_index](i));
    }
  }

  for (int i = 0; i < n_x_ + n_lambda_ + n_u_; i++) {
    delta_k[i] =
        model.addVar(-kVariableBounds, kVariableBounds, 0.0, GRB_CONTINUOUS);
    if (warm_start_index != -1) {
      delta_k[i].set(GRB_DoubleAttr_Start,
                     warm_start_delta_[admm_iteration][warm_start_index](i));
    }
  }

  GRBQuadExpr obj = 0;

  for (int i = 0; i < n_x_ + n_lambda_ + n_u_; i++) {
    obj.addTerm(cost_lin(i), delta_k[i]);
    obj.addTerm(U(i, i), delta_k[i], delta_k[i]);
  }

  model.setObjective(obj, GRB_MINIMIZE);

  int M = 1000;  // big M variable
  double coeff[n_x_ + n_lambda_ + n_u_];
  double coeff2[n_x_ + n_lambda_ + n_u_];

  for (int i = 0; i < n_lambda_; i++) {
    GRBLinExpr lambda_expr = 0;

    /// convert VectorXd to double
    for (int j = 0; j < n_x_ + n_lambda_ + n_u_; j++) {
      coeff[j] = Mcons2(i, j);
    }

    lambda_expr.addTerms(coeff, delta_k, n_x_ + n_lambda_ + n_u_);
    model.addConstr(lambda_expr >= 0);
    model.addConstr(lambda_expr <= M * (1 - binary[i]));

    GRBLinExpr activation_expr = 0;

    /// convert VectorXd to double
    for (int j = 0; j < n_x_ + n_lambda_ + n_u_; j++) {
      coeff2[j] = Mcons1(i, j);
    }

    activation_expr.addTerms(coeff2, delta_k, n_x_ + n_lambda_ + n_u_);
    model.addConstr(activation_expr + c(i) >= 0);
    model.addConstr(activation_expr + c(i) <= M * binary[i]);
  }

  model.optimize();

  VectorXd delta_kc(n_x_ + n_lambda_ + n_u_);
  VectorXd binaryc(n_lambda_);
  for (int i = 0; i < n_x_ + n_lambda_ + n_u_; i++) {
    delta_kc(i) = delta_k[i].get(GRB_DoubleAttr_X);
  }
  for (int i = 0; i < n_lambda_; i++) {
    binaryc(i) = binary[i].get(GRB_DoubleAttr_X);
  }

  if (warm_start_index != -1) {
    warm_start_delta_[admm_iteration][warm_start_index] = delta_kc;
    warm_start_binary_[admm_iteration][warm_start_index] = binaryc;
  }
  return delta_kc;
}

// Need to input friction ratio as a parameter
VectorXd C3MIQP::SolveRobustSingleProjection(
    const MatrixXd& U, const VectorXd& delta_c, const MatrixXd& E,
    const MatrixXd& F, const MatrixXd& H, const VectorXd& c,
    const Eigen::MatrixXd& W_x, const Eigen::MatrixXd& W_l,
    const Eigen::MatrixXd& W_u, const Eigen::VectorXd& w,
    const int admm_iteration, const int& warm_start_index) {

  // set up linear term in cost
  VectorXd cost_lin = -2 * delta_c.transpose() * U;

  // set up for constraints (Ex + F \lambda + Hu + c >= 0)
  MatrixXd Mcons1(n_lambda_, n_x_ + n_lambda_ + n_u_);
  Mcons1 << E, F, H;

  // set up for constraints (\lambda >= 0)
  MatrixXd MM1 = MatrixXd::Zero(n_lambda_, n_x_);
  MatrixXd MM2 = MatrixXd::Identity(n_lambda_, n_lambda_);
  MatrixXd MM3 = MatrixXd::Zero(n_lambda_, n_u_);
  MatrixXd Mcons2(n_lambda_, n_x_ + n_lambda_ + n_u_);
  Mcons2 << MM1, MM2, MM3;

  GRBModel model = GRBModel(env);

  GRBVar delta_k[n_x_ + n_lambda_ + n_u_];
  GRBVar binary[n_lambda_];
  GRBVar reduced_friction_cone_binary[n_lambda_];

  for (int i = 0; i < n_lambda_; i++) {
    binary[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
    if (warm_start_index != -1) {
      binary[i].set(GRB_DoubleAttr_Start,
                    warm_start_binary_[admm_iteration][warm_start_index](i));
    }
  }

  for (int i = 0; i < n_x_ + n_lambda_ + n_u_; i++) {
    delta_k[i] = model.addVar(-100.0, 100.0, 0.0, GRB_CONTINUOUS);
    if (warm_start_index != -1) {
      delta_k[i].set(GRB_DoubleAttr_Start,
                     warm_start_delta_[admm_iteration][warm_start_index](i));
    }
  }

  GRBQuadExpr obj = 0;

  for (int i = 0; i < n_x_ + n_lambda_ + n_u_; i++) {
    obj.addTerm(cost_lin(i), delta_k[i]);
    obj.addTerm(U(i, i), delta_k[i], delta_k[i]);
  }

  // initial state constraint
  if (warm_start_index == 0) {
    for (int i = 0; i < n_x_; ++i) {
      model.addConstr(delta_k[i] == delta_c[i]);
    }
  }

  model.setObjective(obj, GRB_MINIMIZE);

  int M = 1000;  // big M variable
  double coeff[n_x_ + n_lambda_ + n_u_];
  double coeff2[n_x_ + n_lambda_ + n_u_];

  for (int i = 0; i < n_lambda_; i++) {
    GRBLinExpr lambda_expr = 0;

    /// convert VectorXd to double
    for (int j = 0; j < n_x_ + n_lambda_ + n_u_; j++) {
      coeff[j] = Mcons2(i, j);
    }

    lambda_expr.addTerms(coeff, delta_k, n_x_ + n_lambda_ + n_u_);
    model.addConstr(lambda_expr >= 0);
    model.addConstr(lambda_expr <= M * (1 - binary[i]));

    GRBLinExpr activation_expr = 0;

    /// convert VectorXd to double
    for (int j = 0; j < n_x_ + n_lambda_ + n_u_; j++) {
      coeff2[j] = Mcons1(i, j);
    }

    activation_expr.addTerms(coeff2, delta_k, n_x_ + n_lambda_ + n_u_);
    model.addConstr(activation_expr + c(i) >= 0);
    model.addConstr(activation_expr + c(i) <= M * binary[i]);
  }

  double* tangential_velocity_coeffs;
  MatrixXd P_t(n_lambda_, n_x_ + n_lambda_ + n_u_);
  int constraint_rows = n_lambda_;
  P_t << W_x, W_l, W_u;


  for (int i = 0; i < constraint_rows; ++i) {
    reduced_friction_cone_binary[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
    GRBLinExpr tangential_velocity_expr = 0;
    tangential_velocity_coeffs = P_t.row(i).data();
    tangential_velocity_expr.addTerms(tangential_velocity_coeffs, delta_k,
                                      n_x_ + n_lambda_ + n_u_);

    /// Constraint explanation:
    /// Adding a binary decision to either be:
    /// - inside a conservative (mu_l) friction cone (stick)
    /// or
    /// - on the boundary of the original friction cone (mu) (slip)
    /// where mu_l < mu

    /// The active constraint for the sticking condition is:
    /// lambda_1 - lambda_2 <= mu_l / mu * (lambda_1 + lambda_2)

    /// The active constraint for the friction cone boundary is:
    /// lambda_2 == 0
    /// where lambda_2 is the friction force in the sliding direction
    /// tangential velocity expr is the tangential velocity in the SAME
    /// direction as the friction force
    if (i % 2 == 1) {
      model.addConstr(delta_k[n_x_ + i - 1] - delta_k[n_x_ + i] <=
                      (0.2 / 0.4) *
                              (delta_k[n_x_ + i] + delta_k[n_x_ + i - 1]) +
                          M * (reduced_friction_cone_binary[i]));
      model.addConstr(tangential_velocity_expr <=
                      M * (reduced_friction_cone_binary[i]));
      model.addConstr(delta_k[n_x_ + i] <=
                      M * (1 - reduced_friction_cone_binary[i]));
    } else {
      model.addConstr(delta_k[n_x_ + i + 1] - delta_k[n_x_ + i] <=
                      (0.2 / 0.4) *
                              (delta_k[n_x_ + i] + delta_k[n_x_ + i + 1]) +
                          M * (reduced_friction_cone_binary[i]));
      model.addConstr(tangential_velocity_expr <=
                      M * (reduced_friction_cone_binary[i]));
      model.addConstr(delta_k[n_x_ + i] <=
                      M * (1 - reduced_friction_cone_binary[i]));
    }
  }

  model.optimize();

  VectorXd delta_kc(n_x_ + n_lambda_ + n_u_);
  VectorXd binaryc(n_lambda_);
  for (int i = 0; i < n_x_ + n_lambda_ + n_u_; i++) {
    delta_kc(i) = delta_k[i].get(GRB_DoubleAttr_X);
  }
  for (int i = 0; i < n_lambda_; i++) {
    binaryc(i) = binary[i].get(GRB_DoubleAttr_X);
  }

  if (warm_start_index != -1) {
    warm_start_delta_[admm_iteration][warm_start_index] = delta_kc;
    warm_start_binary_[admm_iteration][warm_start_index] = binaryc;
  }

  return delta_kc;
}

std::vector<Eigen::VectorXd> C3MIQP::GetWarmStartDelta() const {
  return warm_start_delta_[0];
}

std::vector<Eigen::VectorXd> C3MIQP::GetWarmStartBinary() const {
  return warm_start_binary_[0];
}

}  // namespace solvers
}  // namespace dairlib
