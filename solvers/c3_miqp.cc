#include "solvers/c3_miqp.h"

namespace dairlib {
namespace solvers {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

C3MIQP::C3MIQP(const LCS& LCS, const CostMatrices& costs,
               const vector<VectorXd>& xdesired, const C3Options& options)
    : C3(LCS, costs, xdesired, options), env_(true) {
  // Create an environment
  //  env_.set("LogToConsole", "0");
  env_.set("OutputFlag", "0");
  env_.set("Threads", "0");
  env_.start();
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
  MatrixXd Mcons1(m_, n_ + m_ + k_);
  Mcons1 << E, F, H;

  // set up for constraints (\lambda >= 0)
  MatrixXd MM1 = MatrixXd::Zero(m_, n_);
  MatrixXd MM2 = MatrixXd::Identity(m_, m_);
  MatrixXd MM3 = MatrixXd::Zero(m_, k_);
  MatrixXd Mcons2(m_, n_ + m_ + k_);
  Mcons2 << MM1, MM2, MM3;

  GRBModel model = GRBModel(env_);

  GRBVar delta_k[n_ + m_ + k_];
  GRBVar binary[m_];

  for (int i = 0; i < m_; i++) {
    binary[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
    if (warm_start_index != -1) {
      binary[i].set(GRB_DoubleAttr_Start,
                    warm_start_binary_[admm_iteration][warm_start_index](i));
    }
  }

  for (int i = 0; i < n_ + m_ + k_; i++) {
    delta_k[i] = model.addVar(-100.0, 100.0, 0.0, GRB_CONTINUOUS);
    if (warm_start_index != -1) {
      delta_k[i].set(GRB_DoubleAttr_Start,
                     warm_start_delta_[admm_iteration][warm_start_index](i));
    }
  }

  GRBQuadExpr obj = 0;

  for (int i = 0; i < n_ + m_ + k_; i++) {
    obj.addTerm(cost_lin(i), delta_k[i]);
    obj.addTerm(U(i, i), delta_k[i], delta_k[i]);
  }

  model.setObjective(obj, GRB_MINIMIZE);

  int M = 1000;  // big M variable
  double coeff[n_ + m_ + k_];
  double coeff2[n_ + m_ + k_];

  for (int i = 0; i < m_; i++) {
    GRBLinExpr lambda_expr = 0;

    /// convert VectorXd to double
    for (int j = 0; j < n_ + m_ + k_; j++) {
      coeff[j] = Mcons2(i, j);
    }

    lambda_expr.addTerms(coeff, delta_k, n_ + m_ + k_);
    model.addConstr(lambda_expr >= 0);
    model.addConstr(lambda_expr <= M * (1 - binary[i]));

    GRBLinExpr activation_expr = 0;

    /// convert VectorXd to double
    for (int j = 0; j < n_ + m_ + k_; j++) {
      coeff2[j] = Mcons1(i, j);
    }

    activation_expr.addTerms(coeff2, delta_k, n_ + m_ + k_);
    model.addConstr(activation_expr + c(i) >= 0);
    model.addConstr(activation_expr + c(i) <= M * binary[i]);
  }

  model.optimize();

  VectorXd delta_kc(n_ + m_ + k_);
  VectorXd binaryc(m_);
  for (int i = 0; i < n_ + m_ + k_; i++) {
    delta_kc(i) = delta_k[i].get(GRB_DoubleAttr_X);
  }
  for (int i = 0; i < m_; i++) {
    binaryc(i) = binary[i].get(GRB_DoubleAttr_X);
  }

  if (warm_start_index != -1) {
    warm_start_delta_[admm_iteration][warm_start_index] = delta_kc;
    warm_start_binary_[admm_iteration][warm_start_index] = binaryc;
  }
  return delta_kc;
}

VectorXd C3MIQP::SolveRobustSingleProjection(
    const MatrixXd& U, const VectorXd& delta_c, const MatrixXd& E,
    const MatrixXd& F, const MatrixXd& H, const VectorXd& c,
    const Eigen::MatrixXd& W_x, const Eigen::MatrixXd& W_l,
    const Eigen::MatrixXd& W_u, const Eigen::VectorXd& w,
    const int admm_iteration, const int& warm_start_index) {
  // set up linear term in cost
  VectorXd cost_lin = -2 * delta_c.transpose() * U;

  // set up for constraints (Ex + F \lambda + Hu + c >= 0)
  MatrixXd Mcons1(m_, n_ + m_ + k_);
  Mcons1 << E, F, H;

  // set up for constraints (\lambda >= 0)
  MatrixXd MM1 = MatrixXd::Zero(m_, n_);
  MatrixXd MM2 = MatrixXd::Identity(m_, m_);
  MatrixXd MM3 = MatrixXd::Zero(m_, k_);
  MatrixXd Mcons2(m_, n_ + m_ + k_);
  Mcons2 << MM1, MM2, MM3;

  GRBModel model = GRBModel(env_);

  GRBVar delta_k[n_ + m_ + k_];
  GRBVar binary[m_];
  GRBVar reduced_friction_cone_binary[m_];

  for (int i = 0; i < m_; i++) {
    binary[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
    if (warm_start_index != -1) {
      binary[i].set(GRB_DoubleAttr_Start,
                    warm_start_binary_[admm_iteration][warm_start_index](i));
    }
  }

  for (int i = 0; i < n_ + m_ + k_; i++) {
    delta_k[i] = model.addVar(-100.0, 100.0, 0.0, GRB_CONTINUOUS);
    if (warm_start_index != -1) {
      delta_k[i].set(GRB_DoubleAttr_Start,
                     warm_start_delta_[admm_iteration][warm_start_index](i));
    }
  }

  GRBQuadExpr obj = 0;

  for (int i = 0; i < n_ + m_ + k_; i++) {
    obj.addTerm(cost_lin(i), delta_k[i]);
    obj.addTerm(U(i, i), delta_k[i], delta_k[i]);
  }

  model.setObjective(obj, GRB_MINIMIZE);

  int M = 1000;  // big M variable
  double coeff[n_ + m_ + k_];
  double coeff2[n_ + m_ + k_];

  for (int i = 0; i < m_; i++) {
    GRBLinExpr lambda_expr = 0;

    /// convert VectorXd to double
    for (int j = 0; j < n_ + m_ + k_; j++) {
      coeff[j] = Mcons2(i, j);
    }

    lambda_expr.addTerms(coeff, delta_k, n_ + m_ + k_);
    model.addConstr(lambda_expr >= 0);
    model.addConstr(lambda_expr <= M * (1 - binary[i]));

    GRBLinExpr activation_expr = 0;

    /// convert VectorXd to double
    for (int j = 0; j < n_ + m_ + k_; j++) {
      coeff2[j] = Mcons1(i, j);
    }

    activation_expr.addTerms(coeff2, delta_k, n_ + m_ + k_);
    model.addConstr(activation_expr + c(i) >= 0);
    model.addConstr(activation_expr + c(i) <= M * binary[i]);
  }

  double subtraction_coeffs[n_ + m_ + k_];
  double addition_coeffs[n_ + m_ + k_];
  double tangential_velocity_coeffs[n_ + m_ + k_];
  double single_lambda_coeffs[n_ + m_ + k_];
  MatrixXd addition_of_aligned_forces = MatrixXd::Zero(m_, n_ + m_ + k_);
  MatrixXd subtraction_of_aligned_forces = MatrixXd::Zero(m_, n_ + m_ + k_);


  MatrixXd P_t(m_, n_ + m_ + k_);
  int constraint_rows = m_;
//  P_t << W_x, W_l, W_u;

  // stewart and trinkle
  P_t << E, F, H;
  MatrixXd P_t2 = P_t.bottomRows(((m_ / 6) * 4) / 7 * 3);
  P_t = P_t2;
  constraint_rows = P_t2.rows();
  for (int i = 0; i < constraint_rows; ++i) {
    addition_of_aligned_forces(i, n_ + 2 * (i / 2) + 0) = 1;
    addition_of_aligned_forces(i, n_ + 2 * (i / 2) + 1) = 1;
    if (i % 2 == 0) {
      subtraction_of_aligned_forces(i, n_ + i + 0) = -1;
      subtraction_of_aligned_forces(i, n_ + i + 1) = 1;
    } else {
      subtraction_of_aligned_forces(i, n_ + i + 0) = -1;
      subtraction_of_aligned_forces(i, n_ + i - 1) = 1;
    }
  }

  for (int i = 0; i < constraint_rows; ++i) {
    reduced_friction_cone_binary[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
    GRBLinExpr subtraction_constraint_expr = 0;
    GRBLinExpr addition_constraint_expr = 0;
    GRBLinExpr tangential_velocity_expr = 0;
    GRBLinExpr single_lambda_expr = 0;

    /// convert VectorXd to double
    for (int j = 0; j < n_ + m_ + k_; j++) {
      addition_coeffs[j] = addition_of_aligned_forces(i, j);
      subtraction_coeffs[j] = subtraction_of_aligned_forces(i, j);
      tangential_velocity_coeffs[j] = P_t(i, j);
      single_lambda_coeffs[j] = Mcons2(i, j);
    }

    subtraction_constraint_expr.addTerms(subtraction_coeffs, delta_k,
                                         n_ + m_ + k_);
    addition_constraint_expr.addTerms(addition_coeffs, delta_k, n_ + m_ + k_);
    tangential_velocity_expr.addTerms(tangential_velocity_coeffs, delta_k,
                                      n_ + m_ + k_);
    single_lambda_expr.addTerms(single_lambda_coeffs, delta_k, n_ + m_ + k_);

    /// Constraint explanation
    /// if i % 2 == 0:
    ///     lambda_1 - lambda_2 <= mu_l / mu * (lambda_1 + lambda_2)
    /// else:
    ///     lambda_2 - lambda_1 <= mu_l / mu * (lambda_1 + lambda_2)
    if (i % 2 == 1) {
      model.addConstr(subtraction_constraint_expr <=
                      (0.4 / 0.6) * (addition_constraint_expr) +
                          M * (reduced_friction_cone_binary[i]));
      model.addConstr(tangential_velocity_expr <=
                      M * (reduced_friction_cone_binary[i]));
    } else {
      model.addConstr(subtraction_constraint_expr <=
                      (0.4 / 0.6) * (addition_constraint_expr) +
                          M * (reduced_friction_cone_binary[i]));
      model.addConstr(tangential_velocity_expr <=
                      M * (reduced_friction_cone_binary[i]));
    }
    /// if i % 2 == 0:
    ///     lambda_1 = 0
    /// else:
    ///     lambda_2 = 0
    model.addConstr(single_lambda_expr <=
                    M * (1 - reduced_friction_cone_binary[i]));
  }

  model.optimize();

  VectorXd delta_kc(n_ + m_ + k_);
  VectorXd binaryc(m_);
  for (int i = 0; i < n_ + m_ + k_; i++) {
    delta_kc(i) = delta_k[i].get(GRB_DoubleAttr_X);
  }
  for (int i = 0; i < m_; i++) {
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
