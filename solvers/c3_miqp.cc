#include "solvers/c3_miqp.h"

namespace dairlib {
namespace solvers {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

C3MIQP::C3MIQP(const LCS& LCS, const vector<MatrixXd>& Q,
               const vector<MatrixXd>& R, const vector<MatrixXd>& G,
               const vector<MatrixXd>& U, const vector<VectorXd>& xdesired,
               const C3Options& options,
               const std::vector<Eigen::VectorXd>& warm_start_delta,
               const std::vector<Eigen::VectorXd>& warm_start_binary,
               const std::vector<Eigen::VectorXd>& warm_start_x,
               const std::vector<Eigen::VectorXd>& warm_start_lambda,
               const std::vector<Eigen::VectorXd>& warm_start_u,
               bool warm_start)
    : C3(LCS, Q, R, G, U, xdesired, options,
         warm_start_delta, warm_start_binary,
         warm_start_x, warm_start_lambda,
         warm_start_u, warm_start), env_(true) {

  // Create an environment
//  env_.set("LogToConsole", "0");
  env_.set("OutputFlag", "0");
  env_.set("Threads", "2");
  env_.start();
}

VectorXd C3MIQP::SolveSingleProjection(const MatrixXd& U,
                                       const VectorXd& delta_c,
                                       const MatrixXd& E, const MatrixXd& F,
                                       const MatrixXd& H, const VectorXd& c,
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

  // Create an empty model
//  GRBEnv env = GRBEnv();
//  env.start();
//  env_.start();

  GRBModel model = GRBModel(env_);
//  model.set(GRB_IntParam_LogToConsole, 1);
//  model.set(GRB_StringParam_LogFile, "grb_debug");
  //model.set("FeasibilityTol", "0.01");
  //model.set("IterationLimit", "40");

  GRBVar delta_k[n_ + m_ + k_];
  GRBVar binary[m_];

  for (int i = 0; i < m_; i++) {
    binary[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
    if (warm_start_index != -1) {
      binary[i].set(GRB_DoubleAttr_Start, warm_start_binary_[warm_start_index](i));
    }
  }

  for (int i = 0; i < n_ + m_ + k_; i++) {
    delta_k[i] = model.addVar(-10000.0, 10000.0, 0.0, GRB_CONTINUOUS);
    if (warm_start_index != -1) {
      delta_k[i].set(GRB_DoubleAttr_Start, warm_start_delta_[warm_start_index](i));
    }
  }

  GRBQuadExpr obj = 0;

  for (int i = 0; i < n_ + m_ + k_; i++) {
    obj.addTerm(cost_lin(i), delta_k[i]);

    for (int j = 0; j < n_ + m_ + k_; j++) {
      obj.addTerm(U(i, j), delta_k[i], delta_k[j]);
    }
  }

  model.setObjective(obj, GRB_MINIMIZE);

  int M = 100000;  // big M variable
  double coeff[n_ + m_ + k_];
  double coeff2[n_ + m_ + k_];

  for (int i = 0; i < m_; i++) {
    GRBLinExpr cexpr = 0;

    /// convert VectorXd to double
    for (int j = 0; j < n_ + m_ + k_; j++) {
      coeff[j] = Mcons2.row(i)(j);
    }

    cexpr.addTerms(coeff, delta_k, n_ + m_ + k_);
    model.addConstr(cexpr >= 0);
    model.addConstr(cexpr <= M * (1 - binary[i]));

    GRBLinExpr cexpr2 = 0;

    /// convert VectorXd to double
    for (int j = 0; j < n_ + m_ + k_; j++) {
      coeff2[j] = Mcons1.row(i)(j);
    }

    cexpr2.addTerms(coeff2, delta_k, n_ + m_ + k_);
    model.addConstr(cexpr2 + c(i) >= 0);
    model.addConstr(cexpr2 + c(i) <= M * binary[i]);
  }

  model.optimize();

  VectorXd delta_kc(n_ + m_ + k_);
  VectorXd binaryc(m_);
  for (int i = 0; i < n_ + m_ + k_; i++) {
    delta_kc(i) = delta_k[i].get(GRB_DoubleAttr_X);
  }
  for (int i = 0; i < m_; i++){
    binaryc(i) = binary[i].get(GRB_DoubleAttr_X);
  }

  if (warm_start_index != -1){
    warm_start_delta_[warm_start_index] = delta_kc;
    warm_start_binary_[warm_start_index] = binaryc;
  }
  return delta_kc;
}

std::vector<Eigen::VectorXd> C3MIQP::GetWarmStartDelta() const{
  return warm_start_delta_;
}

std::vector<Eigen::VectorXd> C3MIQP::GetWarmStartBinary() const{
  return warm_start_binary_;
}

}  // namespace solvers
}  // namespace dairlib
