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
  env_.set("LogToConsole", "0");
  env_.set("OutputFlag", "0");
  // env_.set("Threads", "2");
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
  MatrixXd MM1;
  MM1 = MatrixXd::Zero(m_, n_);
  MatrixXd MM2;
  MM2 = MatrixXd::Identity(m_, m_);
  MatrixXd MM3;
  MM3 = MatrixXd::Zero(m_, k_);
  MatrixXd Mcons2(m_, n_ + m_ + k_);
  Mcons2 << MM1, MM2, MM3;

  // Create an empty model
  GRBModel model = GRBModel(env_);
  //model.set("FeasibilityTol", "0.01");
  // model.set("IterationLimit", "40");
  model.set(GRB_DBL_PAR_TIMELIMIT, "0.5");    // Time limit in seconds for MIQP.

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
      coeff[j] = Mcons1.row(i)(j);
    }

    cexpr2.addTerms(coeff, delta_k, n_ + m_ + k_);
    model.addConstr(cexpr2 + c(i) >= 0);
    model.addConstr(cexpr2 + c(i) <= M * binary[i]);
  }

  // TODO:  Play around with fixing some of these constraints.  Eventually make these adaptive to model size instead of hard-coded for the jack.
  // Any reported rates were achieved using 1 thread and 0 sample locations on Wanxin's computer.
  // for (int i = 0; i < m_; i++) {
  //   model.addConstr(binary[i] == 0);
  // }
  // // Enforce sliding for all contacts (gamma = 0).
  // model.addConstr(binary[0] + binary[1] + binary[2] + binary[3] == 0);
  // // Enforce that only up to one ground contact can lift up (lambda_n = 0).
  // model.addConstr(binary[5] + binary[6] + binary[7] <= 1);
  // Enforce sticking for all contacts.
  // model.addConstr(binary[0] + binary[1] + binary[2] + binary[3] == 0);
  // model.addConstr(binary[0] == 1);
  // model.addConstr(binary[1] == 1);
  // model.addConstr(binary[2] == 1);
  // model.addConstr(binary[3] == 1);
  // Enforce opposing tangential forces can't both be non-zero (or binaries can't both be zero).
  // (This alone doesn't improve speed noticeably; 2-3Hz.)
  model.addConstr(binary[8] + binary[9] >= 1);      // end effector / jack contact.
  model.addConstr(binary[10] + binary[11] >= 1);
  model.addConstr(binary[12] + binary[13] >= 1);    // capsule 1 / ground contact.
  model.addConstr(binary[14] + binary[15] >= 1);
  model.addConstr(binary[16] + binary[17] >= 1);    // capsule 2 / ground contact.
  model.addConstr(binary[18] + binary[19] >= 1);
  model.addConstr(binary[20] + binary[21] >= 1);    // capsule 3 / ground contact.
  model.addConstr(binary[22] + binary[23] >= 1);
  // Enforce there can be tangential forces only if there's corresponding normal force (or binary tangential can
  // be 0 only if binary normal is also 0).
  // (This alone doesn't improve speed noticeably; 1-2Hz.)
  model.addConstr(binary[8] >= binary[4]);          // end effector / jack contact.
  model.addConstr(binary[9] >= binary[4]);
  model.addConstr(binary[10] >= binary[4]);
  model.addConstr(binary[11] >= binary[4]);
  model.addConstr(binary[12] >= binary[5]);         // capsule 1 / ground contact.
  model.addConstr(binary[13] >= binary[5]);
  model.addConstr(binary[14] >= binary[5]);
  model.addConstr(binary[15] >= binary[5]);
  model.addConstr(binary[16] >= binary[6]);         // capsule 2 / ground contact.
  model.addConstr(binary[17] >= binary[6]);
  model.addConstr(binary[18] >= binary[6]);
  model.addConstr(binary[19] >= binary[6]);
  model.addConstr(binary[20] >= binary[7]);         // capsule 3 / ground contact.
  model.addConstr(binary[21] >= binary[7]);
  model.addConstr(binary[22] >= binary[7]);
  model.addConstr(binary[23] >= binary[7]);

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

  // std::cout<<"DELTA:  "<<delta_kc<<std::endl;

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
