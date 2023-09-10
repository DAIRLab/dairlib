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
                                       const int& warm_start_index,
                                       const bool& constrain_first_x,
                                       const Eigen::VectorXd& x0) {
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

  // Create an empty model.
  GRBModel model = GRBModel(env_);

  // Gurobi parameter tuning:  description, default, min/max values.
  // Gurobi's automatic parameter tuning tool suggested Heuristics=0 and BranchDir=-1.  Adding TimeLimit=0.25 as well.
  // model.set("FeasibilityTol", "0.01");     // Primal feasibility tolerance; higher = faster (1e-6 / 1e-9 / 1e-2).
  // model.set("IntFeasTol", "0.01");         // Integer feasibility tolerance; higher = faster (1e-5 / 1e-9 / 1e-1).
  // model.set("OptimalityTol", "0.01");      // Dual feasibility tolerance; higher = faster (1e-6 / 1e-9 / 1e-2).
  // model.set("IterationLimit", "100");      // Simplex iteration limit; lower = faster (infinity / 0 / infinity).
  model.set("Heuristics", "0.0");         // Turn MIP heuristics up or down; lower = faster (0.05 / 0 / 1).
  // model.set("MIPFocus", "1");              // Set the focus of the MIP solver; see docs for details (0 / 0 / 3).
  // model.set("NumericFocus", "1");          // Set the numerical focus; see docs for details (0 / 0 / 3).
  // model.set("Cuts", "1");                  // Global cut generation control; see docs for details (-1 / -1 / 3).
  model.set("TimeLimit", "0.25");          // Time limit in seconds for MIQP; lower = faster (infinity / 0 / infinity).
  // model.set("ScaleFlag", "2");             // Model scaling; 2 uses geometric mean scaling (-1 / -1 / 3).
  // model.set("NormAdjust", "3");            // Pricing norm variant; no information in docs on what these mean (-1 / -1 / 3).
  model.set("BranchDir", "-1");            // What child node to explore first; -1 explores down branch first (0 / -1 / 1).
  // model.set("CoverCuts", "0");             // Cover cut generation; 0 disables cuts (-1 / -1 / 2).
  // model.set("PrePasses", "1");             // Presolve pass limit (-1 / -1 / MAXINT).
  // model.set("MIRCuts", "0");               // MIR cut generation (overrides "Cuts" parameter); 0 disables cuts (-1 / -1 / 2).
  // model.set("PreQLinearize", "0");         // Presolve quadratic linearization; see docs for details (-1 / -1 / 2).


  GRBVar delta_k[n_ + m_ + k_];
  GRBVar binary[m_];

  for (int i = 0; i < m_; i++) {
    binary[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
    if (warm_start_index != -1) {
      binary[i].set(GRB_DoubleAttr_Start, warm_start_binary_[warm_start_index](i));
      // binary[i].set(GRB_DoubleAttr_VarHintVal, warm_start_binary_[warm_start_index](i));
    }
  }

  for (int i = 0; i < n_ + m_ + k_; i++) {
    delta_k[i] = model.addVar(-10000.0, 10000.0, 0.0, GRB_CONTINUOUS);
    if (warm_start_index != -1) {
      delta_k[i].set(GRB_DoubleAttr_Start, warm_start_delta_[warm_start_index](i));
      // delta_k[i].set(GRB_DoubleAttr_VarHintVal, warm_start_delta_[warm_start_index](i));
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

  // Constrain x0 if necessary.
  if (constrain_first_x == true) {
    for (int i = 0; i < n_; i++) {
      model.addConstr(delta_k[i] == x0[i]);
    }
    // std::cout<<"Constraining delta0 to be: "<<x0<<std::endl;
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
  // // Enforce opposing tangential forces can't both be non-zero (or binaries can't both be zero).
  // // (This alone doesn't improve speed noticeably; 2-3Hz.)
  // model.addConstr(binary[8] + binary[9] >= 1);      // end effector / jack contact.
  // model.addConstr(binary[10] + binary[11] >= 1);
  // model.addConstr(binary[12] + binary[13] >= 1);    // capsule 1 / ground contact.
  // model.addConstr(binary[14] + binary[15] >= 1);
  // model.addConstr(binary[16] + binary[17] >= 1);    // capsule 2 / ground contact.
  // model.addConstr(binary[18] + binary[19] >= 1);
  // model.addConstr(binary[20] + binary[21] >= 1);    // capsule 3 / ground contact.
  // model.addConstr(binary[22] + binary[23] >= 1);
  // // Enforce there can be tangential forces only if there's corresponding normal force (or binary tangential can
  // // be 0 only if binary normal is also 0).
  // // (This alone doesn't improve speed noticeably; 1-2Hz.)
  // model.addConstr(binary[8] >= binary[4]);          // end effector / jack contact.
  // model.addConstr(binary[9] >= binary[4]);
  // model.addConstr(binary[10] >= binary[4]);
  // model.addConstr(binary[11] >= binary[4]);
  // model.addConstr(binary[12] >= binary[5]);         // capsule 1 / ground contact.
  // model.addConstr(binary[13] >= binary[5]);
  // model.addConstr(binary[14] >= binary[5]);
  // model.addConstr(binary[15] >= binary[5]);
  // model.addConstr(binary[16] >= binary[6]);         // capsule 2 / ground contact.
  // model.addConstr(binary[17] >= binary[6]);
  // model.addConstr(binary[18] >= binary[6]);
  // model.addConstr(binary[19] >= binary[6]);
  // model.addConstr(binary[20] >= binary[7]);         // capsule 3 / ground contact.
  // model.addConstr(binary[21] >= binary[7]);
  // model.addConstr(binary[22] >= binary[7]);
  // model.addConstr(binary[23] >= binary[7]);

  // Write the model:
  // std::cout<<"EXPORTING GUROBI MODEL"<<std::endl;
  // model.write("examples/cube/franka/gurobi_autotune/jack_miqp_model.mps");
  // std::cout<<"FINISHED EXPORTING GUROBI MODEL"<<std::endl;

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

  // std::cout<<"DELTA: "<<delta_kc<<std::endl;

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
