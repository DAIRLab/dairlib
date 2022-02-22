#include "solvers/c3_miqp.h"


namespace dairlib {
namespace solvers {

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

C3MIQP::C3MIQP(const LCS& LCS, const vector<MatrixXd>& Q, const vector<MatrixXd>& R, const vector<MatrixXd>& G, const vector<MatrixXd>& U,
			 				 const C3Options& options)
		: C3(LCS, Q, R, G, U, options) {

}

    VectorXd C3MIQP::SolveSingleProjection(const MatrixXd& U, const VectorXd& delta_c, const MatrixXd& E, const MatrixXd& F, const MatrixXd& H, const VectorXd& c) {

    //set up linear term in cost
    MatrixXd cost_lin = -2 * delta_c.transpose() * U;

    //set up for constraints (Ex + F \lambda + Hu + c >= 0)
    MatrixXd Mcons1(m_,n_+m_+k_);
    Mcons1 << E, F, H;

    //set up for constraints (\lambda >= 0)
    MatrixXd MM1;
    MM1 = MatrixXd::Zero(m_,n_);
    MatrixXd MM2;
    MM2 = MatrixXd::Identity(m_,m_);
    MatrixXd MM3;
    MM3 = MatrixXd::Zero(m_,k_);
    MatrixXd Mcons2(m_,n_+m_+k_);
    Mcons2 << MM1, MM2, MM3;

    // Create an environment
    GRBEnv env = GRBEnv(true);
    env.set("LogToConsole", "0");
    env.set("OutputFlag", "0");
    env.set("Threads", "1");
    env.start();

    // Create an empty model
    GRBModel model = GRBModel(env);

    // Create variables
    const int n_delta_k = n_+m_+k_;
    const int n_binary = m_;

    GRBVar* delta_k = 0;
    GRBVar* binary = 0;

    delta_k = model.addVars(n_delta_k, GRB_CONTINUOUS);
    for (int i = 0; i < n_delta_k; i++) {
        delta_k[i].set(GRB_DoubleAttr_LB, -GRB_INFINITY);
    }

    binary = model.addVars(n_binary, GRB_BINARY);

    GRBQuadExpr obj = 0;
    for (int i = 0; i < n_delta_k; i++) {

        obj.addTerm(cost_lin(i,1), delta_k[i]);

        for (int j = 0; j < n_delta_k; j++) {
            obj.addTerm( U(i,j), delta_k[i] , delta_k[j]);
        }
    }

    model.setObjective(obj, GRB_MINIMIZE);

    int M = 1000; //big M variable

    for (int i = 0; i < n_binary; i++) {
        GRBLinExpr lin_holder = 0;  //make sure you are reseting lin_holder
        lin_holder.addTerms( Mcons1.row(i).data(), delta_k, n_+m_+k_ );
        model.addConstr( lin_holder + c(i,1) >= 0      );
        model.addConstr( lin_holder + c(i,1) <=  M * binary[i]      );

        GRBLinExpr lin_holder2 = 0;  //make sure you are reseting lin_holder2
        lin_holder2.addTerms( Mcons2.row(i).data(), delta_k, n_+m_+k_ );
        model.addConstr( lin_holder2  >= 0      );
        model.addConstr( lin_holder2  <=  M * (1 - binary[i] )     );
    }

    model.optimize();

    //Solution as MatrixXd
    MatrixXd delta_kc(n_+m_+k_,1);
    for (int i = 0; i < n_delta_k; i++) {
        delta_kc(i,0) = delta_k[i].get(GRB_DoubleAttr_X);

    }

    return delta_kc;
}

} // namespace dairlib
} // namespace solvers
