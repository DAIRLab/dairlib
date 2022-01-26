#include "solvers/c3_miqp.h"


namespace dairlib {
namespace solvers {

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

C3MIQP::C3MIQP(const vector<MatrixXd>& A, const vector<MatrixXd>& B,
			 				 const vector<MatrixXd>& D, const vector<MatrixXd>& d,
			 				 const vector<MatrixXd>& E, const vector<MatrixXd>& F,
			 				 const vector<MatrixXd>& H, const vector<VectorXd>& c, const vector<MatrixXd>& Q, const vector<MatrixXd>& R, const vector<MatrixXd>& G,
			 				 const C3Options& options)
		: C3(A, B, D, d, E, F, H, c, Q, R, G, options) {
	// TODO: any MIQP-specific workspace construction goes here
}


/// Call the time-varying constructor using copies of the time-invariant
/// description
C3MIQP::C3MIQP(const MatrixXd& A, const MatrixXd& B, const MatrixXd& D,
			 const MatrixXd& d, const MatrixXd& E, const MatrixXd& F,
			 const MatrixXd& H, const VectorXd& c,  const MatrixXd& Q, const MatrixXd& R, const MatrixXd& G, const int& N,
			 const C3Options& options)
		: C3MIQP(vector<MatrixXd>(N, A), vector<MatrixXd>(N, B), vector<MatrixXd>(N, D),
				 vector<MatrixXd>(N, d), vector<MatrixXd>(N, E), vector<MatrixXd>(N, F),
				 vector<MatrixXd>(N, H), vector<VectorXd>(N, c), vector<MatrixXd>(N+1, Q), vector<MatrixXd>(N, R), vector<MatrixXd>(N, G),options) {}

    VectorXd C3MIQP::SolveSingleProjection(const MatrixXd& U, const VectorXd& delta_c, const MatrixXd& E, const MatrixXd& F, const MatrixXd& H, const VectorXd& c) {

    //set up linear term in cost
    MatrixXd cost_lin = -2 * delta_c.transpose() * U;

    //std::cout << cost_lin << std::endl;  //debug

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

    //std::cout << Mcons2 << std::endl; //debug

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


    //debug
    //GRBVar x = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "x");
    //GRBVar y = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "y");
    //GRBVar z = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "z");
    //GRBVar x = binary[0];
    //GRBVar y = binary[1];
    //GRBVar z = binary[2];

    //maybe vectorize?
    GRBQuadExpr obj = 0;
    for (int i = 0; i < n_delta_k; i++) {

        obj.addTerm(cost_lin(i,1), delta_k[i]);

        for (int j = 0; j < n_delta_k; j++) {
            obj.addTerm( U(i,j), delta_k[i] , delta_k[j]);
        }
    }

    // Set objective: maximize x + y + 2 z
    model.setObjective(obj, GRB_MINIMIZE);

    int M = 1000; //big M variable

    //Add constraint: M binary >= Ex + F\lambda + Hu + c >= 0
    //Add constraint: M (1-binary) >= \lambda >= 0

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





    // Add constraint: x + 2 y + 3 z <= 4
    //model.addConstr(x + 2 * y + 3 * z <= 4, "c0");

    // Add constraint: x + y >= 1
    //model.addConstr(x + y >= 1, "c1");

    // Optimize model
    model.optimize();

    /*
    cout << x.get(GRB_StringAttr_VarName) << " "
     << x.get(GRB_DoubleAttr_X) << endl;
    cout << y.get(GRB_StringAttr_VarName) << " "
     << y.get(GRB_DoubleAttr_X) << endl;
    cout << z.get(GRB_StringAttr_VarName) << " "
     << z.get(GRB_DoubleAttr_X) << endl;
    */

    //Solution as MatrixXd
    MatrixXd delta_kc(n_+m_+k_,1);
    for (int i = 0; i < n_delta_k; i++) {
        delta_kc(i,0) = delta_k[i].get(GRB_DoubleAttr_X);

    }

    return delta_kc;
}

} // namespace dairlib
} // namespace solvers
