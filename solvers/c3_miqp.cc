#include "solvers/c3_miqp.h"


namespace dairlib {
namespace solvers {

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

C3MIQP::C3MIQP(const LCS& LCS, const vector<MatrixXd>& Q, const vector<MatrixXd>& R, const vector<MatrixXd>& G, const vector<MatrixXd>& U,
			 				 const C3Options& options)
		: C3(LCS, Q, R, G, U, options),
          env_(true) {

    // Create an environment
    env_.set("LogToConsole", "0");
    env_.set("OutputFlag", "0");
    env_.set("Threads", "1");
    env_.start();


}


    VectorXd C3MIQP::SolveSingleProjection(const MatrixXd& U, const VectorXd& delta_c, const MatrixXd& E, const MatrixXd& F, const MatrixXd& H, const VectorXd& c) {

    //set up linear term in cost
    VectorXd cost_lin = -2 * delta_c.transpose() * U;

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


    // Create an empty model
    GRBModel model = GRBModel(env_);

    //GRBVar* delta_k = 0;
    //GRBVar* binary = 0;

    //delta_k = model.addVars(n_delta_k, GRB_CONTINUOUS);
    //for (int i = 0; i < n_delta_k; i++) {
    //    delta_k[i].set(GRB_DoubleAttr_LB, -GRB_INFINITY);
    //}

    //binary = model.addVars(n_binary, GRB_BINARY);

    GRBVar delta_k[n_+m_+k_];
    GRBVar binary[m_];

    for (int i=0;i<m_;i++){
        binary[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
    }

    for (int i=0;i<n_+m_+k_;i++){
        delta_k[i] = model.addVar(-10000.0, 10000.0, 0.0, GRB_CONTINUOUS);
    }



    GRBQuadExpr obj = 0;



    for (int i = 0; i < n_+m_+k_; i++) {

        obj.addTerm(cost_lin(i), delta_k[i]);

        for (int j = 0; j < n_+m_+k_; j++) {

                obj.addTerm( U(i,j), delta_k[i], delta_k[j]);

        }
    }

    model.setObjective(obj, GRB_MINIMIZE);

    int M = 1000; //big M variable
    double coeff[n_ + m_ + k_];

    for (int i = 0; i < m_; i++) {

        GRBLinExpr cexpr = 0;

        ///convert VectorXd to double
        for (int j = 0; j < n_+m_+k_; j++) {
            coeff[j] = Mcons2.row(i)(j);
        }

        cexpr.addTerms(coeff,delta_k,n_+m_+k_);
        model.addConstr( cexpr >= 0  );
        model.addConstr( cexpr  <=  M * (1 - binary[i] )     );

        GRBLinExpr cexpr2 = 0;

        ///convert VectorXd to double
        for (int j = 0; j < n_+m_+k_; j++) {
            coeff[j] = Mcons1.row(i)(j);
        }

        cexpr2.addTerms(coeff,delta_k,n_+m_+k_);
        model.addConstr( cexpr2 + c(i) >= 0  );
        model.addConstr( cexpr2 + c(i)  <=  M * binary[i]     );



    }

   // cexpr.addTerms(coeff,delta_k,n_+m_+k_);
    //model.addConstr( cexpr >= 0      );



    //std::cout << Mcons2.row(0) << std::endl;

        /*


        for (int i = 0; i < n_binary; i++) {
            GRBLinExpr lin_holder = 0;  //make sure you are reseting lin_holder
            lin_holder.addTerms( Mcons1.row(i).data(), delta_k, n_+m_+k_ );
            model.addConstr( lin_holder + c(i) >= 0      );
            model.addConstr( lin_holder + c(i) <=  M * binary[i]      );

            GRBLinExpr lin_holder2 = 0;  //make sure you are reseting lin_holder2
            lin_holder2.addTerms( Mcons2.row(i).data(), delta_k, n_+m_+k_ );
            model.addConstr( lin_holder2  >= 0      );
            model.addConstr( lin_holder2  <=  M * (1 - binary[i] )     );
        }

         */

           // GRBLinExpr lin_holder2 = 0;  //make sure you are reseting lin_holder2
            // lin_holder2.addTerms( Mcons2.row(2).data(), delta_k, n_+m_+k_ );
           // model.addConstr( lin_holder2  >= 0      );

        //model.addConstr( lin_holder2  >= 0      );







        //GRBLinExpr lin_holder2 = 0;  //make sure you are reseting lin_holder2
        //lin_holder2.addTerms( Mcons2.row(2).data(), delta_k, n_+m_+k_ );
        //model.addConstr( lin_holder2  >= 0      );



    model.optimize();

    /*
    //Solution as MatrixXd
    MatrixXd delta_kc(n_+m_+k_,1);
    for (int i = 0; i < n_delta_k; i++) {
        delta_kc(i,0) = delta_k[i].get(GRB_DoubleAttr_X);

    }

     */

    VectorXd delta_kc(n_+m_+k_);
    for (int i = 0; i < n_+m_+k_; i++) {
        delta_kc(i) = delta_k[i].get(GRB_DoubleAttr_X);
    }

       //std::cout << "delta_kc" << std::endl;
       //std::cout << delta_kc << std::endl;

    return delta_kc;


}


} // namespace dairlib
} // namespace solvers
