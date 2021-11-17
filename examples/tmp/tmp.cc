//#include <stdlib.h>
//#include <stdio.h>
//#include "gurobi_c.h"
#include "gurobi_c++.h"
//#include "cassie_utils.h"
#include <Eigen/Dense>
//#include <Eigen/Core>
//#include <iostream>
using namespace std;
using Eigen::MatrixXd;
//

class C3 {
  public:
    virtual MatrixXd SolveSingleProjection(const MatrixXd *G, const MatrixXd *delta_constant, const MatrixXd *Ec, const MatrixXd *Fc, const MatrixXd *cc, const MatrixXd *Hc, int n, int m, int k) {  // Method/function defined inside the class

        //debug
        //std::cout << *G << std::endl;
        //std::cout << n << std::endl;
        //std::cout << m << std::endl;
        //std::cout << k << std::endl;

        //change this part later
        const MatrixXd U = *G;
        const MatrixXd delta_c = *delta_constant;
        const MatrixXd E = *Ec;
        const MatrixXd F = *Fc;
        const MatrixXd c = *cc;
        const MatrixXd H = *Hc;

        //set up linear term in cost
        MatrixXd cost_lin = -2 * delta_c.transpose() * U;

        //std::cout << cost_lin << std::endl;  //debug

        //set up for constraints (Ex + F \lambda + Hu + c >= 0)
        MatrixXd Mcons1(m,n+m+k);
        Mcons1 << E, F, H;

        //set up for constraints (\lambda >= 0)
        MatrixXd MM1;
        MM1 = MatrixXd::Zero(m,n);
        MatrixXd MM2;
        MM2 = MatrixXd::Identity(m,m);
        MatrixXd MM3;
        MM3 = MatrixXd::Zero(m,k);
        MatrixXd Mcons2(m,n+m+k);
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
        const int n_delta_k = n+m+k;
        const int n_binary = m;

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

        int M = 1; //big M variable

        //Add constraint: M binary >= Ex + F\lambda + Hu + c >= 0
        //Add constraint: M (1-binary) >= \lambda >= 0

        for (int i = 0; i < n_binary; i++) {
                GRBLinExpr lin_holder = 0;  //make sure you are reseting lin_holder
                lin_holder.addTerms( Mcons1.row(i).data(), delta_k, n+m+k );
                model.addConstr( lin_holder + c(i,1) >= 0      );
                model.addConstr( lin_holder + c(i,1) <=  M * binary[i]      );

                GRBLinExpr lin_holder2 = 0;  //make sure you are reseting lin_holder2
                lin_holder2.addTerms( Mcons2.row(i).data(), delta_k, n+m+k );
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
        MatrixXd delta_kc(n+m+k,1);
        for (int i = 0; i < n_delta_k; i++) {
            delta_kc(i,0) = delta_k[i].get(GRB_DoubleAttr_X);

            }


        cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

        return delta_kc;

      }
};

int main(int   argc,
     char *argv[])
{

    int n = 2;
    int m = 3;
    int k = 4;

    MatrixXd T1;
    T1 = MatrixXd::Zero(n+m+k,n+m+k);
    MatrixXd T2;
    T2 = MatrixXd::Ones(n+m+k,1);

    MatrixXd E;
    E = MatrixXd::Ones(m,n);
    MatrixXd F;
    F = MatrixXd::Ones(m,m);
    MatrixXd c;
    c = MatrixXd::Ones(m,1);
    MatrixXd H;
    H = MatrixXd::Zero(m,k);

    //MatrixXd m(6,6);
    T1(0,0) = 3;
    T1(1,1) = 3;
    T1(2,2) = 3;
    T1(3,3) = 3;
    T1(4,4) = 3;
    T1(5,5) = 3;
    T1(6,6) = 3;
    T1(7,7) = 3;
    T1(8,8) = 3;

    MatrixXd test;
    test = MatrixXd::Zero(n+m+k,1);

    C3 Alg;
    test = Alg.SolveSingleProjection(&T1, &T2, &E, &F, &c, &H, n, m, k);

    cout << "Delta: "<< test << endl;

}