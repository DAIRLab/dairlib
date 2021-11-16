//#include <stdlib.h>
//#include <stdio.h>
//#include "gurobi_c.h"
#include "gurobi_c++.h"
//#include "cassie_utils.h"
#include <Eigen/Dense>
//#include <iostream>
using namespace std;
using Eigen::MatrixXd;
//

class C3 {
  public:
    virtual int SolveSingleProjection(const MatrixXd *m) {  // Method/function defined inside the class

        std::cout << *m << std::endl;

        // Create an environment
        GRBEnv env = GRBEnv(true);
        env.set("LogToConsole", "0");
        env.set("OutputFlag", "0");
        env.set("Threads", "1");
        env.start();

        // Create an empty model
        GRBModel model = GRBModel(env);

        // Create variables
        const int n_delta_k = 3;
        const int n_binary = 3;
        //double minVal[] = {0, 0};
        //double maxVal[] = {100, 100};

        GRBVar* delta_k = 0;
        GRBVar* binary = 0;

        delta_k = model.addVars(n_delta_k, GRB_CONTINUOUS);
        for (int i = 0; i < n_delta_k; i++) {
        delta_k[i].set(GRB_DoubleAttr_LB, -GRB_INFINITY);
            }

        binary = model.addVars(n_binary, GRB_BINARY);



        //GRBVar x = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "x");
        //GRBVar y = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "y");
        //GRBVar z = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "z");
        GRBVar x = binary[0];
        GRBVar y = binary[1];
        GRBVar z = binary[2];

        GRBQuadExpr obj = x * x;

        // Set objective: maximize x + y + 2 z
        model.setObjective(x * x, GRB_MAXIMIZE);

        // Add constraint: x + 2 y + 3 z <= 4
        //model.addConstr(x + 2 * y + 3 * z <= 4, "c0");

        // Add constraint: x + y >= 1
        //model.addConstr(x + y >= 1, "c1");

        // Optimize model
        model.optimize();

        cout << x.get(GRB_StringAttr_VarName) << " "
         << x.get(GRB_DoubleAttr_X) << endl;
        cout << y.get(GRB_StringAttr_VarName) << " "
         << y.get(GRB_DoubleAttr_X) << endl;
        cout << z.get(GRB_StringAttr_VarName) << " "
         << z.get(GRB_DoubleAttr_X) << endl;

        cout << "New: "<< delta_k[0].get(GRB_DoubleAttr_X) << endl;

        cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

      }
};

int main(int   argc,
     char *argv[])
{
    MatrixXd m(2,2);
    m(0,0) = 3;
    m(1,0) = 2.5;
    m(0,1) = -1;
    m(1,1) = m(1,0) + m(0,1);

    C3 Alg;
    Alg.SolveSingleProjection(&m);


}