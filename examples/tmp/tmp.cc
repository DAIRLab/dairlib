//#include <stdlib.h>
//#include <stdio.h>
//#include "gurobi_c.h"
#include "gurobi_c++.h"
//#include "cassie_utils.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/solve.h"
//#include <iostream>
using namespace std;
using Eigen::MatrixXd;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::SolutionResult;

using drake::solvers::OsqpSolver;
using drake::solvers::OsqpSolverDetails;
using drake::solvers::Solve;
//

class C3 {
  public:
    virtual void SolveQP(const vector<MatrixXd> *AA, const vector<MatrixXd> *BB, const vector<MatrixXd> *DD, const vector<MatrixXd> *zz, const int n, const int m,const int k,const int N, const MatrixXd *xx0, const vector<MatrixXd> *QQ, const vector<MatrixXd> *RR, const vector<MatrixXd> *GG, const vector<MatrixXd> *WMD ){

        const vector<MatrixXd> A = *AA;
        const vector<MatrixXd> B = *BB;
        const vector<MatrixXd> D = *DD;
        const vector<MatrixXd> z = *zz;
        const vector<MatrixXd> Q = *QQ;
        const vector<MatrixXd> R = *RR;
        const vector<MatrixXd> G = *GG;
        const vector<MatrixXd> WD = *WMD;
        const MatrixXd x0 = *xx0;

        //cout << "A: "<< A[0] << endl;

        MathematicalProgram prog;


        const int TOT = N*(n+m+k) + n;
        const int num_var = TOT;
        //std::unique_ptr<drake::solvers::MathematicalProgram> prog_;
        //drake::solvers::VectorXDecisionVariable dv_;
        //dv_ = prog_->NewContinuousVariables(size, "dv");

        drake::solvers::VectorXDecisionVariable dv;
        dv = prog.NewContinuousVariables(num_var, "dv");

        //set up for dynamics constraints (x0 = value)
        MatrixXd DynEq1;
        DynEq1 = MatrixXd::Identity(n,n);
        MatrixXd DynEq2;
        DynEq2 = MatrixXd::Zero(n,N*(n+m+k));
        MatrixXd DynEq3(n,TOT);
        DynEq3 << DynEq1, DynEq2;

        //set up for dynamics constraints (A x_k + D \lambda_k + B u_k + z_k = x_{k+1})
        MatrixXd DynEq4;
        DynEq4 = MatrixXd::Zero(N*n,TOT);
        //Matrix to fill in
        MatrixXd DynReg(n, 2*n+m+k);

        for (int i = 0; i < N; i++) {
            //Matrix to fill in
            DynReg << A[i], D[i], B[i], -DynEq1;
            DynEq4.block(n*i,i*(n+m+k),n,2*n+m+k) = DynReg;
        }

        //set up for dynamics (combine the two)
        MatrixXd DynEq(N*n+n, TOT);
        DynEq << DynEq3, DynEq4;

        //testing for dynamics
        cout << "DynEq: "<< DynEq << endl;

        //set up for dynamics constraint equality
        MatrixXd beq;
        beq = MatrixXd::Zero(N*n+n, 1);

        //set the initial condition
        beq.block(0,0,n,1) = x0;

        for (int i = 0; i < N; i++) {
            //Matrix to fill in
            beq.block(n*(i+1),0,n,1) = z[i];
            //beq.block(1,1,1,1) = 1;
        }

        //testing
        cout << "beq: "<< beq << endl;

        prog.AddLinearEqualityConstraint(DynEq, beq, dv);


        //set up for the cost
        MatrixXd CostQR;
        CostQR = MatrixXd::Zero(TOT,TOT);
        for (int i = 0; i < N+1; i++) {

            CostQR.block( (n+m+k)*i ,(n+m+k)*i,n,n) = Q[i];

            if (i < N) {


                CostQR.block( n+m+(n+m+k)*i ,n+m+(n+m+k)*i,k,k) = R[i];

            }

        }

        MatrixXd CostWD;
        CostWD = MatrixXd::Zero(TOT,TOT);

        for (int i = 0; i < N; i++) {

            CostWD.block( (n+m+k)*i ,(n+m+k)*i,n+m+k,n+m+k) = G[i];


        }

        MatrixXd Cost = CostQR + CostWD;

        //testing
        cout << "Cost: "<< Cost << endl;


        //add the linear term in the cost (-2 WD^T G z_k)

        MatrixXd CostLinearWD;
        CostLinearWD = MatrixXd::Zero(TOT,1);

        for (int i = 0; i < N; i++) {
            CostLinearWD.block( (n+m+k)*(i),0,n+m+k,1) = -2 * WD[i] * G[i] ;  //check the product
        }

        //testing
        cout << "CostLinear: "<< CostLinearWD << endl;

        prog.AddQuadraticCost(2*Cost, CostLinearWD, dv, 1);

        //Eigen::Matrix2d Aeq;
        //Aeq << -1, 2,
        //1, 1;
        //Eigen::Vector2d beq(1, 3);

        //prog.AddLinearEqualityConstraint(DynEq, x0, dv.head<2>());

        drake::solvers::SolverOptions options;
        options.SetOption(OsqpSolver::id(), "verbose", 1);
        drake::solvers::OsqpSolver osqp;
        prog.SetSolverOptions(options);

        //MathematicalProgramResult result = drake::solvers::Solve(prog);
        MathematicalProgramResult result = osqp.Solve(prog);
        SolutionResult solution_result = result.get_solution_result();
        std::cout << result.get_solver_id().name() << "\n";
        bool check_success = result.is_success ();
        Eigen::MatrixXd zSol = result.GetSolution();

        //testing
        cout << "success: "<< check_success << endl;
        cout << "Sol: "<< zSol << endl;
        cout << std::to_string(solution_result) << endl;

        //std::unique_ptr<solvers::FastOsqpSolver> solver_;
        //solver_->Solve(*prog_);





       /*
        auto y = prog.NewContinuousVariables<1>("y");
        Eigen::Vector3d lb(0, 1, 2);
        Eigen::Vector3d ub(1, 2, 3);
        // Imposes the constraint
        // 0 ≤ x(0) ≤ 1
        // 1 ≤ x(1) ≤ 2
        // 2 ≤ y    ≤ 3
        prog.AddBoundingBoxConstraint(lb, ub, {x, y});
        */
    }



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

        int M = 1000; //big M variable

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

        //debug
        //cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

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

    int N = 3;
    std::vector<MatrixXd> AA(N, MatrixXd::Ones(n,n) );
    std::vector<MatrixXd> BB(N, MatrixXd::Ones(n,k) );
    std::vector<MatrixXd> DD(N, MatrixXd::Ones(n,m) );
    std::vector<MatrixXd> zz(N, MatrixXd::Zero(n,1) );
    //std::vector<MatrixXd> xx0(n, MatrixXd::Ones(n,1) );
    std::vector<MatrixXd> QQ(N+1, MatrixXd::Identity(n,n) );
    std::vector<MatrixXd> RR(N, MatrixXd::Identity(k,k) );
    std::vector<MatrixXd> KK(N, MatrixXd::Identity(n+m+k,n+m+k) );
    std::vector<MatrixXd> WMD(N, MatrixXd::Zero(n+m+k,1) );


    MatrixXd xx0;
    xx0 = MatrixXd::Zero(n,1);

    AA[0] = MatrixXd::Zero(n,n);

    cout << "AA: "<< AA[2] << endl;

    Alg.SolveQP(&AA, &BB, &DD, &zz, n, m, k, N, &xx0, &QQ, &RR, &KK, &WMD);

    //debug
    //cout << "Delta: "<< test << endl;

}