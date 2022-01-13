#include "solvers/c3.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/solve.h"

namespace dairlib {
namespace solvers {

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::SolutionResult;

using drake::solvers::OsqpSolver;
using drake::solvers::OsqpSolverDetails;
using drake::solvers::Solve;


C3::C3(const vector<MatrixXd>& A, const vector<MatrixXd>& B,
			 const vector<MatrixXd>& D, const vector<MatrixXd>& d,
			 const vector<MatrixXd>& E, const vector<MatrixXd>& F,
			 const vector<MatrixXd>& H, const vector<VectorXd>& c,
			 const C3Options& options)
		: A_(A),
			B_(B),
			D_(D),
			d_(d),
			E_(E),
			F_(F),
			H_(H),
			c_(c),
			options_(options),
			N_(A.size()) {
	// TODO: build the QP workspace and anything else needed at construction
	// Not a bad idea to also check that all of the vector sizes and matrix/
	// dimensions match one another
}


/// Call the time-varying constructor using copies of the time-invariant
/// description
C3::C3(const MatrixXd& A, const MatrixXd& B, const MatrixXd& D,
			 const MatrixXd& d, const MatrixXd& E, const MatrixXd& F,
			 const MatrixXd& H, const VectorXd& c, const int& N,
			 const C3Options& options)
		: C3(vector<MatrixXd>(N, A), vector<MatrixXd>(N, B), vector<MatrixXd>(N, D),
				 vector<MatrixXd>(N, d), vector<MatrixXd>(N, E), vector<MatrixXd>(N, F),
				 vector<MatrixXd>(N, H), vector<VectorXd>(N, c), options) {}
		


VectorXd C3::Solve(const VectorXd& x0, const VectorXd& z0,
									 const VectorXd& delta0, const VectorXd& w0,
									 VectorXd* z, VectorXd* delta, VectorXd* w) {
	/// Any problem setup that's necessary
	/// For loop that calls ADMMStep
	/// Extract and return u[0], just to help the user out
	return VectorXd(1);

}

void C3::ADMMStep(const VectorXd& z, const VectorXd& delta, const VectorXd& w, 
								  VectorXd* z_n, VectorXd* delta_n, VectorXd* w_n) {
	// TODO: SolveQP and SolveProjection are going to need the proper arguments
	//SolveQP();
	SolveProjection();
	// Update dual variables
	// w_n* = ...
}

void C3::SolveQP(const vector<MatrixXd>& A,const vector<MatrixXd>& B, const vector<MatrixXd>& D, const vector<MatrixXd>& d, const int& n, const int& m,const int& k,const int& N, const VectorXd& x0, const vector<MatrixXd>& Q, const vector<MatrixXd>& R, const vector<MatrixXd>& G, const vector<MatrixXd>& WD ) {

    // TODO: fill this in, along with the arguments
    drake::solvers::MathematicalProgram prog;
    const int TOT = N*(n+m+k) + n;
    const int num_var = TOT;
    drake::solvers::VectorXDecisionVariable dv;
    dv = prog.NewContinuousVariables(num_var, "dv");

    //set up for dynamics constraints (x0 = value)
    MatrixXd DynEq1;
    DynEq1 = MatrixXd::Identity(n,n);
    MatrixXd DynEq2;
    DynEq2 = MatrixXd::Zero(n,N*(n+m+k));
    MatrixXd DynEq3(n,TOT);
    DynEq3 << DynEq1, DynEq2;

    //set up for dynamics constraints (A x_k + D \lambda_k + B u_k + d_k = x_{k+1})
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

    //set up for dynamics constraint equality
    MatrixXd beq;
    beq = MatrixXd::Zero(N*n+n, 1);

    //set the initial condition
    beq.block(0,0,n,1) = x0;

    for (int i = 0; i < N; i++) {
        //Matrix to fill in
        beq.block(n*(i+1),0,n,1) = d[i];
        //beq.block(1,1,1,1) = 1;
    }

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

    //add the linear term in the cost (-2 WD^T G z_k)

    MatrixXd CostLinearWD;
    CostLinearWD = MatrixXd::Zero(TOT,1);

    for (int i = 0; i < N; i++) {
        CostLinearWD.block( (n+m+k)*(i),0,n+m+k,1) = -2 * WD[i] * G[i] ;  //check the product
    }

    prog.AddQuadraticCost(2*Cost, CostLinearWD, dv, 1);

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

}

void C3::SolveProjection() {
	// TODO: fill this in, along with the arguments
	// TODO: replace for loop with parallelization
	for (int i = 0; i < N_; i++) {
		//SolveSingleProjection(U_[i]);
	}
}

} // namespace dairlib
} // namespace solvers
