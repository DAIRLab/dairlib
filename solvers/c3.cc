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
			 const vector<MatrixXd>& H, const vector<VectorXd>& c, const vector<MatrixXd>& Q, const vector<MatrixXd>& R, const vector<MatrixXd>& G,
			 const C3Options& options)
		: A_(A),
			B_(B),
			D_(D),
			d_(d),
			E_(E),
			F_(F),
			H_(H),
			c_(c),
            Q_(Q),
            R_(R),
            G_(G),
			options_(options),
			N_(A.size()),
            n_(A[0].cols()),
            m_(D[0].cols()),
            k_(B[0].cols()),
            hflag_(H[0].isZero(0)) {
            // Build the QP workspace and anything else needed at construction
	// Not a bad idea to also check that all of the vector sizes and matrix/
	// dimensions match one another

}


/// Call the time-varying constructor using copies of the time-invariant
/// description
C3::C3(const MatrixXd& A, const MatrixXd& B, const MatrixXd& D,
			 const MatrixXd& d, const MatrixXd& E, const MatrixXd& F,
			 const MatrixXd& H, const VectorXd& c, const MatrixXd& Q, const MatrixXd& R, const MatrixXd& G, const int& N,
			 const C3Options& options)
		: C3(vector<MatrixXd>(N, A), vector<MatrixXd>(N, B), vector<MatrixXd>(N, D),
				 vector<MatrixXd>(N, d), vector<MatrixXd>(N, E), vector<MatrixXd>(N, F),
				 vector<MatrixXd>(N, H), vector<VectorXd>(N, c),  vector<MatrixXd>(N, Q), vector<MatrixXd>(N, R), vector<MatrixXd>(N, G), options) {}
		


VectorXd C3::Solve(VectorXd& x0, vector<VectorXd>* delta, vector<VectorXd>* w) {

    vector<MatrixXd> Gv = G_;

    VectorXd z;

    for (int i = 0; i < options_.admm_iter; i++) {
        z = ADMMStep(x0, delta, w, &Gv);
    }




	/// Any problem setup that's necessary
	/// For loop that calls ADMMStep
	/// Extract and return u[0], just to help the user out
	return z.block(k_,0,n_+m_,1);

}

VectorXd C3::ADMMStep(VectorXd& x0, vector<VectorXd>* delta, vector<VectorXd>* w, vector<MatrixXd>* Gv) {
	// TODO: SolveQP and SolveProjection are going to need the proper arguments
    vector<VectorXd> WD(N_, VectorXd::Zero(n_+m_+k_) );

    for (int i = 0; i < N_; i++) {
        WD[i] = w->at(i) - delta->at(i);
    }

    vector<VectorXd> z = SolveQP(x0, *Gv, WD);
    //std::cout << "Sol: "<< z[0] << std::endl;


    vector<VectorXd> ZW(N_, VectorXd::Zero(n_+m_+k_) );
    for (int i = 0; i < N_; i++) {
        ZW[i] = w->at(i) + z[i];
    }

    //debug
    //std::vector<MatrixXd> Gvv = *Gv;
    //std::vector<MatrixXd> Gvv(N_, 10*MatrixXd::Identity(n_+m_+k_,n_+m_+k_) );


    *delta = SolveProjection(*Gv, ZW);

	// Update dual variables
    for (int i = 0; i < N_; i++) {
        w->at(i) = w->at(i) + z[i] - delta->at(i);
        w->at(i) = w->at(i) / options_.rho_scale;
        Gv->at(i) = Gv->at(i) * options_.rho_scale;
    }

    return z[0];



    //std::cout << "Sol: "<< hflag_ << std::endl;


}

vector<VectorXd> C3::SolveQP(VectorXd& x0, vector<MatrixXd>& G, vector<VectorXd>& WD ) {

    drake::solvers::MathematicalProgram prog;
    const int TOT = N_*(n_+m_+k_) + n_;
    const int num_var = TOT;
    drake::solvers::VectorXDecisionVariable dv;
    dv = prog.NewContinuousVariables(num_var, "dv");

    //set up for dynamics constraints (x0 = value)
    MatrixXd DynEq1;
    DynEq1 = MatrixXd::Identity(n_,n_);
    MatrixXd DynEq2;
    DynEq2 = MatrixXd::Zero(n_,N_*(n_+m_+k_));
    MatrixXd DynEq3(n_,TOT);
    DynEq3 << DynEq1, DynEq2;

    //set up for dynamics constraints (A x_k + D \lambda_k + B u_k + d_k = x_{k+1})
    MatrixXd DynEq4;
    DynEq4 = MatrixXd::Zero(N_*n_,TOT);
    //Matrix to fill in
    MatrixXd DynReg(n_, 2*n_+m_+k_);
    for (int i = 0; i < N_; i++) {
        //Matrix to fill in
        DynReg << A_[i], D_[i], B_[i], -DynEq1;
        DynEq4.block(n_*i,i*(n_+m_+k_),n_,2*n_+m_+k_) = DynReg;
    }
    //set up for dynamics (combine the two)
    MatrixXd DynEq(N_*n_+n_, TOT);
    DynEq << DynEq3, DynEq4;

    //set up for dynamics constraint equality
    MatrixXd beq;
    beq = MatrixXd::Zero(N_*n_+n_, 1);

    //set the initial condition
    beq.block(0,0,n_,1) = x0;

    for (int i = 0; i < N_; i++) {
        //Matrix to fill in
        beq.block(n_*(i+1),0,n_,1) = d_[i];
        //beq.block(1,1,1,1) = 1;
    }

    prog.AddLinearEqualityConstraint(DynEq, beq, dv);

    //set up for the cost
    MatrixXd CostQR;
    CostQR = MatrixXd::Zero(TOT,TOT);
    for (int i = 0; i < N_+1; i++) {

        CostQR.block( (n_+m_+k_)*i ,(n_+m_+k_)*i,n_,n_ ) = Q_[i];


        if (i < N_) {


            CostQR.block( n_+m_+(n_+m_+k_)*i ,n_+m_+(n_+m_+k_)*i,k_,k_) = R_[i];

        }

    }


    MatrixXd CostWD;
    CostWD = MatrixXd::Zero(TOT,TOT);

    for (int i = 0; i < N_; i++) {

        CostWD.block( (n_+m_+k_)*i ,(n_+m_+k_)*i,n_+m_+k_,n_+m_+k_) = G[i];


    }


    MatrixXd Cost = CostQR + CostWD;

    //add the linear term in the cost (-2 WD^T G z_k)

    MatrixXd CostLinearWD;
    CostLinearWD = MatrixXd::Zero(TOT,1);

    for (int i = 0; i < N_; i++) {
        CostLinearWD.block( (n_+m_+k_)*(i),0,n_+m_+k_,1) = -2 * WD[i] * G[i] ;  //check the product
    }

    prog.AddQuadraticCost(2*Cost, CostLinearWD, dv, 1);

    drake::solvers::SolverOptions options;
    options.SetOption(OsqpSolver::id(), "verbose", 1);
    drake::solvers::OsqpSolver osqp;
    prog.SetSolverOptions(options);


    //MathematicalProgramResult result = drake::solvers::Solve(prog);
    MathematicalProgramResult result = osqp.Solve(prog);
    SolutionResult solution_result = result.get_solution_result();
    //std::cout << result.get_solver_id().name() << "\n";
    //bool check_success = result.is_success ();
    Eigen::VectorXd zSol = result.GetSolution();
    //std::cout << "Sol: "<< zSol << std::endl;

    vector<VectorXd> zz(N_, VectorXd::Zero(n_+m_+k_) );

    for (int i = 0; i < N_; i++) {
        zz[i] = zSol.block(n_+m_+k_*(i+1),0,n_+m_+k_,1);
    }

    return zz;

}

vector<VectorXd> C3::SolveProjection(vector<MatrixXd>& G, vector<VectorXd>& WZ) {
	// TODO: fill this in, along with the arguments
	// TODO: replace for loop with parallelization

    vector<VectorXd> delta(N_, VectorXd::Zero(n_+m_+k_) );

	for (int i = 0; i < N_; i++) {
        delta[i] = SolveSingleProjection(G[i], WZ[i], E_[i], F_[i], H_[i], c_[i]);
	}

    return delta;
}

VectorXd C3::Simulate(VectorXd& x_init, VectorXd& input) {

    VectorXd x_final;

    //calculate force
    VectorXd force;

    x_final = A_[0] * x_init + B_[0] * input + D_[0] * force + d_[0];

    return x_final;
}

} // namespace dairlib
} // namespace solvers
