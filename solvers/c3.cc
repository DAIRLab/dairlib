#include "solvers/c3.h"
#include "solvers/lcs.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/solve.h"
#include "drake/solvers/moby_lcp_solver.h"
#include <omp.h>

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


C3::C3(const LCS& LCS, const vector<MatrixXd>& Q, const vector<MatrixXd>& R, const vector<MatrixXd>& G, const vector<MatrixXd>& U,
			 const C3Options& options)
		: A_(LCS.A_),
			B_(LCS.B_),
			D_(LCS.D_),
			d_(LCS.d_),
			E_(LCS.E_),
			F_(LCS.F_),
			H_(LCS.H_),
			c_(LCS.c_),
            Q_(Q),
            R_(R),
            U_(U),
            G_(G),
			options_(options),
			N_((LCS.A_).size()),
            n_((LCS.A_)[0].cols()),
            m_((LCS.D_)[0].cols()),
            k_((LCS.B_)[0].cols()),
            hflag_(H_[0].isZero(0)) {}

VectorXd C3::Solve(VectorXd& x0, vector<VectorXd>& delta, vector<VectorXd>& w) {

    vector<MatrixXd> Gv = G_;
    VectorXd z;

    for (int i = 0; i < options_.admm_iter; i++) {

        z = ADMMStep(x0, &delta, &w, &Gv);

    }

    return z.segment(n_+m_, k_);

}

VectorXd C3::ADMMStep(VectorXd& x0, vector<VectorXd>* delta, vector<VectorXd>* w, vector<MatrixXd>* Gv) {
    vector<VectorXd> WD(N_, VectorXd::Zero(n_+m_+k_) );

    for (int i = 0; i < N_; i++) {
        WD.at(i) = delta->at(i) - w->at(i);
    }

    vector<VectorXd> z = SolveQP(x0, *Gv, WD);

    vector<VectorXd> ZW(N_, VectorXd::Zero(n_+m_+k_) );
    for (int i = 0; i < N_; i++) {
        ZW[i] = w->at(i) + z[i];
    }

    if(U_[0].isZero(0) == 0){
        vector<MatrixXd> Uv = U_;
        *delta = SolveProjection(Uv, ZW);
    }
    else {
        *delta = SolveProjection(*Gv, ZW);
    }

    for (int i = 0; i < N_; i++) {
        w->at(i) = w->at(i) + z[i] - delta->at(i);
        w->at(i) = w->at(i) / options_.rho_scale;
        Gv->at(i) = Gv->at(i) * options_.rho_scale;
    }

    return z[0];

}


vector<VectorXd> C3::SolveQP(VectorXd& x0, vector<MatrixXd>& G, vector<VectorXd>& WD ) {

    drake::solvers::MathematicalProgram prog;

    vector<drake::solvers::VectorXDecisionVariable> x;
    vector<drake::solvers::VectorXDecisionVariable> u;
    vector<drake::solvers::VectorXDecisionVariable> lambda;


    for (int i = 0; i < N_+1; i++) {
        x.push_back(prog.NewContinuousVariables(n_, "x" + std::to_string(i)));
        if (i < N_) {
            u.push_back(prog.NewContinuousVariables(k_, "k" + std::to_string(i)));
            lambda.push_back(prog.NewContinuousVariables(m_, "lambda" + std::to_string(i)));
        }
}

    prog.AddLinearConstraint(x[0] == x0);


    if (hflag_ == 1) {
        drake::solvers::MobyLCPSolver<double> LCPSolver;
        VectorXd lambda0;
        LCPSolver.SolveLcpLemke(F_[0], E_[0] * x0 + c_[0], &lambda0);
        prog.AddLinearConstraint(lambda[0] == lambda0);
        }



    for (int i = 0; i < N_; i++) {


        prog.AddLinearConstraint(A_.at(i) * x.at(i) + B_.at(i) * u.at(i) + D_.at(i) * lambda.at(i) + d_.at(i) == x.at(i+1));

        //for finger gaiting
        prog.AddLinearConstraint(u[i](2) >= 0);
        prog.AddLinearConstraint(u[i](3) >= 0);
        if (i > 0) {
            prog.AddLinearConstraint(x[i](2) >= 1); prog.AddLinearConstraint(x[i](2) <= 3);
            prog.AddLinearConstraint(x[i](4) >= 3); prog.AddLinearConstraint(x[i](4) <= 5);
        }


    }


    for (int i = 0; i < N_+1 ; i++) {
        prog.AddQuadraticCost(Q_.at(i)*2, VectorXd::Zero(n_), x.at(i));
        if (i < N_) {
            prog.AddQuadraticCost(R_.at(i)*2, VectorXd::Zero(k_), u.at(i));
            prog.AddQuadraticCost(G.at(i).block(0,0,n_,n_) * 2, -2 * G.at(i).block(0,0,n_,n_) * WD.at(i).segment(0,n_), x.at(i) );
            prog.AddQuadraticCost(G.at(i).block(n_,n_,m_,m_) * 2, -2 * G.at(i).block(n_,n_,m_,m_) * WD.at(i).segment(n_,m_), lambda.at(i) );
            prog.AddQuadraticCost(G.at(i).block(n_+m_,n_+m_,k_,k_) * 2,  -2 * G.at(i).block(n_+m_,n_+m_,k_,k_) * WD.at(i).segment(n_+m_,k_), u.at(i) );

        }
    }




    drake::solvers::SolverOptions options;
    options.SetOption(OsqpSolver::id(), "verbose", 0);
    drake::solvers::OsqpSolver osqp;
    prog.SetSolverOptions(options);
    MathematicalProgramResult result = osqp.Solve(prog);
    VectorXd xSol = result.GetSolution(x[0]);
    vector<VectorXd> zz(N_, VectorXd::Zero(n_+m_+k_) );

    for (int i = 0; i < N_; i++) {

        zz.at(i).segment(0,n_) = result.GetSolution(x[i]);
        zz.at(i).segment(n_,m_) = result.GetSolution(lambda[i]);
        zz.at(i).segment(n_+m_,k_) = result.GetSolution(u[i]);
    }

    return zz;

}

vector<VectorXd> C3::SolveProjection(vector<MatrixXd>& G, vector<VectorXd>& WZ) {

    vector<VectorXd> deltaProj(N_, VectorXd::Zero(n_+m_+k_) );
    int i;
    if (options_.num_threads > 0) {
        omp_set_dynamic(0);     // Explicitly disable dynamic teams
        omp_set_num_threads(options_.num_threads); // Set number of threads
    }

    #pragma omp parallel for
	for (i = 0; i < N_; i++) {
        deltaProj[i] = SolveSingleProjection(G[i], WZ[i], E_[i], F_[i], H_[i], c_[i]);
	}

    return deltaProj;
}


} // namespace dairlib
} // namespace solvers
