#include "solvers/lcs.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/solve.h"
#include "drake/solvers/moby_lcp_solver.h"

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace dairlib {
    namespace solvers {

        LCS::LCS(const vector<MatrixXd> &A, const vector<MatrixXd> &B,
                 const vector<MatrixXd> &D, const vector<VectorXd> &d,
                 const vector<MatrixXd> &E, const vector<MatrixXd> &F,
                 const vector<MatrixXd> &H, const vector<VectorXd> &c)
                : A_(A),
                  B_(B),
                  D_(D),
                  d_(d),
                  E_(E),
                  F_(F),
                  H_(H),
                  c_(c),
                  N_(A.size()) {}

        LCS::LCS(const MatrixXd &A, const MatrixXd &B, const MatrixXd &D,
                 const VectorXd &d, const MatrixXd &E, const MatrixXd &F,
                 const MatrixXd &H, const VectorXd &c, const int &N)
                : LCS(vector<MatrixXd>(N, A), vector<MatrixXd>(N, B), vector<MatrixXd>(N, D),
                      vector<VectorXd>(N, d), vector<MatrixXd>(N, E), vector<MatrixXd>(N, F),
                      vector<MatrixXd>(N, H), vector<VectorXd>(N, c)) {}

        VectorXd LCS::Simulate(VectorXd &x_init, VectorXd &input) {

            VectorXd x_final;

            //calculate force
            drake::solvers::MobyLCPSolver<double> LCPSolver;
            VectorXd force;
            //MatrixXd regulator = MatrixXd::Identity(F_[0].size(),F_[0].size() );
            LCPSolver.SolveLcpLemke(F_[0], E_[0] * x_init + c_[0] + H_[0] * input, &force);

            //update
            x_final = A_[0] * x_init + B_[0] * input + D_[0] * force + d_[0];

            return x_final;
        }

    } // namespace dairlib
} // namespace solvers
