#include "solvers/c3_miqp.h"


namespace dairlib {
namespace solvers {

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

C3MIQP::C3MIQP(const vector<MatrixXd>& A, const vector<MatrixXd>& B,
			 				 const vector<MatrixXd>& D, const vector<VectorXd>& d,
			 				 const vector<MatrixXd>& E, const vector<MatrixXd>& F,
			 				 const vector<MatrixXd>& H, const vector<VectorXd>& c,
			 				 const C3Options& options)
		: C3(A, B, D, d, E, F, H, c, options) {
	// TODO: any MIQP-specific workspace construction goes here
}


/// Call the time-varying constructor using copies of the time-invariant
/// description
C3MIQP::C3MIQP(const MatrixXd& A, const MatrixXd& B, const MatrixXd& D,
			 const VectorXd& d, const MatrixXd& E, const MatrixXd& F,
			 const MatrixXd& H, const VectorXd& c, const int& N,
			 const C3Options& options)
		: C3MIQP(vector<MatrixXd>(N, A), vector<MatrixXd>(N, B), vector<MatrixXd>(N, D),
				 vector<VectorXd>(N, d), vector<MatrixXd>(N, E), vector<MatrixXd>(N, F),
				 vector<MatrixXd>(N, H), vector<VectorXd>(N, c), options) {}

void C3MIQP::SolveSingleProjection(const MatrixXd& E) {
	// TODO: do stuff here
}

} // namespace dairlib
} // namespace solvers
