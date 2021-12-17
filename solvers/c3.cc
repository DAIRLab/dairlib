#include "solvers/c3.h"

namespace dairlib {
namespace solvers {

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;


C3::C3(const vector<MatrixXd>& A, const vector<MatrixXd>& B,
			 const vector<MatrixXd>& D, const vector<VectorXd>& d,
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
			 const VectorXd& d, const MatrixXd& E, const MatrixXd& F,
			 const MatrixXd& H, const VectorXd& c, const int& N,
			 const C3Options& options)
		: C3(vector<MatrixXd>(N, A), vector<MatrixXd>(N, B), vector<MatrixXd>(N, D),
				 vector<VectorXd>(N, d), vector<MatrixXd>(N, E), vector<MatrixXd>(N, F),
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
	SolveQP();
	SolveProjection();
	// Update dual variables
	// w_n* = ...
}

void C3::SolveQP() {
	// TODO: fill this in, along with the arguments
}

void C3::SolveProjection() {
	// TODO: fill this in, along with the arguments
	// TODO: replace for loop with parallelization
	for (int i = 0; i < N_; i++) {
		SolveSingleProjection(E_[i]);
	}
}

} // namespace dairlib
} // namespace solvers
