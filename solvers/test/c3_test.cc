#include "solvers/c3_miqp.h"

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace dairlib {
namespace solvers {

int DoMain(int argc, char* argv[]) {
	/// This just moves your test code into the dairlib::solvers namespace
	/// Makes life a bit easier
	
	// You can use this function as a place to test out the different methods
	// in C3
	// Make sure you build a C3MIQP (C3 is virtual, so you can't actually build it)

    const int n = 2;
    const int m = 3;
    const int k = 4;
    const int N = 3;

    const std::vector<MatrixXd> A(N, MatrixXd::Ones(n,n) );
    const std::vector<MatrixXd> B(N, MatrixXd::Ones(n,k) );
    const std::vector<MatrixXd> D(N, MatrixXd::Ones(n,m) );
    const std::vector<MatrixXd> d(N, MatrixXd::Zero(n,1) );
    const std::vector<MatrixXd> E(N, MatrixXd::Ones(m,n) );
    const std::vector<MatrixXd> F(N, MatrixXd::Ones(m,m) );
    const std::vector<VectorXd> c(N, VectorXd::Ones(m) );
    const std::vector<MatrixXd> H(N, MatrixXd::Zero(m,k) );
    const std::vector<MatrixXd> Q(N+1, MatrixXd::Ones(n,n) );
    const std::vector<MatrixXd> R(N, MatrixXd::Ones(k,k) );
    const C3Options options;

    C3MIQP opt(A, B, D, d, E, F, H, c, Q, R, options);

    VectorXd x0 = VectorXd::Zero(n);
    std::vector<VectorXd> delta(N, VectorXd::Ones(m) );
    std::vector<VectorXd> w(N, VectorXd::Ones(m) );

    opt.ADMMStep(x0, delta, w);

    //opt.SolveQP();


	return 0;
}

}
}

int main(int argc, char* argv[]) {
	return dairlib::solvers::DoMain(argc, argv);
}