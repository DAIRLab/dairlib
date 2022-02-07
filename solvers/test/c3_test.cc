#include "solvers/c3_miqp.h"
#include "drake/math/discrete_algebraic_riccati_equation.h"

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

    const int n = 4;
    const int m = 2;
    const int k = 1;
    const int N = 10;

    float g = 9.81;
    float mp = 0.411;
    float mc = 0.978;
    float len_p = 0.6;
    float len_com = 0.4267;
    float d1 = 0.35;
    float d2 = -0.35;
    float ks = 100;
    float Ts = 0.01;

    MatrixXd Ainit(n,n);
    Ainit << 0, 0, 1, 0,
            0, 0, 0, 1,
            0, g*mp/mc, 0, 0,
            0, (g*(mc+mp))/(len_com*mc), 0, 0;
    //std::cout << Ainit;

    MatrixXd Binit(n,k);
    Binit << 0,
            0,
            1/mc,
            1/(len_com*mc);
    //std::cout << Binit;

    MatrixXd Dinit(n,m);
    Dinit << 0, 0,
            0, 0,
            (-1/mc) + (len_p/(mc*len_com)), (1/mc) - (len_p/(mc*len_com)),
            (-1 / (mc*len_com) ) + (len_p*(mc+mp)) / (mc*mp*len_com*len_com), -((-1 / (mc*len_com) ) + (len_p*(mc+mp)) / (mc*mp*len_com*len_com));
    //std::cout << Dinit;

    MatrixXd Einit(m,n);
    Einit << -1, len_p, 0, 0,
            1, -len_p, 0, 0;
    //std::cout << Einit;

    MatrixXd Finit(m,m);
    Finit << 1/ks, 0,
            0, 1/ks;
    //std::cout << Finit;

    VectorXd cinit(m);
    cinit << d1,
            -d2;
    //std::cout << cinit;

    MatrixXd dinit(n,1);
    dinit << 0,
            0,
            0,
            0;
    //std::cout << dinit;

    MatrixXd Hinit = MatrixXd::Zero(m,k);
    //std::cout << Hinit;

    MatrixXd Qinit(n,n);
    Qinit << 10, 0, 0, 0,
            0, 3, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    //std::cout << Qinit;

    MatrixXd Rinit(k,k);
    Rinit << 1;
    //std::cout << Rinit;

    MatrixXd Ginit(n+m+k, n+m+k);
    Ginit = 0.1*MatrixXd::Identity(n+m+k,n+m+k); Ginit(n+m+k-1,n+m+k-1) = 0;
    //std::cout << Ginit;

    MatrixXd QNinit = drake::math::DiscreteAlgebraicRiccatiEquation(Ainit * Ts + MatrixXd::Identity(n,n), Ts*Binit, Qinit, Rinit);
    //std::cout << QNinit;


    std::vector<MatrixXd> Qsetup(N+1, Qinit );
    Qsetup.at(N) = QNinit; //Switch to QNinit, solution of Algebraic Ricatti



    const std::vector<MatrixXd> A(N, Ainit * Ts + MatrixXd::Identity(n,n) );
    const std::vector<MatrixXd> B(N, Ts*Binit );
    const std::vector<MatrixXd> D(N, Ts*Dinit );
    const std::vector<MatrixXd> d(N, Ts*dinit );
    const std::vector<MatrixXd> E(N, Einit );
    const std::vector<MatrixXd> F(N, Finit );
    const std::vector<VectorXd> c(N, cinit );
    const std::vector<MatrixXd> H(N, Hinit );
    const std::vector<MatrixXd> Q = Qsetup;
    const std::vector<MatrixXd> R(N, Rinit );
    const std::vector<MatrixXd> G(N, Ginit );

    C3Options options;
    // options.num_threads = 5;

    C3MIQP opt(A, B, D, d, E, F, H, c, Q, R, G, options);



    //VectorXd x0 = VectorXd::Zero(n);

    VectorXd x0(n);
    x0 << 0.1,
            0,
            0.3,
            0;


    std::vector<VectorXd> delta_reset(N, VectorXd::Zero(n+m+k) );
    std::vector<VectorXd> w_reset(N, VectorXd::Zero(n+m+k) );
    std::vector<VectorXd> delta(N, VectorXd::Zero(n+m+k) );
    std::vector<VectorXd> w(N, VectorXd::Zero(n+m+k) );

    int timesteps = 500;

    std::vector<VectorXd> x(timesteps, VectorXd::Zero(n) );
    std::vector<VectorXd> input(timesteps, VectorXd::Zero(k) );

    x[0] = x0;

    for (int i = 0; i < timesteps-1; i++) {

        //THIS MIGHT NOT WORK
        delta = delta_reset;
        w = w_reset;

        input[i] = opt.Solve(x[i], &delta, &w );
        x[i+1] = opt.Simulate(x[i], input[i]);
        std::cout << "state: "<< x[i+1] << std::endl;

    }

	return 0;
}

}
}

int main(int argc, char* argv[]) {
	return dairlib::solvers::DoMain(argc, argv);
}