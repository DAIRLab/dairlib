#include "solvers/c3_miqp.h"
#include "drake/math/discrete_algebraic_riccati_equation.h"

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

void init(int* n_, int* m_, int* k_, int* N_, vector<MatrixXd>* A_, vector<MatrixXd>* B_, vector<MatrixXd>* D_, vector<MatrixXd>* d_, vector<MatrixXd>* E_, vector<MatrixXd>* F_, vector<MatrixXd>* H_, vector<VectorXd>* c_, vector<MatrixXd>* Q_, vector<MatrixXd>* R_, vector<MatrixXd>* G_, vector<MatrixXd>* U_);

namespace dairlib {
namespace solvers {

int DoMain(int argc, char* argv[]) {

    ///dimensions (n: state dimension, m: complementarity variable dimension, k: input dimension, N: MPC horizon)
    int nd, md, kd, Nd;
    ///variables (LCS)
    vector<MatrixXd> Ad, Bd, Dd, dd, Ed, Fd, Hd;
    vector<VectorXd> cd;
    ///variables (cost, ADMM)
    vector<MatrixXd> Qd, Rd, Gd, Ud;
    ///initialize (parameters for cartpole, change wrt your specific system)
    init(&nd, &md, &kd, &Nd, &Ad, &Bd, &Dd, &dd, &Ed, &Fd, &Hd, &cd, &Qd, &Rd, &Gd, &Ud);
    ///set parameters as const
    const vector<MatrixXd> A = Ad; const vector<MatrixXd> B = Bd; const vector<MatrixXd> D = Dd; const vector<MatrixXd> E = Ed; const vector<MatrixXd> F = Fd; const vector<MatrixXd> H = Hd; const vector<MatrixXd> d = dd; const vector<VectorXd> c = cd; const vector<MatrixXd> Q = Qd; const vector<MatrixXd> R = Rd; const vector<MatrixXd> G = Gd; const vector<MatrixXd> U = Ud; const int N = Nd; const int n = nd; const int m = md; const int k = kd;
    ///options
    C3Options options;

    ///define the LCS class variable
    LCS cartpole(A, B, D, d, E, F, H, c);
    C3MIQP opt(cartpole, Q, R, G, U, options);

    ///initial condition
    VectorXd x0(n);
    x0 << 0.1,
            0,
            0.3,
            0;

    ///initialize ADMM variables (delta, w)
    std::vector<VectorXd> delta(N, VectorXd::Zero(n+m+k) );
    std::vector<VectorXd> w(N, VectorXd::Zero(n+m+k) );

    ///initialize ADMM reset variables (delta, w are reseted to these values)
    std::vector<VectorXd> delta_reset(N, VectorXd::Zero(n+m+k) );
    std::vector<VectorXd> w_reset(N, VectorXd::Zero(n+m+k) );

    ///create state and input arrays
    std::vector<VectorXd> x(options.timesteps, VectorXd::Zero(n) );
    std::vector<VectorXd> input(options.timesteps, VectorXd::Zero(k) );

    ///initialize at x0
    x[0] = x0;

    for (int i = 0; i < options.timesteps-1; i++) {

        ///reset delta and w
        delta = delta_reset;
        w = w_reset;

        ///calculate the input given x[i]
        //input[i] = opt.Solve(x[i], &delta, &w );
        input[i] = opt.Solve(x[i], delta, w );

        ///simulate the LCS
        x[i+1] = cartpole.Simulate(x[i], input[i]);

        ///print the state
        std::cout << "state: "<< x[i+1] << std::endl;

    }

	return 0;
}

}
}

int main(int argc, char* argv[]) {
	return dairlib::solvers::DoMain(argc, argv);
}

///initialize LCS parameters
void init(int* n_, int* m_, int* k_, int* N_, vector<MatrixXd>* A_, vector<MatrixXd>* B_, vector<MatrixXd>* D_, vector<MatrixXd>* d_, vector<MatrixXd>* E_, vector<MatrixXd>* F_, vector<MatrixXd>* H_, vector<VectorXd>* c_, vector<MatrixXd>* Q_, vector<MatrixXd>* R_, vector<MatrixXd>* G_, vector<MatrixXd>* U_) {

    int n = 4;
    int m = 2;
    int k = 1;
    int N = 10;

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

    MatrixXd Binit(n,k);
    Binit << 0,
            0,
            1/mc,
            1/(len_com*mc);

    MatrixXd Dinit(n,m);
    Dinit << 0, 0,
            0, 0,
            (-1/mc) + (len_p/(mc*len_com)), (1/mc) - (len_p/(mc*len_com)),
            (-1 / (mc*len_com) ) + (len_p*(mc+mp)) / (mc*mp*len_com*len_com), -((-1 / (mc*len_com) ) + (len_p*(mc+mp)) / (mc*mp*len_com*len_com));

    MatrixXd Einit(m,n);
    Einit << -1, len_p, 0, 0,
            1, -len_p, 0, 0;

    MatrixXd Finit(m,m);
    Finit << 1/ks, 0,
            0, 1/ks;

    VectorXd cinit(m);
    cinit << d1,
            -d2;

    MatrixXd dinit(n,1);
    dinit << 0,
            0,
            0,
            0;

    MatrixXd Hinit = MatrixXd::Zero(m,k);

    MatrixXd Qinit(n,n);
    Qinit << 10, 0, 0, 0,
            0, 3, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    MatrixXd Rinit(k,k);
    Rinit << 1;

    MatrixXd Ginit(n+m+k, n+m+k);
    Ginit = 0.1*MatrixXd::Identity(n+m+k,n+m+k); Ginit(n+m+k-1,n+m+k-1) = 0;

    MatrixXd QNinit = drake::math::DiscreteAlgebraicRiccatiEquation(Ainit * Ts + MatrixXd::Identity(n,n), Ts*Binit, Qinit, Rinit);

    std::vector<MatrixXd> Qsetup(N+1, Qinit );
    Qsetup.at(N) = QNinit; //Switch to QNinit, solution of Algebraic Ricatti

    std::vector<MatrixXd> A(N, Ainit * Ts + MatrixXd::Identity(n,n) );
    std::vector<MatrixXd> B(N, Ts*Binit );
    std::vector<MatrixXd> D(N, Ts*Dinit );
    std::vector<MatrixXd> d(N, Ts*dinit );
    std::vector<MatrixXd> E(N, Einit );
    std::vector<MatrixXd> F(N, Finit );
    std::vector<VectorXd> c(N, cinit );
    std::vector<MatrixXd> H(N, Hinit );
    std::vector<MatrixXd> Q = Qsetup;
    std::vector<MatrixXd> R(N, Rinit );
    std::vector<MatrixXd> G(N, Ginit );

    MatrixXd Us(n+m+k,n+m+k);
    Us << 1000, 0, 0, 0, 0, 0, 0,
            0, 1000, 0, 0, 0, 0, 0,
            0, 0, 1000, 0, 0, 0, 0,
            0, 0, 0, 1000, 0 ,0 ,0,
            0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 1, 0,
            0,0,0,0,0,0,0;
    vector<MatrixXd> U(N, Us );

    *n_ = n;
    *m_ = m;
    *k_ = k;
    *N_ = N;
    *A_ = A;
    *B_ = B;
    *D_ = D;
    *d_ = d;
    *c_ = c;
    *E_ = E;
    *F_ = F;
    *H_ = H;
    *Q_ = Q;
    *R_ = R;
    *G_ = G;
    *U_ = U;

};