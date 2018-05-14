#include <memory>
#include <chrono>
#include <random>
#include <gflags/gflags.h>


#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"

#include "sgd_iter.h"
#include "src/manifold_constraint.h"
#include "src/file_utils.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::string;

using drake::goldilocks_walking::writeCSV;
using drake::goldilocks_walking::readCSV;
using drake::solvers::VectorXDecisionVariable;

namespace drake{
namespace goldilocks_walking {

void runSGD() {
  std::random_device randgen;
  std::default_random_engine e1(randgen());
  std::uniform_real_distribution<> dist(0, 1);




  // int n_weights = 43;
  int n_weights =  16;
  VectorXd theta_0 = VectorXd::Zero(n_weights);
  theta_0(0) = -0.1;
  theta_0(5) = 1.0;
  writeCSV("data/0_theta.csv", theta_0);

  double length = 0.5;
  double duration = 1;
  int snopt_iter = 200;
  string directory = "data/";
  string init_z = "z_save.csv";
  string weights = "0_theta.csv";
  string prefix = "0_";
  sgdIter(length, duration, snopt_iter, directory, init_z, weights, prefix);

  for (int iter = 1; iter <= 20; iter++) {
    MatrixXd A = readCSV(directory + std::to_string(iter-1) + "_A.csv");
    MatrixXd B = readCSV(directory + std::to_string(iter-1) + "_B.csv");
    MatrixXd H = readCSV(directory + std::to_string(iter-1) + "_H.csv");
    MatrixXd lb = readCSV(directory + std::to_string(iter-1) + "_lb.csv");
    MatrixXd ub = readCSV(directory + std::to_string(iter-1) + "_ub.csv");
    MatrixXd y = readCSV(directory + std::to_string(iter-1) + "_y.csv");
    MatrixXd w = readCSV(directory + std::to_string(iter-1) + "_w.csv");
    MatrixXd z = readCSV(directory + std::to_string(iter-1) + "_z.csv");
    MatrixXd theta = readCSV(directory + std::to_string(iter-1) + "_theta.csv");

    int n_active = 0;
    double tol = 1e-4;
    for (int i = 0; i < y.rows(); i++) {
      if (y(i) >= ub(i) - tol || y(i) <= lb(i) + tol)
        n_active++;
    }

    int nz = A.cols();
    int nt = B.cols();

    MatrixXd A_active(n_active, nz);
    MatrixXd B_active(n_active, nt);
    MatrixXd AB_active(n_active, nz + nt);

    int nl = 0;
    for (int i = 0; i < y.rows(); i++) {
      if (y(i) >= ub(i) - tol || y(i) <= lb(i) + tol) {
        A_active.row(nl) = A.row(i);
        B_active.row(nl) = B.row(i);
        AB_active.row(nl) << A.row(i), B.row(i);
        nl++;
      }
    }


    // double weight = 10000;

    // MatrixXd R = MatrixXd::Zero(nz + nt + nl, nz + nt + nl);
    // R.block(0, 0, nz, nz) = H;
    // R.block(nz, nz, nt, nt) = weight*MatrixXd::Identity(nt,nt);
    // R.block(0, nz + nt, nz + nt, nl) = AB_active.transpose();
    // R.block(nz + nt, 0, nl, nz + nt) = AB_active;

    // VectorXd b = VectorXd::Zero(nz + nt + nl);
    // b.head(nz) = w.col(0);

    // VectorXd ztl = -R.colPivHouseholderQr().solve(b);

    // VectorXd dtheta = ztl.segment(nz,nt);
    // VectorXd dz = ztl.head(nz);
    // auto dcost = 0.5*dz.transpose()*H*dz + w.transpose()*dz;

    // std::cout << "Solve error: " << (R*ztl + b).norm() << std::endl;
    // std::cout << "Delta cost: " << dcost(0) << std::endl;
    // std::cout << "dtheta norm: " << dtheta.norm() << std::endl;

    Eigen::BDCSVD<MatrixXd> svd(AB_active,  Eigen::ComputeFullV);


    MatrixXd N = svd.matrixV().rightCols(AB_active.cols() - svd.rank());

    MatrixXd H_ext = MatrixXd::Zero(nz + nt, nz + nt);
    H_ext.block(0,0,nz,nz) = H;
    H_ext.block(nz,nz,nt,nt) = 1e-2*MatrixXd::Identity(nt,nt);

    VectorXd w_ext(nz+nt,1);
    w_ext << w, MatrixXd::Zero(nt,1);

    auto gradient = N*N.transpose()*w_ext;

    double scale_num= gradient.dot(gradient);
    double scale_den = gradient.dot(H_ext*gradient);

    auto dtheta = -0.1*gradient.tail(nt)*scale_num/scale_den;

    std::cout << "dtheta norm: " << dtheta.norm() << std::endl;

    std::cout << "scale predict: " << scale_num/scale_den << std::endl;

    // MatrixXd Q = N.transpose() * H_ext * N;
    // MatrixXd b = N.transpose() * w_ext;

    // drake::solvers::MathematicalProgram qp_theta;
    // VectorXDecisionVariable  vars = qp_theta.NewContinuousVariables(N.cols(), "v");

    // qp_theta.AddQuadraticCost(Q,b,vars);

    // double trust = .5/(5.0+iter);

    // qp_theta.AddLinearConstraint(N*vars <= MatrixXd::Constant(N.rows(), 1, trust));
    // qp_theta.AddLinearConstraint(N*vars >= MatrixXd::Constant(N.rows(), 1, -trust));
    // // drake::solvers::GurobiSolver solver;
    // // std::cout << solver.available() << std::endl;
    // // std::cout << solver.License() << std::endl;

    // // DRAKE_ASSERT(solver.available());

    // auto result = qp_theta.Solve();
    // // auto result = solver.Solve(qp_theta);
    // auto dtheta = N.bottomRows(nt)*qp_theta.GetSolution(vars);

    writeCSV("data/" + std::to_string(iter) + "_theta.csv", theta + dtheta);

    init_z = std::to_string(iter-1) + "_z.csv";
    weights = std::to_string(iter) +  "_theta.csv";
    prefix = std::to_string(iter) +  "_";

    //randomize distance on [0.3,0.5]
    // length = 0.3 + 0.2*dist(e1);
    duration =  length/0.5; //maintain constaint speed of 0.5 m/s

    std::cout << std::endl << "Iter: " << iter <<  std::endl;
    std::cout << "New length: " << length << std::endl;

    sgdIter(length, duration, snopt_iter, directory, init_z, weights, prefix);

    // <<" dtheta: " <<  std::endl << dtheta << std::endl;
  }
}
}
}

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::srand(time(0));  // Initialize random number generator.

  drake::goldilocks_walking::runSGD();
}
