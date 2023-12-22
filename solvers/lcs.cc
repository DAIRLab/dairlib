#include "solvers/lcs.h"

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/moby_lcp_solver.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/solve.h"

#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

namespace dairlib {
namespace solvers {

LCS::LCS(const vector<MatrixXd>& A, const vector<MatrixXd>& B,
         const vector<MatrixXd>& D, const vector<VectorXd>& d,
         const vector<MatrixXd>& E, const vector<MatrixXd>& F,
         const vector<MatrixXd>& H, const vector<VectorXd>& c)
    : A_(A), B_(B), D_(D), d_(d), E_(E), F_(F), H_(H), c_(c), N_(A.size()) {}

LCS::LCS(const MatrixXd& A, const MatrixXd& B, const MatrixXd& D,
         const VectorXd& d, const MatrixXd& E, const MatrixXd& F,
         const MatrixXd& H, const VectorXd& c, const int& N)
    : LCS(vector<MatrixXd>(N, A), vector<MatrixXd>(N, B),
          vector<MatrixXd>(N, D), vector<VectorXd>(N, d),
          vector<MatrixXd>(N, E), vector<MatrixXd>(N, F),
          vector<MatrixXd>(N, H), vector<VectorXd>(N, c)) {}

LCS::LCS(const LCS& lcs)
    : N_(lcs.N_) {
  A_.resize(N_);
  B_.resize(N_);
  D_.resize(N_);
  d_.resize(N_);
  E_.resize(N_);
  F_.resize(N_);
  H_.resize(N_);
  c_.resize(N_);
  for (int i = 0; i < lcs.N_; ++i) {
    A_.at(i) = lcs.A_.at(i);
    B_.at(i) = lcs.B_.at(i);
    D_.at(i) = lcs.D_.at(i);
    d_.at(i) = lcs.d_.at(i);
    E_.at(i) = lcs.E_.at(i);
    F_.at(i) = lcs.F_.at(i);
    H_.at(i) = lcs.H_.at(i);
    c_.at(i) = lcs.c_.at(i);
  }
}

LCS& LCS::operator=(const LCS& lcs) {
  N_ = lcs.N_;
  A_.resize(N_);
  B_.resize(N_);
  D_.resize(N_);
  d_.resize(N_);
  E_.resize(N_);
  F_.resize(N_);
  H_.resize(N_);
  c_.resize(N_);
  for (int i = 0; i < lcs.N_; ++i) {
    A_.at(i) = lcs.A_.at(i);
    B_.at(i) = lcs.B_.at(i);
    D_.at(i) = lcs.D_.at(i);
    d_.at(i) = lcs.d_.at(i);
    E_.at(i) = lcs.E_.at(i);
    F_.at(i) = lcs.F_.at(i);
    H_.at(i) = lcs.H_.at(i);
    c_.at(i) = lcs.c_.at(i);
  }
}
VectorXd LCS::Simulate(VectorXd& x_init, VectorXd& input) const {
  VectorXd x_final;

  // calculate force
  drake::solvers::MobyLCPSolver<double> LCPSolver;
  VectorXd force;

//  disturbance[3] = -5;
//  disturbance[4] = -5;
//  disturbance[5] = -5;

//VectorXd dummy_input = VectorXd::Zero(9);

  ///hacking for now
  // double scaling = 0.000646507;

  auto flag = LCPSolver.SolveLcpLemkeRegularized(F_[0], E_[0] * x_init * scaling_ + c_[0] * scaling_ + H_[0] * input * scaling_,
                          &force);

  //std::cout << flag << std::endl;

//  VectorXd qval = E_[0] * x_init + c_[0] + H_[0] * input;
//  MatrixXd Fval = F_[0];
////
////
//  auto flag = LCPSolver.SolveLcpLemkeRegularized(Fval,qval, &force);

//  std::cout << flag << std::endl;

//  if (flag == 1){
//
//      std::cout << "LCS force estimate" << std::endl;
//    std::cout << force << std::endl;
//    std::cout << "LCS force estimate" << std::endl;
//  }

//  std::cout << "eig" << std::endl;
//  std::cout << (F_[0] + F_[0].transpose()).eigenvalues() << std::endl;
//  std::cout << "eig" << std::endl;

//  std::cout << "gap" << std::endl;
//  std::cout << E_[0] * x_init + c_[0] + H_[0] * input + F_[0] * force << std::endl;
//  std::cout << "gap" << std::endl;

//  //print force
//  std::cout << "LCS force estimate" << std::endl;
//  std::cout << force << std::endl;
//  std::cout << "LCS force estimate" << std::endl;

//  double count = 0;
//  for (int i = 3; i < 5; i++) {
//    count = count + force(i);
//  }
//
//  if ( count >= 0.00001){
//    std::cout << "here" << std::endl;
//  }

  // update
  x_final = A_[0] * x_init + B_[0] * input + D_[0] * force / scaling_ + d_[0];

  //std::cout << "here" << D_[0] << std::endl;

    if (flag == 0){

    std::cout<<"CAREFUL LCS FAILED TO SOLVE"<<std::endl;


//      std::cout << "Next state prediction" << std::endl;
//    std::cout << x_final<< std::endl;
//    std::cout << "Next state prediction" << std::endl;

//    std::cout << "Jn * v" << std::endl;
//   std::cout << D_[0].block(10,0,9,9) * x_final_v << std::endl;
//    std::cout << "Jn * v" << std::endl;

//
  //  std::cout << "LCS force estimate" << std::endl;
  // std::cout << force << std::endl;
  //  std::cout << "LCS force estimate" << std::endl;

//      std::cout << "D" << std::endl;
//    std::cout << D_[0] << std::endl;
//      std::cout << "D" << std::endl;

//      std::cout << "D times LCS force estimate" << std::endl;
//    std::cout << D_[0] * force + d_[0] << std::endl;
//    std::cout << "D times LCS force estimate" << std::endl;


  }


  return x_final;
}

void LCS::SetScaling(double scaling) {
  scaling_ = scaling;
}

}  // namespace solvers
}  // namespace dairlib