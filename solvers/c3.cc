#include "solvers/c3.h"
#include <chrono>

#include <omp.h>

#include "solvers/lcs.h"

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/moby_lcp_solver.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/solve.h"


namespace dairlib {
namespace solvers {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::SolutionResult;
using drake::solvers::SolverOptions;

using drake::solvers::OsqpSolver;
using drake::solvers::OsqpSolverDetails;
using drake::solvers::Solve;

C3::C3(const LCS& LCS, const vector<MatrixXd>& Q, const vector<MatrixXd>& R,
       const vector<MatrixXd>& G, const vector<MatrixXd>& U,
       const vector<VectorXd>& xdesired, const C3Options& options,
       const std::vector<Eigen::VectorXd>& warm_start_delta,
       const std::vector<Eigen::VectorXd>& warm_start_binary,
       const std::vector<Eigen::VectorXd>& warm_start_x,
       const std::vector<Eigen::VectorXd>& warm_start_lambda,
       const std::vector<Eigen::VectorXd>& warm_start_u,
       bool warm_start)
    : lcs_(LCS),
      A_(LCS.A_),
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
      xdesired_(xdesired),
      options_(options),
      N_((LCS.A_).size()),
      n_((LCS.A_)[0].cols()),
      m_((LCS.D_)[0].cols()),
      k_((LCS.B_)[0].cols()),
      hflag_(H_[0].isZero(0)),
      prog_(MathematicalProgram()),
      OSQPoptions_(SolverOptions()),
      osqp_(OsqpSolver()) {

  // Deep copy warm start
  warm_start_ = warm_start;
  if (warm_start_){
    warm_start_delta_.resize(warm_start_delta.size());
    for (size_t i = 0; i < warm_start_delta.size(); i++){
      warm_start_delta_[i] = warm_start_delta[i];
    }
    warm_start_binary_.resize(warm_start_binary.size());
    for (size_t i = 0; i < warm_start_binary.size(); i++){
      warm_start_binary_[i] = warm_start_binary[i];
    }
    warm_start_x_.resize(warm_start_x.size());
    for (size_t i = 0; i < warm_start_x.size(); i++){
      warm_start_x_[i] = warm_start_x[i];
    }
    warm_start_lambda_.resize(warm_start_lambda.size());
    for (size_t i = 0; i < warm_start_lambda.size(); i++){
      warm_start_lambda_[i] = warm_start_lambda[i];
    }
    warm_start_u_.resize(warm_start_u.size());
    for (size_t i = 0; i < warm_start_u.size(); i++){
      warm_start_u_[i] = warm_start_u[i];
    }
  }

  x_ = vector<drake::solvers::VectorXDecisionVariable>();
  u_ = vector<drake::solvers::VectorXDecisionVariable>();
  lambda_ = vector<drake::solvers::VectorXDecisionVariable>();

  for (int i = 0; i < N_ + 1; i++) {
    x_.push_back(prog_.NewContinuousVariables(n_, "x" + std::to_string(i)));
    if (i < N_) {
      u_.push_back(prog_.NewContinuousVariables(k_, "k" + std::to_string(i)));
      lambda_.push_back(
          prog_.NewContinuousVariables(m_, "lambda" + std::to_string(i)));
    }
  }

  // Add dynamics constraint.
  MatrixXd LinEq(n_, 2 * n_ + k_ + m_);
  LinEq.block(0, n_ + k_ + m_, n_, n_) = -1 * MatrixXd::Identity(n_, n_);

  for (int i = 0; i < N_; i++) {
    LinEq.block(0, 0, n_, n_) = A_.at(i);
    LinEq.block(0, n_, n_, k_) = B_.at(i);
    LinEq.block(0, n_ + k_, n_, m_) = D_.at(i);

    prog_.AddLinearEqualityConstraint(
        LinEq, -d_.at(i), {x_.at(i), u_.at(i), lambda_.at(i), x_.at(i + 1)});
  }

  // Add the inequality portion of the complementarity constraints.
  MatrixXd LinIneq(m_, n_ + k_ + m_);

  for (int i = 0; i < N_; i++) {
    LinIneq.block(0, 0, m_, n_) = E_.at(i);
    LinIneq.block(0, n_, m_, k_) = H_.at(i);
    LinIneq.block(0, n_ + k_, m_, m_) = F_.at(i);

    prog_.AddLinearConstraint(
      LinIneq, -c_.at(i), INFINITY * VectorXd::Ones(m_), {x_.at(i), u_.at(i), lambda_.at(i)}
    );
    // Or an alternative way of doing the Ex + Fl + Hu + c >= 0 term (no observed difference).
    // prog_.AddLinearConstraint(
    //   E_.at(i)*x_.at(i) + F_.at(i)*lambda_.at(i) + H_.at(i)*u_.at(i) + c_.at(i) >= VectorXd::Zero(m_)
    // );
    prog_.AddLinearConstraint(lambda_.at(i) >= VectorXd::Zero(m_));
  }


  // Build cost.
  for (int i = 0; i < N_ + 1; i++) {
    prog_.AddQuadraticCost(Q_.at(i) * 2, -2 * Q_.at(i) * xdesired_.at(i),
                           x_.at(i), 1);
    if (i < N_) {
      prog_.AddQuadraticCost(R_.at(i) * 2, VectorXd::Zero(k_), u_.at(i), 1);
    }
  }

  OSQPoptions_.SetOption(OsqpSolver::id(), "verbose", 0);
  // OSQPoptions_.SetOption(OsqpSolver::id(), "ebs_abs", 1e-7);
  // OSQPoptions_.SetOption(OsqpSolver::id(), "eps_rel", 1e-7);
  // OSQPoptions_.SetOption(OsqpSolver::id(), "eps_prim_inf", 1e-6);
  // OSQPoptions_.SetOption(OsqpSolver::id(), "eps_dual_inf", 1e-6);
  //Commented out temporarily
  // OSQPoptions_.SetOption(OsqpSolver::id(), "max_iter",  100);  //30
  prog_.SetSolverOptions(OSQPoptions_);
}

VectorXd C3::Solve(VectorXd& x0, vector<VectorXd>& delta, vector<VectorXd>& w) {
  vector<MatrixXd> Gv = G_;
  VectorXd z;


  for (int i = 0; i < options_.admm_iter-1; i++) {

    //std::cout << "Iteration" << i <<  std::endl;

    z = ADMMStep(x0, &delta, &w, &Gv);
// std::cout << "new delta" << i <<  std::endl;
//std::cout << delta.at(0).segment(n_,m_) << std::endl;
//    std::cout << "w" << i <<  std::endl;
//    std::cout << w.at(0) << std::endl;

  }

  vector<VectorXd> WD(N_, VectorXd::Zero(n_ + m_ + k_));
  for (int i = 0; i < N_; i++) {
    WD.at(i) = delta.at(i) - w.at(i);
  }

  vector<VectorXd> zfin = SolveQP(x0, Gv, WD);

  z = zfin[0];

//  std::cout <<  "contact prediction" << std::endl;
//      std::cout << zfin[0].segment(n_, m_) << std::endl;

//    std::cout << "violation" << std::endl;
//  std::cout << delta.at(0) << std::endl;

//  std::cout << "delta_force" << std::endl;
//  std::cout << delta.at(0).segment(n_,m_) << std::endl;
//
//  std::cout << "delta_displace" << std::endl;
//  std::cout << delta.at(0).segment(0,n_) << std::endl;

//VectorXd hold = delta.at(0).segment(n_,m_);
  //VectorXd hold = z.segment(n_,m_);

//  double count = 0;
//
//  for (int i = 3; i < 5; i++) {
//    count = count + hold(i);
//  }
//
//  if ( count >= 0.001){
//    std::cout << "guessing_contact" << std::endl;
//  }

//  std::cout << "w" << std::endl;
//  std::cout << w.at(0).segment(n_+3,3) << std::endl;

//      std::cout <<  "input" << std::endl;
//      std::cout << z.segment(n_+m_, k_) << std::endl;

//      std::cout <<  "contact prediction" << std::endl;
//      std::cout << z.segment(n_, m_) << std::endl;

//      std::cout <<  "prediction state" << std::endl;
  //    std::cout << z.segment(0, n_) << std::endl;

  
  // return z.segment(n_ + m_, k_);
  return zfin[0].segment(n_ + m_, k_);    //extract k elements starting from index n+m of the 0th vector in zfin. 
                                          //equivalent to extracting the input vector associated with time step 0 in the MPC problem
}

vector<VectorXd> C3::SolveFullSolution(VectorXd& x0, vector<VectorXd>& delta, vector<VectorXd>& w) {
  vector<MatrixXd> Gv = G_;
  VectorXd z;
  
  // vector<VectorXd> u(N_, VectorXd::Zero(n_ + m_ + k_)); //is this an initializer? from line 332

  for (int i = 0; i < options_.admm_iter-1; i++) {

    //std::cout << "Iteration" << i <<  std::endl;
    z = ADMMStep(x0, &delta, &w, &Gv);
// std::cout << "new delta" << i <<  std::endl;
//std::cout << delta.at(0).segment(n_,m_) << std::endl;
//    std::cout << "w" << i <<  std::endl;
//    std::cout << w.at(0) << std::endl;

  }

  vector<VectorXd> WD(N_, VectorXd::Zero(n_ + m_ + k_));
  for (int i = 0; i < N_; i++) {
    WD.at(i) = delta.at(i) - w.at(i);
  }

  vector<VectorXd> zfin = SolveQP(x0, Gv, WD);
  
  return zfin;
}

vector<VectorXd> C3::OptimalInputSeq(const vector<VectorXd> zfin){
  //This function should return the optimal input sequence along the full horizon given the full Z solution
  vector<VectorXd> UU(N_, VectorXd::Zero(k_));
  
  for (int i = 0; i < N_ ; i++){
      UU[i] = zfin[i].segment(n_ + m_, k_);
      // std::cout << "here" << zfin[i].head(n_) << std::endl;
  }
  
return UU; 
}

// Calculate the C3 cost and feasible trajectory associated with applying a provided control input sequence to
// a system at a provided initial state.
std::pair<double,std::vector<Eigen::VectorXd>> C3::CalcCost(const VectorXd& x0, vector<VectorXd>& UU, bool use_full_cost) const{
  std::vector<Eigen::VectorXd> XX(N_+1, VectorXd::Zero(n_)); //locally extracted state sequence
  XX[0] = x0;

  // Get the N step lcs rollout.
  for (int i = 0; i < N_; i++){
    XX[i+1] = lcs_.Simulate(XX[i], UU[i]);
  }

  // Declare Q_eff and R_eff as the Q and R to use for cost computation.
  std::vector<Eigen::MatrixXd> Q_eff = Q_;
  std::vector<Eigen::MatrixXd> R_eff = R_;

  // If not calculating the full cost, calculate just the ball position error cost.
  if (use_full_cost == false) {
    for (int i = 0; i < N_; i++){
      // Make R all zeros since not penalizing input effort.
      R_eff[i] = R_eff[i] * 0;

      // Use all zeros for Q and R except for portion of Q that corresponds to ball xyz errors.
      for(int j = 0; j < 7; j++){
        Q_eff.at(i)(j,j) = 0;
      }
      for(int j = 10; j < 16; j++){
        Q_eff.at(i)(j,j) = 0;
      }
    }

    // Do Nth step for Q.
    for(int j = 0; j < 7; j++){
      Q_eff.at(N_)(j,j) = 0;
    }
    for(int j = 10; j < 16; j++){
      Q_eff.at(N_)(j,j) = 0;
    }
  }

  // Calculate the cost over the N+1 time steps.
  double cost = 0;
  for (int i = 0; i < N_; i++){
    cost = cost + (XX[i] - xdesired_[i]).transpose()*Q_eff.at(i)*(XX[i] - xdesired_[i]) + UU[i].transpose()*R_eff.at(i)*UU[i];
  }
  cost = cost + (XX[N_]- xdesired_[N_]).transpose()*Q_eff.at(N_)*(XX[N_]- xdesired_[N_]);

  // Return the cost and the rolled out state trajectory.
  std::pair <double, std::vector<VectorXd>> ret (cost, XX);
  return ret;
}



VectorXd C3::ADMMStep(VectorXd& x0, vector<VectorXd>* delta,
                      vector<VectorXd>* w, vector<MatrixXd>* Gv) {
  vector<VectorXd> WD(N_, VectorXd::Zero(n_ + m_ + k_));

  for (int i = 0; i < N_; i++) {
    WD.at(i) = delta->at(i) - w->at(i);
  }

//  auto start = std::chrono::high_resolution_clock::now();

  vector<VectorXd> z = SolveQP(x0, *Gv, WD);

  // std::cout<<"z0: "<<z[0]<<std::endl;
  // std::cout<<"Gap function with z0"<<E_[0]*z[0].segment(0, n_) + F_[0] *z[0].segment(n_, m_) + H_[0]*z[0].segment(n_+m_, k_) + c_[0]<<std::endl;
  // std::cout<<"Ex + c with z0"<<E_[0]*z[0].segment(0, n_) + c_[0]<<std::endl;
  // std::cout<<"z1: "<<z[1]<<std::endl;
  // std::cout<<"Gap function with z1"<<E_[1]*z[1].segment(0, n_) + F_[1] *z[1].segment(n_, m_) + H_[1]*z[1].segment(n_+m_, k_) + c_[1]<<std::endl;
  // std::cout<<"Ex + c with z1"<<E_[1]*z[1].segment(0, n_) + c_[1]<<std::endl;
  

//  auto finish = std::chrono::high_resolution_clock::now();
//std::chrono::duration<double> elapsed = finish - start;
//std::cout << "Solve time:" << elapsed.count() << std::endl;

  vector<VectorXd> ZW(N_, VectorXd::Zero(n_ + m_ + k_));
  for (int i = 0; i < N_; i++) {
    ZW[i] = w->at(i) + z[i];
  }

//auto start = std::chrono::high_resolution_clock::now();

  if (U_[0].isZero(0) == 0) {
    vector<MatrixXd> Uv = U_;

    //std::cout << "W:" << w->at(0) << std::endl;


    *delta = SolveProjection(Uv, ZW, x0);
  } else {
    *delta = SolveProjection(*Gv, ZW, x0);
  }


//auto finish = std::chrono::high_resolution_clock::now();
//std::chrono::duration<double> elapsed = finish - start;
//std::cout << "Solve time:" << elapsed.count() << std::endl;

  for (int i = 0; i < N_; i++) {
    w->at(i) = w->at(i) + z[i] - delta->at(i);
    w->at(i) = w->at(i) / options_.rho_scale;

    //w->at(i) = VectorXd::Zero(58);

    Gv->at(i) = Gv->at(i) * options_.rho_scale;
  }

 //std::cout << "Viol:" << z[1] - delta->at(1) << std::endl;
  //std::cout << "Z" << z[1] << std::endl;

  return z[0];
}

vector<VectorXd> C3::SolveQP(VectorXd& x0, vector<MatrixXd>& G,
                             vector<VectorXd>& WD) {

  for (auto& constraint : constraints_) {
    prog_.RemoveConstraint(constraint);
  }
  constraints_.clear();

  constraints_.push_back(prog_.AddLinearConstraint(x_[0] == x0));

  if (hflag_ == 1) {
    drake::solvers::MobyLCPSolver<double> LCPSolver;
    VectorXd lambda0;
    LCPSolver.SolveLcpLemke(F_[0], E_[0] * x0 + c_[0], &lambda0);
    constraints_.push_back(prog_.AddLinearConstraint(lambda_[0] == lambda0));
  }

  for (auto& cost : costs_) {
    prog_.RemoveCost(cost);
  }
  costs_.clear();

  for (int i = 0; i < N_ + 1; i++) {
    if (i < N_) {
      costs_.push_back(prog_.AddQuadraticCost(
          G.at(i).block(0, 0, n_, n_) * 2,
          -2 * G.at(i).block(0, 0, n_, n_) * WD.at(i).segment(0, n_), x_.at(i),
          1));
      costs_.push_back(prog_.AddQuadraticCost(
          G.at(i).block(n_, n_, m_, m_) * 2,
          -2 * G.at(i).block(n_, n_, m_, m_) * WD.at(i).segment(n_, m_),
          lambda_.at(i), 1));
      costs_.push_back(
          prog_.AddQuadraticCost(G.at(i).block(n_ + m_, n_ + m_, k_, k_) * 2,
                                 -2 * G.at(i).block(n_ + m_, n_ + m_, k_, k_) *
                                     WD.at(i).segment(n_ + m_, k_),
                                 u_.at(i), 1));
    }
  }

//  /// initialize decision variables to warm start
//  if (warm_start_){
//    for (int i = 0; i < N_; i++){
//      prog_.SetInitialGuess(x_[i], warm_start_x_[i]);
//      prog_.SetInitialGuess(lambda_[i], warm_start_lambda_[i]);
//      prog_.SetInitialGuess(u_[i], warm_start_u_[i]);
//    }
//    prog_.SetInitialGuess(x_[N_], warm_start_x_[N_]);
//  }

  MathematicalProgramResult result = osqp_.Solve(prog_);
  VectorXd xSol = result.GetSolution(x_[0]);
  vector<VectorXd> zz(N_, VectorXd::Zero(n_ + m_ + k_));

  for (int i = 0; i < N_; i++) {
    zz.at(i).segment(0, n_) = result.GetSolution(x_[i]);
    zz.at(i).segment(n_, m_) = result.GetSolution(lambda_[i]);
    zz.at(i).segment(n_ + m_, k_) = result.GetSolution(u_[i]);

    if (warm_start_){
      // update warm start parameters
      warm_start_x_[i] = result.GetSolution(x_[i]);
      warm_start_lambda_[i] = result.GetSolution(lambda_[i]);
      warm_start_u_[i] = result.GetSolution(u_[i]);
    }
  }
  if (warm_start_)
    warm_start_x_[N_] = result.GetSolution(x_[N_]);


  return zz;
}

void C3::AddLinearConstraint(Eigen::RowVectorXd& A, double& Lowerbound,
                             double& Upperbound, int& constraint) {
  if (constraint == 1) {
    for (int i = 1; i < N_; i++) {
      userconstraints_.push_back(
          prog_.AddLinearConstraint(A, Lowerbound, Upperbound, x_.at(i)));
    }
  }

  if (constraint == 2) {
    for (int i = 0; i < N_; i++) {
      userconstraints_.push_back(
          prog_.AddLinearConstraint(A, Lowerbound, Upperbound, u_.at(i)));
    }
  }

  if (constraint == 3) {
    for (int i = 0; i < N_; i++) {
      userconstraints_.push_back(
          prog_.AddLinearConstraint(A, Lowerbound, Upperbound, lambda_.at(i)));
    }
  }
}

void C3::RemoveConstraints() {
  for (auto& userconstraint : userconstraints_) {
    prog_.RemoveConstraint(userconstraint);
  }
  userconstraints_.clear();
}

vector<VectorXd> C3::SolveProjection(vector<MatrixXd>& G,
                                     vector<VectorXd>& WZ,
                                     Eigen::VectorXd& x0) {
  vector<VectorXd> deltaProj(N_, VectorXd::Zero(n_ + m_ + k_));
  int i;


  // If doing sampling, omp configuration happens elsewhere so don't overwrite it here.
  if (options_.overwrite_threading_settings == true){
    if (options_.num_threads > 0) {
      omp_set_dynamic(0);                         // Explicitly disable dynamic teams.
      omp_set_num_threads(options_.num_threads);  // Set number of threads.
    }
  }

#pragma omp parallel for
  for (i = 0; i < N_; i++) {
    bool constrain_first_x = false;
    if (i == 0) {
      constrain_first_x = true;
    }

    if (warm_start_){
      deltaProj[i] =
          SolveSingleProjection(G[i], WZ[i], E_[i], F_[i], H_[i], c_[i], i, constrain_first_x, x0);
    }
    else{
      deltaProj[i] =
          SolveSingleProjection(G[i], WZ[i], E_[i], F_[i], H_[i], c_[i], -1, constrain_first_x, x0);
    }
  }

  return deltaProj;
}

std::vector<Eigen::VectorXd> C3::GetWarmStartX() const {
  return warm_start_x_;
}

std::vector<Eigen::VectorXd> C3::GetWarmStartLambda() const {
  return warm_start_lambda_;
}

std::vector<Eigen::VectorXd> C3::GetWarmStartU() const {
  return warm_start_u_;
}

}  // namespace solvers
}  // namespace dairlib
