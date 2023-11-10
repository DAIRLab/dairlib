#include "solvers/c3_approx.h"

#include <vector>

#include <Eigen/Dense>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/moby_lcp_solver.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/solve.h"

#include "solvers/c3_options.h"
#include "solvers/fast_osqp_solver.h"
#include "solvers/lcs.h"


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


C3APPROX::C3APPROX(const LCS& LCS, const vector<MatrixXd>& Q,
               const vector<MatrixXd>& R, const vector<MatrixXd>& G,
               const vector<MatrixXd>& U, const vector<VectorXd>& xdesired,
               const C3Options& options,
              const std::vector<Eigen::VectorXd>& warm_start_delta,
              const std::vector<Eigen::VectorXd>& warm_start_binary,
              const std::vector<Eigen::VectorXd>& warm_start_x,
              const std::vector<Eigen::VectorXd>& warm_start_lambda,
              const std::vector<Eigen::VectorXd>& warm_start_u,
              bool warm_start)
    : C3(LCS, Q, R, G, U, xdesired, options, 
         warm_start_delta, warm_start_binary,
         warm_start_x, warm_start_lambda,
         warm_start_u, warm_start), env_(true) {

//  // Create an environment
//  env_.set("LogToConsole", "0");
//  env_.set("OutputFlag", "0");
//  env_.set("Threads", "2");
//  env_.start();

//  drake::solvers::MathematicalProgram prog;
//  drake::solvers::SolverOptions OSQPoptions;
//  drake::solvers::OsqpSolver osqp;
//  drake::solvers::VectorXDecisionVariable x;
//  drake::solvers::VectorXDecisionVariable lam;
//  drake::solvers::VectorXDecisionVariable u;
  //drake::solvers::Binding<drake::solvers::LinearConstraint>* constraint;
  //drake::solvers::Binding<drake::solvers::QuadraticCost>* cost;

//  xn_ = prog_.NewContinuousVariables(n_, "x");
//  ln_ = prog_.NewContinuousVariables(m_, "lambda");
//  un_ = prog_.NewContinuousVariables(k_, "u");

//  xn_ = drake::solvers::VectorXDecisionVariable();
//  ln_ = drake::solvers::VectorXDecisionVariable();
//  un_ = drake::solvers::VectorXDecisionVariable();

  //std::cout << "here" << std::endl;

}

VectorXd C3APPROX::SolveSingleProjection(const MatrixXd& U,
                                       const VectorXd& delta_c,
                                       const MatrixXd& E, const MatrixXd& F,
                                       const MatrixXd& H, const VectorXd& c,
                                       const int& warm_start_index) {

  drake::solvers::MathematicalProgram projprog_;
  drake::solvers::SolverOptions OSQPoptions_;
  drake::solvers::OsqpSolver osqp_;

  auto xn_ = projprog_.NewContinuousVariables(n_, "x");
  auto ln_ = projprog_.NewContinuousVariables(m_, "lambda");
  auto un_ = projprog_.NewContinuousVariables(k_, "u");

  double scaling = 1000;

  MatrixXd LinIneq(m_, n_ + k_ + m_);
  LinIneq.block(0, 0, m_, n_) = E/scaling;
  LinIneq.block(0, n_, m_, m_) = F/scaling;
  LinIneq.block(0, n_ + m_, m_, k_) = H/scaling;


  projprog_.AddLinearConstraint(LinIneq, -c/scaling, 100000*VectorXd::Ones(m_)-(c/scaling), {xn_, ln_, un_} );
  //prog.AddLinearConstraint(LinIneq, -c, -c, {xn, ln, un} );


  //prog.AddLinearEqualityConstraint(LinIneq, -c/scaling, {xn, ln, un});
  projprog_.AddLinearConstraint(MatrixXd::Identity(m_, m_), VectorXd::Zero(m_), 10000*VectorXd::Ones(m_), ln_);

  double alpha = 0.2;
  MatrixXd New_U = U;
  New_U.block(n_,n_,m_,m_) = alpha * F;

  //std::cout << "alphaF" << std::endl;
  //std::cout << alpha * F << std::endl;


  VectorXd cost_linear = -delta_c.transpose() * New_U;

  projprog_.AddQuadraticCost(New_U, cost_linear, {xn_, ln_, un_}, 1);

  projprog_.AddQuadraticCost((1-alpha) * F , VectorXd::Zero(m_), ln_, 1);



//////KIND OF WORKING
//  VectorXd cost_linear = -delta_c.transpose() * New_U;
//
//  prog.AddQuadraticCost(New_U, cost_linear, {xn, ln, un}, 1);
//
//  prog.AddQuadraticCost((1-alpha) * F / 1000 , VectorXd::Zero(m_), ln, 1);
//////KIND OF WORKING


  //prog.AddQuadraticCost((1-alpha) * MatrixXd::Identity(m_, m_), VectorXd::Zero(m_), ln, 1);



//  OSQPoptions_.SetOption(OsqpSolver::id(), "ebs_abs", 1e-9);
//  OSQPoptions_.SetOption(OsqpSolver::id(), "eps_rel", 1e-9);
//  OSQPoptions_.SetOption(OsqpSolver::id(), "eps_prim_inf", 1e-8);
//  OSQPoptions_.SetOption(OsqpSolver::id(), "eps_dual_inf", 1e-8);
//  OSQPoptions_.SetOption(OsqpSolver::id(), "max_iter",  100);
  OSQPoptions_.SetOption(OsqpSolver::id(), "verbose", 0);
  OSQPoptions_.SetOption(OsqpSolver::id(), "polishing", true);
  projprog_.SetSolverOptions(OSQPoptions_);



  MathematicalProgramResult result = osqp_.Solve(projprog_);


  VectorXd xSol = result.GetSolution(xn_);
  VectorXd lamSol = result.GetSolution(ln_);
  VectorXd uSol = result. GetSolution(un_);

  VectorXd delta_kcc = VectorXd::Zero(n_+m_+k_);
  delta_kcc.segment(0,n_) = xSol;
  delta_kcc.segment(n_,m_) = lamSol;
  delta_kcc.segment(n_+m_, k_) = uSol;

//  std::cout << "here" << std::endl;
//  std::cout << lamSol.transpose() *   (E/scaling * xSol + H/scaling * uSol + F/scaling * lamSol + c/scaling) << std::endl;
//  std::cout << F.eigenvalues() << std::endl;
//  std::cout << lamSol << std::endl;
//  std::cout <<  c << std::endl;
//  std::cout << delta_c.segment(0,n_) - delta_kcc.segment(0,n_) << std::endl;

  //return delta_kc;
  return delta_kcc;
}

std::vector<Eigen::VectorXd> C3APPROX::GetWarmStartDelta() const{
  return warm_start_delta_;
}

std::vector<Eigen::VectorXd> C3APPROX::GetWarmStartBinary() const{
  return warm_start_binary_;
}

}  // namespace solvers
}  // namespace dairlib
