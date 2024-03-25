#include "solvers/c3_qp.h"

#include <vector>

#include <Eigen/Dense>

#include "solvers/c3_options.h"
#include "solvers/lcs.h"

#include "drake/solvers/mathematical_program.h"
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

C3QP::C3QP(const LCS& LCS, const CostMatrices& costs,
           const vector<VectorXd>& xdesired, const C3Options& options)
    : C3(LCS, costs, xdesired, options) {}

VectorXd C3QP::SolveSingleProjection(const MatrixXd& U, const VectorXd& delta_c,
                                     const MatrixXd& E, const MatrixXd& F,
                                     const MatrixXd& H, const VectorXd& c,
                                     const int admm_iteration,
                                     const int& warm_start_index) {
  drake::solvers::MathematicalProgram prog;
  drake::solvers::SolverOptions solver_options;
  drake::solvers::OsqpSolver osqp_;

  auto xn_ = prog.NewContinuousVariables(n_, "x");
  auto ln_ = prog.NewContinuousVariables(m_, "lambda");
  auto un_ = prog.NewContinuousVariables(k_, "u");

  double alpha = 0.01;
  double scaling = 1000;

  MatrixXd EFH(m_, n_ + m_ + k_);
  EFH.block(0, 0, m_, n_) = E / scaling;
  EFH.block(0, n_, m_, m_) = F / scaling;
  EFH.block(0, n_ + m_, m_, k_) = H / scaling;

  prog.AddLinearConstraint(
      EFH, -c / scaling,
      Eigen::VectorXd::Constant(m_, std::numeric_limits<double>::infinity()),
      {xn_, ln_, un_});

  prog.AddLinearConstraint(
      MatrixXd::Identity(m_, m_), VectorXd::Zero(m_),
      Eigen::VectorXd::Constant(m_, std::numeric_limits<double>::infinity()),
      ln_);

  MatrixXd New_U = U;
  New_U.block(n_, n_, m_, m_) = alpha * F;

  VectorXd cost_linear = -delta_c.transpose() * New_U;

  //  prog.AddQuadraticCost(New_U, cost_linear, {xn_, ln_, un_}, 1);
  prog.AddQuadraticCost(New_U, cost_linear, {xn_, ln_, un_}, 1);

  //  prog.AddQuadraticCost((1 - alpha) * F, VectorXd::Zero(m_), ln_, 1);
  prog.AddQuadraticCost((1 - alpha) * F, VectorXd::Zero(m_), ln_, 1);

  solver_options.SetOption(OsqpSolver::id(), "max_iter", 500);
  solver_options.SetOption(OsqpSolver::id(), "verbose", 0);
  solver_options.SetOption(OsqpSolver::id(), "polish", 1);
  solver_options.SetOption(OsqpSolver::id(), "polish_refine_iter", 1);
  solver_options.SetOption(OsqpSolver::id(), "rho", 1e-4);
  solver_options.SetOption(OsqpSolver::id(), "scaled_termination", 1);
  solver_options.SetOption(OsqpSolver::id(), "linsys_solver", 0);
  prog.SetSolverOptions(solver_options);

  MathematicalProgramResult result = osqp_.Solve(prog);

  VectorXd xSol = result.GetSolution(xn_);
  VectorXd lamSol = result.GetSolution(ln_);
  VectorXd uSol = result.GetSolution(un_);

  VectorXd delta_kc = VectorXd::Zero(n_ + m_ + k_);
  delta_kc.segment(0, n_) = xSol;
  delta_kc.segment(n_, m_) = lamSol;
  delta_kc.segment(n_ + m_, k_) = uSol;

  return delta_kc;
}

std::vector<Eigen::VectorXd> C3QP::GetWarmStartDelta() const {
  return warm_start_delta_[0];
}

std::vector<Eigen::VectorXd> C3QP::GetWarmStartBinary() const {
  return warm_start_binary_[0];
}

}  // namespace solvers
}  // namespace dairlib
