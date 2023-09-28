#pragma once

#include <vector>

#include <Eigen/Dense>

#include "solvers/c3_options.h"
#include "solvers/fast_osqp_solver.h"
#include "solvers/lcs.h"

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/solve.h"

#include "drake/solvers/branch_and_bound.h"
//#include "solvers/optimization_utils.h"
#include "drake/solvers/gurobi_solver.h"

namespace dairlib {
namespace solvers {
class MIQP {
 public:
  /// @param LCS LCS parameters
  /// @param Q, R, G, U Cost function parameters
  MIQP(const LCS& LCS, const std::vector<Eigen::MatrixXd>& Q,
     const std::vector<Eigen::MatrixXd>& R,
     const std::vector<Eigen::MatrixXd>& G,
     const std::vector<Eigen::MatrixXd>& U,
     const std::vector<Eigen::VectorXd>& xdesired, 
     const C3Options& options,
     double scaling,
     const std::vector<Eigen::VectorXd>& warm_start_delta = {},
     const std::vector<Eigen::VectorXd>& warm_start_binary = {},
     const std::vector<Eigen::VectorXd>& warm_start_x_ = {},
     const std::vector<Eigen::VectorXd>& warm_start_lambda_ = {},
     const std::vector<Eigen::VectorXd>& warm_start_u_ = {},
     bool warm_start = false);

  /// Solve the MPC problem
  /// @param x0 The initial state of the system
  /// @param delta Copy variable solution
  /// @param w Scaled dual variable solution
  /// @return The first control action to take, u[0]
  Eigen::VectorXd Solve(Eigen::VectorXd& x0,
                        std::vector<Eigen::VectorXd>& delta,
                        std::vector<Eigen::VectorXd>& w);


  /// Solve a single QP
  /// @param x0 The initial state of the system
  /// @param WD The (w - delta) variables
  /// @param G The G variables from previous step
  std::vector<Eigen::VectorXd> SolveQP(Eigen::VectorXd& x0,
                                       std::vector<Eigen::MatrixXd>& G,
                                       std::vector<Eigen::VectorXd>& WD);


 public:
  const std::vector<Eigen::MatrixXd> A_;
  const std::vector<Eigen::MatrixXd> B_;
  const std::vector<Eigen::MatrixXd> D_;
  const std::vector<Eigen::VectorXd> d_;
  const std::vector<Eigen::MatrixXd> E_;
  const std::vector<Eigen::MatrixXd> F_;
  const std::vector<Eigen::MatrixXd> H_;
  const std::vector<Eigen::VectorXd> c_;
  const std::vector<Eigen::MatrixXd> Q_;
  const std::vector<Eigen::MatrixXd> R_;
  const std::vector<Eigen::MatrixXd> U_;
  const std::vector<Eigen::MatrixXd> G_;
  const std::vector<Eigen::VectorXd> xdesired_;
  const C3Options options_;
  const int N_;
  const int n_;
  const int m_;
  const int k_;
  const bool hflag_;
  double scaling_;

protected:
  std::vector<Eigen::VectorXd> warm_start_delta_;
  std::vector<Eigen::VectorXd> warm_start_binary_;
  std::vector<Eigen::VectorXd> warm_start_x_;
  std::vector<Eigen::VectorXd> warm_start_lambda_;
  std::vector<Eigen::VectorXd> warm_start_u_;
  bool warm_start_;

 private:
  drake::solvers::MathematicalProgram prog_;
  drake::solvers::SolverOptions OSQPoptions_;
  drake::solvers::GurobiSolver osqp_;
  std::vector<drake::solvers::VectorXDecisionVariable> x_;
  std::vector<drake::solvers::VectorXDecisionVariable> u_;
  std::vector<drake::solvers::VectorXDecisionVariable> lambda_;
  std::vector<drake::solvers::VectorXDecisionVariable> binary_;
  std::vector<drake::solvers::Binding<drake::solvers::QuadraticCost>> costs_;
  std::vector<drake::solvers::Binding<drake::solvers::LinearConstraint>>
      constraints_;
  std::vector<drake::solvers::Binding<drake::solvers::LinearConstraint>>
      userconstraints_;
};

}  // namespace solvers
}  // namespace dairlib
