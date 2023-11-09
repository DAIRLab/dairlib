#pragma once

#include <vector>

#include <Eigen/Dense>

#include "gurobi_c++.h"
#include "solvers/c3.h"
#include "solvers/lcs.h"

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/moby_lcp_solver.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/solve.h"

#include "solvers/c3_options.h"
#include "solvers/fast_osqp_solver.h"


namespace dairlib {
namespace solvers {

class C3QP : public C3 {
 public:
  /// Default constructor for time-varying LCS
  C3QP(const LCS& LCS, const std::vector<Eigen::MatrixXd>& Q,
       const std::vector<Eigen::MatrixXd>& R,
       const std::vector<Eigen::MatrixXd>& G,
       const std::vector<Eigen::MatrixXd>& U,
       const std::vector<Eigen::VectorXd>& xdesired,
       const C3Options& options,
       const std::vector<Eigen::VectorXd>& warm_start_delta = {},
       const std::vector<Eigen::VectorXd>& warm_start_binary = {},
       const std::vector<Eigen::VectorXd>& warm_start_x = {},
       const std::vector<Eigen::VectorXd>& warm_start_lambda = {},
       const std::vector<Eigen::VectorXd>& warm_start_u = {},
       bool warm_start = false);

  /// Virtual projection method
  Eigen::VectorXd SolveSingleProjection(const Eigen::MatrixXd& U,
                                        const Eigen::VectorXd& delta_c,
                                        const Eigen::MatrixXd& E,
                                        const Eigen::MatrixXd& F,
                                        const Eigen::MatrixXd& H,
                                        const Eigen::VectorXd& c,
                                        const int& warm_start_index = -1);
  std::vector<Eigen::VectorXd> GetWarmStartDelta() const;
  std::vector<Eigen::VectorXd> GetWarmStartBinary() const;


// private:
//  GRBEnv env_;
  //drake::solvers::MathematicalProgram projprog_;
  //drake::solvers::SolverOptions OSQPoptions_;
  //drake::solvers::OsqpSolver osqp_;
//  drake::solvers::VectorXDecisionVariable xn_;
//  drake::solvers::VectorXDecisionVariable un_;
//  drake::solvers::VectorXDecisionVariable ln_;
};

}  // namespace solvers
}  // namespace dairlib
