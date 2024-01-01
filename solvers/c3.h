#pragma once

#include <vector>

#include <Eigen/Dense>

#include "solvers/c3_options.h"
#include "solvers/fast_osqp_solver.h"
#include "solvers/lcs.h"

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/solve.h"

namespace dairlib {
namespace solvers {
class C3 {
 public:
  /// @param LCS LCS parameters
  /// @param Q, R, G, U Cost function parameters
  C3(const LCS& LCS, const std::vector<Eigen::MatrixXd>& Q,
     const std::vector<Eigen::MatrixXd>& R,
     const std::vector<Eigen::MatrixXd>& G,
     const std::vector<Eigen::MatrixXd>& U,
     const std::vector<Eigen::VectorXd>& xdesired, const C3Options& options,
     const std::vector<Eigen::VectorXd>& warm_start_delta = {},
     const std::vector<Eigen::VectorXd>& warm_start_binary = {},
     const std::vector<Eigen::VectorXd>& warm_start_x_ = {},
     const std::vector<Eigen::VectorXd>& warm_start_lambda_ = {},
     const std::vector<Eigen::VectorXd>& warm_start_u_ = {},
     bool warm_start = false);

  virtual ~C3() = default;

  /// Solve the MPC problem
  /// @param x0 The initial state of the system
  /// @param delta A pointer to the copy variable solution
  /// @param w A pointer to the scaled dual variable solution
  /// @return The first control action to take, u[0]
  void Solve(const Eigen::VectorXd& x0, std::vector<Eigen::VectorXd>& delta,
             std::vector<Eigen::VectorXd>& w);

  /// Solve a single ADMM step
  /// @param x0 The initial state of the system
  /// @param delta The copy variables from the previous step
  /// @param w The scaled dual variables from the previous step
  /// @param G A pointer to the G variables from previous step
  void ADMMStep(const Eigen::VectorXd& x0, std::vector<Eigen::VectorXd>* delta,
                std::vector<Eigen::VectorXd>* w,
                std::vector<Eigen::MatrixXd>* G);

  /// Solve a single QP
  /// @param x0 The initial state of the system
  /// @param WD A pointer to the (w - delta) variables
  /// @param G A pointer to the G variables from previous step
  std::vector<Eigen::VectorXd> SolveQP(const Eigen::VectorXd& x0,
                                       const std::vector<Eigen::MatrixXd>& G,
                                       const std::vector<Eigen::VectorXd>& WD,
                                       bool is_final_solve = false);

  /// Solve the projection problem for all timesteps
  /// @param WZ A pointer to the (z + w) variables
  /// @param G A pointer to the G variables from previous step
  std::vector<Eigen::VectorXd> SolveProjection(
      const std::vector<Eigen::MatrixXd>& G, std::vector<Eigen::VectorXd>& WZ);

  /// allow users to add constraints (adds for all timesteps)
  /// @param A, lower_bound, upper_bound lower_bound <= A^T x <= upper_bound
  /// @param constraint inputconstraint, stateconstraint, forceconstraint
  void AddLinearConstraint(Eigen::RowVectorXd& A, double lower_bound,
                           double upper_bound, int constraint);

  /// allow user to remove all constraints
  void RemoveConstraints();

  /// Get QP warm start
  std::vector<Eigen::VectorXd> GetWarmStartX() const;
  std::vector<Eigen::VectorXd> GetWarmStartLambda() const;
  std::vector<Eigen::VectorXd> GetWarmStartU() const;

  /// Solve a single projection step
  /// @param E, F, H, c LCS parameters
  /// @param U A pointer to the U variables
  /// @param delta_c A pointer to the copy of (z + w) variables
  virtual Eigen::VectorXd SolveSingleProjection(
      const Eigen::MatrixXd& U, const Eigen::VectorXd& delta_c,
      const Eigen::MatrixXd& E, const Eigen::MatrixXd& F,
      const Eigen::MatrixXd& H, const Eigen::VectorXd& c,
      const int& warm_start_index) = 0;

  void SetOsqpSolverOptions(const drake::solvers::SolverOptions& options) {
    solver_options_ = options;
  }

  std::vector<Eigen::VectorXd> GetFullSolution() { return *z_sol_; }
  std::vector<Eigen::VectorXd> GetStateSolution() { return *x_sol_; }
  std::vector<Eigen::VectorXd> GetForceSolution() { return *lambda_sol_; }
  std::vector<Eigen::VectorXd> GetInputSolution() { return *u_sol_; }

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
  const bool h_is_zero_;

 protected:
  std::vector<Eigen::VectorXd> warm_start_delta_;
  std::vector<Eigen::VectorXd> warm_start_binary_;
  std::vector<Eigen::VectorXd> warm_start_x_;
  std::vector<Eigen::VectorXd> warm_start_lambda_;
  std::vector<Eigen::VectorXd> warm_start_u_;
  bool warm_start_;

 private:
  drake::solvers::MathematicalProgram prog_;
  drake::solvers::SolverOptions solver_options_;
  drake::solvers::OsqpSolver osqp_;
  std::vector<drake::solvers::VectorXDecisionVariable> x_;
  std::vector<drake::solvers::VectorXDecisionVariable> u_;
  std::vector<drake::solvers::VectorXDecisionVariable> lambda_;
  std::vector<drake::solvers::Binding<drake::solvers::QuadraticCost>> costs_;
  std::vector<std::shared_ptr<drake::solvers::QuadraticCost>> input_costs_;
  std::vector<drake::solvers::Binding<drake::solvers::LinearConstraint>>
      constraints_;
  std::vector<drake::solvers::Binding<drake::solvers::LinearConstraint>>
      user_constraints_;

  // Solutions

  std::unique_ptr<std::vector<Eigen::VectorXd>> z_sol_;
  std::unique_ptr<std::vector<Eigen::VectorXd>> x_sol_;
  std::unique_ptr<std::vector<Eigen::VectorXd>> lambda_sol_;
  std::unique_ptr<std::vector<Eigen::VectorXd>> u_sol_;
};

}  // namespace solvers
}  // namespace dairlib
