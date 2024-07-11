#pragma once

#include <vector>

#include <Eigen/Dense>

#include "solvers/c3_options.h"
#include "solvers/lcs.h"

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/solve.h"

namespace dairlib {
namespace solvers {

class C3 {
 public:
  struct CostMatrices {
    CostMatrices(const std::vector<Eigen::MatrixXd>& Q,
                 const std::vector<Eigen::MatrixXd>& R,
                 const std::vector<Eigen::MatrixXd>& G,
                 const std::vector<Eigen::MatrixXd>& U);
    std::vector<Eigen::MatrixXd> Q;
    std::vector<Eigen::MatrixXd> R;
    std::vector<Eigen::MatrixXd> G;
    std::vector<Eigen::MatrixXd> U;
  };
  /// @param LCS LCS parameters
  /// @param Q, R, G, U Cost function parameters
  C3(const LCS& LCS, const CostMatrices& costs,
     const std::vector<Eigen::VectorXd>& x_desired, const C3Options& options);

  virtual ~C3() = default;

  /// Solve the MPC problem
  /// @param x0 The initial state of the system
  /// @param delta A pointer to the copy variable solution
  /// @param w A pointer to the scaled dual variable solution
  /// @return The first control action to take, u[0]
  void Solve(const Eigen::VectorXd& x0);

  /// Update the LCS without needing to reconstruct the C3 object
  void UpdateLCS(const LCS& lcs);
  /// Update the target without needing to reconstruct the C3 object
  void UpdateTarget(const std::vector<Eigen::VectorXd>& x_des);

  /// allow users to add constraints (adds for all timesteps)
  /// @param A, lower_bound, upper_bound lower_bound <= A x <= upper_bound
  /// @param constraint inputconstraint, stateconstraint, forceconstraint
  void AddLinearConstraint(Eigen::MatrixXd& A, Eigen::VectorXd& lower_bound,
                           Eigen::VectorXd& upper_bound, int constraint);

  /// allow user to remove all constraints added by AddLinearConstraint
  void RemoveConstraints();

  /// Solve a single ADMM step
  /// @param x0 The initial state of the system
  /// @param delta The copy variables from the previous step
  /// @param w The scaled dual variables from the previous step
  /// @param G A pointer to the G variables from previous step
  void ADMMStep(const Eigen::VectorXd& x0, std::vector<Eigen::VectorXd>* delta,
                std::vector<Eigen::VectorXd>* w,
                std::vector<Eigen::MatrixXd>* G, int admm_iteration);

  /// Solve a single QP
  /// @param x0 The initial state of the system
  /// @param WD A pointer to the (w - delta) variables
  /// @param G A pointer to the G variables from previous step
  std::vector<Eigen::VectorXd> SolveQP(const Eigen::VectorXd& x0,
                                       const std::vector<Eigen::MatrixXd>& G,
                                       const std::vector<Eigen::VectorXd>& WD,
                                       int admm_iteration,
                                       bool is_final_solve = false);

  /// Solve the projection problem for all timesteps
  /// @param WZ A pointer to the (z + w) variables
  /// @param G A pointer to the G variables from previous step
  std::vector<Eigen::VectorXd> SolveProjection(
      const std::vector<Eigen::MatrixXd>& G, std::vector<Eigen::VectorXd>& WZ,
      int admm_iteration);

  /// Solve a single projection step
  /// @param E, F, H, c LCS parameters
  /// @param U A pointer to the U variables
  /// @param delta_c A pointer to the copy of (z + w) variables
  virtual Eigen::VectorXd SolveSingleProjection(
      const Eigen::MatrixXd& U, const Eigen::VectorXd& delta_c,
      const Eigen::MatrixXd& E, const Eigen::MatrixXd& F,
      const Eigen::MatrixXd& H, const Eigen::VectorXd& c,
      const int admm_iteration, const int& warm_start_index) = 0;

  /// Solve a robust (friction cone) projection step for a single knot point k
  /// @param U Matrix for consensus cost
  /// @param delta_c A pointer to the copy of (z + w) variables
  /// @param E, F, H, c LCS contact parameters
  /// @param W_x, W_l, W_u, w Linearization of J_t v_{k+1} wrt x_k, lambda_k,
  /// u_k
  /// @param admm_iteration ADMM iteration for accurate warm starting
  /// @param warm_start_index knot point index for warm starting
  /// @return delta_k
  virtual Eigen::VectorXd SolveRobustSingleProjection(
      const Eigen::MatrixXd& U, const Eigen::VectorXd& delta_c,
      const Eigen::MatrixXd& E, const Eigen::MatrixXd& F,
      const Eigen::MatrixXd& H, const Eigen::VectorXd& c,
      const Eigen::MatrixXd& W_x, const Eigen::MatrixXd& W_l,
      const Eigen::MatrixXd& W_u, const Eigen::VectorXd& w,
      const int admm_iteration, const int& warm_start_index) = 0;

  /// Solve a robust (friction cone) projection step for a single knot point
  void SetOsqpSolverOptions(const drake::solvers::SolverOptions& options) {
    prog_.SetSolverOptions(options);
  }

  std::vector<Eigen::VectorXd> GetFullSolution() { return *z_sol_; }
  std::vector<Eigen::VectorXd> GetStateSolution() { return *x_sol_; }
  std::vector<Eigen::VectorXd> GetForceSolution() { return *lambda_sol_; }
  std::vector<Eigen::VectorXd> GetInputSolution() { return *u_sol_; }
  std::vector<Eigen::VectorXd> GetDualDeltaSolution() { return *delta_sol_; }
  std::vector<Eigen::VectorXd> GetDualWSolution() { return *w_sol_; }

 protected:
  std::vector<std::vector<Eigen::VectorXd>> warm_start_delta_;
  std::vector<std::vector<Eigen::VectorXd>> warm_start_binary_;
  std::vector<std::vector<Eigen::VectorXd>> warm_start_x_;
  std::vector<std::vector<Eigen::VectorXd>> warm_start_lambda_;
  std::vector<std::vector<Eigen::VectorXd>> warm_start_u_;
  bool warm_start_;
  const int N_;
  const int n_x_;  // n
  const int n_lambda_;  // m
  const int n_u_;  // k

 private:
  LCS lcs_;
  double AnDn_ = 1.0; // Scaling factor for lambda
  const CostMatrices cost_matrices_;
  std::vector<Eigen::VectorXd> x_desired_;
  const C3Options options_;
  double solve_time_ = 0;
  bool h_is_zero_;

  drake::solvers::MathematicalProgram prog_;
  // QP step variables
  drake::solvers::OsqpSolver osqp_;
  std::vector<drake::solvers::VectorXDecisionVariable> x_;
  std::vector<drake::solvers::VectorXDecisionVariable> u_;
  std::vector<drake::solvers::VectorXDecisionVariable> lambda_;
  // QP step constraints
  std::shared_ptr<drake::solvers::LinearEqualityConstraint> initial_state_constraint_;
  std::vector<drake::solvers::LinearEqualityConstraint*> dynamics_constraints_;
  std::vector<drake::solvers::Binding<drake::solvers::LinearConstraint>>
      constraints_;
  std::vector<drake::solvers::Binding<drake::solvers::LinearConstraint>>
      user_constraints_;

  /// Projection step variables are defined outside of the MathematicalProgram
  /// interface

  std::vector<drake::solvers::QuadraticCost*> target_cost_;
  std::vector<drake::solvers::Binding<drake::solvers::QuadraticCost>> costs_;
  std::vector<std::shared_ptr<drake::solvers::QuadraticCost>> input_costs_;

  // Solutions
  std::unique_ptr<std::vector<Eigen::VectorXd>> x_sol_;
  std::unique_ptr<std::vector<Eigen::VectorXd>> lambda_sol_;
  std::unique_ptr<std::vector<Eigen::VectorXd>> u_sol_;

  std::unique_ptr<std::vector<Eigen::VectorXd>> z_sol_;
  std::unique_ptr<std::vector<Eigen::VectorXd>> delta_sol_;
  std::unique_ptr<std::vector<Eigen::VectorXd>> w_sol_;
};

}  // namespace solvers
}  // namespace dairlib
