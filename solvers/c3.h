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
  struct CostMatrices {
    CostMatrices() = default;
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
  /// @param Q, R, G, U Cost Matrices
  C3(const LCS& LCS, const CostMatrices& costs,
     const std::vector<Eigen::VectorXd>& x_desired, const C3Options& options);

  virtual ~C3() = default;

  /// Solve the MPC problem
  /// @param x0 The initial state of the system
  /// @return void
  void Solve(const Eigen::VectorXd& x0);

  /// Solve a single ADMM step
  /// @param x0 The initial state of the system
  /// @param delta The copy variables from the previous step
  /// @param w The scaled dual variables from the previous step
  /// @param G A pointer to the G variables from previous step
  /// @param admm_iteration ADMM iteration for accurate warm starting
  /// @return solution is saved in C3 object
  void ADMMStep(const Eigen::VectorXd& x0, std::vector<Eigen::VectorXd>* delta,
                std::vector<Eigen::VectorXd>* w,
                std::vector<Eigen::MatrixXd>* G, int admm_iteration);

  /// Solve a single QP
  /// @param x0 The initial state of the system
  /// @param G A pointer to the G variables from previous step
  /// @param WD A pointer to the (w - delta) variables
  /// @param admm_iteration ADMM iteration for accurate warm starting
  /// @param is_final_solve Indicating final admm iteration in case of any
  /// polishing steps
  /// @return z MPC solution
  std::vector<Eigen::VectorXd> SolveQP(const Eigen::VectorXd& x0,
                                       const std::vector<Eigen::MatrixXd>& G,
                                       const std::vector<Eigen::VectorXd>& WD,
                                       int admm_iteration,
                                       bool is_final_solve = false);

  /// Solve the projection problem for all timesteps
  /// @param U Matrix for consensus cost
  /// @param WZ (z + w) variables
  /// @param admm_iteration ADMM iteration for accurate warm starting
  std::vector<Eigen::VectorXd> SolveProjection(
      const std::vector<Eigen::MatrixXd>& U, std::vector<Eigen::VectorXd>& WZ,
      int admm_iteration);

  /// allow users to add constraints (adds for all timesteps)
  /// @param A, lower_bound, upper_bound lower_bound <= A^T x <= upper_bound
  /// @param constraint inputconstraint, stateconstraint, forceconstraint
  void AddLinearConstraint(Eigen::RowVectorXd& A, double lower_bound,
                           double upper_bound, int constraint);

  /// remove all constraints
  void RemoveConstraints();

  /// Solve a projection step for a single knot point k
  /// @param U Matrix for consensus cost
  /// @param delta_c A pointer to the copy of (z + w) variables
  /// @param E, F, H, c LCS contact parameters
  /// @param admm_iteration ADMM iteration for accurate warm starting
  /// @param warm_start_index knot point index for warm starting
  /// @return delta_k
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

 public:
  void UpdateLCS(const LCS& lcs);
  void UpdateTarget(const std::vector<Eigen::VectorXd>& x_des);

 protected:
  std::vector<std::vector<Eigen::VectorXd>> warm_start_delta_;
  std::vector<std::vector<Eigen::VectorXd>> warm_start_binary_;
  std::vector<std::vector<Eigen::VectorXd>> warm_start_x_;
  std::vector<std::vector<Eigen::VectorXd>> warm_start_lambda_;
  std::vector<std::vector<Eigen::VectorXd>> warm_start_u_;
  bool warm_start_;
  const int N_;
  const int n_; // n_x
  const int m_; // n_lambda
  const int k_; // n_u

 private:
  std::vector<Eigen::MatrixXd> A_;
  std::vector<Eigen::MatrixXd> B_;
  std::vector<Eigen::MatrixXd> D_;
  std::vector<Eigen::VectorXd> d_;
  std::vector<Eigen::MatrixXd> E_;
  std::vector<Eigen::MatrixXd> F_;
  std::vector<Eigen::MatrixXd> H_;
  std::vector<Eigen::VectorXd> c_;
  Eigen::MatrixXd W_x_;
  Eigen::MatrixXd W_l_;
  Eigen::MatrixXd W_u_;
  Eigen::VectorXd w_;
  double AnDn_ = 1.0;
  const std::vector<Eigen::MatrixXd> Q_;
  const std::vector<Eigen::MatrixXd> R_;
  const std::vector<Eigen::MatrixXd> U_;
  const std::vector<Eigen::MatrixXd> G_;
  std::vector<Eigen::VectorXd> x_desired_;
  const C3Options options_;
  double dt_ = 0;
  double solve_time_ = 0;
  bool h_is_zero_;

  /// MathematicalProgram for QP step
  drake::solvers::OsqpSolver osqp_;
  drake::solvers::MathematicalProgram prog_;
  /// Decision variables for QP step
  std::vector<drake::solvers::VectorXDecisionVariable> x_;
  std::vector<drake::solvers::VectorXDecisionVariable> u_;
  std::vector<drake::solvers::VectorXDecisionVariable> lambda_;
  /// QP step constraints
  std::vector<drake::solvers::LinearEqualityConstraint*> dynamics_constraints_;
  // initial state constraint
  std::vector<drake::solvers::Binding<drake::solvers::LinearConstraint>>
      constraints_;
  // workspace and input limit constraints
  std::vector<drake::solvers::Binding<drake::solvers::LinearConstraint>>
      user_constraints_;
  /// QP step costs
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
