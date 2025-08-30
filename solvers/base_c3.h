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

class C3Base {
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

  /// @param lcs       Parameters defining the LCS (Linear Complementarity
  /// System).
  /// @param costs     Cost matrices used in the optimization.
  /// @param x_des     Desired goal state.
  /// @param options   Options specific to the C3 formulation.
  /// @note Using this constructor will set z_size to the default value, which
  /// is size_x + size_u + size_lambda
  C3Base(const LCS& LCS, const CostMatrices& costs,
         const std::vector<Eigen::VectorXd>& x_des, const C3Options& options);

  virtual ~C3Base() = default;

  /// Solve the MPC problem.
  /// @param x0 The initial state of the system
  /// @param verbose Whether to print additional information
  /// @return void
  void Solve(const Eigen::VectorXd& x0, bool verbose = false);

  /// Compute the MPC cost, using previously solved MPC solution.
  /// @param cost_type The method of computing the cost
  /// @param Kp_for_ee_pd_rollout Proportional gain for simulated EE PD control
  /// used for some of the cost types
  /// @param Kd_for_ee_pd_rollout Derivative gain for simulated EE PD control
  /// used for some of the cost types
  /// @param force_tracking_disabled Whether to simulate EE PD control with
  /// feedforward u from the MPC solution
  /// @param print_cost_breakdown Whether to print the cost breakdown
  /// @param verbose Whether to print additional information
  /// @return The cost and its associated state trajectory
  std::pair<double, std::vector<Eigen::VectorXd>> CalcCost(
      C3CostComputationType cost_type = kSimLCSReplaceC3EEPlan,
      std::vector<double> Kp_for_ee_pd_rollout = {0.0, 0.0, 0.0},  
      std::vector<double> Kd_for_ee_pd_rollout = {0.0, 0.0, 0.0},
      bool force_tracking_disabled = false, int num_objects = 1,
      bool print_cost_breakdown = false, bool verbose = false) const;

  /// Helper function to simulate the dynamics with PD control on the EE
  /// location and velocity plans, and the control input plans.  Used for cost
  /// types that simulate the impedance control.
  /// @param Kp_for_ee_pd_rollout Proportional gain for simulated EE PD control
  /// @param Kd_for_ee_pd_rollout Derivative gain for simulated EE PD control
  /// @param force_tracking_disabled Whether to simulate EE PD control with
  /// feedforward u from the MPC solution
  /// @param verbose Whether to print additional information
  /// @return the simulated state and input trajectories
  std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>>
  SimulatePDControl(std::vector<double> Kp_for_ee_pd_rollout = {0.0, 0.0, 0.0},
                    std::vector<double> Kd_for_ee_pd_rollout = {0.0, 0.0, 0.0}, int num_objects = 1,
                    bool force_tracking_disabled = false,
                    bool verbose = false) const;

  /// Solve a single ADMM step.
  /// @param x0 The initial state of the system
  /// @param delta The copy variables from the previous step
  /// @param w The scaled dual variables from the previous step
  /// @param G A pointer to the G variables from previous step
  /// @param admm_iteration ADMM iteration for accurate warm starting
  /// @param verbose Whether to print additional information
  /// @return solution is saved in C3 object
  void ADMMStep(const Eigen::VectorXd& x0, std::vector<Eigen::VectorXd>* delta,
                std::vector<Eigen::VectorXd>* w,
                std::vector<Eigen::MatrixXd>* G, int admm_iteration,
                bool verbose = false);

  /// Solve a single QP.
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
                                       const std::vector<Eigen::VectorXd>& delta,
                                       int admm_iteration,
                                       bool is_final_solve = false);

  /// Solve the projection problem for all timesteps.
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
  void RemoveUserConstraints();

  /// Solve a projection step for a single knot point k.
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

  void SetOsqpSolverOptions(const drake::solvers::SolverOptions& options) {
    prog_.SetSolverOptions(options);
  }

  std::vector<Eigen::VectorXd> GetFullSolution() { return *z_sol_; }
  std::vector<Eigen::VectorXd> GetStateSolution() { return *x_sol_; }
  std::vector<Eigen::VectorXd> GetForceSolution() { return *lambda_sol_; }
  std::vector<Eigen::VectorXd> GetInputSolution() { return *u_sol_; }
  std::vector<Eigen::VectorXd> GetDualDeltaSolution() { return *delta_sol_; }
  std::vector<Eigen::VectorXd> GetDualWSolution() { return *w_sol_; }

  int GetZSize() { return z_size_; }

  void UpdateCostMatrices(const C3Base::CostMatrices& costs);
  virtual void UpdateLCS(const LCS& lcs);

  /// Update the LCS used for cost computation.  This can differ from the LCS
  /// used for the solve, e.g. if more contacts are used for cost than for
  /// computing the plan.  NOTE:  This does not update the internal cost
  /// matrices used for the solve (Q_, R_, G_, U_).  Those are updated via
  /// UpdateCostMatrices.
  void UpdateCostLCS(const LCS& lcs_for_cost);
  void UpdateTarget(const std::vector<Eigen::VectorXd>& x_des);

 protected:
  /// @param lcs      Parameters defining the LCS.
  /// @param costs    Cost matrices used in the optimization.
  /// @param x_des    Desired goal state trajectory.
  /// @param options  Options specific to the C3 formulation.
  /// @param z_size   Size of the z vector, which depends on the specific C3
  /// variant.
  ///                 For example:
  ///                   - C3MIQP / C3QP: z = [x, u, lambda]
  ///                   - C3Plus:        z = [x, u, lambda, eta]
  ///
  /// This constructor is intended for internal use only. The public constructor
  /// delegates to this one, passing in an explicitly computed z vector size.
  C3Base(const LCS& lcs, const CostMatrices& costs,
         const std::vector<Eigen::VectorXd>& x_des, const C3Options& options,
         int z_size);

  // Helper functions for C3Base constructor
  void ScaleLCS();
  void InitializeWarmStarts();
  void InitializeOptimizationVariables();
  void InitializeDynamicsConstraints();
  void InitializeStateAndInputCosts();

  void UpdateDynamicsConstraints();

  // Helper functions for QP step
  virtual void AddAugmentedCostsQPStep(const std::vector<Eigen::MatrixXd>& G,
                                       const std::vector<Eigen::VectorXd>& WD,
                                       const std::vector<Eigen::VectorXd>& delta,
                                       bool is_final_solve);
  virtual void SetInitialGuessQPStep(const Eigen::VectorXd& x0,
                                     int admm_iteration);
  virtual void ExtractQPSolution(
      const drake::solvers::MathematicalProgramResult& result,
      int admm_iteration, bool is_final_solve);
  virtual void UpdateWarmStarts(
      const drake::solvers::MathematicalProgramResult& result,
      int admm_iteration);

  std::vector<std::vector<Eigen::VectorXd>> warm_start_delta_;
  std::vector<std::vector<Eigen::VectorXd>> warm_start_binary_;
  std::vector<std::vector<Eigen::VectorXd>> warm_start_x_;
  std::vector<std::vector<Eigen::VectorXd>> warm_start_lambda_;
  std::vector<std::vector<Eigen::VectorXd>> warm_start_u_;
  bool warm_start_;
  const int N_;
  const int n_;  // n_x
  const int m_;  // n_lambda
  const int k_;  // n_u
  const int z_size_;
  const C3Options options_;
  bool use_parallelization_in_projection_ = true;

  // TODO:  storing the LCS as a class variable makes the LCS matrices
  // redundant.  Could consider removing LCS matrices as class variables.
  mutable LCS lcs_;
  std::unique_ptr<LCS> lcs_for_cost_;
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
  std::vector<Eigen::MatrixXd> Q_;
  std::vector<Eigen::MatrixXd> R_;
  std::vector<Eigen::MatrixXd> U_;
  std::vector<Eigen::MatrixXd> G_;
  std::vector<Eigen::VectorXd> x_desired_;
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
  mutable std::vector<Eigen::VectorXd> zfin_;
  std::unique_ptr<std::vector<Eigen::VectorXd>> x_sol_;
  std::unique_ptr<std::vector<Eigen::VectorXd>> lambda_sol_;
  std::unique_ptr<std::vector<Eigen::VectorXd>> u_sol_;

  std::unique_ptr<std::vector<Eigen::VectorXd>> z_sol_;
  std::unique_ptr<std::vector<Eigen::VectorXd>> delta_sol_;
  std::unique_ptr<std::vector<Eigen::VectorXd>> w_sol_;
};

}  // namespace solvers
}  // namespace dairlib
