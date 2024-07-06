#pragma once
#include <array>
#include <chrono>
#include <map>

#include "nonlinear_pendulum_utils.h"
#include "geometry/convex_polygon_set.h"
#include "solvers/optimization_utils.h"

#include "drake/solvers/decision_variable.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/common/drake_assert.h"

namespace dairlib {
namespace systems {
namespace controllers {

using std::vector;
using drake::solvers::Binding;
using drake::solvers::QuadraticCost;
using drake::solvers::DecisionVariable;
using drake::solvers::LinearConstraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::BoundingBoxConstraint;
using drake::solvers::VectorXDecisionVariable;
using drake::solvers::LinearEqualityConstraint;

using solvers::LinearBigMConstraint;
using solvers::LinearBigMEqualityConstraint;

struct cf_mpfc_solution {
  bool init = false;
  std::vector<drake::Vector6d> xc{}; // nonlinear_pendulum state
  std::vector<Eigen::VectorXd> uu{}; // nonlinear_pendulum input
  Eigen::Vector4d xi{};              // ALIP state at beginning of next stance phase
  std::vector<Eigen::Vector4d> xx{}; // step-to-step alip state
  std::vector<Eigen::VectorXd> ee{}; // workspace soft constraint slack variable
  std::vector<Eigen::Vector3d> pp{}; // footstep positions
  alip_utils::Stance stance{};

  bool success{};
  double t_nom{};
  double total_time{};
  double optimizer_time{};
  Eigen::Vector2d desired_velocity;
  drake::solvers::SolutionResult solution_result;
};

struct cf_mpfc_params {
  alip_utils::AlipGaitParams gait_params{};
  int nmodes{};
  int nknots{};
  double mu{};
  double foot_length{};
  double time_regularization{};
  double soft_constraint_cost{};
  double rddot_rate_limit = 100; // limit accel change per
  Eigen::Vector2d com_pos_bound{};
  Eigen::Vector2d com_vel_bound{};
  Eigen::Vector2d input_bounds{};
  Eigen::MatrixXd Q{};
  Eigen::MatrixXd R{};
  Eigen::MatrixXd Qf{};
  Eigen::MatrixXd Qc{};
  Eigen::MatrixXd Rc{};
  alip_utils::AlipTrackingCostType tracking_cost_type =
      alip_utils::AlipTrackingCostType::kVelocity;
  drake::solvers::SolverOptions solver_options{};
};

class CFMPFC {
 public:
  explicit CFMPFC(cf_mpfc_params params);

  cf_mpfc_solution Solve(
      const drake::Vector6d& x, const Eigen::Vector3d& p, double t,
      const Eigen::Vector2d& vdes, alip_utils::Stance stance,
      const geometry::ConvexPolygonSet& footholds,
      const cf_mpfc_solution& prev_sol);

  const cf_mpfc_params& params() { return params_; }
 protected:
  void MakeMPCVariables();
  void MakeMPCCosts();
  void MakeFootstepConstraints();
  void MakeComplexInputConstraints();
  void MakeStateConstraints();
  void MakeDynamicsConstraints();
  void MakeInitialConditionsConstraints();

  void UpdateInitialConditions(
      const drake::Vector6d& x, const Eigen::Vector3d& p);
  void UpdateComplexDynamicsConstraints(
      const vector<drake::Vector6d>& xc, const vector<Eigen::Vector2d>& uu, double t);
  void UpdateInputRateLimitConstraint(double t);
  void UpdateInitialInputConstraint(
      const vector<Eigen::Vector2d>& uu, double t_prev, double t);
  void UpdateModelSwitchConstraint(
      const drake::Vector6d& x, const Eigen::Vector3d& p_pre,const Eigen::Vector3d& p_post);
  void UpdateCrossoverConstraint(alip_utils::Stance stance);
  void UpdateComplexModelCosts(const Eigen::Vector2d& vdes, alip_utils::Stance stance);
  void UpdateFootstepCost(const Eigen::Vector2d& vdes, alip_utils::Stance stance);
  void UpdateFootholdConstraints(const geometry::ConvexPolygonSet& footholds);
  void UpdateTrackingCost(const Eigen::Vector2d& vdes, alip_utils::Stance stance);
  void UpdateTrackingCostVelocity(const Eigen::Vector2d& vdes);
  void UpdateTerminalCostVelocity(const Eigen::Vector2d& vdes);
  void UpdateTrackingCostGait(const Eigen::Vector2d& vdes,  alip_utils::Stance stance);
  void UpdateTerminalCostGait(const Eigen::Vector2d& vdes,  alip_utils::Stance stance);
  Eigen::Vector3d CalcS2SLQRInput(
      const drake::Vector6d& x, const Eigen::Vector2d& vdes,
      double t, alip_utils::Stance stance) const;

  void ValidateParams() const {
    DRAKE_DEMAND(params_.nmodes >= 2); // need to take 1 footstep (2 modes)
    DRAKE_DEMAND(params_.nknots >= 2);
    DRAKE_DEMAND(params_.gait_params.double_stance_duration > 0);
    DRAKE_DEMAND(params_.soft_constraint_cost >= 0);
  }

  void Check() {
    DRAKE_DEMAND(initial_foot_c_ != nullptr);
    DRAKE_DEMAND(initial_state_c_ != nullptr);
    DRAKE_DEMAND(initial_alip_state_c_ != nullptr);
    DRAKE_DEMAND(complex_dynamics_c_ != nullptr);
    DRAKE_DEMAND(alip_dynamics_c_ != nullptr);
    DRAKE_DEMAND(terminal_cost_ != nullptr);
    DRAKE_DEMAND(model_switch_c_ != nullptr);
    DRAKE_DEMAND(terminal_cost_ != nullptr);
    DRAKE_DEMAND(not workspace_c_.empty());
    DRAKE_DEMAND(not no_crossover_c_.empty());
    DRAKE_DEMAND(not reachability_c_.empty());
    DRAKE_DEMAND(not complex_state_cost_.empty());
    DRAKE_DEMAND(not complex_input_cost_.empty());
    DRAKE_DEMAND(not tracking_cost_.empty());
    DRAKE_DEMAND(not footstep_cost_.empty());
    DRAKE_DEMAND(not soft_constraint_cost_.empty());
    DRAKE_DEMAND(not complex_input_constraints_.empty());
  };

  // problem data:
  cf_mpfc_params params_;

  // program and decision variables
  drake::copyable_unique_ptr<MathematicalProgram> prog_{
    std::make_unique<MathematicalProgram>()
  };

  drake::solvers::GurobiSolver solver_;

  // Decision Variables
  vector<VectorXDecisionVariable> pp_{}; // footstep sequence
  vector<VectorXDecisionVariable> xc_{}; // Centroidal (SRBD) state
  vector<VectorXDecisionVariable> xx_{}; // ALIP state
  vector<VectorXDecisionVariable> uu_{}; // Centroidal (SRBD) input
  vector<VectorXDecisionVariable> ee_{}; // workspace soft constraint slack var
  vector<VectorXDecisionVariable> mu_{}; // binary variables
  VectorXDecisionVariable xi_;           // initial ALIP State

  // Equality Constraints                                                       | Init  | Updater
  std::shared_ptr<LinearEqualityConstraint> initial_foot_c_ = nullptr;       // |   x   |   x
  std::shared_ptr<LinearEqualityConstraint> initial_state_c_ = nullptr;      // |   x   |   x
  std::shared_ptr<LinearEqualityConstraint> initial_alip_state_c_ = nullptr; // |   x   |  N/A
  std::shared_ptr<LinearEqualityConstraint> model_switch_c_ = nullptr;       // |   x   |   x
  std::shared_ptr<LinearEqualityConstraint> complex_dynamics_c_ = nullptr;       // |   x   |   x
  std::shared_ptr<LinearEqualityConstraint> alip_dynamics_c_ = nullptr;      // |   x   |  N/A
  std::shared_ptr<LinearConstraint> initial_input_constraint_ = nullptr;
  std::shared_ptr<LinearEqualityConstraint> footstep_choice_c_ = nullptr;
  //                                                          | Init  | Updater
  vector<Binding<LinearConstraint>> workspace_c_{};        // |   x   |  N/A
  vector<Binding<LinearConstraint>> no_crossover_c_{};     // |   x   |   x
  vector<Binding<LinearConstraint>> reachability_c_{};     // |   x   |  N/A
  vector<Binding<LinearConstraint>> complex_input_constraints_{};  // |   x   |  N/A
  vector<Binding<LinearConstraint>> input_rate_constraints_{};
  vector<Binding<BoundingBoxConstraint>> complex_input_bounds_{};
  vector<vector<LinearBigMConstraint>> footstep_c_{};
  vector<vector<LinearBigMEqualityConstraint>> footstep_c_eq_{};


                                                           // | Init  | Updater
  std::shared_ptr<QuadraticCost> terminal_cost_ = nullptr; // |   x   |   x
  std::shared_ptr<QuadraticCost> complex_state_final_cost_ = nullptr;
  vector<Binding<QuadraticCost>> complex_state_cost_{}; // |   x   |   x
  vector<Binding<QuadraticCost>> complex_input_cost_{}; // |   x   |   x
  vector<Binding<QuadraticCost>> tracking_cost_{};         // |   x   |   x
  vector<Binding<QuadraticCost>> footstep_cost_{};         // |   x   |   x
  vector<Binding<QuadraticCost>> soft_constraint_cost_{};  // |   x   |  N/A

  // some useful matrices and dynamics quantities for ALIP step to step portion
  Eigen::Matrix4d A_;
  Eigen::Matrix<double, 4, 2> B_;
  Eigen::Matrix<double, 2, 4> lqr_K_;
  Eigen::Matrix4d lqr_S_;
  Eigen::Matrix4d Q_proj_;
  Eigen::Matrix4d Q_proj_f_;
  Eigen::Matrix<double, 4, 2> g_proj_p1_;
  Eigen::Matrix<double, 4, 2> g_proj_p2_;
  Eigen::Matrix4d p2o_premul_;
  Eigen::Matrix4d projection_to_p2o_complement_;
  Eigen::Matrix<double, 4, 2> p2o_orthogonal_complement_;
  Eigen::Matrix<double, 4, 2> p2o_basis_;

};


}
}
}