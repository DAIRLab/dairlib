#pragma once
#include <array>
#include <chrono>
#include <map>

#include "alip_utils.h"
#include "geometry/convex_polygon_set.h"
#include "solvers/optimization_utils.h"

#include "drake/solvers/decision_variable.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/gurobi_solver.h"

#include "alip_s2s_mpfc_params.h"

namespace dairlib::systems::controllers {

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

using std::vector;

struct alip_s2s_mpfc_solution {
  // copy of solution info
  vector<Eigen::Vector3d> pp{}; // footstep sequence
  vector<Eigen::Vector4d> xx{}; // ALIP state
  vector<Eigen::VectorXd> ee{}; // workspace soft constraint slack var
  vector<Eigen::VectorXd> mu{}; // binary variables
  double t_sol;                 // first footstep duration

  // some debug info
  bool success;
  double t_nom;
  double total_time;
  double optimizer_time;
  Eigen::Vector2d desired_velocity;
  drake::solvers::SolutionResult solution_result;
  geometry::ConvexPolygonSet input_footholds;
};

class AlipS2SMPFC {
 public:

  /// Constructor takes a nominal gait and cost parameters
  AlipS2SMPFC(alip_s2s_mpfc_params params);

  alip_s2s_mpfc_solution Solve(
      const Eigen::Vector4d& x,
      const Eigen::Vector3d& p,
      double t,
      const Eigen::Vector2d& vdes,
      alip_utils::Stance stance,
      const geometry::ConvexPolygonSet& footholds
  );

  const alip_utils::AlipGaitParams& gait_params() const {
    return params_.gait_params;
  }

 protected:
  static constexpr size_t kMaxFootholds = 20;
  void MakeMPCVariables();
  void MakeMPCCosts();
  void MakeInputConstraints();
  void MakeStateConstraints();
  void MakeDynamicsConstraint();
  void MakeInitialConditionsConstraints();

  void UpdateInitialConditions(
      const Eigen::Vector4d& x, const Eigen::Vector3d& p, double t);
  void UpdateCrossoverConstraint(alip_utils::Stance stance);
  void UpdateFootholdConstraints(const geometry::ConvexPolygonSet& footholds);
  void UpdateInputCost(const Eigen::Vector2d& vdes, alip_utils::Stance stance);
  void UpdateTrackingCost(const Eigen::Vector2d& vdes);
  void UpdateTerminalCost(const Eigen::Vector2d& vdes);
  void UpdateTimeRegularization(double t);

  void ValidateParams() const {
    DRAKE_DEMAND(params_.nmodes >= 2); // need to take 1 footstep (2 modes)
    DRAKE_DEMAND(params_.tmin > 0);
    DRAKE_DEMAND(params_.tmax >= params_.tmin);
    DRAKE_DEMAND(params_.gait_params.single_stance_duration <= params_.tmax);
    DRAKE_DEMAND(params_.gait_params.single_stance_duration >= params_.tmin);
    DRAKE_DEMAND(params_.gait_params.double_stance_duration > 0);
    DRAKE_DEMAND(params_.soft_constraint_cost >= 0);
  }

  void Check() {
    DRAKE_DEMAND(initial_foot_c_ != nullptr);
    DRAKE_DEMAND(initial_state_c_ != nullptr);
    DRAKE_DEMAND(initial_time_constraint_ != nullptr);
    DRAKE_DEMAND(dynamics_c_ != nullptr);
    DRAKE_DEMAND(footstep_choice_c_ != nullptr);
    DRAKE_DEMAND(terminal_cost_ != nullptr);
    DRAKE_DEMAND(time_regularization_ != nullptr);
  };

  // helper
  static vector<Eigen::Vector2d> MakeP2Orbit(
      alip_utils::AlipGaitParams gait_params);

  // problem data:
  alip_s2s_mpfc_params params_;
  const int np_ = 3;
  const int nx_ = 4;

  // program and decision variables
  drake::copyable_unique_ptr<MathematicalProgram> prog_{
    std::make_unique<MathematicalProgram>()
  };

  drake::solvers::GurobiSolver solver_;

  vector<VectorXDecisionVariable> pp_{}; // footstep sequence
  vector<VectorXDecisionVariable> xx_{}; // ALIP state
  vector<VectorXDecisionVariable> ee_{}; // workspace soft constraint slack var
  vector<VectorXDecisionVariable> mu_{}; // binary variables
  VectorXDecisionVariable tau_;           // first footstep duration

  std::shared_ptr<LinearEqualityConstraint> initial_foot_c_ = nullptr;
  std::shared_ptr<LinearEqualityConstraint> initial_state_c_ = nullptr;
  std::shared_ptr<BoundingBoxConstraint> initial_time_constraint_ = nullptr;
  std::shared_ptr<LinearEqualityConstraint> dynamics_c_ = nullptr;
  std::shared_ptr<LinearEqualityConstraint> footstep_choice_c_ = nullptr;

  vector<Binding<LinearConstraint>> workspace_c_{};
  vector<Binding<LinearConstraint>> no_crossover_c_{};
  vector<vector<LinearBigMConstraint>> footstep_c_{};
  vector<vector<LinearBigMEqualityConstraint>> footstep_c_eq_{};

  std::shared_ptr<QuadraticCost> time_regularization_ = nullptr;
  std::shared_ptr<QuadraticCost> terminal_cost_ = nullptr;
  vector<Binding<QuadraticCost>> tracking_cost_{};
  vector<Binding<QuadraticCost>> input_cost_{};
  vector<Binding<QuadraticCost>> soft_constraint_cost_{};

  // some useful matrices and dynamics quantities
  Eigen::Matrix4d A_;
  Eigen::Matrix<double, 4, 2> B_;
  Eigen::Matrix4d p2o_premul_;
  Eigen::Matrix4d p2o_cost_hessian_;
  Eigen::Matrix<double, 2, 4> p2o_orthogonal_complement_;
  Eigen::Matrix<double, 4, 2> p2o_basis_;
  Eigen::Matrix<double, 4, 2> p2o_cost_gradient_factor_p1_;
  Eigen::Matrix<double, 4, 2> p2o_cost_gradient_factor_p2_;
};
}
