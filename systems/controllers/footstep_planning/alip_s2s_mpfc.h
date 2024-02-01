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


struct alip_s2s_mpfc_params {
  alip_utils::AlipGaitParams gait_params;
  int nmodes;
  double tmin;
  double tmax;
  double soft_constraint_cost;
  Eigen::Vector2d com_pos_bound;
  Eigen::Vector2d com_vel_bound;
  Eigen::MatrixXd Q;
  Eigen::MatrixXd R;
  Eigen::MatrixXd Qf;
  drake::solvers::SolverOptions solver_options;
};


class AlipS2SMPFC {
 public:

  /// Constructor takes a nominal gait and cost parameters
  AlipS2SMPFC(alip_s2s_mpfc_params params) : params_(params){
    DRAKE_DEMAND(params_.nmodes >= 2); // controller defined over 2 stances min
    void MakeMPCVariables();
    void MakeMPCCosts();
    void MakeInputConstraints();
    void MakeStateConstraints();
  };

  std::pair<Eigen::Vector4d, double> CalculateOptimalFootstepAndTiming(
      const Eigen::Vector4d& x,
      const Eigen::Vector3d& p,
      double t,
      alip_utils::Stance stance
  );

 protected:
  static constexpr int kMaxFootholds = 20;
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

  void Check();

  // problem data:
  alip_s2s_mpfc_params params_;
  const int np_ = 3;
  const int nx_ = 4;

  // program and decision variables
  drake::copyable_unique_ptr<MathematicalProgram> prog_{
    std::make_unique<MathematicalProgram>()
  };
  vector<VectorXDecisionVariable> pp_{}; // footstep sequence
  vector<VectorXDecisionVariable> xx_{}; // ALIP state
  vector<VectorXDecisionVariable> ee_{}; // workspace soft constraint slack var
  vector<VectorXDecisionVariable> mu_{}; // binary variables
  VectorXDecisionVariable t0_;           // first footstep duration

  std::shared_ptr<LinearEqualityConstraint> initial_foot_c_ = nullptr;
  std::shared_ptr<LinearEqualityConstraint> initial_state_c_ = nullptr;
  std::shared_ptr<BoundingBoxConstraint> initial_time_constraint_ = nullptr;
  std::shared_ptr<LinearEqualityConstraint> dynamics_c_ = nullptr;
  std::shared_ptr<LinearEqualityConstraint> footstep_choice_c_ = nullptr;

  vector<Binding<LinearConstraint>> workspace_c_{};
  vector<Binding<LinearConstraint>> no_crossover_c_{};
  vector<vector<LinearBigMConstraint>> footstep_c_{};
  vector<vector<LinearBigMEqualityConstraint>> footstep_c_eq_{};


  std::shared_ptr<QuadraticCost> terminal_cost_ = nullptr;
  vector<Binding<QuadraticCost>> tracking_cost_{};
  vector<Binding<QuadraticCost>> input_cost_{};
  vector<Binding<QuadraticCost>> soft_constraint_cost_{};

  // Bookkeeping
  bool built_ = false;
};
}
