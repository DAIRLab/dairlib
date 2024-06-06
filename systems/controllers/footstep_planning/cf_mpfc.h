#pragma once
#include <array>
#include <chrono>
#include <map>

#include "cf_mpfc_utils.h"
#include "geometry/convex_polygon_set.h"
#include "solvers/optimization_utils.h"

#include "drake/solvers/decision_variable.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/gurobi_solver.h"
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

struct cf_mpfc_solution {
  std::vector<Eigen::VectorXd> xc; // SRB State
  std::vector<Eigen::VectorXd> uc; // SRB Input
  Eigen::Vector4d xi;              // ALIP state at beginning of next stance phase
  std::vector<Eigen::Vector4d> xx; // step-to-step alip state
  std::vector<Eigen::Vector3d> pp; // footstep positions
};

struct cf_mpfc_params {
  alip_utils::AlipGaitParams gait_params;
  int nmodes;
  double time_regularization;
  double soft_constraint_cost;
  Eigen::Vector2d com_pos_bound;
  Eigen::Vector2d com_vel_bound;
  Eigen::MatrixXd Q;
  Eigen::MatrixXd R;
  Eigen::MatrixXd Qf;
  drake::solvers::SolverOptions solver_options;
  double ankle_torque_regularization = 1.0;
};

class CFMPFC {
 public:
  CFMPFC(cf_mpfc_params params);

 protected:
  void MakeMPCVariables();
  void MakeMPCCosts();
  void MakeInputConstraints();
  void MakeStateConstraints();
  void MakeDynamicsConstraint();
  void MakeInitialConditionsConstraints();

  void UpdateInitialConditions(
      const Eigen::Vector4d& x, const Eigen::Vector3d& p,
      double t, double tmin, double tmax);
  void UpdateCrossoverConstraint(alip_utils::Stance stance);
  void UpdateFootholdConstraints(const geometry::ConvexPolygonSet& footholds);
  void UpdateInputCost(const Eigen::Vector2d& vdes, alip_utils::Stance stance);
  void UpdateTrackingCost(const Eigen::Vector2d& vdes, alip_utils::Stance stance);
  void UpdateTrackingCostVelocity(const Eigen::Vector2d& vdes);
  void UpdateTerminalCostVelocity(const Eigen::Vector2d& vdes);
  void UpdateTrackingCostGait(const Eigen::Vector2d& vdes,  alip_utils::Stance stance);
  void UpdateTerminalCostGait(const Eigen::Vector2d& vdes,  alip_utils::Stance stance);
  void UpdateTimeRegularization(double t);

  void ValidateParams() const {
    DRAKE_DEMAND(params_.nmodes >= 2); // need to take 1 footstep (2 modes)
    DRAKE_DEMAND(params_.gait_params.double_stance_duration > 0);
    DRAKE_DEMAND(params_.soft_constraint_cost >= 0);
  }

  void Check() {
    DRAKE_DEMAND(initial_foot_c_ != nullptr);
    DRAKE_DEMAND(initial_state_c_ != nullptr);
    DRAKE_DEMAND(dynamics_c_ != nullptr);
    DRAKE_DEMAND(footstep_choice_c_ != nullptr);
    DRAKE_DEMAND(terminal_cost_ != nullptr);
    DRAKE_DEMAND(time_regularization_ != nullptr);
    DRAKE_DEMAND(model_switch_c_ != nullptr);
  };

  // problem data:
  cf_mpfc_params params_;
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
  VectorXDecisionVariable tau_;          // first footstep duration
  VectorXDecisionVariable u_;            // Integrated ankle torque

  std::shared_ptr<LinearEqualityConstraint> initial_foot_c_ = nullptr;
  std::shared_ptr<LinearEqualityConstraint> initial_state_c_ = nullptr;
  std::shared_ptr<LinearEqualityConstraint> initial_alip_state_c_ = nullptr;
  std::shared_ptr<LinearEqualityConstraint> model_switch_c_ = nullptr;
  std::shared_ptr<LinearEqualityConstraint> dynamics_c_ = nullptr;
  std::shared_ptr<LinearEqualityConstraint> alip_dynamics_c_ = nullptr;
  std::shared_ptr<LinearEqualityConstraint> footstep_choice_c_ = nullptr;

  vector<Binding<LinearConstraint>> workspace_c_{};
  vector<Binding<LinearConstraint>> no_crossover_c_{};
  vector<Binding<LinearConstraint>> reachability_c_{};

  std::shared_ptr<QuadraticCost> ankle_torque_regularization_ = nullptr;
  std::shared_ptr<QuadraticCost> time_regularization_ = nullptr;
  std::shared_ptr<QuadraticCost> terminal_cost_ = nullptr;
  vector<Binding<QuadraticCost>> tracking_cost_{};
  vector<Binding<QuadraticCost>> input_cost_{};
  vector<Binding<QuadraticCost>> soft_constraint_cost_{};

  // some useful matrices and dynamics quantities
  Eigen::Matrix4d A_;
  Eigen::Matrix<double, 4, 2> B_;
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