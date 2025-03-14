#pragma once

#include "solvers/admm/ncqp_solver.h"
#include "solvers/admm/convex_polygon_set_constraint.h"
#include "solvers/sqp/sqp_quadratic_cost.h"

#include "walking_utils.h"
#include "systems/controllers/id_mpc/id_mpc.h"
#include "systems/controllers/id_mpc/constraints/point_position_constraint.h"
#include "systems/controllers/id_mpc/constraints/alip_mapping_constraint.h"

namespace dairlib::systems::controllers::id_mpc {

class IDMPCWalking {
 public:
  IDMPCWalking(IDMPCParams mpc_params,
               std::unique_ptr<ConstrainedDynamicsInfo> dynamics,
               GaitParams gait_params);

  [[nodiscard]] const IDMPC& mpc() const {
    return mpc_;
  }
  [[nodiscard]] IDMPC& mutable_mpc() {
    return mpc_;
  }
  [[nodiscard]] const ConstrainedDynamicsInfo& dynamics() const {
    return mpc_.dynamics();
  }

  void UpdateProblemData(const MPCReference& reference,
                         const Eigen::VectorXd& initial_state,
                         const Eigen::VectorXd& prev_sol,
                         const geometry::ConvexPolygonSet& footholds);

  void SetFootstepInitialGuess(const std::vector<Eigen::Vector3d>& pp);
  void UpdateFootstepLocationsInStackedVariables(
      const std::vector<Eigen::Vector3d>& pp, Eigen::VectorXd* z) const;

  int n_footsteps() const { return params_.footstep_horizon; }

  solvers::NCQPSolver::SetMembershipConstraints GetFootholdConstraints(
      const Eigen::VectorXd& z);

  std::vector<Eigen::VectorXd> get_footstep_solutions(const Eigen::VectorXd& z) const;

 private:
  void UpdateFootstepConstraints(
      const std::vector<std::string>& foot_names,
      const std::vector<Eigen::Vector3d>& contact_points);

  void UpdateALIPTerms(const MPCReference& reference);

  void MakeFootsteps();
  void MakeALIPTerms();
  void MakeSwingTrajCosts();
  void MakeGroundConstraints();
  void MakeFootLevelingCosts();


  IDMPC mpc_;
  GaitParams params_;

  std::vector<drake::solvers::VectorXDecisionVariable> pp_; // Footstep pos
  std::vector<drake::solvers::VectorXDecisionVariable> xa_; // ALIP states
  std::vector<std::shared_ptr<PointPositionConstraint<AutoDiffXd>>>
  td_constraints_;

  std::vector<std::shared_ptr<solvers::ConvexPolygonSetConstraint>> footholds_;
  std::vector<drake::solvers::Binding<drake::solvers::Constraint>>
  foothold_bindings_;

  std::shared_ptr<ALIPMappingConstraint> alip_mapping_constraint_;
  drake::solvers::LinearEqualityConstraint* initial_s2s_state_constraint_;
  std::vector<drake::solvers::LinearEqualityConstraint*> alip_dynamics_;
  std::vector<std::shared_ptr<solvers::sqp::SqpQuadraticCost>>
  alip_state_costs_;
  std::vector<std::shared_ptr<solvers::sqp::SqpQuadraticCost>>
  alip_footstep_costs_;

};

}