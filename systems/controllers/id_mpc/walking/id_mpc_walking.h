#pragma once

#include "solvers/admm/convex_polygon_set_constraint.h"
#include "solvers/admm/ncqp_solver.h"

#include "walking_utils.h"
#include "systems/controllers/id_mpc/id_mpc.h"
#include "systems/controllers/id_mpc/constraints/point_position_constraint.h"

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
                         const geometry::ConvexPolygonSet& footholds);

  void SetFootstepInitialGuess(const std::vector<Eigen::Vector3d>& pp);
  void UpdateFootstepLocationsInStackedVariables(
      const std::vector<Eigen::Vector3d>& pp, Eigen::VectorXd* z) const;

  int n_footsteps() const { return params_.footstep_horizon; }

  solvers::NCQPSolver::SetMembershipConstraints GetFootholdConstraints(
      const Eigen::VectorXd& z);

 private:
  void UpdateFootstepConstraints(
      const std::vector<std::string>& foot_names,
      const std::vector<Eigen::Vector3d>& contact_points);

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
};

}