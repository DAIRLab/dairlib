#pragma once
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
                         const Eigen::VectorXd& initial_state);

  void SetFootstepInitialGuess(const std::vector<Eigen::Vector3d>& pp);
  void UpdateFootstepLocationsInStackedVariables(
      const std::vector<Eigen::Vector3d>& pp, Eigen::VectorXd* z) const;

  int n_footsteps() const { return params_.footstep_horizon; }

 private:
  void UpdateFootstepConstraints(
      const std::vector<std::string>& foot_names,
      const std::vector<Eigen::Vector3d>& contact_points);

  void MakeFootsteps();
  void MakeSwingTrajCosts();
  void MakeGroundConstraints();
  void MakeFootLevelingCosts();

  IDMPC mpc_;
  GaitParams params_;

  std::vector<drake::solvers::VectorXDecisionVariable> pp_;
  std::vector<std::shared_ptr<PointPositionConstraint<AutoDiffXd>>>
  td_constraints_;

};

}