#pragma once

#include "systems/controllers/id_mpc/id_mpc.h"
#include "walking_utils.h"


namespace dairlib::systems::controllers::id_mpc {

class IDMPCWalking {
 public:
  IDMPCWalking(IDMPCParams mpc_params,
               std::unique_ptr<ConstrainedDynamicsInfo> dynamics,
               GaitParams gait_params);

  const IDMPC& mpc() const {
    return mpc_;
  }

 private:
  void UpdateFootstepConstraints(
      const std::vector<std::string>& foot_names,
      const std::vector<Eigen::Vector3d>& contact_points);

  void MakeFootsteps();
  void MakeGroundConstraints();

  std::vector<drake::solvers::VectorXDecisionVariable> pp_;
  IDMPC mpc_;

};

}