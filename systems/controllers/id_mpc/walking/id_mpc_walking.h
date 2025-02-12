#pragma once

#include "systems/controllers/id_mpc/id_mpc.h"
#include "walking_utils.h"


namespace dairlib::systems::controllers::id_mpc {

class IDMPCWalking {
 public:
  IDMPCWalking(IDMPCParams mpc_params,
               std::unique_ptr<ConstrainedDynamicsInfo> dynamics,
               GaitParams gait_params);
 private:
  void MakeGroundConstraints();

};

}