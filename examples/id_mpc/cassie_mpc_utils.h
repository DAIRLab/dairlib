#pragma once

#include "systems/controllers/id_mpc/id_mpc.h"
#include "systems/controllers/id_mpc/systems/walking_reference_system.h"


namespace dairlib {

std::unique_ptr<systems::controllers::id_mpc::ConstrainedDynamicsInfo>
    MakeCassieDynamics();

systems::controllers::id_mpc::GaitParams
MakeCassieGaitParams(const systems::controllers::id_mpc::IDMPCParams&
                     mpc_params);

}