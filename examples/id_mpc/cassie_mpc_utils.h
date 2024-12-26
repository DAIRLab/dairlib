#pragma once

#include "systems/controllers/id_mpc/id_mpc.h"

namespace dairlib {

std::unique_ptr<systems::controllers::id_mpc::ConstrainedDynamicsInfo>
    MakeCassieDynamics();

}