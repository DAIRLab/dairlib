#pragma once

#include "constrained_dynamics_info.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib::systems::controllers::id_mpc {

class IDMPC {
 public:
  IDMPC(std::string urdf);

 private:
  ConstrainedDynamicsInfo dynamics_info_;
};

}