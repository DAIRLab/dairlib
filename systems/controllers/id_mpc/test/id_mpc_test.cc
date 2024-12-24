#include <iostream>
#include "systems/controllers/id_mpc/constrained_dynamics_info.h"
#include "systems/controllers/id_mpc/id_mpc.h"

namespace dairlib::systems::controllers::id_mpc {

using Eigen::VectorXd;
using Eigen::Vector3d;

int DoMain() {
  std::string urdf = "examples/Cassie/urdf/cassie_fixed_springs.urdf";
  auto dynamics_info = std::make_unique<ConstrainedDynamicsInfo>(urdf);

  dynamics_info->AddContactPoint(
      "toe_left_front",
      "toe_left",
      Vector3d(-0.0457, 0.112, 0),
      {0, 1, 2},
      0.8
  );

  dynamics_info->AddDistanceConstraint(
      "thigh_left",
      Vector3d(0.0, 0.0, 0.045),
      "heel_spring_left",
      Vector3d(.11877, -.01, 0.0),
      0.5012
  );

  IDMPCParams params;
  params.N = 10;
  params.dt = 0.1;

  IDMPC mpc(params, std::move(dynamics_info));

  return 0;
}

}

int main(int argc, char**argv) {
  return dairlib::systems::controllers::id_mpc::DoMain();
}