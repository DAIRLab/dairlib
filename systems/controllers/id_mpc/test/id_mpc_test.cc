#include "systems/controllers/id_mpc/constrained_dynamics_info.h"

namespace dairlib::systems::controllers::id_mpc {

using Eigen::Vector3d;

int DoMain() {
  std::string urdf = "examples/Cassie/urdf/cassie_fixed_springs.urdf";
  ConstrainedDynamicsInfo dynamics_info(urdf);

  dynamics_info.AddContactPoint(
      "toe_left_front",
      "toe_left",
      Vector3d(-0.0457, 0.112, 0),
      {0, 1, 2},
      0.8
  );

  dynamics_info.AddDistanceConstraint(
      "thigh_left",
      Vector3d(0.0, 0.0, 0.045),
      "heel_spring_left",
      Vector3d(.11877, -.01, 0.0),
      0.5012
  );

  return 0;
}

}

int main(int argc, char**argv) {
  return dairlib::systems::controllers::id_mpc::DoMain();
}