#include <memory>
#include "cassie_mpc_utils.h"


namespace dairlib {

using systems::controllers::id_mpc::ConstrainedDynamicsInfo;
using Eigen::Vector3d;

std::unique_ptr<ConstrainedDynamicsInfo> MakeCassieDynamics() {
  std::string urdf = "examples/Cassie/urdf/cassie_fixed_springs.urdf";
  auto dynamics_info = std::make_unique<ConstrainedDynamicsInfo>(urdf);

  dynamics_info->AddContactPoint(
      "toe_left_front",
      "toe_left",
      Vector3d(-0.0457, 0.112, 0),
      {0, 1, 2},
      0.8
  );
  dynamics_info->AddContactPoint(
      "toe_left_rear",
      "toe_left",
      Vector3d(0.088, 0, 0),
      {1, 2},
      0.8
  );
  dynamics_info->AddContactPoint(
      "toe_right_front",
      "toe_right",
      Vector3d(-0.0457, 0.112, 0),
      {0, 1, 2},
      0.8
  );
  dynamics_info->AddContactPoint(
      "toe_right_rear",
      "toe_right",
      Vector3d(0.088, 0, 0),
      {1, 2},
      0.8
  );

  dynamics_info->AddDistanceConstraint(
      "thigh_left",
      Vector3d(0.0, 0.0, 0.045),
      "heel_spring_left",
      Vector3d(.11877, -.01, 0.0),
      0.5012
  );

  dynamics_info->AddDistanceConstraint(
      "thigh_right",
      Vector3d(0.0, 0.0, -0.045),
      "heel_spring_right",
      Vector3d(.11877, -.01, 0.0),
      0.5012
  );

  return std::move(dynamics_info);
}

}