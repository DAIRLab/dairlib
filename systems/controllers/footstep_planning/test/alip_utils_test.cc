#include <iostream>
#include "systems/controllers/footstep_planning/alip_utils.h"

namespace dairlib::systems::controllers::alip_utils {

using Eigen::Vector2d;
using Eigen::Matrix4d;
using Eigen::Matrix;

int DoMain() {

  auto test_gait = AlipGaitParams {
      0.85,
      32.0,
      0.3,
      0.1,
      0.3,
      Vector2d::UnitX(),
      Stance::kLeft,
      ResetDiscretization::kZOH
  };

  const auto& [x0, x1] = MakePeriodicAlipGait(test_gait);

  Matrix4d Pi0;
  Matrix4d Pi1;
  Matrix<double, 4, 2> d0;
  Matrix<double, 4, 2> d1;

  MakeProjectionToP2Orbit(test_gait, Pi0, Pi1, d0, d1);

  std::cout << Pi0 * (x0 - d0.col(0)) << "\n" << std::endl;
  std::cout << Pi1 * (x1 - d1.col(0)) << std::endl;

  return 0;
}

}


int main(int argc, char **argv) {
  return dairlib::systems::controllers::alip_utils::DoMain();
}