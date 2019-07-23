#include "examples/Cassie/osc_walk/cp_control_common_func.h"

using Eigen::Vector2d;

namespace dairlib {
namespace cassie {
namespace osc_walk {

double GetDesiredYawAngle(double pelvis_yaw_pos,
                            Vector2d com_to_target_pos,
                            Eigen::Vector2d params) {
  double weight = 1 / (1 + exp(-params(0)*(com_to_target_pos.norm()-params(1))));
  std::cout << weight << std::endl;
  return (1-weight) * pelvis_yaw_pos +
      weight * atan2(com_to_target_pos(1), com_to_target_pos(0));
}

}  // namespace osc_walk
}  // namespace cassie
}  // namespace dairlib


