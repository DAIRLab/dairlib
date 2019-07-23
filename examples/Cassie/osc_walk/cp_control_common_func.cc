#include "examples/Cassie/osc_walk/cp_control_common_func.h"

using Eigen::Vector2d;

namespace dairlib {
namespace cassie {
namespace osc_walk {

double GetDesiredHeadingPos(double pelvis_yaw_pos,
                            Vector2d global_CoM_to_target_pos,
                            double circle_radius_of_no_turning) {
  if (global_CoM_to_target_pos.norm() < circle_radius_of_no_turning) {
    return pelvis_yaw_pos;
  } else {
    return atan2(global_CoM_to_target_pos(1),
                 global_CoM_to_target_pos(0));
  }
}

}  // namespace osc_walk
}  // namespace cassie
}  // namespace dairlib


