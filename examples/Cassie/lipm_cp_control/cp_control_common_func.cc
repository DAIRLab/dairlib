#pragma once
#include "examples/Cassie/lipm_cp_control/cp_control_common_func.h"

using Eigen::Vector2d;

namespace dairlib {
namespace cassie {
namespace cp_control {

double getDesiredHeadingPos(double pelvis_yaw_pos,
                            Vector2d global_CoM_to_target_pos,
                            double circle_radius_of_no_turning) {
  if (global_CoM_to_target_pos.norm() < circle_radius_of_no_turning) {
    return pelvis_yaw_pos;
  } else {
    return atan2(global_CoM_to_target_pos(1),
                 global_CoM_to_target_pos(0));
  }
}

}  // namespace cp_control
}  // namespace cassie
}  // namespace dairlib


