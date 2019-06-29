#pragma once
#include "systems/framework/output_vector.h"

namespace dairlib {
namespace cassie {
namespace cp_control {

double getDesiredHeadingPos(double pelvis_yaw_pos,
                            Eigen::Vector2d global_CoM_to_target_pos,
                            double circle_radius_of_no_turning);

}  // namespace cp_control
}  // namespace cassie
}  // namespace dairlib


