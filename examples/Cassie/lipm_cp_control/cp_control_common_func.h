#pragma once
#include "systems/framework/output_vector.h"

using Eigen::Vector2d;

namespace dairlib {
namespace cassie {
namespace cp_control {

double GetDesiredHeadingPos(double pelvis_yaw_pos,
                            Vector2d global_CoM_to_target_pos, 
                            double circle_radius_of_no_turning);

}  // namespace cp_control
}  // namespace cassie
}  // namespace dairlib


