#pragma once
#include "systems/framework/output_vector.h"

namespace dairlib {
namespace cassie {
namespace osc_walking_control {

double GetDesiredHeadingPos(double pelvis_yaw_pos,
                            Eigen::Vector2d global_CoM_to_target_pos,
                            double circle_radius_of_no_turning);

}  // namespace osc_walking_control
}  // namespace cassie
}  // namespace dairlib


