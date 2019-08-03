#pragma once
#include "systems/framework/output_vector.h"

namespace dairlib {
namespace cassie {
namespace osc_walk {

/// GetDesiredYawAngle() calculates the disired yaw angle in global frame for
/// Cassie pelvis.
///
/// Control philosophy:
///   If the current center of mass (com) position is close to the target
///   position, then the desired pevlis yaw is the current yaw, y_c.
///   On the other hand, if the com is far from target position, then the
///   desired yaw is y_t, the angle between global x axis and the line from com
///   to target position.
///
/// We use logistic function to implement the weighting for the current position
/// y_c and y_t.
/// Logistic function = 1 / (1 - params_1*exp(x-params_2))
/// Function visualization: https://www.desmos.com/calculator/agxuc5gip8
double GetDesiredYawAngle(double pelvis_yaw_pos,
                            Eigen::Vector2d com_to_target_pos,
                            Eigen::Vector2d params);

}  // namespace osc_walk
}  // namespace cassie
}  // namespace dairlib


