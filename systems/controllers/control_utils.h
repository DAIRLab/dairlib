#pragma once

#include <memory>
#include <Eigen/Dense>

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/copyable_unique_ptr.h"

namespace dairlib {
namespace systems {

/// ImposeHalfplaneGuard() updates the foot placement position by restricting
/// it in a halfplane.
/// The halfplane is defined by two things:
///  - A point on the line (the edge of the halfplace).
///    The point is either shifted CoM or stance foot depending on the motion of
///    the robot. This CoM is shifted by a distance `center_line_offset` in
///    order to avoid foot collision.
///  - The slope/direction of the line.
///    The direction of the line is the robot's yaw angle.
///
/// Inputs:
///  - `foot_placement_pos` planned foot placement position
///  - `CoM` center of mass position
///  - `left_or_right_stance` a flag that indicates whether it's currently
///     in left stance (left_or_right_stance = true) or
///     in right stance (left_or_right_stance = false).
///  - `yaw` current yaw angle
///  - `CoM` current center of mass position
///  - `stance_foot_pos` current stance foot position
///  - `center_line_offset` offset of the center line
///
/// Requirement:
///  - This function is designed for bipedal robots.
Eigen::Vector2d ImposeHalfplaneGuard(Eigen::Vector2d foot_placement_pos,
    bool left_or_right_stance,
    double yaw, Eigen::Vector2d CoM, Eigen::Vector2d stance_foot_pos,
    double center_line_offset);


/// ImposeStepLengthGuard() updates the foot placement position by imposing a
/// step length limit.
///
/// Inputs:
///  - `foot_placement_pos` planned foot placement position
///  - `CoM` center of mass position
///  - `max_dist` maximum distance from center of mass to foot placement
///    position
Eigen::Vector2d ImposeStepLengthGuard(Eigen::Vector2d foot_placement_pos,
    Eigen::Vector2d CoM, double max_dist);
}  // namespace systems
}  // namespace dairlib
