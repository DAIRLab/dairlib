#pragma once

#include <Eigen/Dense>

namespace dairlib {
namespace systems {

/// ImposeHalfplaneGaurd() updates the foot placement position by restricting
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
///  - `left_stance_state` left stance state (of finite state machine)
///  - `right_stance_state` right stance state (of finite state machine)
///  - `state` current state of finite state machine
///  - `yaw` current yaw angle
///  - `CoM` current center of mass position
///  - `stance_foot_pos` current stance foot position
///  - `center_line_offset` offset of the center line
///
/// Requirement:
///  - The controller is used with two-state finite state machine
Eigen::Vector2d ImposeHalfplaneGaurd(Eigen::Vector2d foot_placement_pos,
    int left_stance_state, int right_stance_state, int state,
    double yaw, Eigen::Vector2d CoM, Eigen::Vector2d stance_foot_pos,
    double center_line_offset);


/// ImposeStepLengthGaurd() updates the foot placement position by imposing a
/// step length limit.
///
/// Inputs:
///  - `foot_placement_pos` planned foot placement position
///  - `CoM` center of mass position
///  - `max_dist` maximum distance from center of mass to foot placement
///    position
Eigen::Vector2d ImposeStepLengthGaurd(Eigen::Vector2d foot_placement_pos,
    Eigen::Vector2d CoM, double max_dist);
}  // namespace systems
}  // namespace dairlib
