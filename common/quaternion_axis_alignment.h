#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace dairlib {

// Angle between `tracked_axis_body` (a unit vector in the object's body
// frame) as rotated into world frame by curr_quat vs. by reference_quat.
// Stateless: well-defined and continuous even when the two directions are
// antiparallel.
double ComputeAxisMisalignmentAngle(const Eigen::Quaterniond& curr_quat,
                                    const Eigen::Quaterniond& reference_quat,
                                    const Eigen::Vector3d& tracked_axis_body);

// Returns goal_quat = q_align * curr_quat, where q_align is the minimal
// rotation that swings tracked_axis_body's current world-frame direction
// (under curr_quat) onto its direction under reference_quat. Because q_align's
// rotation axis is always perpendicular to tracked_axis_body's current world
// direction, this introduces zero twist about that axis: goal_quat differs from
// curr_quat only in the tracked axis's alignment.
//
// *hysteresis_axis_state must be owned by the caller and persisted across calls
// (initialize to Eigen::Vector3d::Zero()); it disambiguates the swing axis when
// the two directions are nearly parallel or antiparallel, and applies
// hysteresis near the antipodal singularity so the chosen axis does not flip
// back and forth between consecutive calls.
Eigen::Quaterniond ComputeAxisAlignedGoalQuaternion(
    const Eigen::Quaterniond& curr_quat,
    const Eigen::Quaterniond& reference_quat,
    const Eigen::Vector3d& tracked_axis_body, double angle_hysteresis,
    Eigen::Vector3d* hysteresis_axis_state);

}  // namespace dairlib
