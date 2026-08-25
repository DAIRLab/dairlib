#include "quaternion_axis_alignment.h"

#include <cmath>

using Eigen::AngleAxisd;
using Eigen::Quaterniond;
using Eigen::Vector3d;

namespace dairlib {

double ComputeAxisMisalignmentAngle(const Quaterniond& curr_quat,
                                    const Quaterniond& reference_quat,
                                    const Vector3d& tracked_axis_body) {
  // Eigen's quaternion-vector rotation assumes a unit quaternion; inputs (e.g.
  // YAML literals like 0.707, or un-renormalized simulator state) aren't
  // guaranteed to be unit quaternions, so normalize explicitly to avoid a
  // curr_quat/reference_quat-dependent directional bias.
  Quaterniond curr_quat_n = curr_quat.normalized();
  Quaterniond reference_quat_n = reference_quat.normalized();
  Vector3d axis_body = tracked_axis_body.normalized();
  Vector3d a_curr_world = curr_quat_n * axis_body;
  Vector3d a_target_world = reference_quat_n * axis_body;
  return std::atan2(a_curr_world.cross(a_target_world).norm(),
                    a_curr_world.dot(a_target_world));
}

Quaterniond ComputeAxisAlignedGoalQuaternion(const Quaterniond& curr_quat,
                                             const Quaterniond& reference_quat,
                                             const Vector3d& tracked_axis_body,
                                             double angle_hysteresis,
                                             Vector3d* hysteresis_axis_state) {
  constexpr double kAxisEpsilon = 1e-9;

  // See the normalization comment in ComputeAxisMisalignmentAngle above -- also
  // needed here so the returned quaternion is a proper unit quaternion rather
  // than inheriting curr_quat's arbitrary norm.
  Quaterniond curr_quat_n = curr_quat.normalized();
  Quaterniond reference_quat_n = reference_quat.normalized();
  Vector3d axis_body = tracked_axis_body.normalized();
  Vector3d a_curr_world = curr_quat_n * axis_body;
  Vector3d a_target_world = reference_quat_n * axis_body;

  Vector3d cross = a_curr_world.cross(a_target_world);
  double angle = std::atan2(cross.norm(), a_curr_world.dot(a_target_world));

  Vector3d axis;
  if (cross.norm() > kAxisEpsilon) {
    axis = cross.normalized();
  } else {
    // Near-parallel or near-antiparallel: the cross product doesn't give a
    // reliable axis. Fall back to the previous hysteresis axis, projected to be
    // perpendicular to a_curr_world, to stay continuous across calls.
    Vector3d fallback = *hysteresis_axis_state;
    if (fallback.squaredNorm() < kAxisEpsilon) {
      fallback = a_curr_world.unitOrthogonal();
    }
    Vector3d projected = fallback - a_curr_world * a_curr_world.dot(fallback);
    if (projected.squaredNorm() < kAxisEpsilon) {
      projected = a_curr_world.unitOrthogonal();
    }
    axis = projected.normalized();
  }

  // Enforce consistency near 180 degrees, mirroring the hysteresis logic in
  // GenerateLineTrajectoryWithLookahead.
  if (axis.dot(*hysteresis_axis_state) < 0 &&
      (M_PI - angle < angle_hysteresis)) {
    angle = 2 * M_PI - angle;
    axis = -axis;
  }
  *hysteresis_axis_state = axis;

  Quaterniond q_align(AngleAxisd(angle, axis));
  return q_align * curr_quat_n;
}

}  // namespace dairlib
