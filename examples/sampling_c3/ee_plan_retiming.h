#pragma once

// End effector plan retiming, used by the sampling C3 controller to enforce
// plans respect end effector velocity limits (particularly important for the 3D
// printer demos, since the hardware has a very restrictive z-axis velocity
// limit).
//
// Two steps that are used together but are useful separately:
//
//   1. RetimeEEPlanToVelocityLimits stretches a plan's time grid until no
//      segment asks the end effector to move faster than the machine can.  The
//      geometric path is untouched; the plan just takes longer.  This is what
//      SC3 publishes for execution.
//
//   2. ResampleEEPlanOnUniformGrid takes that stretched trajectory and samples
//      it back onto the original uniform time grid, which trades the extra
//      duration back for a shorter path.  This is more fair for comparing
//      sample costs.

#include <vector>

#include <Eigen/Dense>

namespace dairlib {

/// Stretches @p time_vector in place so that, under a first order hold over
/// (*time_vector, ee_position_traj), no segment exceeds the horizontal (xy)
/// speed @p v_xy_max or the vertical (z) speed @p v_z_max.  Only ever slows the
/// plan down: knot 0 keeps its time and later knots slide out, so the path is
/// untouched and no discontinuity is introduced.  Times are kept strictly
/// increasing.
///
/// @p ee_position_traj is 3 x n; @p time_vector has n entries.  Both speeds
/// must be positive.
void RetimeEEPlanToVelocityLimits(
    const Eigen::Ref<const Eigen::MatrixXd>& ee_position_traj, double v_xy_max,
    double v_z_max, Eigen::VectorXd* time_vector);

/// Where each of a plan's original knot times lands once the plan has been
/// slowed down, together with the resampled path itself.  All fields have one
/// entry (or column) per sample.
struct RetimedPlanSampling {
  /// The resampled end effector positions, 3 x n.
  Eigen::MatrixXd positions;
  /// The retimed segment the sample fell in, in [0, n-2].
  std::vector<int> segment;
  /// How far into that segment the sample fell, in [0, 1].
  Eigen::VectorXd fraction;
  /// dt divided by that segment's retimed duration, in (0, 1]: the factor by
  /// which a velocity along the original plan has to be scaled to be traversed
  /// at the slowed pace.  Never scales a velocity up, so it is exactly 1 for a
  /// segment the retiming left alone.
  Eigen::VectorXd time_scale;
};

/// Samples the trajectory (@p ee_position_traj columns at @p retimed_times)
/// back onto the uniform grid t_i = i * @p dt, i = 0 ... n-1, where n is the
/// number of columns.  @p retimed_times must start at 0 and increase; it is
/// expected to come from RetimeEEPlanToVelocityLimits, so it spans at least
/// (n-1) * dt and no sample extrapolates.  A sample past the end saturates at
/// the last knot rather than extrapolating.
///
/// The `segment`, `fraction` and `time_scale` fields of the result let callers
/// carry the rest of a plan onto the same map: interpolate a per knot quantity
/// with `fraction` (scaling a velocity by `time_scale`), or hold a per segment
/// quantity such as an input from `segment`.
///
/// Sample 0 always reproduces knot 0 exactly, and if no segment was stretched
/// the sampling is the identity: sample i is knot i.
RetimedPlanSampling ResampleEEPlanOnUniformGrid(
    const Eigen::Ref<const Eigen::MatrixXd>& ee_position_traj,
    const Eigen::VectorXd& retimed_times, double dt);

}  // namespace dairlib
