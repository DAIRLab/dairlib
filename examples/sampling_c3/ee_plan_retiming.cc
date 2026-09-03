#include "examples/sampling_c3/ee_plan_retiming.h"

#include <algorithm>
#include <cmath>

#include "drake/common/drake_throw.h"

namespace dairlib {

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace {
constexpr double kMinKnotDt = 1e-4;
constexpr double kRelTol = 1e-12;
}  // namespace

void RetimeEEPlanToVelocityLimits(
    const Eigen::Ref<const MatrixXd>& ee_position_traj, double v_xy_max,
    double v_z_max, VectorXd* time_vector) {
  DRAKE_THROW_UNLESS(ee_position_traj.rows() == 3);
  DRAKE_THROW_UNLESS(ee_position_traj.cols() == time_vector->size());
  DRAKE_THROW_UNLESS(v_xy_max > 0.0);
  DRAKE_THROW_UNLESS(v_z_max > 0.0);

  for (int i = 1; i < ee_position_traj.cols(); ++i) {
    const Vector3d step = ee_position_traj.col(i) - ee_position_traj.col(i - 1);
    const double dxy = step.head(2).norm();
    const double dz = std::abs(step(2));
    const double dt_min = std::max({(*time_vector)(i) - (*time_vector)(i - 1),
                                    dxy / v_xy_max, dz / v_z_max, kMinKnotDt});
    (*time_vector)(i) = (*time_vector)(i - 1) + dt_min;
  }
}

RetimedPlanSampling ResampleEEPlanOnUniformGrid(
    const Eigen::Ref<const MatrixXd>& ee_position_traj,
    const VectorXd& retimed_times, double dt) {
  const int n = ee_position_traj.cols();
  DRAKE_THROW_UNLESS(ee_position_traj.rows() == 3);
  DRAKE_THROW_UNLESS(retimed_times.size() == n);
  DRAKE_THROW_UNLESS(n >= 2);
  DRAKE_THROW_UNLESS(dt > 0.0);
  DRAKE_THROW_UNLESS(retimed_times(0) == 0.0);

  RetimedPlanSampling sampling;
  sampling.positions.resize(3, n);
  sampling.segment.assign(n, 0);
  sampling.fraction = VectorXd::Zero(n);
  sampling.time_scale = VectorXd::Ones(n);

  // The sample times only increase, so walk the retimed grid alongside them
  // instead of searching it per sample.
  int k = 0;
  for (int i = 0; i < n; ++i) {
    const double t = i * dt;
    while (k < n - 2 && retimed_times(k + 1) <= t) {
      ++k;
    }
    const double seg_dt = retimed_times(k + 1) - retimed_times(k);
    if (seg_dt > kRelTol) {
      // Only update if the time difference is large enough to avoid rounding
      // issues.
      sampling.fraction(i) =
          std::clamp((t - retimed_times(k)) / seg_dt, 0.0, 1.0);
      sampling.time_scale(i) =
          seg_dt > dt * (1.0 + kRelTol) ? dt / seg_dt : 1.0;
    }
    sampling.positions.col(i) =
        ee_position_traj.col(k) +
        sampling.fraction(i) *
            (ee_position_traj.col(k + 1) - ee_position_traj.col(k));
    sampling.segment.at(i) = k;
  }
  return sampling;
}

}  // namespace dairlib
