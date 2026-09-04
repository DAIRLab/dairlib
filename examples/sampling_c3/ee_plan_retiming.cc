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

void RetimeAndResampleEEPlan(double dt, double v_xy_max, double v_z_max,
                             int ee_velocity_offset,
                             std::vector<VectorXd>* x_plan,
                             std::vector<VectorXd>* u_plan,
                             std::vector<VectorXd>* lambda_plan) {
  const int n = x_plan->size();
  DRAKE_THROW_UNLESS(n >= 2);
  DRAKE_THROW_UNLESS(static_cast<int>(u_plan->size()) == n - 1);
  DRAKE_THROW_UNLESS(lambda_plan == nullptr ||
                     static_cast<int>(lambda_plan->size()) == n - 1);
  DRAKE_THROW_UNLESS(dt > 0.0);
  DRAKE_THROW_UNLESS(ee_velocity_offset >= 3);

  MatrixXd ee_positions(3, n);
  VectorXd times(n);
  for (int i = 0; i < n; i++) {
    ee_positions.col(i) = x_plan->at(i).head(3);
    times(i) = i * dt;
  }
  RetimeEEPlanToVelocityLimits(ee_positions, v_xy_max, v_z_max, &times);

  const RetimedPlanSampling sampling =
      ResampleEEPlanOnUniformGrid(ee_positions, times, dt);

  // The plan has to be read before any of it is overwritten, since the segment
  // a resampled knot came from is always at or behind that knot.
  const std::vector<VectorXd> x_plan_original = *x_plan;
  const std::vector<VectorXd> u_plan_original = *u_plan;
  for (int i = 0; i < n; i++) {
    const int k = sampling.segment.at(i);
    x_plan->at(i).head(3) = sampling.positions.col(i);
    // Track the same planned EE velocity, scaled down by however much this
    // segment had to be slowed to respect the limits.  Nothing is changed when
    // the segment was not slowed down.
    const Vector3d v_plan_k =
        x_plan_original.at(k).segment(ee_velocity_offset, 3);
    const Vector3d v_plan_next =
        x_plan_original.at(k + 1).segment(ee_velocity_offset, 3);
    x_plan->at(i).segment(ee_velocity_offset, 3) =
        sampling.time_scale(i) *
        (v_plan_k + sampling.fraction(i) * (v_plan_next - v_plan_k));
    if (i < n - 1) {
      u_plan->at(i) = u_plan_original.at(k);
    }
  }
  // lambda is held from the same segments, but has to be read from its own
  // untouched copy for the same reason the states did.
  if (lambda_plan != nullptr) {
    const std::vector<VectorXd> lambda_plan_original = *lambda_plan;
    for (int i = 0; i < n - 1; i++) {
      lambda_plan->at(i) = lambda_plan_original.at(sampling.segment.at(i));
    }
  }
}

}  // namespace dairlib
