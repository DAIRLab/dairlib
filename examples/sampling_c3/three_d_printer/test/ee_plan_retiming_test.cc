// Unit tests for the EE plan retiming and resampling
// (examples/sampling_c3/ee_plan_retiming.h) that the sampling C3 controller
// uses both to publish an executable plan and to score candidate samples over a
// fixed amount of time.  Everything here runs on hand-built knots whose answers
// are known by inspection.

#include "examples/sampling_c3/ee_plan_retiming.h"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

namespace dairlib {
namespace {

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::vector;

// The 3D printer's configured limits.
constexpr double kVxyMax = 0.12;
constexpr double kVzMax = 0.015;
// The printer's planning_dt.
constexpr double kDt = 0.075;
constexpr double kTol = 1e-12;

// A plan of `n` knots stepping by `step` each segment, starting at the origin.
MatrixXd MakeStraightPlan(int n, const Vector3d& step) {
  MatrixXd knots(3, n);
  for (int i = 0; i < n; i++) {
    knots.col(i) = i * step;
  }
  return knots;
}

// Built the same way the resampler builds the times it samples at, so an
// unstretched plan resamples to itself exactly rather than to within rounding.
VectorXd MakeUniformTimes(int n, double dt) {
  VectorXd times(n);
  for (int i = 0; i < n; i++) {
    times(i) = i * dt;
  }
  return times;
}

// Checks that no segment of (times, knots) exceeds the speed limits.
void ExpectWithinVelocityLimits(const MatrixXd& knots, const VectorXd& times) {
  for (int i = 1; i < knots.cols(); i++) {
    const Vector3d step = knots.col(i) - knots.col(i - 1);
    const double seg_dt = times(i) - times(i - 1);
    ASSERT_GT(seg_dt, 0.0);
    EXPECT_LE(step.head(2).norm() / seg_dt, kVxyMax * (1 + 1e-9));
    EXPECT_LE(std::abs(step(2)) / seg_dt, kVzMax * (1 + 1e-9));
  }
}

// A plan already within the limits is left exactly alone by the retiming, and
// resampling it reproduces the original knots.
TEST(EEPlanRetimingTest, FeasiblePlanIsUnchanged) {
  const int n = 8;
  // Half the horizontal limit and half the vertical limit per segment.
  const Vector3d step(0.5 * kVxyMax * kDt, 0.0, 0.5 * kVzMax * kDt);
  const MatrixXd knots = MakeStraightPlan(n, step);

  VectorXd times = MakeUniformTimes(n, kDt);
  RetimeEEPlanToVelocityLimits(knots, kVxyMax, kVzMax, &times);
  EXPECT_EQ(times, MakeUniformTimes(n, kDt));

  const RetimedPlanSampling sampling =
      ResampleEEPlanOnUniformGrid(knots, times, kDt);

  // The resampling is exactly the identity: sample i is knot i, at the plan's
  // own pace.  This is what lets a cost computed from the resampled plan match
  // one computed from the raw plan whenever no limit binds.
  EXPECT_EQ(sampling.positions, knots);
  for (int i = 0; i < n; i++) {
    EXPECT_EQ(sampling.segment[i], std::min(i, n - 2));
    // Every sample but the last lands on the start of its segment; the last
    // saturates at the end of the final one, which is the same knot.
    EXPECT_EQ(sampling.fraction(i), i < n - 1 ? 0.0 : 1.0);
    EXPECT_EQ(sampling.time_scale(i), 1.0);
  }
}

// A plan asking for 5x the vertical limit is stretched 5x, and resampling it
// walks the same path at a fifth of the spacing -- covering a fifth of the
// original path over the same number of knots.
TEST(EEPlanRetimingTest, OverspeedVerticalPlanIsSlowedAndTruncated) {
  const int n = 6;
  const double stretch = 5.0;
  const Vector3d step(0.0, 0.0, stretch * kVzMax * kDt);
  const MatrixXd knots = MakeStraightPlan(n, step);

  VectorXd times = MakeUniformTimes(n, kDt);
  RetimeEEPlanToVelocityLimits(knots, kVxyMax, kVzMax, &times);
  EXPECT_TRUE(times.isApprox(MakeUniformTimes(n, stretch * kDt)));
  ExpectWithinVelocityLimits(knots, times);

  const RetimedPlanSampling sampling =
      ResampleEEPlanOnUniformGrid(knots, times, kDt);

  // Sample i sits a fifth of a segment further along than sample i-1.
  for (int i = 0; i < n; i++) {
    EXPECT_NEAR((sampling.positions.col(i) - (i / stretch) * step).norm(), 0.0,
                kTol);
    // Five samples fall inside each stretched segment, and the sixth lands
    // exactly on the boundary, which counts as the start of the next one.
    EXPECT_EQ(sampling.segment[i], i / static_cast<int>(stretch));
    // Any velocity carried along this plan is slowed by the same factor the
    // segment was stretched by.
    EXPECT_NEAR(sampling.time_scale(i), 1.0 / stretch, kTol);
  }
  // The resampled plan covers a fifth of the original path.
  EXPECT_NEAR(sampling.positions.col(n - 1).norm(),
              knots.col(n - 1).norm() / stretch, kTol);

  // The resampled plan is itself feasible on the original uniform grid.
  ExpectWithinVelocityLimits(sampling.positions, MakeUniformTimes(n, kDt));
}

// Each segment is stretched by whichever axis binds hardest, independently.
TEST(EEPlanRetimingTest, BindingAxisSetsEachSegmentDuration) {
  MatrixXd knots(3, 4);
  knots.col(0) = Vector3d::Zero();
  // Segment 0: within both limits.
  knots.col(1) = knots.col(0) + Vector3d(0.5 * kVxyMax * kDt, 0.0, 0.0);
  // Segment 1: 3x the horizontal limit, vertically idle.
  knots.col(2) = knots.col(1) + Vector3d(3.0 * kVxyMax * kDt, 0.0, 0.0);
  // Segment 2: within the horizontal limit but 2x the vertical one.
  knots.col(3) =
      knots.col(2) + Vector3d(0.5 * kVxyMax * kDt, 0.0, 2.0 * kVzMax * kDt);

  VectorXd times = MakeUniformTimes(4, kDt);
  RetimeEEPlanToVelocityLimits(knots, kVxyMax, kVzMax, &times);

  EXPECT_NEAR(times(1) - times(0), kDt, kTol);
  EXPECT_NEAR(times(2) - times(1), 3.0 * kDt, kTol);
  EXPECT_NEAR(times(3) - times(2), 2.0 * kDt, kTol);
  ExpectWithinVelocityLimits(knots, times);
}

// Retiming only ever slows a plan down, whatever the path.
TEST(EEPlanRetimingTest, RetimingNeverSpeedsUp) {
  MatrixXd knots(3, 5);
  knots.col(0) = Vector3d(0.1, -0.2, 0.05);
  knots.col(1) = knots.col(0);  // A zero-length segment.
  knots.col(2) = knots.col(1) + Vector3d(-0.03, 0.001, 0.0);
  knots.col(3) = knots.col(2) + Vector3d(0.0, 0.0, -0.004);
  knots.col(4) = knots.col(3) + Vector3d(0.02, 0.02, 0.002);

  VectorXd times = MakeUniformTimes(5, kDt);
  RetimeEEPlanToVelocityLimits(knots, kVxyMax, kVzMax, &times);

  const VectorXd nominal = MakeUniformTimes(5, kDt);
  for (int i = 0; i < 5; i++) {
    EXPECT_GE(times(i), nominal(i) - kTol);
  }
  // A zero-length segment keeps the nominal dt rather than collapsing.
  EXPECT_NEAR(times(1) - times(0), kDt, kTol);
  ExpectWithinVelocityLimits(knots, times);
}

// A segment's floor is the nominal time still left for it, not a full dt, so a
// plan that overran earlier is allowed to catch back up (at the limits) rather
// than pushing the whole rest of the plan out.  Every knot still lands no
// earlier than it nominally would, which is what keeps resampling from having
// to extrapolate.
TEST(EEPlanRetimingTest, LaterSegmentsCatchUpButNeverArriveEarly) {
  MatrixXd knots(3, 4);
  knots.col(0) = Vector3d::Zero();
  // Segment 0: 4x the vertical limit, so it takes 4 nominal steps.
  knots.col(1) = knots.col(0) + Vector3d(0.0, 0.0, 4.0 * kVzMax * kDt);
  // Segments 1 and 2: nominally due long before segment 0 ends, so they run at
  // whatever the limits allow instead of at a full dt each.
  knots.col(2) = knots.col(1) + Vector3d(kVxyMax * kDt / 3.0, 0.0, 0.0);
  knots.col(3) = knots.col(2) + Vector3d(0.0, 0.0, kVzMax * kDt / 2.0);

  const VectorXd nominal = MakeUniformTimes(4, kDt);
  VectorXd times = nominal;
  RetimeEEPlanToVelocityLimits(knots, kVxyMax, kVzMax, &times);

  EXPECT_NEAR(times(1) - times(0), 4.0 * kDt, kTol);
  EXPECT_NEAR(times(2) - times(1), kDt / 3.0, kTol);
  EXPECT_NEAR(times(3) - times(2), kDt / 2.0, kTol);
  for (int i = 0; i < 4; i++) {
    EXPECT_GE(times(i), nominal(i) - kTol);
  }
  ExpectWithinVelocityLimits(knots, times);

  // So no sample extrapolates, and the ones inside the compressed segments are
  // left at the plan's own pace rather than sped up.
  const RetimedPlanSampling sampling =
      ResampleEEPlanOnUniformGrid(knots, times, kDt);
  for (int i = 0; i < 4; i++) {
    EXPECT_LE(sampling.fraction(i), 1.0);
    EXPECT_LE(sampling.time_scale(i), 1.0);
  }
  EXPECT_EQ(sampling.segment[3], 0);
  EXPECT_NEAR(sampling.time_scale(3), 0.25, kTol);
}

// Resampling a mixed plan stays on the path, reports the segment each sample
// came from, and paces anything carried along it by the segment's stretch.
TEST(EEPlanRetimingTest, ResampleTracksPathAndSourceSegments) {
  MatrixXd knots(3, 5);
  knots.col(0) = Vector3d::Zero();
  knots.col(1) = knots.col(0) + Vector3d(0.0, 0.0, 2.0 * kVzMax * kDt);
  knots.col(2) = knots.col(1) + Vector3d(0.5 * kVxyMax * kDt, 0.0, 0.0);
  knots.col(3) = knots.col(2) + Vector3d(0.0, 4.0 * kVxyMax * kDt, 0.0);
  knots.col(4) = knots.col(3) + Vector3d(0.0, 0.0, kVzMax * kDt);

  VectorXd times = MakeUniformTimes(5, kDt);
  RetimeEEPlanToVelocityLimits(knots, kVxyMax, kVzMax, &times);

  const RetimedPlanSampling sampling =
      ResampleEEPlanOnUniformGrid(knots, times, kDt);

  // Sample 0 always reproduces knot 0 exactly, since retiming pins it.
  EXPECT_EQ(sampling.positions.col(0), knots.col(0));

  int previous_segment = 0;
  for (int i = 0; i < 5; i++) {
    const int k = sampling.segment[i];
    EXPECT_GE(k, previous_segment);
    EXPECT_LE(k, 3);
    previous_segment = k;

    // The sample lies on segment k, at the fraction its time implies.
    const double frac = (i * kDt - times(k)) / (times(k + 1) - times(k));
    EXPECT_NEAR(sampling.fraction(i), frac, kTol);
    EXPECT_GE(sampling.fraction(i), 0.0);
    EXPECT_LE(sampling.fraction(i), 1.0);
    const Vector3d expected =
        knots.col(k) + frac * (knots.col(k + 1) - knots.col(k));
    EXPECT_NEAR((sampling.positions.col(i) - expected).norm(), 0.0, kTol);

    // A segment that had to be stretched slows anything carried along it by
    // exactly its stretch factor; one that did not is left at the plan's own
    // pace, never sped up.
    const double seg_dt = times(k + 1) - times(k);
    EXPECT_NEAR(sampling.time_scale(i), std::min(kDt / seg_dt, 1.0), kTol);
    EXPECT_GT(sampling.time_scale(i), 0.0);
    EXPECT_LE(sampling.time_scale(i), 1.0);
  }

  ExpectWithinVelocityLimits(sampling.positions, MakeUniformTimes(5, kDt));
}

}  // namespace
}  // namespace dairlib
