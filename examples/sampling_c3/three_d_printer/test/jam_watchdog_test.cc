// Unit tests for the live jam watchdog added after the "Cone Wedge at the Ramp
// Lip" hardware post-mortem (hwlog-000003 / hwlog-000005): the arming, dwell
// and hysteresis rules in JamLatch, the retreat splice in
// RepositionWithRetreat, and the shipped thresholds themselves.
//
// The controller's two queries -- C3's knot-0 EE<->object contact force and the
// EE-to-object signed distance -- are not exercised here; those need a whole
// plant.  What is exercised is every decision made on top of them, which is
// where the subtlety lives.

#include <cmath>
#include <limits>

#include <gtest/gtest.h>
#include <optional>

#include "examples/sampling_c3/jamming_metrics.h"
#include "examples/sampling_c3/parameter_headers/progress_params.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_controller_params.h"
#include "examples/sampling_c3/reposition.h"

#include "drake/common/yaml/yaml_io.h"

namespace dairlib {
namespace systems {
namespace {

constexpr char kConeControllerParams[] =
    "examples/sampling_c3/three_d_printer/cone/parameters/"
    "sampling_c3_controller_params.yaml";

// The detector's behaviour rules, with thresholds kept here rather than read
// from yaml so these stay fixed if the yaml is retuned; the yaml itself is
// pinned separately below.
JamLatchThresholds MakeThresholds() {
  return JamLatchThresholds{
      .force_trip = 6.0,
      .force_release = 4.0,
      .force_gate_gap = 0.002,
      .gap_trip = -0.010,
      .gap_release = -0.004,
      // The travel term off, so every rule below is exercised on its own.  Its
      // own tests use MakeThresholdsWithTravel().
      .object_travel_trip = std::numeric_limits<double>::infinity(),
      .object_travel_release = std::numeric_limits<double>::infinity(),
      .trip_hold_seconds = 0.25,
      .release_hold_seconds = 0.25};
}

// The same rules with the object-travel term on.  A jam is contact with no
// object progress; penetration on its own cannot say which, because the object
// pose estimate reaches deeper apparent penetration than the real jams do.
JamLatchThresholds MakeThresholdsWithTravel() {
  JamLatchThresholds thresholds = MakeThresholds();
  thresholds.object_travel_trip = 0.004;
  thresholds.object_travel_release = 0.008;
  return thresholds;
}

// Deliberately not a whole number of kLoop:  a dwell that lands exactly on a
// loop boundary makes every test a coin flip on whether the accumulated clock
// rounds just under it.  0.25 s is crossed on the 4th loop with room to spare.
constexpr double kLoop = 0.1;        // a plausible control period [s].
constexpr double kContact = -0.001;  // touching, but nowhere near gap_trip.
constexpr double kClear = 0.020;     // well outside the force gate.

// Walks the latch forward at kLoop, returning the rising edge of the last call.
bool Step(JamLatch* latch, double* now, int loops, double force,
          std::optional<double> gap,
          std::optional<double> travel = std::nullopt) {
  bool edge = false;
  for (int i = 0; i < loops; i++) {
    edge = latch->Update(*now, force, gap, travel);
    *now += kLoop;
  }
  return edge;
}

// The dwell is a duration: three loops at 0.1 s is 0.2 s of elapsed arming,
// short of the 0.25 s threshold, and the fourth is the rising edge.
TEST(JamLatchTest, ArmingMustHoldForTheFullDwell) {
  JamLatch latch(MakeThresholds());
  double now = 0.0;
  for (int i = 0; i < 3; i++) {
    EXPECT_FALSE(latch.Update(now, 8.0, kContact, std::nullopt))
        << "loop " << i;
    EXPECT_FALSE(latch.tripped()) << "loop " << i;
    now += kLoop;
  }
  EXPECT_TRUE(latch.Update(now, 8.0, kContact, std::nullopt));
  EXPECT_TRUE(latch.tripped());
}

// A slower loop reaches the same dwell in fewer updates, which is the point of
// making it a duration:  measured loop spacing over hwlog-000003/000005 runs
// 0.039-0.179 s, so a fixed loop count meant anywhere from 0.15 s to 0.71 s.
TEST(JamLatchTest, TheDwellIsATimeNotALoopCount) {
  JamLatch latch(MakeThresholds());
  EXPECT_FALSE(latch.Update(0.0, 8.0, kContact, std::nullopt));
  EXPECT_TRUE(latch.Update(0.3, 8.0, kContact, std::nullopt));
  EXPECT_TRUE(latch.tripped());
}

// A gap in the dwell resets it outright rather than accumulating.
TEST(JamLatchTest, TheDwellResetsOnAnyQuietLoop) {
  JamLatch latch(MakeThresholds());
  double now = 0.0;
  Step(&latch, &now, 3, 8.0, kContact);
  Step(&latch, &now, 1, 1.0, kContact);  // quiet
  EXPECT_EQ(latch.trip_seconds(), 0.0);
  EXPECT_FALSE(Step(&latch, &now, 3, 8.0, kContact));
  EXPECT_FALSE(latch.tripped());
  EXPECT_TRUE(latch.Update(now, 8.0, kContact, std::nullopt));
}

// The regression this gate exists for.  C3's knot-0 lambda is an ADMM iterate
// whose complementarity is only relaxed, so it does not vanish at a positive
// gap:  replayed over hwlog-000003 it reads 8-16 N with the end effector
// 15-33 mm clear of the cone, and 18 N at 82 mm on hwlog-000005.  Ungated,
// that latched the guard three times on no contact at all.
TEST(JamLatchTest, AHugeForceAwayFromTheObjectCannotArmTheLatch) {
  JamLatch latch(MakeThresholds());
  double now = 0.0;
  for (int i = 0; i < 40; i++) {
    EXPECT_FALSE(latch.Update(now, 16.0, kClear, std::nullopt)) << "loop " << i;
    now += kLoop;
  }
  EXPECT_FALSE(latch.tripped());

  // The same force does arm once the end effector is actually in contact.
  EXPECT_TRUE(Step(&latch, &now, 4, 16.0, kContact));
}

// The rising edge fires exactly once; the controller keys the unsuccessful
// buffer write off it, and a second write would fill the buffer with near
// duplicates of one spot.
TEST(JamLatchTest, TheRisingEdgeFiresOnce) {
  JamLatch latch(MakeThresholds());
  double now = 0.0;
  ASSERT_TRUE(Step(&latch, &now, 4, 8.0, kContact));
  for (int i = 0; i < 10; i++) {
    EXPECT_FALSE(latch.Update(now, 8.0, kContact, std::nullopt))
        << "loop " << i;
    EXPECT_TRUE(latch.tripped()) << "loop " << i;
    now += kLoop;
  }
}

// Hysteresis: while still in contact, dropping back under the trip threshold is
// not enough to release.  5 N is below force_trip but above force_release, and
// hwlog-000005 shows C3's lambda dipping through exactly that band mid-jam.
TEST(JamLatchTest, ReleaseNeedsTheLowerThresholdNotJustTheTripOne) {
  JamLatch latch(MakeThresholds());
  double now = 0.0;
  ASSERT_TRUE(Step(&latch, &now, 4, 8.0, kContact));

  for (int i = 0; i < 10; i++) {
    latch.Update(now, 5.0, kContact, std::nullopt);
    EXPECT_TRUE(latch.tripped()) << "loop " << i;
    now += kLoop;
  }
  Step(&latch, &now, 4, 3.0, kContact);
  EXPECT_FALSE(latch.tripped());
}

// Releasing has its own dwell, for the same reason arming does.  Without it the
// guard disarms itself:  one loop of the retreat it commands is enough to
// satisfy gap_release, and the first hwlog-000003 trip released after 0.30 s
// with the escape barely started.
TEST(JamLatchTest, ReleaseMustAlsoHold) {
  JamLatch latch(MakeThresholds());
  double now = 0.0;
  ASSERT_TRUE(Step(&latch, &now, 4, 8.0, kContact));

  // One clear loop is not a release.
  latch.Update(now, 1.0, kClear, std::nullopt);
  now += kLoop;
  EXPECT_TRUE(latch.tripped());
  // Nor is falling back into contact partway through one.
  latch.Update(now, 8.0, -0.020, std::nullopt);
  now += kLoop;
  EXPECT_TRUE(latch.tripped());
  // Held for the full release dwell, it clears.
  Step(&latch, &now, 4, 1.0, kClear);
  EXPECT_FALSE(latch.tripped());
}

// The release side of the same gate, and the reason the latch needs it: once
// the end effector is demonstrably clear, lambda must not get a vote.  Replayed
// on hwlog-000005 the retreat reached 142 mm clear while lambda sat at ~5 N,
// which held the guard set for the whole window -- C3 plans to re-approach, so
// its knot-0 lambda never falls to zero just because contact ended.
TEST(JamLatchTest, AForceReadingAwayFromTheObjectCannotHoldAJamOpen) {
  JamLatch latch(MakeThresholds());
  double now = 0.0;
  ASSERT_TRUE(Step(&latch, &now, 4, 8.0, kContact));

  // 5 N is above force_release, but 142 mm clear is far outside the gate.
  Step(&latch, &now, 4, 5.0, 0.142);
  EXPECT_FALSE(latch.tripped());
}

// The penetration guard alone must arm, with no help from the force term.
// This is episode B of hwlog-000005, whose forces never exceeded 7.2 N but
// whose apparent penetration reached -22.1 mm.
TEST(JamLatchTest, ThePenetrationGuardArmsOnItsOwn) {
  JamLatch latch(MakeThresholds());
  double now = 0.0;
  EXPECT_FALSE(Step(&latch, &now, 3, 0.5, -0.022));
  EXPECT_TRUE(latch.Update(now, 0.5, -0.022, std::nullopt));
  now += kLoop;
  EXPECT_TRUE(latch.tripped());

  // And releases only once the gap is back above gap_release, not merely back
  // above gap_trip -- and then only after the release dwell.
  Step(&latch, &now, 10, 0.5, -0.008);
  EXPECT_TRUE(latch.tripped());
  Step(&latch, &now, 4, 0.5, -0.002);
  EXPECT_FALSE(latch.tripped());
}

// An untrustworthy signed-distance reading (the controller passes nullopt) must
// not manufacture a jam.  With the force term gated on the gap, no reading
// means nothing can arm at all -- the fail-safe direction.
TEST(JamLatchTest, AMissingGapReadingCannotArmTheLatch) {
  JamLatch latch(MakeThresholds());
  double now = 0.0;
  for (int i = 0; i < 20; i++) {
    EXPECT_FALSE(latch.Update(now, 16.0, std::nullopt, std::nullopt))
        << "loop " << i;
    now += kLoop;
  }
  EXPECT_FALSE(latch.tripped());
}

// The reason the travel term exists.  Penetration deep enough to trip is
// something the object pose estimate reaches on its own -- the 2026-09-17
// hardware logs show -22 mm from estimation error alone -- so a deep gap while
// the object is visibly moving is a push being made, not a jam.  Before this
// term, nine of the 26 hardware trips fired exactly here and the retreat
// aborted a push that had already broken free.
TEST(JamLatchTest, AMovingObjectBlocksTheGapGuard) {
  JamLatch latch(MakeThresholdsWithTravel());
  double now = 0.0;
  // 12 mm of travel per window, against a 4 mm trip.
  EXPECT_FALSE(Step(&latch, &now, 20, 0.5, -0.022, 0.012));
  EXPECT_FALSE(latch.tripped());
}

// The same penetration with the object held still is the jam, and still arms.
TEST(JamLatchTest, TheGapGuardStillArmsWhenTheObjectIsStalled) {
  JamLatch latch(MakeThresholdsWithTravel());
  double now = 0.0;
  EXPECT_FALSE(Step(&latch, &now, 3, 0.5, -0.022, 0.001));
  EXPECT_TRUE(latch.Update(now, 0.5, -0.022, 0.001));
  EXPECT_TRUE(latch.tripped());
}

// The object moving again ends the jam even while the gap still reads as deep
// penetration -- which it will, since the retreat has barely started and the
// pose estimate is what made it look deep in the first place.  This is an
// alternative to the gap clearing, not a second condition on top of it.
TEST(JamLatchTest, TheObjectMovingAgainReleasesTheLatch) {
  JamLatch latch(MakeThresholdsWithTravel());
  double now = 0.0;
  ASSERT_TRUE(Step(&latch, &now, 4, 0.5, -0.022, 0.001));

  // Still wedged: neither the gap nor the travel says otherwise.
  Step(&latch, &now, 10, 0.5, -0.022, 0.001);
  EXPECT_TRUE(latch.tripped());
  // Moving again, with the gap unchanged.
  Step(&latch, &now, 4, 0.5, -0.022, 0.012);
  EXPECT_FALSE(latch.tripped());
}

// A partial travel window under-reports how far the object has gone, which
// would arm the latch on a stillness nobody has observed yet.  The controller
// passes nullopt until the history spans the window; that must fail safe the
// same way a missing gap does.
TEST(JamLatchTest, AMissingTravelReadingCannotArmTheGapGuard) {
  JamLatch latch(MakeThresholdsWithTravel());
  double now = 0.0;
  for (int i = 0; i < 20; i++) {
    EXPECT_FALSE(latch.Update(now, 0.5, -0.022, std::nullopt)) << "loop " << i;
    now += kLoop;
  }
  EXPECT_FALSE(latch.tripped());
}

// ...but a missing reading must not hold a latch open either, so on the
// release side it simply does not vote and the gap decides alone.
TEST(JamLatchTest, AMissingTravelReadingDoesNotHoldTheLatchOpen) {
  JamLatch latch(MakeThresholdsWithTravel());
  double now = 0.0;
  ASSERT_TRUE(Step(&latch, &now, 4, 0.5, -0.022, 0.001));
  Step(&latch, &now, 4, 0.5, -0.002, std::nullopt);
  EXPECT_FALSE(latch.tripped());
}

// The force term is dormant but still wired, and it is deliberately NOT gated
// on travel:  it is the backstop for the case where the travel history is
// unavailable entirely.
TEST(JamLatchTest, TheForceGuardIsNotGatedOnTravel) {
  JamLatch latch(MakeThresholdsWithTravel());
  double now = 0.0;
  // 12 mm of travel would block the gap guard; the force guard arms anyway.
  EXPECT_TRUE(Step(&latch, &now, 4, 8.0, kContact, 0.012));
  EXPECT_TRUE(latch.tripped());
}

// Leaving object_travel_trip at its infinite default must reproduce the
// gap-only latch exactly, including for a caller that passes no travel at all.
TEST(JamLatchTest, AnUnsetTravelThresholdIsANoOp) {
  JamLatch latch(MakeThresholds());
  double now = 0.0;
  EXPECT_FALSE(Step(&latch, &now, 3, 0.5, -0.022, std::nullopt));
  EXPECT_TRUE(latch.Update(now, 0.5, -0.022, std::nullopt));
  EXPECT_TRUE(latch.tripped());
}

// Nor hold an existing jam open: with no gap reading, the force term decides
// the release alone.
TEST(JamLatchTest, AMissingGapReadingDoesNotHoldAJamOpen) {
  JamLatch latch(MakeThresholds());
  double now = 0.0;
  ASSERT_TRUE(Step(&latch, &now, 4, 8.0, -0.020));
  Step(&latch, &now, 4, 3.0, std::nullopt);
  EXPECT_FALSE(latch.tripped());
}

// ---------------------------------------------------------------------------
// The retreat splice.

constexpr int kNq = 10;  // 3 EE + 7 object (quat + xyz)
constexpr int kNx = 19;  // + 3 EE vel + 6 object vel
constexpr int kN = 10;
constexpr double kDt = 0.075;

Eigen::VectorXd Row5(double a, double b, double c, double d, double e) {
  Eigen::VectorXd v(5);
  v << a, b, c, d, e;
  return v;
}

SamplingC3RepositionParams MakeRepositionParams() {
  SamplingC3RepositionParams p{};
  p.traj_type = RepositioningTrajectoryType::kPiecewiseLinear;
  p.speed_horizontal = 0.12;
  p.speed_vertical = 0.015;
  p.use_straight_line_traj_under_spline = 0.12;
  p.use_straight_line_traj_within_angle = 0.3;
  p.use_straight_line_traj_under_piecewise_linear = 0.008;
  p.spline_width = 0.17;
  p.sphere_radius = 0.18;
  p.circle_radius = 0.20;
  p.circle_height = 0.0;
  p.pwl_waypoint_height = 0.15;
  p.pwl_adaptive_waypoint_height = false;
  p.pwl_clearance_margin = 0.01;
  p.pwl_height_search_step = 0.01;
  p.pwl_num_path_collision_samples = 12;
  p.max_tilt_angle = 20;
  return p;
}

SamplingC3Options MakeOptions() {
  SamplingC3Options o{};
  o.workspace_limits = {Row5(1, 0, 0, 0.0, 0.35), Row5(0, 1, 0, 0.0, 0.35),
                        Row5(0, 0, 1, 0.006, 0.248)};
  o.workspace_margins = 0.002;
  return o;
}

Eigen::VectorXd MakeLcsState(const Eigen::Vector3d& ee) {
  Eigen::VectorXd x = Eigen::VectorXd::Zero(kNx);
  x.head(3) = ee;
  x.segment(kNq - 3, 3) = Eigen::Vector3d(0.2, 0.2, 0.02);  // object position
  return x;
}

// Knot 0 must stay exactly where the end effector already is, or the published
// plan is discontinuous with what the OSC is tracking.  The next knots are one
// knot period of travel apart along the escape direction -- purely horizontal
// here, so speed_horizontal * kDt = 0.12 * 0.075 = 9 mm each.
TEST(RepositionWithRetreatTest, TheRetreatLeavesKnotZeroWhereTheEEIs) {
  const Eigen::Vector3d ee(0.10, 0.10, 0.02);
  const Eigen::Vector3d target(0.30, 0.30, 0.02);
  const Eigen::Vector3d escape(1.0, 0.0, 0.0);  // +x, out of the object
  constexpr double kStep = 0.009;

  const Eigen::MatrixXd knots = RepositionWithRetreat(
      kNq, kNx, kN, MakeLcsState(ee), target, kDt, /*is_doing_c3=*/false,
      escape, /*num_retreat_knots=*/3, MakeRepositionParams(), MakeOptions());

  ASSERT_EQ(knots.rows(), kNx);
  ASSERT_EQ(knots.cols(), kN);
  EXPECT_NEAR((knots.col(0).head(3) - ee).norm(), 0.0, 1e-12);
  EXPECT_NEAR((knots.col(1).head(3) - (ee + kStep * escape)).norm(), 0.0,
              1e-12);
  EXPECT_NEAR((knots.col(2).head(3) - (ee + 2 * kStep * escape)).norm(), 0.0,
              1e-12);
  // Knot 3 starts the repositioning leg, from the end of the retreat.
  EXPECT_NEAR((knots.col(3).head(3) - (ee + 3 * kStep * escape)).norm(), 0.0,
              1e-12);
}

// The retreat is a commanded motion like any other, so it must be one the
// machine can actually execute.  A straight-up escape is capped by the
// printer's much slower vertical axis, and a diagonal by whichever axis
// saturates first -- not by the horizontal limit applied to the 3D distance.
TEST(RepositionWithRetreatTest, TheRetreatRespectsBothPrinterSpeedLimits) {
  const SamplingC3RepositionParams params = MakeRepositionParams();
  const Eigen::Vector3d ee(0.10, 0.10, 0.05);

  // Purely vertical: the 0.015 m/s axis binds.
  EXPECT_NEAR(MaxSpeedAlongDirection(Eigen::Vector3d(0, 0, 1), params), 0.015,
              1e-12);
  // Purely horizontal, and unnormalized, to show the direction's length is not
  // the speed: the 0.12 m/s axis binds.
  EXPECT_NEAR(MaxSpeedAlongDirection(Eigen::Vector3d(0, 7.0, 0), params), 0.12,
              1e-12);
  // 45 degrees out of the xy plane.  Both components are 1/sqrt(2) of the
  // speed, so the vertical cap binds first: 0.015 * sqrt(2).
  EXPECT_NEAR(MaxSpeedAlongDirection(Eigen::Vector3d(1, 0, 1), params),
              0.015 * std::sqrt(2.0), 1e-12);
  // Nothing to move along.
  EXPECT_EQ(MaxSpeedAlongDirection(Eigen::Vector3d::Zero(), params), 0.0);

  // And the knots the retreat actually plans obey both limits per period.
  for (const Eigen::Vector3d& escape :
       {Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(1, 0, 1),
        Eigen::Vector3d(0.3, -0.4, 0.2)}) {
    const Eigen::MatrixXd knots = RepositionWithRetreat(
        kNq, kNx, kN, MakeLcsState(ee), Eigen::Vector3d(0.30, 0.30, 0.05), kDt,
        /*is_doing_c3=*/false, escape, /*num_retreat_knots=*/3, params,
        MakeOptions());
    const Eigen::Vector3d step = knots.col(1).head(3) - knots.col(0).head(3);
    EXPECT_LE(step.head(2).norm(), params.speed_horizontal * kDt + 1e-12)
        << "escape " << escape.transpose();
    EXPECT_LE(std::abs(step(2)), params.speed_vertical * kDt + 1e-12)
        << "escape " << escape.transpose();
    // Saturated, not merely legal: one of the two limits is met exactly.
    EXPECT_TRUE(
        std::abs(step.head(2).norm() - params.speed_horizontal * kDt) < 1e-12 ||
        std::abs(std::abs(step(2)) - params.speed_vertical * kDt) < 1e-12)
        << "escape " << escape.transpose();
  }
}

// Past the retreat, the plan is exactly what the configured strategy would have
// produced from the retreat endpoint -- the retreat biases where repositioning
// starts, it does not replace it.
TEST(RepositionWithRetreatTest, TheTailIsPlainRepositioningFromTheEndpoint) {
  const Eigen::Vector3d ee(0.10, 0.10, 0.02);
  const Eigen::Vector3d target(0.30, 0.30, 0.02);
  const Eigen::Vector3d escape(1.0, 0.0, 0.0);
  constexpr int kRetreatKnots = 2;
  constexpr double kStep = 0.009;  // speed_horizontal * kDt, escape is +x

  const Eigen::MatrixXd spliced = RepositionWithRetreat(
      kNq, kNx, kN, MakeLcsState(ee), target, kDt, /*is_doing_c3=*/false,
      escape, kRetreatKnots, MakeRepositionParams(), MakeOptions());

  bool unused_finished_flag = false;
  const Eigen::MatrixXd plain =
      Reposition(kNq, kNx, kN - kRetreatKnots,
                 MakeLcsState(ee + kRetreatKnots * kStep * escape), target, kDt,
                 /*is_doing_c3=*/false, unused_finished_flag,
                 MakeRepositionParams(), MakeOptions());

  EXPECT_TRUE(spliced.rightCols(kN - kRetreatKnots).isApprox(plain, 1e-12));
}

// The object rows are frozen across the retreat, the same way Reposition()
// freezes them: the retreat moves the end effector, not the world.
TEST(RepositionWithRetreatTest, TheObjectPoseIsFrozenAcrossTheRetreat) {
  const Eigen::Vector3d ee(0.10, 0.10, 0.02);
  const Eigen::VectorXd x_lcs = MakeLcsState(ee);
  const Eigen::MatrixXd knots = RepositionWithRetreat(
      kNq, kNx, kN, x_lcs, Eigen::Vector3d(0.30, 0.30, 0.02), kDt,
      /*is_doing_c3=*/false, Eigen::Vector3d(1, 0, 0),
      /*num_retreat_knots=*/3, MakeRepositionParams(), MakeOptions());

  for (int col = 0; col < 3; col++) {
    EXPECT_TRUE(knots.col(col)
                    .segment(3, kNq - 3)
                    .isApprox(x_lcs.segment(3, kNq - 3), 1e-12))
        << "knot " << col;
  }
}

// With no usable escape direction there is nothing to retreat along, so the
// plan is plain repositioning rather than an invented direction.
TEST(RepositionWithRetreatTest, AZeroEscapeDirectionFallsBackToRepositioning) {
  const Eigen::Vector3d ee(0.10, 0.10, 0.02);
  const Eigen::Vector3d target(0.30, 0.30, 0.02);

  const Eigen::MatrixXd retreated = RepositionWithRetreat(
      kNq, kNx, kN, MakeLcsState(ee), target, kDt, /*is_doing_c3=*/false,
      Eigen::Vector3d::Zero(), /*num_retreat_knots=*/3, MakeRepositionParams(),
      MakeOptions());

  bool unused_finished_flag = false;
  const Eigen::MatrixXd plain =
      Reposition(kNq, kNx, kN, MakeLcsState(ee), target, kDt,
                 /*is_doing_c3=*/false, unused_finished_flag,
                 MakeRepositionParams(), MakeOptions());

  EXPECT_TRUE(retreated.isApprox(plain, 1e-12));
}

// An over-long retreat is clamped to N - 1 knots so the repositioning leg
// always has somewhere to exist.  With exactly one knot left that leg is just
// its own start point, which is the degenerate but well-formed end of the
// clamp -- the plan is still N knots and still starts where the EE is.
TEST(RepositionWithRetreatTest, TheRetreatCannotConsumeTheWholeHorizon) {
  const Eigen::Vector3d ee(0.10, 0.10, 0.02);
  const Eigen::Vector3d escape(1.0, 0.0, 0.0);
  constexpr double kStep = 0.009;  // speed_horizontal * kDt, escape is +x
  const Eigen::MatrixXd knots = RepositionWithRetreat(
      kNq, kNx, kN, MakeLcsState(ee), Eigen::Vector3d(0.30, 0.30, 0.02), kDt,
      /*is_doing_c3=*/false, escape,
      /*num_retreat_knots=*/kN + 5, MakeRepositionParams(), MakeOptions());

  ASSERT_EQ(knots.rows(), kNx);
  ASSERT_EQ(knots.cols(), kN);
  EXPECT_NEAR((knots.col(0).head(3) - ee).norm(), 0.0, 1e-12);
  // kN - 1 retreat knots, then the one-knot repositioning leg pinned to the
  // retreat endpoint.
  EXPECT_NEAR(
      (knots.col(kN - 2).head(3) - (ee + (kN - 2) * kStep * escape)).norm(),
      0.0, 1e-12);
  EXPECT_NEAR(
      (knots.col(kN - 1).head(3) - (ee + (kN - 1) * kStep * escape)).norm(),
      0.0, 1e-12);
}

// ---------------------------------------------------------------------------
// The shipped configuration.

// Pins the cone demo's thresholds to the detector actually measured against
// hwlog-000003 and hwlog-000005 by replaying the controller at HEAD over them
// (three_d_printer/test/hybrid_jam_replay).  The post-mortem's own 6 N sits
// inside the productive-push band once u_{horizontal,vertical}_limits are
// +/-15, and fired 1-3 s early on both logs; 8 N fires 0.2 s into the press on
// both.  See the yaml's comment for the measured bands.  Loosening either trip
// value silently stops describing that.
TEST(JamGuardParamsTest, TheConeYamlShipsTheDetectorTheReportScored) {
  const SamplingC3ControllerParams controller_params =
      drake::yaml::LoadYamlFile<SamplingC3ControllerParams>(
          kConeControllerParams);
  ASSERT_TRUE(controller_params.progress_params.jam_guard.has_value());
  const JamGuardParams& jam_guard =
      controller_params.progress_params.jam_guard.value();

  // These pin what the cone demo ships so a retune is a deliberate edit rather
  // than a silent one.  They are NOT a validation that the values are right --
  // that comes from scoring the logged SAMPLING_C3_DEBUG signal against the
  // simulator's contact forces, as the yaml's own comment block records.
  //
  // gap_trip staying at -0.010 is the load-bearing one:  loosening it scored
  // worse on BOTH the 2026-09-22 sim logs and the 2026-09-17 hardware logs, and
  // this file is shared by the cone demo's sim and hardware launches.
  EXPECT_EQ(jam_guard.gap_trip, -0.010);
  EXPECT_EQ(jam_guard.object_travel_trip, 0.003);
  EXPECT_EQ(jam_guard.object_travel_release, 0.006);
  EXPECT_EQ(jam_guard.object_travel_window_seconds, 0.3);
  EXPECT_EQ(jam_guard.force_trip, 9.0);
  EXPECT_EQ(jam_guard.force_gate_gap, 0.002);
  EXPECT_EQ(jam_guard.force_release, 4.0);
  EXPECT_EQ(jam_guard.gap_release, 0.002);
  EXPECT_EQ(jam_guard.trip_hold_seconds, 0.3);
  EXPECT_EQ(jam_guard.release_hold_seconds, 0.5);

  // The invariants the controller DRAKE_DEMANDs, checked here so a bad yaml
  // fails the test rather than the demo.
  EXPECT_LT(jam_guard.force_release, jam_guard.force_trip);
  EXPECT_GT(jam_guard.gap_release, jam_guard.gap_trip);
  EXPECT_GT(jam_guard.object_travel_release, jam_guard.object_travel_trip);
  EXPECT_GT(jam_guard.object_travel_trip, 0.0);
  EXPECT_GT(jam_guard.object_travel_window_seconds, 0.0);
  EXPECT_GE(jam_guard.trip_hold_seconds, 0.0);
  EXPECT_GE(jam_guard.release_hold_seconds, 0.0);
  EXPECT_GT(jam_guard.retreat_knots, 0);
  // The force gate has to sit outside the gap trip, or the force term could
  // only ever arm where the gap term already had.
  EXPECT_GT(jam_guard.force_gate_gap, jam_guard.gap_trip);
}

// The gate: jam_guard is what turns the watchdog on, and every demo that omits
// it -- jacktoy, anything, push_t -- must keep loading.
TEST(JamGuardParamsTest, TheKeyIsOptional) {
  const auto jacktoy = drake::yaml::LoadYamlFile<SamplingC3ProgressParams>(
      "examples/sampling_c3/jacktoy/parameters/progress_params.yaml");
  EXPECT_FALSE(jacktoy.jam_guard.has_value());
}

}  // namespace
}  // namespace systems
}  // namespace dairlib
