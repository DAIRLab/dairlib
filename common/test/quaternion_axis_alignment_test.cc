#include "common/quaternion_axis_alignment.h"

#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

namespace dairlib {
namespace {

using Eigen::AngleAxisd;
using Eigen::Quaterniond;
using Eigen::Vector3d;

constexpr double kTol = 1e-9;

TEST(QuaternionAxisAlignmentTest, IdenticalOrientationsHaveZeroError) {
  Quaterniond curr_quat = Quaterniond::Identity();
  Quaterniond reference_quat = Quaterniond::Identity();
  Vector3d axis(1, 0, 0);

  EXPECT_NEAR(ComputeAxisMisalignmentAngle(curr_quat, reference_quat, axis), 0,
              kTol);

  Vector3d hysteresis_state = Vector3d::Zero();
  Quaterniond goal_quat = ComputeAxisAlignedGoalQuaternion(
      curr_quat, reference_quat, axis, 0.4, &hysteresis_state);
  EXPECT_TRUE(goal_quat.isApprox(curr_quat, kTol));
}

TEST(QuaternionAxisAlignmentTest, PureTwistDifferenceIsIgnored) {
  Vector3d axis(1, 0, 0);
  Quaterniond curr_quat = Quaterniond::Identity();
  // Twist the reference by 47 degrees about the tracked axis itself -- this
  // should be invisible to both functions.
  Quaterniond reference_quat(AngleAxisd(47 * M_PI / 180, axis));

  EXPECT_NEAR(ComputeAxisMisalignmentAngle(curr_quat, reference_quat, axis), 0,
              kTol);

  Vector3d hysteresis_state = Vector3d::Zero();
  Quaterniond goal_quat = ComputeAxisAlignedGoalQuaternion(
      curr_quat, reference_quat, axis, 0.4, &hysteresis_state);
  EXPECT_TRUE(goal_quat.isApprox(curr_quat, kTol));
}

TEST(QuaternionAxisAlignmentTest, NinetyDegreeMisalignment) {
  Vector3d axis_body(1, 0, 0);
  Quaterniond curr_quat = Quaterniond::Identity();
  // Reference orientation rotates the tracked axis 90 degrees, from world +x
  // to world +y.
  Quaterniond reference_quat(AngleAxisd(M_PI / 2, Vector3d(0, 0, 1)));

  EXPECT_NEAR(
      ComputeAxisMisalignmentAngle(curr_quat, reference_quat, axis_body),
      M_PI / 2, kTol);

  Vector3d hysteresis_state = Vector3d::Zero();
  Quaterniond goal_quat = ComputeAxisAlignedGoalQuaternion(
      curr_quat, reference_quat, axis_body, 0.4, &hysteresis_state);
  Vector3d aligned_axis_world = goal_quat * axis_body;
  Vector3d target_axis_world = reference_quat * axis_body;
  EXPECT_TRUE(aligned_axis_world.isApprox(target_axis_world, kTol));
}

TEST(QuaternionAxisAlignmentTest, NearAntipodalMisalignmentIsWellDefined) {
  Vector3d axis_body(1, 0, 0);
  Quaterniond curr_quat = Quaterniond::Identity();
  // Reference orientation rotates the tracked axis to nearly the opposite
  // direction (179 degrees).
  Quaterniond reference_quat(AngleAxisd(179 * M_PI / 180, Vector3d(0, 0, 1)));

  double angle =
      ComputeAxisMisalignmentAngle(curr_quat, reference_quat, axis_body);
  EXPECT_NEAR(angle, 179 * M_PI / 180, kTol);

  Vector3d hysteresis_state = Vector3d::Zero();
  Quaterniond goal_quat = ComputeAxisAlignedGoalQuaternion(
      curr_quat, reference_quat, axis_body, 0.4, &hysteresis_state);
  Vector3d aligned_axis_world = goal_quat * axis_body;
  Vector3d target_axis_world = reference_quat * axis_body;
  EXPECT_TRUE(aligned_axis_world.isApprox(target_axis_world, 1e-6));
}

TEST(QuaternionAxisAlignmentTest, ExactAntipodalDoesNotNanOrThrow) {
  Vector3d axis_body(1, 0, 0);
  Quaterniond curr_quat = Quaterniond::Identity();
  Quaterniond reference_quat(AngleAxisd(M_PI, Vector3d(0, 0, 1)));

  Vector3d hysteresis_state = Vector3d::Zero();
  Quaterniond goal_quat = ComputeAxisAlignedGoalQuaternion(
      curr_quat, reference_quat, axis_body, 0.4, &hysteresis_state);
  EXPECT_FALSE(std::isnan(goal_quat.norm()));
  Vector3d aligned_axis_world = goal_quat * axis_body;
  Vector3d target_axis_world = reference_quat * axis_body;
  EXPECT_TRUE(aligned_axis_world.isApprox(target_axis_world, 1e-6));
}

// Regression test for the hysteresis composition risk: as curr_quat sweeps
// through the antipodal region relative to a fixed reference, the chosen
// swing axis should not flip sign from one call to the next.
TEST(QuaternionAxisAlignmentTest,
     HysteresisAxisStaysContinuousNearSingularity) {
  Vector3d axis_body(1, 0, 0);
  Quaterniond reference_quat = Quaterniond::Identity();
  double angle_hysteresis = 0.4;
  Vector3d hysteresis_state = Vector3d::Zero();

  Vector3d previous_axis = Vector3d::Zero();
  bool first = true;
  for (double deg = 170; deg <= 190; deg += 0.5) {
    Quaterniond curr_quat(AngleAxisd(deg * M_PI / 180, Vector3d(0, 1, 0)));
    ComputeAxisAlignedGoalQuaternion(curr_quat, reference_quat, axis_body,
                                     angle_hysteresis, &hysteresis_state);
    if (!first) {
      EXPECT_GT(hysteresis_state.dot(previous_axis), 0)
          << "Hysteresis axis flipped sign at " << deg << " degrees";
    }
    previous_axis = hysteresis_state;
    first = false;
  }
}

// The reported swing (swing_angle, swing_axis) must rebuild the returned goal
// quaternion: goal_quat == AngleAxisd(swing_angle, swing_axis) * curr, up to the
// quaternion double cover. Covers a spread of misalignments plus the reflex case
// where the hysteresis deliberately returns swing_angle > pi.
TEST(QuaternionAxisAlignmentTest, SwingOutputsReconstructGoalQuaternion) {
  Vector3d axis_body(1, 0, 0);
  // An arbitrary current orientation that also carries twist about the tracked
  // axis, to make sure the swing reconstruction isn't accidentally relying on
  // curr_quat being identity.
  Quaterniond curr_quat(AngleAxisd(0.3, Vector3d(0, 0, 1)) *
                        AngleAxisd(0.7, axis_body));

  for (double deg : {30.0, 90.0, 150.0, 179.0}) {
    Quaterniond reference_quat(AngleAxisd(deg * M_PI / 180, Vector3d(0, 1, 0)));
    Vector3d hysteresis_state = Vector3d::Zero();
    double swing_angle = 0;
    Vector3d swing_axis = Vector3d::Zero();
    Quaterniond goal_quat = ComputeAxisAlignedGoalQuaternion(
        curr_quat, reference_quat, axis_body, 0.4, &hysteresis_state,
        &swing_angle, &swing_axis);

    Quaterniond reconstructed =
        Quaterniond(AngleAxisd(swing_angle, swing_axis)) * curr_quat.normalized();
    EXPECT_TRUE(reconstructed.isApprox(goal_quat, kTol) ||
                reconstructed.coeffs().isApprox(-goal_quat.coeffs(), kTol))
        << "deg=" << deg;
  }

  // Reflex case: prime the hysteresis state opposite the natural swing so the
  // near-antipodal call takes the long way, yielding swing_angle > pi.
  {
    Quaterniond reference_quat(AngleAxisd(M_PI / 2, Vector3d(0, 1, 0)));
    Vector3d hysteresis_state(0, 1, 0);
    Quaterniond near_antipodal_curr(AngleAxisd(-M_PI / 2 - 0.03, Vector3d(0, 1, 0)));
    double swing_angle = 0;
    Vector3d swing_axis = Vector3d::Zero();
    Quaterniond goal_quat = ComputeAxisAlignedGoalQuaternion(
        near_antipodal_curr, reference_quat, axis_body, 0.4, &hysteresis_state,
        &swing_angle, &swing_axis);

    EXPECT_GT(swing_angle, M_PI) << "expected the hysteresis to take the long way";
    Quaterniond reconstructed = Quaterniond(AngleAxisd(swing_angle, swing_axis)) *
                                near_antipodal_curr.normalized();
    EXPECT_TRUE(reconstructed.isApprox(goal_quat, kTol) ||
                reconstructed.coeffs().isApprox(-goal_quat.coeffs(), kTol));
  }
}

// Regression test for the 3D-printer cone bug: with the object's tracked axis
// hovering near the exact antipode of a fixed target and jittering to both sides
// of it every step, the reported swing -- and the lookahead-truncated rotation
// built from it -- must not flip between two orientations from call to call.
// Before the swing outputs existed, callers re-derived (angle, axis) from the
// goal quaternion via AngleAxisd, which canonicalizes to [0, pi] and negates the
// axis, so a clamped lookahead alternated between +/-2 rad about opposite axes.
TEST(QuaternionAxisAlignmentTest, SwingOutputsSurviveAntipodalJitter) {
  Vector3d axis_body(1, 0, 0);
  // Fixed target: tracked axis points world -z.
  Quaterniond reference_quat(AngleAxisd(M_PI / 2, Vector3d(0, 1, 0)));
  const double angle_hysteresis = 0.4;
  const double lookahead_angle = 2.0;
  Vector3d hysteresis_state = Vector3d::Zero();

  double prev_swing_angle = 0;
  Vector3d prev_swing_axis = Vector3d::Zero();
  Quaterniond prev_rel = Quaterniond::Identity();
  bool first = true;
  for (int k = 0; k < 40; ++k) {
    // Tracked axis stays near world +z (antipodal to the target); the wobble
    // alternates sign each step so the raw cross product flips direction.
    double wobble = (k % 2 == 0 ? 0.02 : -0.02) + 0.01 * std::sin(0.3 * k);
    Quaterniond curr_quat(AngleAxisd(-M_PI / 2 + wobble, Vector3d(0, 1, 0)) *
                          AngleAxisd(0.05 * k, axis_body));  // drifting twist

    double swing_angle = 0;
    Vector3d swing_axis = Vector3d::Zero();
    ComputeAxisAlignedGoalQuaternion(curr_quat, reference_quat, axis_body,
                                     angle_hysteresis, &hysteresis_state,
                                     &swing_angle, &swing_axis);
    Quaterniond rel(AngleAxisd(std::min(swing_angle, lookahead_angle), swing_axis));

    if (!first) {
      EXPECT_GT(swing_axis.dot(prev_swing_axis), 0)
          << "swing axis flipped sign at k=" << k;
      EXPECT_LT(std::abs(swing_angle - prev_swing_angle), 0.5)
          << "swing angle jumped at k=" << k;
      EXPECT_GT(std::abs(rel.coeffs().dot(prev_rel.coeffs())), 1 - 1e-2)
          << "lookahead-truncated rotation flipped at k=" << k;
    }
    prev_swing_angle = swing_angle;
    prev_swing_axis = swing_axis;
    prev_rel = rel;
    first = false;
  }
}

}  // namespace
}  // namespace dairlib
