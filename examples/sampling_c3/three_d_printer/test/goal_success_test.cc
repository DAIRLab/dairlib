// Tests for SamplingC3GoalParams::IsObjectOnTarget, the single definition of
// "this object has reached its goal" shared by SamplingC3GoalGenerator (which
// uses it to advance the goal) and SamplingC3Controller (which uses it to
// decide which objects still need samples, and when to park at a terminal fixed
// goal).

#include <gtest/gtest.h>

#include "examples/sampling_c3/parameter_headers/goal_params.h"

namespace dairlib {
namespace {

using Eigen::AngleAxisd;
using Eigen::Quaterniond;
using Eigen::Vector3d;

// The 3D printer cone demo's settings:  success in xy only, the cone's body +x
// symmetry axis tracked (twist about it free), 2 cm and 0.1 rad thresholds.
SamplingC3GoalParams MakeConeParams() {
  SamplingC3GoalParams params;
  params.position_success_threshold = 0.02;
  params.orientation_success_threshold = 0.1;
  params.only_use_xy_position = true;
  params.tracked_orientation_axis = {Vector3d::UnitX()};
  return params;
}

// A tilt of `angle` about world z misaligns a body +x axis by exactly `angle`.
Quaterniond TiltedByZ(double angle) {
  return Quaterniond(AngleAxisd(angle, Vector3d::UnitZ()));
}

// The regression:  the object is at the waypoint's xy but tilted more than the
// orientation threshold, as happens transiently while it is being pushed.  The
// controller used to skip the orientation term here (whenever the goal step's
// cost switching threshold kept crossed_cost_switching_threshold_ false) and
// call the goal met, while the goal generator -- correctly -- would not advance
// the sequence.  Three such loops parked the robot for good.
TEST(GoalSuccessTest, TiltedAtWaypointIsNotOnTarget) {
  const SamplingC3GoalParams params = MakeConeParams();
  const Vector3d p_goal(0.280, 0.07, 0.0);

  EXPECT_FALSE(params.IsObjectOnTarget(0, p_goal,
                                       TiltedByZ(10.0 * M_PI / 180.0), p_goal,
                                       Quaterniond::Identity()));
}

// Success stays xy-only:  z estimation error must not mask a real achievement.
TEST(GoalSuccessTest, HeightErrorIgnoredWhenOnlyUsingXY) {
  const SamplingC3GoalParams params = MakeConeParams();
  const Vector3d p_goal(0.069, 0.07, 0.05);
  const Vector3d p_curr(0.069, 0.07, 0.05 - 0.5);

  EXPECT_TRUE(params.IsObjectOnTarget(0, p_curr, Quaterniond::Identity(),
                                      p_goal, Quaterniond::Identity()));
}

TEST(GoalSuccessTest, HeightErrorCountsWhenUsingFullPosition) {
  SamplingC3GoalParams params = MakeConeParams();
  params.only_use_xy_position = false;
  const Vector3d p_goal(0.069, 0.07, 0.05);
  const Vector3d p_curr(0.069, 0.07, 0.05 - 0.5);

  EXPECT_FALSE(params.IsObjectOnTarget(0, p_curr, Quaterniond::Identity(),
                                       p_goal, Quaterniond::Identity()));
}

// Twist about the tracked axis is free:  a cone spun about its own symmetry
// axis is still on target.  The controller previously used the raw quaternion
// error here, which is not what the tracked-axis convention means.
TEST(GoalSuccessTest, TwistAboutTrackedAxisIsOnTarget) {
  const SamplingC3GoalParams params = MakeConeParams();
  const Vector3d p_goal(0.069, 0.07, 0.05);
  const Quaterniond q_spun(AngleAxisd(M_PI_2, Vector3d::UnitX()));

  EXPECT_TRUE(params.IsObjectOnTarget(0, p_goal, q_spun, p_goal,
                                      Quaterniond::Identity()));
}

// With no tracked axis for this object, the full orientation is compared, so
// the same spin is a failure.
TEST(GoalSuccessTest, TwistFailsWithoutATrackedAxis) {
  SamplingC3GoalParams params = MakeConeParams();
  params.tracked_orientation_axis = {Vector3d::Zero()};
  const Vector3d p_goal(0.069, 0.07, 0.05);
  const Quaterniond q_spun(AngleAxisd(M_PI_2, Vector3d::UnitX()));

  EXPECT_FALSE(params.IsObjectOnTarget(0, p_goal, q_spun, p_goal,
                                       Quaterniond::Identity()));
}

TEST(GoalSuccessTest, PositionThresholdEdges) {
  const SamplingC3GoalParams params = MakeConeParams();
  const Vector3d p_goal(0.280, 0.07, 0.0);

  EXPECT_TRUE(params.IsObjectOnTarget(0, p_goal + Vector3d(0.019, 0.0, 0.0),
                                      Quaterniond::Identity(), p_goal,
                                      Quaterniond::Identity()));
  EXPECT_FALSE(params.IsObjectOnTarget(0, p_goal + Vector3d(0.021, 0.0, 0.0),
                                       Quaterniond::Identity(), p_goal,
                                       Quaterniond::Identity()));
}

TEST(GoalSuccessTest, OrientationThresholdEdges) {
  const SamplingC3GoalParams params = MakeConeParams();
  const Vector3d p_goal(0.280, 0.07, 0.0);

  EXPECT_TRUE(params.IsObjectOnTarget(0, p_goal, TiltedByZ(0.09), p_goal,
                                      Quaterniond::Identity()));
  EXPECT_FALSE(params.IsObjectOnTarget(0, p_goal, TiltedByZ(0.11), p_goal,
                                       Quaterniond::Identity()));
}

}  // namespace
}  // namespace dairlib
