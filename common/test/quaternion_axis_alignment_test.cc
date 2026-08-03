#include "common/quaternion_axis_alignment.h"

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

}  // namespace
}  // namespace dairlib
