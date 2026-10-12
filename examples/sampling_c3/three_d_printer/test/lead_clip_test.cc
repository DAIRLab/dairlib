// ClipEEPlanToFingerBend (reposition.h), roadmap C's lead clip.
//
// The printer's finger bends sideways under load, and printer latency lets the
// gantry run 25-35 mm past a caught fingertip before a jam trip acts (toe
// pushes in the 2026-10-06/08 Track A runs).  The clip moves the first knot
// whose predicted bend exceeds the budget back toward the tip, and holds it.

#include <algorithm>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "examples/sampling_c3/reposition.h"

namespace dairlib {
namespace systems {
namespace {

using Eigen::MatrixXd;
using Eigen::Vector3d;

// A push in -x at 10 mm per knot, from x = 0.25 m at z = 0.04 m.
MatrixXd PushPlan(int n) {
  MatrixXd plan(3, n);
  for (int k = 0; k < n; ++k) {
    plan.col(k) = Vector3d(0.25 - 0.01 * k, 0.07, 0.04);
  }
  return plan;
}

TEST(LeadClipTest, LeavesAPlanUnderTheBudgetAlone) {
  MatrixXd plan = PushPlan(6);
  const MatrixXd original = plan;
  const std::vector<Vector3d> bends(6, Vector3d(0.012, 0, 0));
  EXPECT_EQ(ClipEEPlanToFingerBend(bends, 0.015, &plan), -1);
  EXPECT_TRUE(plan.isApprox(original));
}

TEST(LeadClipTest, PullsTheFirstOverBudgetKnotBackAndHoldsIt) {
  MatrixXd plan = PushPlan(6);
  const MatrixXd original = plan;
  // The tip catches on the cone at knot 2; the gantry then leads it in -x, so
  // the bend (tip minus unbent tip) points +x and grows 10 mm per knot.
  std::vector<Vector3d> bends;
  for (int k = 0; k < 6; ++k) {
    bends.push_back(Vector3d(0.01 * std::max(0, k - 1), 0, 0));
  }
  const int knot = ClipEEPlanToFingerBend(bends, 0.015, &plan);
  ASSERT_EQ(knot, 3);  // bends 0, 0, 10, 20, 30, 40 mm
  for (int k = 0; k < 3; ++k) {
    EXPECT_TRUE(plan.col(k).isApprox(original.col(k)));
  }
  // Knot 3 led the tip by 20 mm, so it comes back 5 mm in +x.
  const Vector3d held(original(0, 3) + 0.005, original(1, 3), original(2, 3));
  for (int k = 3; k < 6; ++k) EXPECT_TRUE(plan.col(k).isApprox(held, 1e-12));
}

TEST(LeadClipTest, IgnoresTheVerticalPartOfABend) {
  MatrixXd plan = PushPlan(4);
  const MatrixXd original = plan;
  // 3 mm sideways and 30 mm vertical:  the finger is stiff in z, so this is a
  // 3 mm bend.
  const std::vector<Vector3d> bends(4, Vector3d(0.0, 0.003, -0.03));
  EXPECT_EQ(ClipEEPlanToFingerBend(bends, 0.015, &plan), -1);
  EXPECT_TRUE(plan.isApprox(original));
}

TEST(LeadClipTest, KeepsTheHeightAndPullsBackAlongTheBend) {
  MatrixXd plan = PushPlan(3);
  // A diagonal bend of 25 mm, the tip caught behind and to +y of the gantry.
  const Vector3d bend = Vector3d(0.015, 0.02, 0.0);
  const std::vector<Vector3d> bends(3, bend);
  const MatrixXd original = plan;
  ASSERT_EQ(ClipEEPlanToFingerBend(bends, 0.010, &plan), 0);
  const Vector3d moved = plan.col(0) - original.col(0);
  EXPECT_NEAR(moved.norm(), 0.015, 1e-12);
  EXPECT_NEAR(moved.z(), 0.0, 1e-15);
  EXPECT_NEAR(moved.head<2>().normalized().dot(bend.head<2>().normalized()),
              1.0, 1e-12);
}

TEST(LeadClipTest, ChecksOnlyTheKnotsItHasBendsFor) {
  MatrixXd plan = PushPlan(5);
  const MatrixXd original = plan;
  const std::vector<Vector3d> bends(2, Vector3d::Zero());
  EXPECT_EQ(ClipEEPlanToFingerBend(bends, 0.015, &plan), -1);
  EXPECT_TRUE(plan.isApprox(original));
}

}  // namespace
}  // namespace systems
}  // namespace dairlib
