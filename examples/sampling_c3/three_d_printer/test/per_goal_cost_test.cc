// Covers the per-goal-step override of q_vector_position added for the 3D
// printer cone demo.
//
// The motivating failure:  goal step 0 slides the cone in -Y while holding
// x = 0.280, which sits close to the workspace's +X limit of 0.35 -- the build
// plate edge.  With an isotropic object-position cost, C3 had no reason to
// prefer sliding along Y over drifting in X, and could push the cone off the
// plate.  q_vector_position_sequence lets that one goal weight X more heavily
// without touching the goals that follow.
//
// These tests pin the loading and indexing side of that feature; the live
// controller consumes it through SamplingC3Options::GetC3Options(false,
// goal_step) with the step that RefreshPerGoalSettings() last selected.

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "examples/sampling_c3/parameter_headers/sampling_c3_controller_params.h"

#include "drake/common/yaml/yaml_io.h"

namespace dairlib {
namespace {

constexpr char kConeControllerParams[] =
    "examples/sampling_c3/three_d_printer/cone/parameters/"
    "sampling_c3_controller_params.yaml";

// A demo that leaves q_vector_position_sequence unset, to pin the fallback.
// Loaded as bare options rather than through SamplingC3ControllerParams, since
// only the cost block matters here.
constexpr char kJacktoyOptions[] =
    "examples/sampling_c3/jacktoy/parameters/sampling_c3plus_options.yaml";

// Indices into the LCS state:  EE position is 0-2, object orientation 3-6, and
// object position 7-9.
constexpr int kObjectX = 7;
constexpr int kObjectY = 8;
constexpr int kObjectZ = 9;

constexpr bool kPositionTracking = false;
constexpr bool kPoseTracking = true;

SamplingC3Options LoadConeOptions() {
  return drake::yaml::LoadYamlFile<SamplingC3ControllerParams>(
             kConeControllerParams)
      .sampling_c3_options;
}

// ---------------------------------------------------------------------------
// The shipped cone configuration
// ---------------------------------------------------------------------------

TEST(PerGoalCostTest, ConeYamlWeightsObjectXHarderForGoalZeroOnly) {
  const SamplingC3Options options = LoadConeOptions();
  ASSERT_TRUE(options.has_per_goal_position_cost());

  const auto Q0 = options.GetC3Options(kPositionTracking, 0).Q;
  const auto Q1 = options.GetC3Options(kPositionTracking, 1).Q;
  const auto Q2 = options.GetC3Options(kPositionTracking, 2).Q;

  // Goal 0 is the -Y slide that must not drift toward the +X plate edge.
  EXPECT_DOUBLE_EQ(Q0(kObjectX, kObjectX), 4.0 * Q1(kObjectX, kObjectX));
  EXPECT_GT(Q0(kObjectX, kObjectX), Q0(kObjectY, kObjectY));
  EXPECT_GT(Q0(kObjectX, kObjectX), Q0(kObjectZ, kObjectZ));

  // Goal 0 changes the X weight and nothing else.
  EXPECT_DOUBLE_EQ(Q0(kObjectY, kObjectY), Q1(kObjectY, kObjectY));
  EXPECT_DOUBLE_EQ(Q0(kObjectZ, kObjectZ), Q1(kObjectZ, kObjectZ));

  // Goals 1 and 2 declare an empty entry, so they keep q_vector_position.
  EXPECT_TRUE(Q1.isApprox(Q2));
  EXPECT_TRUE(Q1.isApprox(options.c3_options_position.Q));
}

// The override is scoped to position tracking; pose tracking always uses
// q_vector, which no goal overrides.
TEST(PerGoalCostTest, PoseTrackingCostIsTheSameForEveryGoal) {
  const SamplingC3Options options = LoadConeOptions();
  const auto Q0 = options.GetC3Options(kPoseTracking, 0).Q;
  EXPECT_TRUE(Q0.isApprox(options.GetC3Options(kPoseTracking, 1).Q));
  EXPECT_TRUE(Q0.isApprox(options.GetC3Options(kPoseTracking, 2).Q));
  EXPECT_TRUE(Q0.isApprox(options.c3_options_pose.Q));
}

// Only Q depends on the goal:  the R/G/U penalties and the LCS factory options
// are shared, so a per-goal solve differs from its neighbours in exactly one
// matrix.
TEST(PerGoalCostTest, OnlyQVariesAcrossGoals) {
  const SamplingC3Options options = LoadConeOptions();
  const auto goal_0 = options.GetC3Options(kPositionTracking, 0);
  const auto goal_1 = options.GetC3Options(kPositionTracking, 1);
  EXPECT_TRUE(goal_0.R.isApprox(goal_1.R));
  EXPECT_TRUE(goal_0.G.isApprox(goal_1.G));
  EXPECT_TRUE(goal_0.U.isApprox(goal_1.U));
  EXPECT_FALSE(goal_0.Q.isApprox(goal_1.Q));
}

// ---------------------------------------------------------------------------
// Indexing
// ---------------------------------------------------------------------------

// RefreshPerGoalSettings() clamps the step it is handed, and so does the
// selector, so a step past the end of the sequence holds the last goal's cost
// rather than reading out of bounds.  This is the state the controller parks in
// once the final fixed goal is reached.
TEST(PerGoalCostTest, GoalStepIsClampedToTheSequenceBounds) {
  const SamplingC3Options options = LoadConeOptions();
  const auto last = options.GetC3Options(kPositionTracking, 2).Q;
  EXPECT_TRUE(options.GetC3Options(kPositionTracking, 3).Q.isApprox(last));
  EXPECT_TRUE(options.GetC3Options(kPositionTracking, 99).Q.isApprox(last));

  // detected_goal_changes_ starts at -1, so a negative step must land on 0.
  const auto first = options.GetC3Options(kPositionTracking, 0).Q;
  EXPECT_TRUE(options.GetC3Options(kPositionTracking, -1).Q.isApprox(first));
}

// ---------------------------------------------------------------------------
// Fallback for demos that do not opt in
// ---------------------------------------------------------------------------

TEST(PerGoalCostTest, DemoWithoutTheSequenceUsesOneCostForEveryGoal) {
  const SamplingC3Options options =
      drake::yaml::LoadYamlFile<SamplingC3Options>(kJacktoyOptions);
  ASSERT_FALSE(options.q_vector_position_sequence.has_value());
  EXPECT_FALSE(options.has_per_goal_position_cost());

  const auto& expected = options.c3_options_position.Q;
  for (int goal_step : {0, 1, 2, 7}) {
    EXPECT_TRUE(
        options.GetC3Options(kPositionTracking, goal_step).Q.isApprox(expected))
        << "goal step " << goal_step;
  }
}

}  // namespace
}  // namespace dairlib
