#include "systems/controllers/pose_latch_release.h"

#include <gtest/gtest.h>

namespace dairlib {
namespace systems {
namespace {

constexpr double kRelease = 0.09;
constexpr double kHold = 2.0;

TEST(PoseLatchReleaseTimerTest, InsideNeverReleases) {
  PoseLatchReleaseTimer timer;
  for (double t = 0; t < 10; t += 0.1) {
    EXPECT_FALSE(timer.Update(0.085, kRelease, kHold, t));
  }
}

TEST(PoseLatchReleaseTimerTest, SustainedSlideBackReleasesAfterHold) {
  PoseLatchReleaseTimer timer;
  EXPECT_FALSE(timer.Update(0.13, kRelease, kHold, 41.7));
  EXPECT_FALSE(timer.Update(0.13, kRelease, kHold, 43.6));
  EXPECT_TRUE(timer.Update(0.13, kRelease, kHold, 43.7));
}

TEST(PoseLatchReleaseTimerTest, BriefExcursionsRestartTheTimer) {
  PoseLatchReleaseTimer timer;
  // 1.5 s out, one loop back inside, 1.5 s out again:  never 2 s in a row.
  for (double t = 0; t < 1.5; t += 0.1) {
    EXPECT_FALSE(timer.Update(0.095, kRelease, kHold, t));
  }
  EXPECT_FALSE(timer.Update(0.08, kRelease, kHold, 1.5));
  for (double t = 1.6; t < 3.1; t += 0.1) {
    EXPECT_FALSE(timer.Update(0.095, kRelease, kHold, t));
  }
}

TEST(PoseLatchReleaseTimerTest, ResetRestartsTheTimer) {
  PoseLatchReleaseTimer timer;
  EXPECT_FALSE(timer.Update(0.12, kRelease, kHold, 0.0));
  timer.Reset();
  EXPECT_FALSE(timer.Update(0.12, kRelease, kHold, 2.5));
  EXPECT_TRUE(timer.Update(0.12, kRelease, kHold, 4.5));
}

}  // namespace
}  // namespace systems
}  // namespace dairlib
