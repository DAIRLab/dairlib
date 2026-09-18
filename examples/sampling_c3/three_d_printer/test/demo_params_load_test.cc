// Regression test: every demo's full parameter chain must load.
//
// SamplingC3ControllerParams pulls in each demo's goal, sampling, progress,
// reposition and C3 option files, and Drake's YAML archive requires a key for
// every non-optional field.  So a field added for one demo silently breaks
// every other demo's startup until their yamls catch up -- which is how both
// of these landed:
//
//   * fixed_target_position_sequence / fixed_target_orientation_sequence came
//     in with goal mode 3 (kFixedGoalSequence) for the 3D printer.  The other
//     demos run goal mode 0 and never got the keys, so their goal params threw
//     "missing entry for ... fixed_target_position_sequence" even though
//     ValidateFixedGoalSequence() deliberately skips their mode.
//   * unsuccessful_min_retention_seconds is a std::optional whose range check
//     compared the optional itself against 0.0.  An empty std::optional orders
//     below every double, so omitting the key aborted on startup.
//
// Walking every demo here means the next such field cannot break the rest
// quietly.  See also sampling_params_test for the optional's value semantics.

#include <gtest/gtest.h>

#include "examples/sampling_c3/parameter_headers/sampling_c3_controller_params.h"

#include "drake/common/yaml/yaml_io.h"

namespace dairlib {
namespace {

constexpr const char* kAllDemoControllerParams[] = {
    "examples/sampling_c3/jacktoy/parameters/"
    "sampling_c3_controller_params.yaml",
    "examples/sampling_c3/push_t/parameters/"
    "sampling_c3_controller_params.yaml",
    "examples/sampling_c3/anything/parameters/"
    "sampling_c3_controller_params.yaml",
    "examples/sampling_c3/three_d_printer/cone/parameters/"
    "sampling_c3_controller_params.yaml",
};

TEST(DemoParamsLoadTest, EveryDemosParametersLoad) {
  for (const char* path : kAllDemoControllerParams) {
    SCOPED_TRACE(path);
    EXPECT_NO_THROW(
        { drake::yaml::LoadYamlFile<SamplingC3ControllerParams>(path); });
  }
}

// The goal sequence is read only in goal mode 3, and only the printer uses it.
// The rest declare the keys empty, which ValidateFixedGoalSequence() tolerates
// precisely because it returns early for their mode.
TEST(DemoParamsLoadTest, OnlyTheSequenceDemoPopulatesTheGoalSequence) {
  for (const char* path : kAllDemoControllerParams) {
    SCOPED_TRACE(path);
    const auto& goal_params =
        drake::yaml::LoadYamlFile<SamplingC3ControllerParams>(path)
            .goal_params;
    if (goal_params.goal_mode == GoalMode::kFixedGoalSequence) {
      EXPECT_FALSE(goal_params.fixed_target_position_sequence.empty());
      EXPECT_EQ(goal_params.fixed_target_position_sequence.size(),
                goal_params.fixed_target_orientation_sequence.size());
    } else {
      EXPECT_TRUE(goal_params.fixed_target_position_sequence.empty());
      EXPECT_TRUE(goal_params.fixed_target_orientation_sequence.empty());
    }
  }
}

}  // namespace
}  // namespace dairlib
