// Regression tests for SamplingParams' own validation.
//
// unsuccessful_min_retention_seconds is a std::optional, and leaving it out is
// documented to mean "no floor" -- the controller reads it with
// .value_or(0.0).  Its range check used to compare the optional itself against
// 0.0, and since an empty std::optional orders below every double, that aborted
// on startup for every demo that omits the key (all of them but the 3D
// printer's).  These pin both spellings.

#include <gtest/gtest.h>

#include "examples/sampling_c3/parameter_headers/sampling_params.h"

#include "drake/common/yaml/yaml_io.h"

namespace dairlib {
namespace {

// Omits unsuccessful_min_retention_seconds.
constexpr char kJacktoySamplingParams[] =
    "examples/sampling_c3/jacktoy/parameters/sampling_params.yaml";

// Sets it, to 6.0.
constexpr char kPrinterSamplingParams[] =
    "examples/sampling_c3/three_d_printer/printer_shared_parameters/"
    "sampling_params.yaml";

TEST(SamplingParamsTest, OmittingTheRetentionFloorLoadsAndMeansNoFloor) {
  SamplingParams params;
  ASSERT_NO_THROW({
    params = drake::yaml::LoadYamlFile<SamplingParams>(kJacktoySamplingParams);
  });
  EXPECT_FALSE(params.unsuccessful_min_retention_seconds.has_value());
  // How the controller resolves it; see SamplingC3Controller's constructor.
  EXPECT_DOUBLE_EQ(params.unsuccessful_min_retention_seconds.value_or(0.0),
                   0.0);
}

TEST(SamplingParamsTest, SettingTheRetentionFloorIsReadBack) {
  const auto params =
      drake::yaml::LoadYamlFile<SamplingParams>(kPrinterSamplingParams);
  ASSERT_TRUE(params.unsuccessful_min_retention_seconds.has_value());
  EXPECT_GT(*params.unsuccessful_min_retention_seconds, 0.0);
}

}  // namespace
}  // namespace dairlib
