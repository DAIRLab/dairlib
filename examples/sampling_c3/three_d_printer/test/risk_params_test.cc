// Pins the cone demo's shipped risk_params.yaml to the configuration the
// offline label study actually measured (see the recommended settings in
// examples/sampling_c3/fast_jamming_label.h).
//
// No sim here -- this is the cheap guard.  If either the yaml or the config's
// defaults drift, the agreement figures quoted throughout fast_jamming_label.h
// stop describing what the controller runs, and that is exactly the kind of
// drift nothing else would catch.

#include <optional>
#include <string>

#include <gtest/gtest.h>

#include "examples/sampling_c3/fast_jamming_label.h"
#include "examples/sampling_c3/parameter_headers/risk_params.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_controller_params.h"

#include "drake/common/yaml/yaml_io.h"

namespace dairlib {
namespace systems {
namespace {

constexpr char kConeControllerParams[] =
    "examples/sampling_c3/three_d_printer/cone/parameters/"
    "sampling_c3_controller_params.yaml";

// The whole configuration in one string, so a single assertion covers every
// knob and a diff reads as the change that was made.
TEST(RiskParamsTest, ConeYamlIsTheConfigurationTheStudyRecommended) {
  const SampleRiskParams risk_params =
      drake::yaml::LoadYamlFile<SampleRiskParams>(
          "examples/sampling_c3/three_d_printer/cone/parameters/"
          "risk_params.yaml");
  EXPECT_EQ(MakeFastJammingLabelConfig(risk_params).Describe(),
            "dt4ms_hydro_settle25_thr1.1mm_exit");
}

// The rollout replays the SIM's cone, not the controller's simplified LCS
// model.  Getting this wrong would silently label a different scene than the
// one the agreement was measured on, and would still run.
TEST(RiskParamsTest, TheLabellerSimulatesTheSimsObjectNotTheLcsModel) {
  const SamplingC3ControllerParams controller_params =
      drake::yaml::LoadYamlFile<SamplingC3ControllerParams>(
          kConeControllerParams);
  ASSERT_TRUE(controller_params.risk_params.has_value());
  ASSERT_EQ(controller_params.risk_params->object_models.size(), 1);
  EXPECT_EQ(controller_params.risk_params->object_models.at(0),
            "examples/sampling_c3/urdf/cone/cone.sdf");
  EXPECT_NE(controller_params.risk_params->object_models.at(0),
            controller_params.object_models.at(0));
}

// The gate itself: risk_params_file is what turns jam labelling on, and every
// demo that omits the key -- jacktoy, anything, push_t -- must be able to leave
// it out.  Read through a struct carrying only that key, so this stays a test
// of the gate rather than of whether some other demo's unrelated parameters
// happen to be up to date (jacktoy's goal_params.yaml currently is not).
struct JustTheRiskParamsKey {
  std::optional<std::string> risk_params_file;
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(risk_params_file));
  }
};

TEST(RiskParamsTest, TheKeyIsOptional) {
  drake::yaml::LoadYamlOptions ignore_the_other_keys;
  ignore_the_other_keys.allow_yaml_with_no_cpp = true;

  const auto cone = drake::yaml::LoadYamlFile<JustTheRiskParamsKey>(
      kConeControllerParams, {}, {}, ignore_the_other_keys);
  EXPECT_TRUE(cone.risk_params_file.has_value());

  const auto jacktoy = drake::yaml::LoadYamlFile<JustTheRiskParamsKey>(
      "examples/sampling_c3/jacktoy/parameters/"
      "sampling_c3_controller_params.yaml",
      {}, {}, ignore_the_other_keys);
  EXPECT_FALSE(jacktoy.risk_params_file.has_value());
}

}  // namespace
}  // namespace systems
}  // namespace dairlib
