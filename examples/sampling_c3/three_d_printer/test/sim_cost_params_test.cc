// Pins the printer's shipped progress_params_c3plus.yaml to the cost type and
// hysteresis fractions the cost-type-7 sweeps actually measured, and pins the
// no-op defaults of the sim cost's shape knobs.
//
// No sim here -- this is the cheap guard.  The fractions below are not round
// numbers anyone would arrive at twice:  they come from Monte Carlo over two
// 2000-draw jamming_sweep runs (endgame and ramp base, both --label_with_sim),
// and a cost type change or an inherited LCS-era value silently makes them
// describe a distribution that no longer exists.  A cost type 7 distribution
// has a coefficient of variation near 0.018, so an LCS-era fraction like 0.85
// gates nothing at all and the controller simply stops switching modes.

#include <optional>
#include <string>

#include <gtest/gtest.h>

#include "examples/sampling_c3/parameter_headers/progress_params.h"

#include "drake/common/yaml/yaml_io.h"

namespace dairlib {
namespace systems {
namespace {

constexpr char kPrinterProgressParams[] =
    "examples/sampling_c3/three_d_printer/printer_shared_parameters/"
    "progress_params_c3plus.yaml";

SamplingC3ProgressParams Load() {
  return drake::yaml::LoadYamlFile<SamplingC3ProgressParams>(
      kPrinterProgressParams);
}

// Both phases score with the real Drake sim.  The two are pinned together
// because the hysteresis fractions below are only valid for cost type 7's
// distribution, and cost_type_position governs every goal step before the last.
TEST(SimCostParamsTest, PrinterScoresBothPhasesWithTheDrakeSim) {
  const SamplingC3ProgressParams params = Load();
  EXPECT_EQ(params.cost_type, C3CostComputationType::kSimDrakeObjectOnly);
  EXPECT_EQ(params.cost_type_position,
            C3CostComputationType::kSimDrakeObjectOnly);
}

// The fractions, with the per-loop firing rate each was chosen for.  Relative
// rather than absolute on purpose: the two scenes sit 16x apart in cost level
// (403k endgame, 24.6k ramp base) but have comparable relative spread.
TEST(SimCostParamsTest, HysteresisFractionsMatchTheSweptValues) {
  const SamplingC3ProgressParams params = Load();
  EXPECT_TRUE(params.use_relative_hysteresis);

  // ~7.5% of loops at the endgame, ~6.9% at the ramp base.
  EXPECT_EQ(params.hyst_c3_to_repos_frac, 0.015);
  EXPECT_EQ(params.hyst_c3_to_repos_frac_position, 0.03);

  // Deliberately rarer (~2.3% and ~4.5%): abandoning a reposition part-way is
  // the exceptional route out, not the usual one.
  EXPECT_EQ(params.hyst_repos_to_c3_frac, 0.005);
  EXPECT_EQ(params.hyst_repos_to_c3_frac_position, 0.01);

  // Left at the LCS-era value so the controller stays committed to a
  // repositioning target once it picks one; 0.3 is unreachable on cost 7.
  EXPECT_EQ(params.hyst_repos_to_repos_frac, 0.3);
  EXPECT_EQ(params.hyst_repos_to_repos_frac_position, 0.3);
}

// The shape and speed knobs are written out explicitly rather than left unset,
// so the file says what rollout cost type 7 is running.  Every value here is
// the one that reproduces the full "ground truth" rollout the cost has always
// used, so the block is behaviourally identical to deleting it -- which is
// what this pins:  a knob whose shipped value drifted away from its own unset
// default would silently rescale every cost the fractions above were tuned
// against, and a knob whose unset default drifted away from the value here
// would do the same to the six demos that omit the block.
TEST(SimCostParamsTest, ShippedSimCostKnobsAreTheFullGroundTruthRollout) {
  const SamplingC3ProgressParams params = Load();

  // Score the plan's own N+1 knots, weighting them equally.
  EXPECT_EQ(params.sim_cost_settle_fraction.value_or(0.0), 0.0);
  EXPECT_EQ(params.sim_cost_terminal_weight.value_or(1.0), 1.0);
  // The cone demo's sim_params.yaml dt, in both phases, which is what an unset
  // value falls back to.
  EXPECT_EQ(params.sim_cost_dt.value_or(0.001), 0.001);
  EXPECT_EQ(params.sim_cost_dt_position.value_or(0.001), 0.001);
  // The default hydroelastic contact model and the printer's real PD-tracked
  // end effector.  Both measured at 1.0x, so neither has a speed case, and
  // both change the physics the cost is meant to be reporting.
  EXPECT_FALSE(params.sim_cost_point_contact.value_or(false));
  EXPECT_FALSE(params.sim_cost_prescribed_ee.value_or(false));
}

// The knobs are std::optional so the six demos that never select cost type 7
// need no entry at all.  Their unset defaults have to be exactly the values
// pinned above; this is the half of that guard the printer's own file cannot
// check, since it sets all six.
TEST(SimCostParamsTest, ADemoThatOmitsTheKnobsGetsTheSameRollout) {
  const SamplingC3ProgressParams params = Load();
  EXPECT_TRUE(params.sim_cost_settle_fraction.has_value());
  EXPECT_TRUE(params.sim_cost_dt.has_value());

  SamplingC3ProgressParams unset;
  EXPECT_FALSE(unset.sim_cost_settle_fraction.has_value());
  EXPECT_FALSE(unset.sim_cost_terminal_weight.has_value());
  EXPECT_FALSE(unset.sim_cost_dt.has_value());
  EXPECT_FALSE(unset.sim_cost_dt_position.has_value());
  EXPECT_FALSE(unset.sim_cost_point_contact.has_value());
  EXPECT_FALSE(unset.sim_cost_prescribed_ee.has_value());

  EXPECT_EQ(unset.sim_cost_settle_fraction.value_or(0.0),
            params.sim_cost_settle_fraction.value());
  EXPECT_EQ(unset.sim_cost_terminal_weight.value_or(1.0),
            params.sim_cost_terminal_weight.value());
  EXPECT_EQ(unset.sim_cost_point_contact.value_or(false),
            params.sim_cost_point_contact.value());
  EXPECT_EQ(unset.sim_cost_prescribed_ee.value_or(false),
            params.sim_cost_prescribed_ee.value());
  // sim_cost_dt has no counterpart here: unset means "the demo's
  // sim_params.yaml dt", which this struct cannot see.  That the shipped 0.001
  // is that dt is pinned above instead.
}

}  // namespace
}  // namespace systems
}  // namespace dairlib
