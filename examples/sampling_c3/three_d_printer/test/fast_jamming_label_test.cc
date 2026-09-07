// Tests for the fast approximate labeller
// (examples/sampling_c3/fast_jamming_label.h).  The class exists to trade
// accuracy for speed, so the assertions here are about the things that must NOT
// change with the knobs: that the reference configuration still reproduces the
// ground truth's verdict, that the early exit is exact rather than approximate,
// and that Label() is safe to call concurrently.
//
// How far each knob moves the label is a measurement, not a test -- the sweep's
// --label_variants mode produces that -- so nothing here pins a distance.

#include "examples/sampling_c3/fast_jamming_label.h"

#include <cmath>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "examples/sampling_c3/jamming_ground_truth.h"

namespace dairlib {
namespace systems {
namespace {

using Eigen::Vector3d;
using Eigen::Vector4d;
using std::string;
using std::vector;

// The cone demo's sim, as sim_params.yaml configures it.
constexpr double kSimDt = 0.001;
const vector<string> kObjectModels = {
    "examples/sampling_c3/urdf/cone/cone.sdf"};

// The frozen scene the sweep studies: the cone toppled onto its side at the
// foot of the ramp, part way through goal 1 -> goal 2.
const Vector4d kConeQuaternion(0.707, 0.0, -0.707, 0.0);
const Vector3d kConePosition(0.234, 0.07, 0.0);

constexpr double kKnotDt = 0.075;
constexpr int kNumKnots = 11;

// The two fixtures the ground truth test pins its verdicts on: a plan that
// holds still and achieves nothing, and one that drives into the cone.
const Vector3d kParked(0.30, 0.07, 0.15);
const Vector3d kPushStart(0.285, 0.07, 0.025);
const Vector3d kPushEnd(0.215, 0.07, 0.025);

vector<Vector3d> MakePlan(const Vector3d& start, const Vector3d& end) {
  vector<Vector3d> plan;
  for (int i = 0; i < kNumKnots; ++i) {
    const double alpha = static_cast<double>(i) / (kNumKnots - 1);
    plan.push_back(start + alpha * (end - start));
  }
  return plan;
}

// A configuration cheap enough to use where the test is about bookkeeping
// rather than physics.
FastJammingLabelConfig FastConfig() {
  FastJammingLabelConfig config;
  config.sim_dt = 0.004;
  config.point_contact = true;
  config.settle_fraction = 0.25;
  config.travel_threshold = 2e-3;
  return config;
}

// The reference configuration reproduces the ground truth's verdict.  The one
// thing it cannot reproduce from its own knobs is the passive baseline the
// ground truth subtracts, which is a constant per scene -- so it is measured
// here, from the ground truth itself, and folded into the threshold.  Nothing
// in the library hard-codes it.
TEST(FastJammingLabelTest, ReferenceConfigReproducesTheGroundTruthVerdict) {
  JammingGroundTruthSim reference(kObjectModels, kSimDt);

  const vector<vector<Vector3d>> fixtures = {MakePlan(kParked, kParked),
                                             MakePlan(kPushStart, kPushEnd)};
  for (const vector<Vector3d>& plan : fixtures) {
    const GroundTruthLabel truth = reference.Label(
        kConeQuaternion, kConePosition, plan, kKnotDt, /*plan_is_real=*/1);

    FastJammingLabelConfig config = FastJammingLabelConfig::Reference();
    // progress = travel - passive, so thresholding raw travel at the ground
    // truth's threshold plus that passive is the same test on the same number.
    config.travel_threshold += truth.sim_object_travel_passive;
    FastJammingLabelSim fast(kObjectModels, config);
    const FastJammingLabel label = fast.Label(
        kConeQuaternion, kConePosition, plan, kKnotDt, /*plan_is_real=*/1);

    EXPECT_NEAR(label.travel, truth.sim_object_travel, 1e-12);
    EXPECT_NEAR(label.jammed, truth.jammed, 1e-12);
  }
}

// Both classes resolve the tip offset from the plant rather than assuming it,
// and they have to agree or they are not simulating the same machine.
TEST(FastJammingLabelTest, AgreesWithTheReferenceOnTheEEOffset) {
  JammingGroundTruthSim reference(kObjectModels, kSimDt);
  FastJammingLabelSim fast(kObjectModels, FastJammingLabelConfig::Reference());
  EXPECT_LT((fast.ee_to_joint_offset() - reference.ee_to_joint_offset()).norm(),
            1e-12);
}

// A discrete plant is required, for the same reason the reference requires one:
// the printer's actuator PD gains are only installed when the plant has a time
// step.  A threshold has to be a real length, too, or every sample is jammed.
TEST(FastJammingLabelTest, RejectsAConfigItCannotHonour) {
  FastJammingLabelConfig continuous = FastConfig();
  continuous.sim_dt = 0.0;
  EXPECT_THROW(FastJammingLabelSim(kObjectModels, continuous), std::exception);

  FastJammingLabelConfig no_threshold = FastConfig();
  no_threshold.travel_threshold = 0.0;
  EXPECT_THROW(FastJammingLabelSim(kObjectModels, no_threshold),
               std::exception);
}

// The early exit is exact, not an approximation: travel is a running maximum,
// so a crossing cannot be undone and stopping at it cannot change the verdict.
TEST(FastJammingLabelTest, TheEarlyExitDoesNotChangeTheVerdict) {
  const vector<vector<Vector3d>> fixtures = {MakePlan(kParked, kParked),
                                             MakePlan(kPushStart, kPushEnd)};
  FastJammingLabelConfig patient = FastConfig();
  patient.early_exit = false;
  FastJammingLabelConfig impatient = FastConfig();
  impatient.early_exit = true;

  FastJammingLabelSim patient_sim(kObjectModels, patient);
  FastJammingLabelSim impatient_sim(kObjectModels, impatient);

  for (const vector<Vector3d>& plan : fixtures) {
    const FastJammingLabel full = patient_sim.Label(
        kConeQuaternion, kConePosition, plan, kKnotDt, /*plan_is_real=*/1);
    const FastJammingLabel early = impatient_sim.Label(
        kConeQuaternion, kConePosition, plan, kKnotDt, /*plan_is_real=*/1);

    EXPECT_NEAR(early.jammed, full.jammed, 1e-12);
    if (early.exited_early) {
      // Only a sample that cleared the bar may stop, and its travel is then a
      // lower bound on the maximum rather than the maximum itself.
      EXPECT_GE(early.travel, impatient.travel_threshold);
      EXPECT_LE(early.travel, full.travel + 1e-12);
    } else {
      EXPECT_NEAR(early.travel, full.travel, 1e-12);
    }
  }
}

// A no-op plan cannot be jammed by a push it never made, and a caller that
// cannot say either way gets NaN rather than a guess -- the same contract the
// ground truth labeller offers.
TEST(FastJammingLabelTest, ANoOpPlanIsNotLabelledJammed) {
  FastJammingLabelSim fast(kObjectModels, FastConfig());
  const vector<Vector3d> plan = MakePlan(kParked, kParked);

  const FastJammingLabel no_op = fast.Label(kConeQuaternion, kConePosition,
                                            plan, kKnotDt, /*plan_is_real=*/0);
  EXPECT_NEAR(no_op.jammed, 0.0, 1e-12);
  // The measurement is still reported -- only the verdict is withheld.
  EXPECT_TRUE(std::isfinite(no_op.travel));

  const FastJammingLabel unknown = fast.Label(
      kConeQuaternion, kConePosition, plan, kKnotDt, /*plan_is_real=*/-1);
  EXPECT_TRUE(std::isnan(unknown.jammed));
  EXPECT_TRUE(std::isfinite(unknown.travel));
}

// The point of the class is to run inside the controller's per-sample parallel
// loop, so one instance has to serve many threads at once.  Label() holds no
// state, so concurrent calls must agree with the serial answer exactly.
TEST(FastJammingLabelTest, LabelIsSafeToCallConcurrently) {
  FastJammingLabelSim fast(kObjectModels, FastConfig());
  const vector<Vector3d> plan = MakePlan(kPushStart, kPushEnd);
  const FastJammingLabel expected = fast.Label(
      kConeQuaternion, kConePosition, plan, kKnotDt, /*plan_is_real=*/1);

  constexpr int kNumThreads = 4;
  vector<FastJammingLabel> results(kNumThreads);
  vector<std::thread> threads;
  for (int i = 0; i < kNumThreads; ++i) {
    threads.emplace_back([&fast, &plan, &results, i]() {
      results.at(i) = fast.Label(kConeQuaternion, kConePosition, plan, kKnotDt,
                                 /*plan_is_real=*/1);
    });
  }
  for (std::thread& thread : threads) thread.join();

  for (const FastJammingLabel& result : results) {
    EXPECT_EQ(result.travel, expected.travel);
    EXPECT_EQ(result.jammed, expected.jammed);
  }
}

// The study writes one column per variant, so configurations that differ in any
// knob have to be distinguishable by name.
TEST(FastJammingLabelTest, DescribeNamesEachKnob) {
  const string reference = FastJammingLabelConfig::Reference().Describe();
  EXPECT_NE(reference.find("dt1ms"), string::npos);
  EXPECT_NE(reference.find("hydro"), string::npos);

  FastJammingLabelConfig config = FastConfig();
  config.early_exit = true;
  config.prescribed_ee = true;
  const string described = config.Describe();
  EXPECT_NE(described.find("dt4ms"), string::npos);
  EXPECT_NE(described.find("point"), string::npos);
  EXPECT_NE(described.find("settle25"), string::npos);
  EXPECT_NE(described.find("thr2.0mm"), string::npos);
  EXPECT_NE(described.find("exit"), string::npos);
  EXPECT_NE(described.find("kinee"), string::npos);
  EXPECT_NE(described, reference);
}

}  // namespace
}  // namespace systems
}  // namespace dairlib
