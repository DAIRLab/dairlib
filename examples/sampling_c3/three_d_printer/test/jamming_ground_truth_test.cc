// Tests for the ground truth labeller
// (examples/sampling_c3/jamming_ground_truth.h). Unlike
// jamming_metrics_test.cc, these run the demo's real Drake sim -- that is the
// whole point of the class -- so they assert relationships that hold for any
// reasonable contact model rather than numbers read off one run.

#include "examples/sampling_c3/jamming_ground_truth.h"

#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

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

// A plan is a list of EE positions at the planning knot spacing.
constexpr double kKnotDt = 0.075;
constexpr int kNumKnots = 11;

// A plan that walks the end effector from @p start to @p end in equal steps.
vector<Vector3d> MakePlan(const Vector3d& start, const Vector3d& end) {
  vector<Vector3d> plan;
  for (int i = 0; i < kNumKnots; ++i) {
    const double alpha = static_cast<double>(i) / (kNumKnots - 1);
    plan.push_back(start + alpha * (end - start));
  }
  return plan;
}

// The printer's axes have to be world-aligned for a single offset to map an EE
// position to joint coordinates, and the constructor throws if they are not --
// so building one at all is the assertion.  The offset itself is a pure z drop
// from the carriage to the tool tip.
TEST(JammingGroundTruthTest, EEToJointOffsetIsResolvedFromThePlant) {
  JammingGroundTruthSim sim(kObjectModels, kSimDt);
  const Vector3d& offset = sim.ee_to_joint_offset();
  EXPECT_NEAR(offset(0), 0.0, 1e-9);
  EXPECT_NEAR(offset(1), 0.0, 1e-9);
  EXPECT_LT(offset(2), 0.0);  // the tip hangs below the carriage
}

// A discrete plant is required: the printer's actuator PD gains, which are what
// makes the end effector track a commanded position at all, are only installed
// when the plant has a time step.
TEST(JammingGroundTruthTest, RejectsAContinuousPlant) {
  EXPECT_THROW(JammingGroundTruthSim(kObjectModels, 0.0), std::exception);
}

// The end effector has to actually follow the plan, or every label describes a
// push that was never made.  Driving through free space well clear of the cone
// isolates the tracking from any contact.
TEST(JammingGroundTruthTest, TheEndEffectorTracksThePlanItIsGiven) {
  JammingGroundTruthSim sim(kObjectModels, kSimDt);
  // High above the scene, moving at the printer's horizontal speed limit.
  const vector<Vector3d> plan =
      MakePlan(Vector3d(0.30, 0.07, 0.15), Vector3d(0.21, 0.07, 0.15));
  const GroundTruthLabel label = sim.Label(kConeQuaternion, kConePosition, plan,
                                           kKnotDt, /*plan_is_real=*/1);

  EXPECT_NEAR(label.plan_ee_displacement, 0.09, 1e-9);
  // A tenth of the commanded motion is a generous bound on the printer's own
  // lag; anything worse means the velocity feedforward has stopped working.
  EXPECT_LT(label.sim_ee_tracking_error, 0.1 * label.plan_ee_displacement);
}

// The label is a difference from doing nothing, so a plan that holds the end
// effector still has to come out at zero progress however far the cone settles
// on its own.
TEST(JammingGroundTruthTest, HoldingStillMakesNoProgress) {
  JammingGroundTruthSim sim(kObjectModels, kSimDt);
  const Vector3d parked(0.30, 0.07, 0.15);
  const GroundTruthLabel label =
      sim.Label(kConeQuaternion, kConePosition, MakePlan(parked, parked),
                kKnotDt, /*plan_is_real=*/1);

  EXPECT_NEAR(label.sim_object_progress, 0.0, 1e-9);
  // And the settle it cancels is real, not zero: a passive baseline that never
  // moved would make the subtraction pointless.
  EXPECT_GT(label.sim_object_travel_passive, 0.0);
  // A plan that commanded effort and achieved nothing is the jam.
  EXPECT_NEAR(label.jammed, 1.0, 1e-12);
}

// ... and a push that connects has to beat that baseline.  Asserted as a
// relationship rather than a distance, so it does not depend on the contact
// parameters happening to stay where they are today.
TEST(JammingGroundTruthTest, PushingTheConeBeatsTheStandingStillBaseline) {
  JammingGroundTruthSim sim(kObjectModels, kSimDt);
  // Into the cone's far face at mid-height, driving toward the ramp.
  const vector<Vector3d> plan =
      MakePlan(Vector3d(0.285, 0.07, 0.025), Vector3d(0.215, 0.07, 0.025));
  const GroundTruthLabel label = sim.Label(kConeQuaternion, kConePosition, plan,
                                           kKnotDt, /*plan_is_real=*/1);

  EXPECT_GT(label.sim_object_progress, kJammedProgressThreshold);
  EXPECT_GT(label.sim_object_travel, label.sim_object_travel_passive);
  EXPECT_NEAR(label.jammed, 0.0, 1e-12);
  EXPECT_GT(label.sim_max_contact_force, 0.0);
}

// A solve that produced nothing to execute cannot be jammed by a push it never
// made, and a caller that cannot say either way gets NaN rather than a guess.
TEST(JammingGroundTruthTest, ANoOpPlanIsNotLabelledJammed) {
  JammingGroundTruthSim sim(kObjectModels, kSimDt);
  const Vector3d parked(0.30, 0.07, 0.15);
  const vector<Vector3d> plan = MakePlan(parked, parked);

  const GroundTruthLabel no_op = sim.Label(kConeQuaternion, kConePosition, plan,
                                           kKnotDt, /*plan_is_real=*/0);
  EXPECT_NEAR(no_op.jammed, 0.0, 1e-12);
  // The measurements are still reported -- only the verdict is withheld.
  EXPECT_TRUE(std::isfinite(no_op.sim_object_travel));

  const GroundTruthLabel unknown = sim.Label(
      kConeQuaternion, kConePosition, plan, kKnotDt, /*plan_is_real=*/-1);
  EXPECT_TRUE(std::isnan(unknown.jammed));
}

TEST(JammingGroundTruthTest, RowCarriesTheColumnsInHeaderOrder) {
  const vector<string> names = JammingGroundTruthSim::ColumnNames();
  GroundTruthLabel label;
  label.sim_object_travel = 0.05;
  label.sim_object_rotation = 0.3;
  label.sim_object_travel_passive = 0.001;
  label.sim_object_progress = 0.049;
  label.sim_ee_tracking_error = 0.002;
  label.plan_ee_displacement = 0.09;
  label.sim_max_contact_force = 1.5;
  label.jammed = 0.0;
  const Eigen::VectorXd row = JammingGroundTruthSim::AsRow(label);

  ASSERT_EQ(row.size(), static_cast<int>(names.size()));
  const auto index_of = [&names](const string& name) {
    const auto it = std::find(names.begin(), names.end(), name);
    EXPECT_NE(it, names.end()) << name << " is missing from the header";
    return it - names.begin();
  };
  EXPECT_NEAR(row(index_of("sim_object_progress")), 0.049, 1e-12);
  EXPECT_NEAR(row(index_of("sim_ee_tracking_error")), 0.002, 1e-12);
  EXPECT_NEAR(row(index_of("plan_ee_displacement")), 0.09, 1e-12);
  EXPECT_NEAR(row(index_of("jammed")), 0.0, 1e-12);
}

}  // namespace
}  // namespace systems
}  // namespace dairlib
