// Tests for the ground truth labeller
// (examples/sampling_c3/jamming_ground_truth.h). Unlike
// jamming_metrics_test.cc, these run the demo's real Drake sim -- that is the
// whole point of the class -- so they assert relationships that hold for any
// reasonable contact model rather than numbers read off one run.

#include "examples/sampling_c3/jamming_ground_truth.h"

#include <cmath>
#include <string>
#include <thread>
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

// The per-knot states a cost type scores.  With no settle window the rollout
// reports one state per knot: the frozen scene, then the result of each step.
TEST(JammingGroundTruthTest, RolloutReportsOneStatePerKnot) {
  JammingGroundTruthSim sim(kObjectModels, kSimDt, /*settle_fraction=*/0.0);
  const Vector3d start(0.30, 0.07, 0.15);
  const vector<Vector3d> plan = MakePlan(start, Vector3d(0.21, 0.07, 0.15));

  double travel = 0.0;
  double rotation = 0.0;
  vector<Eigen::VectorXd> knot_states;
  sim.Rollout(kConeQuaternion, kConePosition, plan, kKnotDt, &travel, &rotation,
              nullptr, nullptr, {}, &knot_states);

  ASSERT_EQ(knot_states.size(), plan.size());
  for (const Eigen::VectorXd& state : knot_states) {
    ASSERT_EQ(state.size(), 19);
  }
  // Entry 0 is the scene before anything moves: the end effector parked at the
  // plan's first knot, the cone where it was put, everything at rest.
  EXPECT_LT((knot_states.front().head(3) - start).norm(), 1e-9);
  EXPECT_LT((knot_states.front().segment(7, 3) - kConePosition).norm(), 1e-9);
  EXPECT_LT(knot_states.front().tail(9).norm(), 1e-9);
  // The quaternion is stored w-first, as the LCS state and Drake both do.
  EXPECT_NEAR(knot_states.front().segment(3, 4).normalized()(0),
              kConeQuaternion.normalized()(0), 1e-9);
  // And the end effector column follows the plan it was given, to within the
  // same lag bound the tracking test allows.
  for (size_t i = 0; i < plan.size(); ++i) {
    EXPECT_LT((knot_states.at(i).head(3) - plan.at(i)).norm(),
              0.1 * (plan.back() - plan.front()).norm());
  }
}

// The prescribed end effector writes the interpolated axis positions straight
// into the state instead of asking the PD to reach them, so it goes where the
// plan says whatever is in the way.  Driving the tip down through the build
// plate is the cleanest demonstration: the PD-tracked printer stalls against a
// surface it cannot penetrate, and the prescribed one does not notice it.
//
// This is the knob's cost as well as its description.  A jam IS the end
// effector stalling, so a rollout under this cannot report one.
TEST(JammingGroundTruthTest, APrescribedEndEffectorIgnoresWhatIsInItsWay) {
  // Straight down from clear air to well below the plate.
  const vector<Vector3d> plan =
      MakePlan(Vector3d(0.30, 0.07, 0.05), Vector3d(0.30, 0.07, -0.02));

  const JammingGroundTruthSim tracked(kObjectModels, kSimDt,
                                      /*settle_fraction=*/0.0);
  const GroundTruthLabel pd = tracked.Label(kConeQuaternion, kConePosition,
                                            plan, kKnotDt, /*plan_is_real=*/1);

  const JammingGroundTruthSim prescribed(kObjectModels, kSimDt,
                                         /*settle_fraction=*/0.0,
                                         /*point_contact=*/false,
                                         /*prescribed_ee=*/true);
  const GroundTruthLabel written = prescribed.Label(
      kConeQuaternion, kConePosition, plan, kKnotDt, /*plan_is_real=*/1);

  // The plate stops the real printer well short of where it was asked to go.
  EXPECT_GT(pd.sim_ee_tracking_error, 0.01);
  // The prescribed one arrives, so its tracking error is the interpolation
  // residual and nothing else.
  EXPECT_LT(written.sim_ee_tracking_error, 1e-4);
}

// Point contact is a different contact model, not a cheaper solve of the same
// one, so a rollout under it is a different rollout wherever the default model
// was doing hydroelastic work.  Asserted as "the object does not do exactly the
// same thing" rather than as a direction, since which model predicts more
// motion is not a property either one guarantees.
TEST(JammingGroundTruthTest, PointContactIsADifferentContactModel) {
  // The cone up on the ramp, which is where the two models actually differ:
  // at the foot of the ramp every pair in the scene already falls back to
  // point contact, so switching models there is bit-for-bit a no-op.
  const Vector4d ramp_quaternion(-0.150330, -0.232180, -0.481138, 0.831865);
  const Vector3d ramp_position(0.097918, 0.069893, 0.057393);
  const Vector3d parked(0.14, 0.07, 0.15);
  const vector<Vector3d> plan = MakePlan(parked, parked);

  const JammingGroundTruthSim hydroelastic(kObjectModels, kSimDt,
                                           /*settle_fraction=*/0.0);
  const GroundTruthLabel default_model = hydroelastic.Label(
      ramp_quaternion, ramp_position, plan, kKnotDt, /*plan_is_real=*/1);

  const JammingGroundTruthSim point(kObjectModels, kSimDt,
                                    /*settle_fraction=*/0.0,
                                    /*point_contact=*/true);
  const GroundTruthLabel point_model = point.Label(
      ramp_quaternion, ramp_position, plan, kKnotDt, /*plan_is_real=*/1);
  // Every collision in this scene declares a point contact stiffness, so the
  // point model is fully parameterised: the cone is still held up by the ramp
  // rather than falling through it.
  EXPECT_GT(default_model.sim_max_contact_force, 0.0);
  EXPECT_GT(point_model.sim_max_contact_force, 0.0);
  // But it is not the same physics.  The cone resting on the ramp creeps about
  // three times as far under the point model over the same window.
  EXPECT_NE(point_model.sim_object_travel, default_model.sim_object_travel);
  EXPECT_LT(point_model.sim_object_travel, 1e-3);  // still a resting cone
}

// A rollout that assumes a scene at rest describes a different push than the
// one a moving scene is actually undergoing, so the initial velocities have to
// reach the sim.  A cone already sliding toward the ramp ends up further along
// than the same cone started from rest.
TEST(JammingGroundTruthTest, InitialObjectVelocityChangesWhereTheConeEndsUp) {
  JammingGroundTruthSim sim(kObjectModels, kSimDt, /*settle_fraction=*/0.0);
  // Parked well clear of the cone, so the difference is the scene's own motion
  // rather than anything the end effector did.
  const Vector3d parked(0.30, 0.07, 0.15);
  const vector<Vector3d> plan = MakePlan(parked, parked);

  double at_rest = 0.0;
  double moving = 0.0;
  double rotation = 0.0;
  sim.Rollout(kConeQuaternion, kConePosition, plan, kKnotDt, &at_rest,
              &rotation, nullptr, nullptr);

  RolloutInitialVelocities initial_velocities;
  initial_velocities.object_linear_velocity = Vector3d(-0.1, 0.0, 0.0);
  sim.Rollout(kConeQuaternion, kConePosition, plan, kKnotDt, &moving, &rotation,
              nullptr, nullptr, initial_velocities);

  EXPECT_GT(moving, at_rest);
}

// The controller scores its samples in a parallel loop, so rollouts run
// concurrently on one instance.  Identical results from many threads is the
// assertion that the class holds no state they could be sharing.
TEST(JammingGroundTruthTest, ConcurrentRolloutsMatchTheSerialOne) {
  JammingGroundTruthSim sim(kObjectModels, kSimDt, /*settle_fraction=*/0.0);
  const vector<Vector3d> plan =
      MakePlan(Vector3d(0.285, 0.07, 0.025), Vector3d(0.215, 0.07, 0.025));

  auto roll = [&sim, &plan]() {
    double travel = 0.0;
    double rotation = 0.0;
    vector<Eigen::VectorXd> knot_states;
    sim.Rollout(kConeQuaternion, kConePosition, plan, kKnotDt, &travel,
                &rotation, nullptr, nullptr, {}, &knot_states);
    return knot_states;
  };

  const vector<Eigen::VectorXd> expected = roll();
  constexpr int kNumThreads = 8;
  vector<vector<Eigen::VectorXd>> results(kNumThreads);
  vector<std::thread> threads;
  for (int i = 0; i < kNumThreads; ++i) {
    threads.emplace_back([&results, &roll, i]() { results.at(i) = roll(); });
  }
  for (std::thread& thread : threads) {
    thread.join();
  }

  for (const vector<Eigen::VectorXd>& result : results) {
    ASSERT_EQ(result.size(), expected.size());
    for (size_t i = 0; i < expected.size(); ++i) {
      EXPECT_TRUE(result.at(i) == expected.at(i));
    }
  }
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
