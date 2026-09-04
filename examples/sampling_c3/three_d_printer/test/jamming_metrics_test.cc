// Unit tests for the jamming predictors
// (examples/sampling_c3/jamming_metrics.h) that
// examples/sampling_c3/three_d_printer/test/jamming_sweep.cc dumps: the peak
// end effector effort read off a C3 plan, and the peak read off a PD rollout of
// that plan.  Everything here runs against hand-built LCSs whose answers are
// known by inspection, so no C3 solve is involved.

#include "examples/sampling_c3/jamming_metrics.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "examples/sampling_c3/ee_plan_retiming.h"
#include "systems/senders/cost_colormap.h"

namespace dairlib {
namespace systems {
namespace {

using c3::LCS;
using c3::LCSSimulateConfig;
using c3::multibody::LCSContactDescription;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::vector;

// A trivial state: 3 EE positions then 3 EE velocities.
constexpr int kNx = 6;
// Where the EE velocity block starts, i.e. n_q for this layout.
constexpr int kEEVelocityOffset = 3;
constexpr int kNu = 3;
constexpr int kNLambda = 1;

// An LCS whose single complementarity variable is forced to zero (0 <= lambda
// perp lambda + 1 >= 0) and contributes nothing to the dynamics, so only A and
// B matter.  With @p b_scale = 0 the input has no effect at all, which makes
// the resulting tracking error independent of any gain choice.
LCS MakeTrivialLcs(int N, double dt, double b_scale) {
  MatrixXd A = MatrixXd::Identity(kNx, kNx);
  MatrixXd B = MatrixXd::Zero(kNx, kNu);
  // Inputs push the EE velocities, which integrate into the EE positions.
  B.block(3, 0, 3, 3) = b_scale * MatrixXd::Identity(3, 3);
  A.block(0, 3, 3, 3) = dt * MatrixXd::Identity(3, 3);

  MatrixXd D = MatrixXd::Zero(kNx, kNLambda);
  VectorXd d = VectorXd::Zero(kNx);
  MatrixXd E = MatrixXd::Zero(kNLambda, kNx);
  MatrixXd F = MatrixXd::Identity(kNLambda, kNLambda);
  MatrixXd H = MatrixXd::Zero(kNLambda, kNu);
  VectorXd c = VectorXd::Ones(kNLambda);
  return LCS(A, B, D, d, E, F, H, c, N, dt);
}

VectorXd MakeGains(double value, bool position_rows) {
  VectorXd gains = VectorXd::Zero(kNx);
  // SimulatePDControlWithLCS requires exactly n_u non-zero entries, which it
  // reads as the actuated position (Kp) and velocity (Kd) indices.
  gains.segment(position_rows ? 0 : 3, 3) = value * Vector3d::Ones();
  return gains;
}

// One knot of a C3 solution: z = [x; lambda; u].
VectorXd MakeZ(const VectorXd& x, const VectorXd& lambda, const VectorXd& u) {
  VectorXd z(x.size() + lambda.size() + u.size());
  z << x, lambda, u;
  return z;
}

LCSContactDescription MakeDescription(const Vector3d& force_basis) {
  LCSContactDescription description;
  description.witness_point_A = Vector3d::Zero();
  description.witness_point_B = Vector3d::Zero();
  description.force_basis = force_basis;
  description.is_slack = false;
  return description;
}

// --- The object-side layout, for the metrics that ask whether the thing being
// --- pushed actually goes anywhere. ------------------------------------------

// A state laid out the way the sampling LCS lays one out: 3 EE positions, then
// one object's quaternion and position, then the EE and object velocities.
constexpr int kNxObject = 19;
constexpr int kObjectQuatOffset = 3;
constexpr int kObjectPosOffset = 7;
constexpr int kObjectEEVelocityOffset = 10;

// An LCS in that layout whose object translates @p travel_per_step along +x and
// rotates @p angle_per_step about +z on every step, no matter what the input
// does (B is zero).  The translation rides on the constant offset d; the
// rotation rides on A, which for a fixed axis is just a planar rotation by half
// the angle in the quaternion's (w, z) coordinates.  So the object motion the
// rollout produces is known exactly, without solving anything.
LCS MakeObjectLcs(int N, double dt, double travel_per_step,
                  double angle_per_step) {
  MatrixXd A = MatrixXd::Identity(kNxObject, kNxObject);
  const double half = 0.5 * angle_per_step;
  A(kObjectQuatOffset, kObjectQuatOffset) = std::cos(half);
  A(kObjectQuatOffset, kObjectQuatOffset + 3) = -std::sin(half);
  A(kObjectQuatOffset + 3, kObjectQuatOffset) = std::sin(half);
  A(kObjectQuatOffset + 3, kObjectQuatOffset + 3) = std::cos(half);

  MatrixXd B = MatrixXd::Zero(kNxObject, kNu);
  MatrixXd D = MatrixXd::Zero(kNxObject, kNLambda);
  VectorXd d = VectorXd::Zero(kNxObject);
  d(kObjectPosOffset) = travel_per_step;
  MatrixXd E = MatrixXd::Zero(kNLambda, kNxObject);
  MatrixXd F = MatrixXd::Identity(kNLambda, kNLambda);
  MatrixXd H = MatrixXd::Zero(kNLambda, kNu);
  VectorXd c = VectorXd::Ones(kNLambda);
  return LCS(A, B, D, d, E, F, H, c, N, dt);
}

VectorXd MakeObjectGains(double value, bool position_rows) {
  VectorXd gains = VectorXd::Zero(kNxObject);
  gains.segment(position_rows ? 0 : kObjectEEVelocityOffset, 3) =
      value * Vector3d::Ones();
  return gains;
}

// A plan that holds still at the identity orientation, so every bit of object
// motion the rollout reports comes from the LCS above rather than from the
// reference being chased.
vector<VectorXd> MakeStationaryObjectPlan(int num_knots) {
  VectorXd x = VectorXd::Zero(kNxObject);
  x(kObjectQuatOffset) = 1.0;  // A unit quaternion needs w = 1, not w = 0.
  return vector<VectorXd>(num_knots, x);
}

// UnpackC3Plan reads u out of z at the [n_x + n_lambda, +n_u) offset -- not
// from the head of z -- and ComputeC3PlanMetrics reports the peak over the
// horizon.
TEST(JammingMetricsTest, C3PlanMetricsReadUAtTheRightOffsetAndTakeTheMax) {
  // Three knots whose inputs are (1, 0, 0), (0, 3, 4) and (0, 0, 2).  The
  // states and lambdas are deliberately larger than any of the inputs, so a
  // metric that read the wrong slice of z would come out too big.
  vector<VectorXd> z_plan;
  z_plan.push_back(MakeZ(VectorXd::Constant(kNx, 100.0),
                         VectorXd::Constant(kNLambda, 50.0),
                         Vector3d(1.0, 0.0, 0.0)));
  z_plan.push_back(MakeZ(VectorXd::Constant(kNx, 100.0),
                         VectorXd::Constant(kNLambda, 50.0),
                         Vector3d(0.0, 3.0, 4.0)));
  z_plan.push_back(MakeZ(VectorXd::Constant(kNx, 100.0),
                         VectorXd::Constant(kNLambda, 50.0),
                         Vector3d(0.0, 0.0, 2.0)));

  vector<VectorXd> x_plan;
  vector<VectorXd> lambda_plan;
  vector<VectorXd> u_plan;
  UnpackC3Plan(z_plan, kNx, kNLambda, kNu, &x_plan, &lambda_plan, &u_plan);

  const JammingForceBases empty_bases;
  JammingMetrics metrics;
  ComputeC3PlanMetrics(u_plan, lambda_plan, empty_bases, &metrics);

  EXPECT_NEAR(metrics.max_u_norm_c3, 5.0, 1e-12);  // |(0, 3, 4)|
  EXPECT_NEAR(metrics.max_u_xy_c3, 3.0, 1e-12);    // |(0, 3)|
  EXPECT_NEAR(metrics.max_u_z_c3, 4.0, 1e-12);
  // No EE contacts declared, so no contact force is attributed to the EE.
  EXPECT_NEAR(metrics.max_ee_contact_force_c3, 0.0, 1e-12);
}

// Without input limits there is nothing to measure saturation against, so the
// fractions must stay NaN rather than reading as "never saturated" -- a 0.0
// would be indistinguishable from a plan that genuinely stays clear of its
// bound.
TEST(JammingMetricsTest, SaturationIsNaNWithoutInputLimits) {
  const vector<VectorXd> u_plan = {Vector3d(1.0, 0.0, 0.0)};
  const vector<VectorXd> lambda_plan = {VectorXd::Zero(kNLambda)};

  JammingMetrics metrics;
  ComputeC3PlanMetrics(u_plan, lambda_plan, JammingForceBases(), &metrics);

  EXPECT_TRUE(std::isnan(metrics.frac_knots_u_xy_at_limit));
  EXPECT_TRUE(std::isnan(metrics.frac_knots_u_z_at_limit));
}

// MakeUInputLimits collapses the options' [lo, hi] pairs the same way the
// controller collapses the EE velocity pair, and refuses anything unusable
// rather than throwing.
TEST(JammingMetricsTest, MakeUInputLimitsCollapsesAndRejects) {
  const UInputLimits limits = MakeUInputLimits({-25.0, 20.0}, {-3.0, 25.0});
  ASSERT_TRUE(limits.IsValid());
  EXPECT_NEAR(limits.u_xy_max, 20.0, 1e-12);  // min(|-25|, |20|)
  EXPECT_NEAR(limits.u_z_max, 3.0, 1e-12);    // min(|-3|, |25|)

  EXPECT_FALSE(MakeUInputLimits({}, {}).IsValid());
  EXPECT_FALSE(MakeUInputLimits({-25.0}, {-25.0, 25.0}).IsValid());
  EXPECT_FALSE(MakeUInputLimits({0.0, 25.0}, {-25.0, 25.0}).IsValid());
}

// A plan comfortably inside its bounds is not saturated anywhere.
TEST(JammingMetricsTest, SaturationIsZeroForAPlanInsideItsBounds) {
  const vector<VectorXd> u_plan = {Vector3d(1.0, 0.0, 0.5),
                                   Vector3d(0.0, 2.0, 1.0)};
  const vector<VectorXd> lambda_plan(u_plan.size(), VectorXd::Zero(kNLambda));

  JammingMetrics metrics;
  ComputeC3PlanMetrics(u_plan, lambda_plan, JammingForceBases(), &metrics,
                       MakeUInputLimits({-25.0, 25.0}, {-25.0, 25.0}));

  EXPECT_NEAR(metrics.frac_knots_u_xy_at_limit, 0.0, 1e-12);
  EXPECT_NEAR(metrics.frac_knots_u_z_at_limit, 0.0, 1e-12);
}

// The fraction counts knots, not peaks: two of four knots pinned on z reads 0.5
// even though max_u_z_c3 saturates identically in both cases.  The two axes are
// counted independently.
TEST(JammingMetricsTest, SaturationCountsTheFractionOfPinnedKnots) {
  const vector<VectorXd> u_plan = {
      Vector3d(0.0, 0.0, 25.0),   // z pinned, xy slack
      Vector3d(1.0, 0.0, -25.0),  // z pinned (sign ignored), xy slack
      Vector3d(0.0, 25.0, 0.0),   // xy pinned, z slack
      Vector3d(1.0, 1.0, 1.0)};   // neither
  const vector<VectorXd> lambda_plan(u_plan.size(), VectorXd::Zero(kNLambda));

  JammingMetrics metrics;
  ComputeC3PlanMetrics(u_plan, lambda_plan, JammingForceBases(), &metrics,
                       MakeUInputLimits({-25.0, 25.0}, {-25.0, 25.0}));

  EXPECT_NEAR(metrics.frac_knots_u_xy_at_limit, 0.25, 1e-12);
  EXPECT_NEAR(metrics.frac_knots_u_z_at_limit, 0.5, 1e-12);
}

// The C3+ projection imposes no u bounds, and the solution is read out of a z
// the half-step rollout overwrote, so a knot can land marginally *outside* its
// bound.  That has to count as saturated -- the naive |u| == limit test would
// let exactly the worst knots fall through -- and the fraction must stay in
// [0, 1].
TEST(JammingMetricsTest, SaturationCountsKnotsThatOvershootTheBound) {
  const vector<VectorXd> u_plan = {Vector3d(0.0, 0.0, 25.0 + 1e-6),
                                   Vector3d(25.0 + 1e-3, 0.0, 25.0 * 1.5)};
  const vector<VectorXd> lambda_plan(u_plan.size(), VectorXd::Zero(kNLambda));

  JammingMetrics metrics;
  ComputeC3PlanMetrics(u_plan, lambda_plan, JammingForceBases(), &metrics,
                       MakeUInputLimits({-25.0, 25.0}, {-25.0, 25.0}));

  EXPECT_NEAR(metrics.frac_knots_u_z_at_limit, 1.0, 1e-12);
  EXPECT_NEAR(metrics.frac_knots_u_xy_at_limit, 0.5, 1e-12);
}

// The flattened row and its column names have to stay in step, including for
// the two columns added for saturation.
TEST(JammingMetricsTest, RowCarriesTheSaturationColumnsInHeaderOrder) {
  const vector<std::string> names = JammingMetricsColumnNames();
  JammingMetrics metrics;
  metrics.frac_knots_u_xy_at_limit = 0.25;
  metrics.frac_knots_u_z_at_limit = 0.75;
  const VectorXd row = JammingMetricsAsRow(metrics);

  ASSERT_EQ(row.size(), static_cast<int>(names.size()));
  const auto index_of = [&names](const std::string& name) {
    return std::find(names.begin(), names.end(), name) - names.begin();
  };
  ASSERT_LT(index_of("frac_knots_u_xy_at_limit"),
            static_cast<long>(names.size()));
  EXPECT_NEAR(row(index_of("frac_knots_u_xy_at_limit")), 0.25, 1e-12);
  EXPECT_NEAR(row(index_of("frac_knots_u_z_at_limit")), 0.75, 1e-12);
}

TEST(JammingMetricsTest, EEContactForceSumsBasisTimesLambda) {
  // Two EE lambda entries pushing along +x and +y, plus a trailing entry that
  // belongs to some other contact and must be ignored.
  const vector<LCSContactDescription> descriptions = {
      MakeDescription(Vector3d::UnitX()), MakeDescription(Vector3d::UnitY()),
      MakeDescription(Vector3d::UnitZ())};
  const ContactForceBasis basis =
      MakeEEContactForceBasis(descriptions, /*num_ee_lambda_entries=*/2);
  ASSERT_EQ(basis.lambda_indices.size(), 2u);

  VectorXd lambda(3);
  lambda << 3.0, 4.0, 1000.0;  // the third entry is not the EE's
  EXPECT_NEAR(ContactForceMagnitude(lambda, basis), 5.0, 1e-12);

  EXPECT_NEAR(ContactForceMagnitude(VectorXd::Zero(3), basis), 0.0, 1e-12);
}

TEST(JammingMetricsTest, EEContactForceSkipsSlackEntries) {
  vector<LCSContactDescription> descriptions = {
      MakeDescription(Vector3d::UnitX()),
      LCSContactDescription::CreateSlackVariableDescription(),
      MakeDescription(Vector3d::UnitX())};
  const ContactForceBasis basis =
      MakeEEContactForceBasis(descriptions, /*num_ee_lambda_entries=*/3);
  // The slack entry is dropped, so only two entries contribute.
  ASSERT_EQ(basis.lambda_indices.size(), 2u);
  EXPECT_EQ(basis.lambda_indices[0], 0);
  EXPECT_EQ(basis.lambda_indices[1], 2);

  VectorXd lambda(3);
  lambda << 1.0, 999.0, 2.0;
  EXPECT_NEAR(ContactForceMagnitude(lambda, basis), 3.0, 1e-12);
}

TEST(JammingMetricsTest, EEContactForceBasisRejectsTooManyEntries) {
  const vector<LCSContactDescription> descriptions = {
      MakeDescription(Vector3d::UnitX())};
  EXPECT_THROW(MakeEEContactForceBasis(descriptions, 2), std::exception);
}

// With an LCS whose B is zero the rollout cannot move at all, so the EE
// tracking error is exactly the plan's own drift away from its start -- and,
// crucially, is the same no matter how Kp is scaled.  That is the property that
// makes reporting delta_x in meters rather than as a P-term force worthwhile.
TEST(JammingMetricsTest, DeltaXIsInMetersAndIndependentOfKp) {
  constexpr int kNCoarse = 2;
  constexpr int kUpsample = 2;
  const LCS coarse = MakeTrivialLcs(kNCoarse, 0.1, /*b_scale=*/0.0);
  const LCS fine = MakeTrivialLcs(kNCoarse * kUpsample, 0.05, /*b_scale=*/0.0);

  // A plan that climbs 0.01 m in z per coarse knot, starting from the origin.
  vector<VectorXd> x_plan;
  for (int i = 0; i < kNCoarse + 1; ++i) {
    VectorXd x = VectorXd::Zero(kNx);
    x(2) = 0.01 * i;
    x_plan.push_back(x);
  }
  const vector<VectorXd> u_plan(kNCoarse, VectorXd::Zero(kNu));

  const LCSSimulateConfig config;
  JammingMetrics metrics;
  ComputePdRolloutMetrics(x_plan, u_plan, MakeGains(100.0, true),
                          MakeGains(0.5, false), coarse, fine, config,
                          &metrics);

  // The rollout is pinned at the plan's first knot, so the zero-order-held plan
  // gets 0.01 m ahead of it by the third fine step.
  EXPECT_NEAR(metrics.max_delta_x_ee_norm, 0.01, 1e-12);
  EXPECT_NEAR(metrics.max_delta_x_ee_z, 0.01, 1e-12);
  EXPECT_NEAR(metrics.max_delta_x_ee_xy, 0.0, 1e-12);

  // Scaling Kp by 10 changes the forces but not the error, because the input
  // cannot move this system.
  JammingMetrics stiffer;
  ComputePdRolloutMetrics(x_plan, u_plan, MakeGains(1000.0, true),
                          MakeGains(0.5, false), coarse, fine, config,
                          &stiffer);
  EXPECT_NEAR(stiffer.max_delta_x_ee_norm, metrics.max_delta_x_ee_norm, 1e-12);
  EXPECT_GT(stiffer.max_u_norm_pd, metrics.max_u_norm_pd);
}

// The coarse/fine rollout the controller calls downsamples its inputs before
// returning them, so its peak can only ever be a subsample of the fine peak.
TEST(JammingMetricsTest, FineRolloutPeakIsNeverBelowTheDownsampledPeak) {
  constexpr int kNCoarse = 4;
  constexpr int kUpsample = 4;
  const LCS coarse = MakeTrivialLcs(kNCoarse, 0.1, /*b_scale=*/1.0);
  const LCS fine = MakeTrivialLcs(kNCoarse * kUpsample, 0.025, /*b_scale=*/1.0);

  // A plan that jumps around, so the rollout's error -- and therefore its
  // input -- varies within each coarse step.
  vector<VectorXd> x_plan;
  for (int i = 0; i < kNCoarse + 1; ++i) {
    VectorXd x = VectorXd::Zero(kNx);
    x(0) = 0.05 * ((i % 2 == 0) ? 1 : -1);
    x(2) = 0.02 * i;
    x_plan.push_back(x);
  }
  const vector<VectorXd> u_plan(kNCoarse, VectorXd::Zero(kNu));

  JammingMetrics metrics;
  ComputePdRolloutMetrics(x_plan, u_plan, MakeGains(100.0, true),
                          MakeGains(0.5, false), coarse, fine,
                          LCSSimulateConfig(), &metrics);

  EXPECT_GT(metrics.max_u_norm_pd, 0.0);
  EXPECT_GE(metrics.max_u_norm_pd, metrics.max_u_norm_pd_coarse);
}

// --- Generalizing the force basis past the EE's block. -----------------------

// The object-ground group sits immediately after the EE's entries, so reading
// it means starting partway into the descriptions rather than at zero.
TEST(JammingMetricsTest, ContactForceBasisReadsAnInteriorBlock) {
  // Two EE entries, then two object-ground entries, then a trailing entry
  // belonging to a group neither basis covers.
  const vector<LCSContactDescription> descriptions = {
      MakeDescription(Vector3d::UnitX()), MakeDescription(Vector3d::UnitY()),
      MakeDescription(Vector3d::UnitZ()), MakeDescription(Vector3d::UnitX()),
      MakeDescription(Vector3d::UnitY())};
  const ContactForceBasis object_ground =
      MakeContactForceBasis(descriptions, /*first_entry=*/2, /*num_entries=*/2);
  ASSERT_EQ(object_ground.lambda_indices.size(), 2u);
  EXPECT_EQ(object_ground.lambda_indices[0], 2);
  EXPECT_EQ(object_ground.lambda_indices[1], 3);

  VectorXd lambda(5);
  lambda << 100.0, 100.0, 3.0, 4.0, 100.0;  // only entries 2 and 3 are ours
  EXPECT_NEAR(ContactForceMagnitude(lambda, object_ground), 5.0, 1e-12);
}

// The EE wrapper has to stay exactly the leading-block case of the general
// form, since everything already measured was measured through it.
TEST(JammingMetricsTest, EEBasisIsTheGeneralFormAtOffsetZero) {
  vector<LCSContactDescription> descriptions = {
      MakeDescription(Vector3d::UnitX()),
      LCSContactDescription::CreateSlackVariableDescription(),
      MakeDescription(Vector3d::UnitY()), MakeDescription(Vector3d::UnitZ())};
  const ContactForceBasis wrapped =
      MakeEEContactForceBasis(descriptions, /*num_ee_lambda_entries=*/3);
  const ContactForceBasis general =
      MakeContactForceBasis(descriptions, /*first_entry=*/0, /*num_entries=*/3);

  EXPECT_EQ(wrapped.lambda_indices, general.lambda_indices);
  ASSERT_EQ(wrapped.force_bases.size(), general.force_bases.size());
  for (size_t i = 0; i < wrapped.force_bases.size(); ++i) {
    EXPECT_TRUE(wrapped.force_bases[i] == general.force_bases[i]);
  }
}

TEST(JammingMetricsTest, ContactForceBasisRejectsABlockPastTheEnd) {
  const vector<LCSContactDescription> descriptions = {
      MakeDescription(Vector3d::UnitX()), MakeDescription(Vector3d::UnitY())};
  EXPECT_THROW(MakeContactForceBasis(descriptions, 1, 2), std::exception);
  EXPECT_THROW(MakeContactForceBasis(descriptions, -1, 1), std::exception);
}

// How hard the EE pushes and how hard the world pushes back are different
// numbers off the same lambda, and neither may pick up the other's entries.
TEST(JammingMetricsTest, ObjectGroundForceIsMeasuredApartFromTheEEs) {
  const vector<LCSContactDescription> descriptions = {
      MakeDescription(Vector3d::UnitX()), MakeDescription(Vector3d::UnitY()),
      MakeDescription(Vector3d::UnitZ())};
  JammingForceBases bases;
  bases.ee = MakeEEContactForceBasis(descriptions, /*num_ee_lambda_entries=*/1);
  bases.object_ground =
      MakeContactForceBasis(descriptions, /*first_entry=*/1, /*num_entries=*/2);

  VectorXd lambda(3);
  lambda << 2.0, 3.0, 4.0;
  const vector<VectorXd> u_plan = {Vector3d::Zero()};
  const vector<VectorXd> lambda_plan = {lambda};

  JammingMetrics metrics;
  ComputeC3PlanMetrics(u_plan, lambda_plan, bases, &metrics);

  EXPECT_NEAR(metrics.max_ee_contact_force_c3, 2.0, 1e-12);     // |2 x|
  EXPECT_NEAR(metrics.max_object_ground_force_c3, 5.0, 1e-12);  // |3y+4z|
}

// A caller with no object-ground contacts to point at gets zero, not the EE's
// force leaking across.
TEST(JammingMetricsTest, ObjectGroundForceIsZeroWithoutABasis) {
  const vector<LCSContactDescription> descriptions = {
      MakeDescription(Vector3d::UnitX())};
  JammingForceBases bases;
  bases.ee = MakeEEContactForceBasis(descriptions, 1);

  const vector<VectorXd> u_plan = {Vector3d::Zero()};
  const vector<VectorXd> lambda_plan = {VectorXd::Constant(1, 7.0)};

  JammingMetrics metrics;
  ComputeC3PlanMetrics(u_plan, lambda_plan, bases, &metrics);

  EXPECT_NEAR(metrics.max_ee_contact_force_c3, 7.0, 1e-12);
  EXPECT_NEAR(metrics.max_object_ground_force_c3, 0.0, 1e-12);
}

// --- Does the object move? ---------------------------------------------------

// The object columns come off the simulated rollout states, so an LCS that
// marches the object a known amount per step has to read back as exactly that
// much travel and that much rotation.
TEST(JammingMetricsTest, ObjectTravelAndRotationAreReadOffTheRollout) {
  constexpr int kNKnots = 4;
  constexpr double kTravelPerStep = 0.003;
  constexpr double kAnglePerStep = 0.05;
  // Same N and dt for both, so the fine rollout takes one step per plan knot
  // and the totals below are exact.
  const LCS coarse = MakeObjectLcs(kNKnots, 0.1, kTravelPerStep, kAnglePerStep);
  const LCS fine = MakeObjectLcs(kNKnots, 0.1, kTravelPerStep, kAnglePerStep);

  const vector<VectorXd> x_plan = MakeStationaryObjectPlan(kNKnots + 1);
  const vector<VectorXd> u_plan(kNKnots, VectorXd::Zero(kNu));

  JammingMetrics metrics;
  ComputePdRolloutMetrics(
      x_plan, u_plan, MakeObjectGains(100.0, true), MakeObjectGains(0.5, false),
      coarse, fine, LCSSimulateConfig(), &metrics, MakeObjectStateLayout(0));

  EXPECT_NEAR(metrics.object_travel_in_rollout, kNKnots * kTravelPerStep, 1e-9);
  EXPECT_NEAR(metrics.object_rotation_in_rollout, kNKnots * kAnglePerStep,
              1e-9);
}

// A solve that produced nothing to execute leaves the horizon commanding a
// small fraction of the budget; a plan that pushes hard and achieves nothing is
// a different thing entirely and must not be swept up with it.  Getting this
// backwards would mask exactly the jams the sweep exists to find.
TEST(JammingMetricsTest, NoOpPlanFlagsAnEffortlessPlanNotAFutileOne) {
  const UInputLimits limits = MakeUInputLimits({-15.0, 15.0}, {-15.0, 15.0});
  const vector<VectorXd> lambda_plan(2, VectorXd::Zero(kNLambda));

  // Roughly what the degenerate cone samples command: the EE holding itself up.
  const vector<VectorXd> effortless = {Vector3d(0.2, 0.1, 0.3),
                                       Vector3d(0.0, 0.0, 0.4)};
  JammingMetrics no_op;
  ComputeC3PlanMetrics(effortless, lambda_plan, JammingForceBases(), &no_op,
                       limits);
  EXPECT_NEAR(no_op.no_op_plan, 1.0, 1e-12);

  // A real plan, still nowhere near its bound.  4.7 N is the weakest genuine
  // plan seen in the cone sweep, and it must not read as a no-op.
  const vector<VectorXd> pushing = {Vector3d(0.2, 0.1, 0.3),
                                    Vector3d(4.7, 0.0, 0.0)};
  JammingMetrics real;
  ComputeC3PlanMetrics(pushing, lambda_plan, JammingForceBases(), &real,
                       limits);
  EXPECT_NEAR(real.no_op_plan, 0.0, 1e-12);
}

// The flag scales with the configured box rather than with an absolute force,
// so the same plan reads differently against a budget it barely dents.
TEST(JammingMetricsTest, NoOpPlanIsRelativeToTheInputBudget) {
  const vector<VectorXd> u_plan = {Vector3d(4.7, 0.0, 0.0)};
  const vector<VectorXd> lambda_plan(1, VectorXd::Zero(kNLambda));

  JammingMetrics tight;
  ComputeC3PlanMetrics(u_plan, lambda_plan, JammingForceBases(), &tight,
                       MakeUInputLimits({-15.0, 15.0}, {-15.0, 15.0}));
  EXPECT_NEAR(tight.no_op_plan, 0.0, 1e-12);

  JammingMetrics roomy;
  ComputeC3PlanMetrics(u_plan, lambda_plan, JammingForceBases(), &roomy,
                       MakeUInputLimits({-500.0, 500.0}, {-500.0, 500.0}));
  EXPECT_NEAR(roomy.no_op_plan, 1.0, 1e-12);
}

// Without limits there is no budget to be a fraction of, so the flag stays NaN
// rather than claiming every plan is real.
TEST(JammingMetricsTest, NoOpPlanIsNaNWithoutInputLimits) {
  const vector<VectorXd> u_plan = {Vector3d(0.2, 0.0, 0.0)};
  const vector<VectorXd> lambda_plan(1, VectorXd::Zero(kNLambda));

  JammingMetrics metrics;
  ComputeC3PlanMetrics(u_plan, lambda_plan, JammingForceBases(), &metrics);
  EXPECT_TRUE(std::isnan(metrics.no_op_plan));
}

// Rotation is counted about whatever axis the object turns on.  A sample that
// spins the object without translating it is the one case the jam index cannot
// see -- its denominator is travel alone -- which is why the rotation is
// emitted as its own column rather than folded in.
TEST(JammingMetricsTest, RotationIsCountedWithoutAnyTranslation) {
  constexpr int kNKnots = 4;
  constexpr double kAnglePerStep = 0.05;
  const LCS lcs =
      MakeObjectLcs(kNKnots, 0.1, /*travel_per_step=*/0.0, kAnglePerStep);
  const vector<VectorXd> x_plan = MakeStationaryObjectPlan(kNKnots + 1);
  const vector<VectorXd> u_plan(kNKnots, VectorXd::Zero(kNu));

  JammingMetrics metrics;
  ComputePdRolloutMetrics(
      x_plan, u_plan, MakeObjectGains(100.0, true), MakeObjectGains(0.5, false),
      lcs, lcs, LCSSimulateConfig(), &metrics, MakeObjectStateLayout(0));

  EXPECT_NEAR(metrics.object_rotation_in_rollout, kNKnots * kAnglePerStep,
              1e-9);
  EXPECT_NEAR(metrics.object_travel_in_rollout, 0.0, 1e-12);
}

// Without a layout there is nowhere to read the object's pose from, and the
// columns must stay NaN rather than reading as "the object did not move" --
// a 0.0 would be indistinguishable from a genuine jam.
TEST(JammingMetricsTest, ObjectColumnsAreNaNWithoutALayout) {
  constexpr int kNCoarse = 2;
  const LCS lcs = MakeTrivialLcs(kNCoarse, 0.1, /*b_scale=*/0.0);
  const vector<VectorXd> x_plan(kNCoarse + 1, VectorXd::Zero(kNx));
  const vector<VectorXd> u_plan(kNCoarse, VectorXd::Zero(kNu));

  JammingMetrics metrics;
  ComputePdRolloutMetrics(x_plan, u_plan, MakeGains(100.0, true),
                          MakeGains(0.5, false), lcs, lcs, LCSSimulateConfig(),
                          &metrics);

  EXPECT_TRUE(std::isnan(metrics.object_travel_in_rollout));
  EXPECT_TRUE(std::isnan(metrics.object_rotation_in_rollout));

  // And the jam index that divides by the travel stays NaN with it, rather
  // than dividing by the floor and inventing a number.
  ComputeDerivedMetrics(&metrics);
  EXPECT_TRUE(std::isnan(metrics.object_ground_force_per_travel));
}

// The LCS packs the EE's three prismatic positions first and then 7 per
// object, which is the packing every object column above reads through.
TEST(JammingMetricsTest, ObjectStateLayoutMatchesTheLcsPacking) {
  const ObjectStateLayout first = MakeObjectStateLayout(0);
  ASSERT_TRUE(first.IsValid());
  EXPECT_EQ(first.quaternion_offset, kObjectQuatOffset);
  EXPECT_EQ(first.position_offset, kObjectPosOffset);

  const ObjectStateLayout second = MakeObjectStateLayout(1);
  ASSERT_TRUE(second.IsValid());
  EXPECT_EQ(second.quaternion_offset, kObjectQuatOffset + 7);
  EXPECT_EQ(second.position_offset, kObjectPosOffset + 7);

  EXPECT_FALSE(ObjectStateLayout().IsValid());
}

// --- The jam index. ----------------------------------------------------------

// Newtons per meter moved, with the denominator floored so a plan that pushes
// hard and moves nothing reads as a large finite number rather than infinity.
TEST(JammingMetricsTest, JamIndexIsForcePerMeterAndFloorsTheDenominator) {
  JammingMetrics moving;
  moving.max_object_ground_force_c3 = 6.0;
  moving.object_travel_in_rollout = 0.002;
  ComputeDerivedMetrics(&moving);
  EXPECT_NEAR(moving.object_ground_force_per_travel, 3000.0, 1e-9);

  // A jam: the same reaction force, nothing moving.  Finite, and far larger.
  JammingMetrics wedged;
  wedged.max_object_ground_force_c3 = 6.0;
  wedged.object_travel_in_rollout = 0.0;
  ComputeDerivedMetrics(&wedged);
  EXPECT_TRUE(std::isfinite(wedged.object_ground_force_per_travel));
  EXPECT_NEAR(wedged.object_ground_force_per_travel,
              6.0 / kMinObjectTravelForJamIndex, 1e-6);
  EXPECT_GT(wedged.object_ground_force_per_travel,
            moving.object_ground_force_per_travel);

  // Travel just above the floor is not clamped.
  JammingMetrics crawling;
  crawling.max_object_ground_force_c3 = 6.0;
  crawling.object_travel_in_rollout = 10.0 * kMinObjectTravelForJamIndex;
  ComputeDerivedMetrics(&crawling);
  EXPECT_NEAR(crawling.object_ground_force_per_travel,
              6.0 / (10.0 * kMinObjectTravelForJamIndex), 1e-6);
}

// The object-side columns have to reach the sweep file in the same order the
// header names them, the same way the saturation columns do.
TEST(JammingMetricsTest, RowCarriesTheObjectColumnsInHeaderOrder) {
  const vector<std::string> names = JammingMetricsColumnNames();
  JammingMetrics metrics;
  metrics.object_travel_in_rollout = 0.011;
  metrics.object_rotation_in_rollout = 0.22;
  metrics.no_op_plan = 1.0;
  metrics.max_object_ground_force_c3 = 3.3;
  metrics.object_ground_force_per_travel = 44.0;
  const VectorXd row = JammingMetricsAsRow(metrics);

  ASSERT_EQ(row.size(), static_cast<int>(names.size()));
  const auto index_of = [&names](const std::string& name) {
    const auto it = std::find(names.begin(), names.end(), name);
    EXPECT_NE(it, names.end()) << name << " is missing from the header";
    return it - names.begin();
  };
  EXPECT_NEAR(row(index_of("object_travel_in_rollout")), 0.011, 1e-12);
  EXPECT_NEAR(row(index_of("object_rotation_in_rollout")), 0.22, 1e-12);
  EXPECT_NEAR(row(index_of("no_op_plan")), 1.0, 1e-12);
  EXPECT_NEAR(row(index_of("max_object_ground_force_c3")), 3.3, 1e-12);
  EXPECT_NEAR(row(index_of("object_ground_force_per_travel")), 44.0, 1e-12);
}

// --- The retiming the metrics are measured through. --------------------------
//
// ComputeJammingMetrics measures the C3 plan slowed to the EE velocity limits
// and resampled onto its own knot times.  Reaching it end to end needs a solved
// c3::C3, which this file deliberately does without, so what is covered here is
// the transform it applies and how the measured plan differs from the plan as
// solved.  The unretimed numbers below stand in for what the metrics would
// report with the limits unconfigured.

// A plan climbing `dz` in z per knot with a per knot input of ascending
// magnitude, so the retiming has something to slow and the late knots carry
// the largest forces.
struct OverspeedPlan {
  vector<VectorXd> x_plan;  // n knots
  vector<VectorXd> u_plan;  // n - 1
  vector<VectorXd> lambda_plan;
  static constexpr int kN = 4;  // coarse horizon, so n = kN + 1 knots
};

OverspeedPlan MakeOverspeedPlan(double dz) {
  OverspeedPlan plan;
  for (int i = 0; i < OverspeedPlan::kN + 1; ++i) {
    VectorXd x = VectorXd::Zero(kNx);
    x(2) = dz * i;
    // A nonzero EE z velocity, so the retiming's velocity scaling is
    // exercised rather than multiplying zeros.
    x(kEEVelocityOffset + 2) = dz;
    plan.x_plan.push_back(x);
  }
  for (int i = 0; i < OverspeedPlan::kN; ++i) {
    plan.u_plan.push_back(Vector3d(0.0, 0.0, 1.0 + i));
    plan.lambda_plan.push_back(VectorXd::Constant(kNLambda, 1.0 + i));
  }
  return plan;
}

// An unconfigured limit must not read as "retiming available"; the plan is
// then measured as solved rather than retimed against a garbage speed.
TEST(JammingMetricsTest, EEVelocityLimitsRejectUnconfiguredValues) {
  EXPECT_FALSE(EEVelocityLimits{}.IsValid());
  EXPECT_FALSE((EEVelocityLimits{0.0, 0.015, 3}).IsValid());
  EXPECT_FALSE((EEVelocityLimits{0.12, 0.0, 3}).IsValid());
  // An EE velocity block overlapping the EE positions is a misconfiguration,
  // not a plan the retiming can rewrite.
  EXPECT_FALSE((EEVelocityLimits{0.12, 0.015, 2}).IsValid());
  EXPECT_TRUE((EEVelocityLimits{0.12, 0.015, 3}).IsValid());
}

// A plan the printer can already execute is left exactly alone, so retiming it
// reports the same numbers as measuring it as solved.
TEST(JammingMetricsTest, RetimingAFeasiblePlanReproducesTheUnretimedMetrics) {
  constexpr double kDt = 0.1;
  // 0.001 m per 0.1 s is 0.01 m/s, inside the vertical limit below.
  const OverspeedPlan plan = MakeOverspeedPlan(0.001);

  const LCS coarse = MakeTrivialLcs(OverspeedPlan::kN, kDt, /*b_scale=*/0.0);
  const LCS fine =
      MakeTrivialLcs(OverspeedPlan::kN * 2, kDt / 2, /*b_scale=*/0.0);

  JammingMetrics unretimed;
  ComputeC3PlanMetrics(plan.u_plan, plan.lambda_plan, JammingForceBases(),
                       &unretimed);
  ComputePdRolloutMetrics(plan.x_plan, plan.u_plan, MakeGains(100.0, true),
                          MakeGains(0.5, false), coarse, fine,
                          LCSSimulateConfig(), &unretimed);

  vector<VectorXd> x_retimed = plan.x_plan;
  vector<VectorXd> u_retimed = plan.u_plan;
  vector<VectorXd> lambda_retimed = plan.lambda_plan;
  RetimeAndResampleEEPlan(kDt, /*v_xy_max=*/0.12, /*v_z_max=*/0.015,
                          kEEVelocityOffset, &x_retimed, &u_retimed,
                          &lambda_retimed);

  JammingMetrics retimed;
  ComputeC3PlanMetrics(u_retimed, lambda_retimed, JammingForceBases(),
                       &retimed);
  ComputePdRolloutMetrics(x_retimed, u_retimed, MakeGains(100.0, true),
                          MakeGains(0.5, false), coarse, fine,
                          LCSSimulateConfig(), &retimed);

  EXPECT_NEAR(retimed.max_u_norm_c3, unretimed.max_u_norm_c3, 1e-12);
  EXPECT_NEAR(retimed.max_u_z_c3, unretimed.max_u_z_c3, 1e-12);
  EXPECT_NEAR(retimed.max_u_norm_pd, unretimed.max_u_norm_pd, 1e-12);
  EXPECT_NEAR(retimed.max_delta_x_ee_norm, unretimed.max_delta_x_ee_norm,
              1e-12);
  EXPECT_NEAR(retimed.max_delta_x_ee_z, unretimed.max_delta_x_ee_z, 1e-12);
}

// Retiming does not invent forces: the retimed u and lambda are a
// hold-and-repeat of the solved plan's knot values, so a plan whose late knots
// push hardest loses those peaks once the path is truncated to what the printer
// can reach.
TEST(JammingMetricsTest, RetimingCanOnlyLowerTheC3PlanForcePeak) {
  constexpr double kDt = 0.1;
  // 0.05 m per 0.1 s is 0.5 m/s vertically, 33x the limit used below, so the
  // resampled knots all fall inside the plan's first segment.
  const OverspeedPlan plan = MakeOverspeedPlan(0.05);

  JammingMetrics unretimed;
  ComputeC3PlanMetrics(plan.u_plan, plan.lambda_plan, JammingForceBases(),
                       &unretimed);

  vector<VectorXd> x_retimed = plan.x_plan;
  vector<VectorXd> u_retimed = plan.u_plan;
  vector<VectorXd> lambda_retimed = plan.lambda_plan;
  RetimeAndResampleEEPlan(kDt, /*v_xy_max=*/0.12, /*v_z_max=*/0.015,
                          kEEVelocityOffset, &x_retimed, &u_retimed,
                          &lambda_retimed);

  JammingMetrics retimed;
  ComputeC3PlanMetrics(u_retimed, lambda_retimed, JammingForceBases(),
                       &retimed);

  // The last knot's u of 4 N is never reached; only the first segment's 1 N is.
  EXPECT_NEAR(unretimed.max_u_z_c3, 4.0, 1e-12);
  EXPECT_NEAR(retimed.max_u_z_c3, 1.0, 1e-12);
  EXPECT_LE(retimed.max_u_norm_c3, unretimed.max_u_norm_c3);
}

// The retimed rollout is a genuinely different simulation, not a subsample of
// the unretimed one.  For this plan -- badly overspeed, tracked by a rollout
// that cannot move at all -- the slowed reference covers far less path and so
// does not run away from the rollout.  The direction is a property of this
// case, not a general law: on real sweeps the retimed tracking error comes out
// larger than the unretimed one on a fair fraction of samples.  What is general
// is that the two differ, which is why the retimed one is the one recorded.
TEST(JammingMetricsTest, RetimedRolloutTracksTheSlowedReference) {
  constexpr double kDt = 0.1;
  const OverspeedPlan plan = MakeOverspeedPlan(0.05);

  // b_scale = 0 means the input cannot move the rollout at all, so delta_x is
  // exactly how far the reference gets ahead of the plan's first knot -- a
  // gain-free comparison of the two references.
  const LCS coarse = MakeTrivialLcs(OverspeedPlan::kN, kDt, /*b_scale=*/0.0);
  const LCS fine =
      MakeTrivialLcs(OverspeedPlan::kN * 2, kDt / 2, /*b_scale=*/0.0);

  JammingMetrics unretimed;
  ComputePdRolloutMetrics(plan.x_plan, plan.u_plan, MakeGains(100.0, true),
                          MakeGains(0.5, false), coarse, fine,
                          LCSSimulateConfig(), &unretimed);

  vector<VectorXd> x_retimed = plan.x_plan;
  vector<VectorXd> u_retimed = plan.u_plan;
  RetimeAndResampleEEPlan(kDt, /*v_xy_max=*/0.12, /*v_z_max=*/0.015,
                          kEEVelocityOffset, &x_retimed, &u_retimed);

  JammingMetrics retimed;
  ComputePdRolloutMetrics(x_retimed, u_retimed, MakeGains(100.0, true),
                          MakeGains(0.5, false), coarse, fine,
                          LCSSimulateConfig(), &retimed);

  EXPECT_GT(unretimed.max_delta_x_ee_z, 0.0);
  EXPECT_GT(retimed.max_delta_x_ee_z, 0.0);
  EXPECT_LT(retimed.max_delta_x_ee_z, unretimed.max_delta_x_ee_z);
  // Less tracking error means less commanded force out of the same gains,
  // which is why measuring the retimed plan scores samples differently.
  EXPECT_LT(retimed.max_u_norm_pd, unretimed.max_u_norm_pd);
}

// The colormap the jamming visualizer paints samples with.  Low must read
// green and high must read red, or the picture means the opposite of what the
// legend says.
TEST(JammingColormapTest, LowIsGreenAndHighIsRed) {
  const Eigen::Vector3i low = RdYlGnReversedColor(0.0);
  EXPECT_GT(low(1), low(0));  // green channel dominates red
  EXPECT_GT(low(1), low(2));

  const Eigen::Vector3i high = RdYlGnReversedColor(1.0);
  EXPECT_GT(high(0), high(1));  // red channel dominates green
  EXPECT_GT(high(0), high(2));
}

// RdYlGn is a diverging map: it runs green -> yellow -> red, so no single
// channel is monotone (red peaks at 255 in the yellow middle, then falls to 165
// at the dark-red end).  What must hold is that "greenness", g - r, is ordered
// across the ramp, and that the middle really is yellow.
TEST(JammingColormapTest, RunsGreenThroughYellowToRed) {
  auto greenness = [](double t) {
    const Eigen::Vector3i color = RdYlGnReversedColor(t);
    return color(1) - color(0);
  };
  EXPECT_GT(greenness(0.0), greenness(0.5));
  EXPECT_GT(greenness(0.5), greenness(1.0));
  EXPECT_GT(greenness(0.0), 0);
  EXPECT_LT(greenness(1.0), 0);

  const Eigen::Vector3i middle = RdYlGnReversedColor(0.5);
  EXPECT_GT(middle(0), 200);  // yellow: red and green high, blue low
  EXPECT_GT(middle(1), 200);
  EXPECT_LT(middle(2), 200);
}

TEST(JammingColormapTest, OutOfRangeClampsRatherThanWrapping) {
  // A value past an endpoint must saturate; wrapping would paint the single
  // hottest sample green.
  EXPECT_EQ(RdYlGnReversedColor(-5.0), RdYlGnReversedColor(0.0));
  EXPECT_EQ(RdYlGnReversedColor(37.0), RdYlGnReversedColor(1.0));
  // NaN has to land somewhere in range rather than indexing out of bounds.
  const Eigen::Vector3i nan_color =
      RdYlGnReversedColor(std::numeric_limits<double>::quiet_NaN());
  EXPECT_GE(nan_color.minCoeff(), 0);
  EXPECT_LE(nan_color.maxCoeff(), 255);
}

TEST(JammingColormapTest, TableIsTheExpectedShapeAndInRange) {
  const Eigen::MatrixXi& table = RdYlGnReversedTable();
  ASSERT_EQ(table.rows(), kNumColormapEntries);
  ASSERT_EQ(table.cols(), 3);
  EXPECT_GE(table.minCoeff(), 0);
  EXPECT_LE(table.maxCoeff(), 255);

  const Eigen::VectorXf& positions = RdYlGnReversedPositions();
  ASSERT_EQ(positions.size(), kNumColormapEntries);
  EXPECT_FLOAT_EQ(positions(0), 0.0f);
  EXPECT_FLOAT_EQ(positions(kNumColormapEntries - 1), 1.0f);
}

TEST(JammingMetricsTest, PdRolloutLambdaIsNotAvailableAtThePinnedC3Commit) {
  // SimulatePDControlWithLCSAndForces does not exist at the c3 commit dairlib
  // pins, so this field stays NaN; the sweep's plots must treat it as missing
  // rather than as a zero force.
  const JammingMetrics metrics;
  EXPECT_TRUE(std::isnan(metrics.max_ee_contact_force_pd));
}

}  // namespace
}  // namespace systems
}  // namespace dairlib
