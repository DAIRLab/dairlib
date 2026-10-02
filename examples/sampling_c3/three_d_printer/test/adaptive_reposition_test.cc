// Unit tests for the piecewise-linear repositioning knobs added for the 3D
// printer's slow z axis (see reposition_params.h / the "Faster piecewise-linear
// repositioning" plan): the adaptive cruise height in RepositionPiecewiseLinear,
// and the collision-check gating (ComputeRepositionClearance / the direct-
// diagonal route inside Reposition) exercised against a small SceneGraph.

#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "examples/sampling_c3/reposition.h"

#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"

namespace dairlib {
namespace systems {
namespace {

using drake::geometry::Box;
using drake::geometry::GeometryId;
using drake::geometry::GeometryInstance;
using drake::geometry::QueryObject;
using drake::geometry::SceneGraph;
using drake::math::RigidTransformd;

constexpr int kNq = 10;  // 3 EE + 7 object (quat + xyz)
constexpr int kNx = 19;  // + 3 EE vel + 6 object vel
constexpr int kN = 10;
constexpr double kDt = 0.075;

SamplingC3RepositionParams MakeParams() {
  SamplingC3RepositionParams p{};
  p.traj_type = RepositioningTrajectoryType::kPiecewiseLinear;
  p.speed_horizontal = 0.12;
  p.speed_vertical = 0.015;
  p.use_straight_line_traj_under_spline = 0.12;
  p.use_straight_line_traj_within_angle = 0.3;
  p.use_straight_line_traj_under_piecewise_linear = 0.008;
  p.spline_width = 0.17;
  p.sphere_radius = 0.18;
  p.circle_radius = 0.20;
  p.circle_height = 0.0;
  p.pwl_waypoint_height = 0.15;
  p.pwl_adaptive_waypoint_height = true;
  p.pwl_clearance_margin = 0.01;
  p.pwl_height_search_step = 0.01;
  p.pwl_num_path_collision_samples = 12;
  p.max_tilt_angle = 20;
  return p;
}

Eigen::VectorXd Row5(double a, double b, double c, double d, double e) {
  Eigen::VectorXd v(5);
  v << a, b, c, d, e;
  return v;
}

SamplingC3Options MakeOptions() {
  SamplingC3Options o{};
  o.workspace_limits = {Row5(1, 0, 0, 0.0, 0.35), Row5(0, 1, 0, 0.0, 0.35),
                        Row5(0, 0, 1, 0.006, 0.248)};
  o.workspace_margins = 0.002;
  return o;
}

Eigen::VectorXd MakeLcsState(const Eigen::Vector3d& ee) {
  Eigen::VectorXd x = Eigen::VectorXd::Zero(kNx);
  x.head(3) = ee;
  x.segment(kNq - 3, 3) = Eigen::Vector3d(0.2, 0.2, 0.02);  // object position
  return x;
}

// A scene with one anchored box obstacle plus a throwaway "EE" geometry (so we
// have an id to exclude).  `obstacle_pose` places the box centre.
struct Scene {
  std::unique_ptr<SceneGraph<double>> scene_graph;
  std::unique_ptr<drake::systems::Context<double>> context;
  GeometryId ee_id;

  const QueryObject<double>& query() const {
    return scene_graph->get_query_output_port().Eval<QueryObject<double>>(
        *context);
  }
};

Scene MakeScene(const RigidTransformd& obstacle_pose,
                const Eigen::Vector3d& obstacle_size) {
  Scene s;
  s.scene_graph = std::make_unique<SceneGraph<double>>();
  const auto source = s.scene_graph->RegisterSource("test");

  auto obstacle = std::make_unique<GeometryInstance>(
      obstacle_pose,
      std::make_unique<Box>(obstacle_size.x(), obstacle_size.y(),
                            obstacle_size.z()),
      "obstacle");
  obstacle->set_proximity_properties(drake::geometry::ProximityProperties{});
  s.scene_graph->RegisterAnchoredGeometry(source, std::move(obstacle));

  auto ee = std::make_unique<GeometryInstance>(
      RigidTransformd(Eigen::Vector3d(5, 5, 5)),
      std::make_unique<Box>(0.01, 0.01, 0.01), "ee");
  ee->set_proximity_properties(drake::geometry::ProximityProperties{});
  s.ee_id = s.scene_graph->RegisterAnchoredGeometry(source, std::move(ee));

  s.context = s.scene_graph->CreateDefaultContext();
  return s;
}

// With no scene supplied, adaptive repositioning is a no-op: the plan uses the
// fixed pwl_waypoint_height (it climbs toward 0.15, not a lower height).
TEST(AdaptiveRepositionTest, NoSceneUsesFixedWaypointHeight) {
  const auto params = MakeParams();
  const auto options = MakeOptions();
  const Eigen::Vector3d ee(0.10, 0.10, 0.02);
  const Eigen::Vector3d target(0.30, 0.30, 0.02);
  bool finished = false;

  Eigen::MatrixXd knots =
      Reposition(kNq, kNx, 400, MakeLcsState(ee), target, kDt,
                 /*is_doing_c3=*/false, finished, params, options);

  double peak_z = 0.0;
  for (int i = 0; i < knots.cols(); ++i) peak_z = std::max(peak_z, knots(2, i));
  EXPECT_NEAR(peak_z, params.pwl_waypoint_height, 1e-6);
}

// ComputeRepositionClearance: a low, thin obstacle straddling the xy path is
// cleared by a modest cruise height (below pwl_waypoint_height), and a tall
// obstacle blocks the direct diagonal.
TEST(AdaptiveRepositionTest, ClearanceHeightClearsLowObstacle) {
  const auto params = MakeParams();
  const auto options = MakeOptions();
  const Eigen::Vector3d ee(0.05, 0.05, 0.02);
  const Eigen::Vector3d target(0.30, 0.30, 0.02);
  // Box centred on the path, top at z = 0.04.
  Scene s = MakeScene(RigidTransformd(Eigen::Vector3d(0.175, 0.175, 0.0)),
                      Eigen::Vector3d(0.05, 0.05, 0.08));

  auto [direct_clear, cruise] = ComputeRepositionClearance(
      s.query(), s.ee_id, ee, target, /*ee_radius=*/0.005, params, options);

  EXPECT_FALSE(direct_clear);  // the box sits on the straight line
  EXPECT_GT(cruise, 0.04);     // must clear the box top + margins
  EXPECT_LT(cruise, params.pwl_waypoint_height);  // but well below the cap
}

TEST(AdaptiveRepositionTest, DirectDiagonalWhenPathIsClear) {
  const auto params = MakeParams();
  const auto options = MakeOptions();
  const Eigen::Vector3d ee(0.05, 0.05, 0.08);
  const Eigen::Vector3d target(0.30, 0.30, 0.06);
  // Obstacle well below the (already high) straight-line path.
  Scene s = MakeScene(RigidTransformd(Eigen::Vector3d(0.175, 0.175, -0.05)),
                      Eigen::Vector3d(0.05, 0.05, 0.06));

  auto [direct_clear, cruise] = ComputeRepositionClearance(
      s.query(), s.ee_id, ee, target, /*ee_radius=*/0.005, params, options);
  EXPECT_TRUE(direct_clear);

  bool finished = false;
  Eigen::MatrixXd knots = Reposition(
      kNq, kNx, kN, MakeLcsState(ee), target, kDt, /*is_doing_c3=*/false,
      finished, params, options, &s.query(), s.ee_id, /*ee_radius=*/0.005);

  // A clear direct path => straight EE->target diagonal, no climb.
  const Eigen::Vector3d dir = (target - ee).normalized();
  const double zmax = std::max(ee.z(), target.z());
  for (int i = 0; i < knots.cols(); ++i) {
    const Eigen::Vector3d p = knots.col(i).head(3);
    EXPECT_LT((p - ee).cross(dir).norm(), 1e-9) << "knot " << i;
    EXPECT_LE(p.z(), zmax + 1e-9) << "knot " << i;
  }
}

// RepositionPiecewiseLinear directly: a lower cruise height is honored over a
// long horizon and never exceeded.
TEST(AdaptiveRepositionTest, PiecewiseLinearHonorsCruiseHeight) {
  const auto params = MakeParams();
  const Eigen::Vector3d ee(0.10, 0.10, 0.02);
  const Eigen::Vector3d target(0.30, 0.30, 0.02);
  const double cruise = 0.06;  // below pwl_waypoint_height (0.15)
  bool finished = false;

  Eigen::MatrixXd knots = Eigen::MatrixXd::Zero(kNx, 400);
  RepositionPiecewiseLinear(knots, 400, MakeLcsState(ee), target, kDt,
                            /*is_doing_c3=*/false, finished, params, cruise);

  double peak_z = 0.0;
  for (int i = 0; i < knots.cols(); ++i) peak_z = std::max(peak_z, knots(2, i));
  EXPECT_NEAR(peak_z, cruise, 1e-6);
  EXPECT_LT(peak_z, params.pwl_waypoint_height);
}

// When the EE already sits above the cruise height, there is no descent to it
// first -- the first move is horizontal (z stays put) and x starts advancing.
TEST(AdaptiveRepositionTest, NoNeedlessDipWhenAlreadyHigh) {
  const auto params = MakeParams();
  const Eigen::Vector3d ee(0.10, 0.10, 0.12);
  const Eigen::Vector3d target(0.30, 0.30, 0.02);
  bool finished = false;

  Eigen::MatrixXd knots = Eigen::MatrixXd::Zero(kNx, kN);
  RepositionPiecewiseLinear(knots, kN, MakeLcsState(ee), target, kDt,
                            /*is_doing_c3=*/false, finished, params,
                            /*adaptive_waypoint_height=*/0.06);

  for (int i = 0; i < knots.cols(); ++i) {
    EXPECT_GE(knots(2, i), ee.z() - 1e-9) << "knot " << i;
  }
  EXPECT_GT(knots(0, 2), ee.x());  // has started moving in x by knot 2
}

// Short hops (under 18 mm of xy travel below the cruise height) go straight to
// the target only when StraightHopIsClear.  These scenes use a 3 mm EE, so the
// hop clearance is 5 mm and the direct-path clearance 15 mm.
constexpr double kHopEERadius = 0.003;

// Distance from p to an axis-aligned box.
double DistanceToBox(const Eigen::Vector3d& p, const Eigen::Vector3d& centre,
                     const Eigen::Vector3d& size) {
  const Eigen::Vector3d outside =
      ((p - centre).cwiseAbs() - size / 2).cwiseMax(0.0);
  return outside.norm();
}

// The smallest EE-centre distance to the box along the straight segments
// between consecutive knots.
double MinPathDistanceToBox(const Eigen::MatrixXd& knots,
                            const Eigen::Vector3d& centre,
                            const Eigen::Vector3d& size) {
  double min_distance = std::numeric_limits<double>::infinity();
  for (int i = 0; i + 1 < knots.cols(); ++i) {
    const Eigen::Vector3d a = knots.col(i).head(3);
    const Eigen::Vector3d b = knots.col(i + 1).head(3);
    for (int k = 0; k <= 20; ++k) {
      min_distance = std::min(
          min_distance, DistanceToBox(a + (k / 20.0) * (b - a), centre, size));
    }
  }
  return min_distance;
}

// Whether every knot lies on the straight line from start to target.
bool KnotsOnSegment(const Eigen::MatrixXd& knots, const Eigen::Vector3d& start,
                    const Eigen::Vector3d& target) {
  const Eigen::Vector3d dir = (target - start).normalized();
  for (int i = 0; i < knots.cols(); ++i) {
    const Eigen::Vector3d p = knots.col(i).head(3);
    if ((p - start).cross(dir).norm() > 1e-9) return false;
  }
  return true;
}

// The issue-#4 regression: a short hop whose straight line runs through a thin
// wall between the EE and its target lifts first and goes over the wall,
// instead of cutting through it unchecked.
TEST(AdaptiveRepositionTest, ShortHopThroughObstacleLiftsFirst) {
  const auto params = MakeParams();
  const auto options = MakeOptions();
  // A 2 mm wall across the hop, top at z = 0.05; both ends 6 mm off its faces.
  const Eigen::Vector3d wall_centre(0.2, 0.2, 0.0);
  const Eigen::Vector3d wall_size(0.002, 0.05, 0.1);
  Scene s = MakeScene(RigidTransformd(wall_centre), wall_size);
  const Eigen::Vector3d ee(0.193, 0.2, 0.03);
  const Eigen::Vector3d target(0.207, 0.2, 0.03);

  EXPECT_FALSE(StraightHopIsClear(s.query(), s.ee_id, ee, target, kHopEERadius,
                                  params, options));

  bool finished = false;
  Eigen::MatrixXd knots = Reposition(kNq, kNx, 400, MakeLcsState(ee), target,
                                     kDt, /*is_doing_c3=*/false, finished,
                                     params, options, &s.query(), s.ee_id,
                                     kHopEERadius);

  // Lift first: knot 1 rises straight above knot 0.
  EXPECT_LT((knots.col(1).head(2) - ee.head(2)).norm(), 1e-9);
  EXPECT_GT(knots(2, 1), ee.z());
  // Over the wall, never through it, and on to the target.
  EXPECT_GE(MinPathDistanceToBox(knots, wall_centre, wall_size),
            kHopEERadius);
  EXPECT_LT((knots.col(knots.cols() - 1).head(3) - target).norm(), 1e-9);
}

// A short hop past a wall that it never comes within the hop clearance of
// still goes straight, though the wider direct-path clearance calls it
// blocked: no needless slow lift.
TEST(AdaptiveRepositionTest, ClearShortHopGoesStraight) {
  const auto params = MakeParams();
  const auto options = MakeOptions();
  // A wall parallel to the hop, its face 9 mm to the side of it.
  Scene s = MakeScene(RigidTransformd(Eigen::Vector3d(0.2, 0.21, 0.0)),
                      Eigen::Vector3d(0.05, 0.002, 0.1));
  const Eigen::Vector3d ee(0.193, 0.2, 0.03);
  const Eigen::Vector3d target(0.207, 0.2, 0.03);

  EXPECT_FALSE(ComputeRepositionClearance(s.query(), s.ee_id, ee, target,
                                          kHopEERadius, params, options)
                   .first);
  EXPECT_TRUE(StraightHopIsClear(s.query(), s.ee_id, ee, target, kHopEERadius,
                                 params, options));

  bool finished = false;
  Eigen::MatrixXd knots = Reposition(kNq, kNx, kN, MakeLcsState(ee), target,
                                     kDt, /*is_doing_c3=*/false, finished,
                                     params, options, &s.query(), s.ee_id,
                                     kHopEERadius);
  EXPECT_TRUE(KnotsOnSegment(knots, ee, target));
}

// An EE resting closer to an object than the clearance (it just let go of it,
// say) may hop straight away from it.
TEST(AdaptiveRepositionTest, ShortHopAwayFromObstacleGoesStraight) {
  const auto params = MakeParams();
  const auto options = MakeOptions();
  Scene s = MakeScene(RigidTransformd(Eigen::Vector3d(0.2, 0.2, 0.0)),
                      Eigen::Vector3d(0.002, 0.05, 0.1));
  const Eigen::Vector3d ee(0.204, 0.2, 0.03);  // 3 mm off the face
  const Eigen::Vector3d target(0.216, 0.2, 0.03);

  EXPECT_TRUE(StraightHopIsClear(s.query(), s.ee_id, ee, target, kHopEERadius,
                                 params, options));

  bool finished = false;
  Eigen::MatrixXd knots = Reposition(kNq, kNx, kN, MakeLcsState(ee), target,
                                     kDt, /*is_doing_c3=*/false, finished,
                                     params, options, &s.query(), s.ee_id,
                                     kHopEERadius);
  EXPECT_TRUE(KnotsOnSegment(knots, ee, target));
}

// But from inside the clearance it may not slide closer: a hop past the corner
// of a post it starts 4.2 mm from comes within 3 mm of it, so it lifts.
TEST(AdaptiveRepositionTest, ShortHopSlidingCloserLiftsFirst) {
  const auto params = MakeParams();
  const auto options = MakeOptions();
  const Eigen::Vector3d post_centre(0.2, 0.2, 0.0);
  const Eigen::Vector3d post_size(0.002, 0.004, 0.1);
  Scene s = MakeScene(RigidTransformd(post_centre), post_size);
  const Eigen::Vector3d ee(0.204, 0.195, 0.03);
  const Eigen::Vector3d target(0.204, 0.207, 0.03);
  ASSERT_LT(DistanceToBox(ee, post_centre, post_size), 0.005);
  ASSERT_GT(DistanceToBox(target, post_centre, post_size), 0.005);

  EXPECT_FALSE(StraightHopIsClear(s.query(), s.ee_id, ee, target, kHopEERadius,
                                  params, options));

  bool finished = false;
  Eigen::MatrixXd knots = Reposition(kNq, kNx, kN, MakeLcsState(ee), target,
                                     kDt, /*is_doing_c3=*/false, finished,
                                     params, options, &s.query(), s.ee_id,
                                     kHopEERadius);
  EXPECT_LT((knots.col(1).head(2) - ee.head(2)).norm(), 1e-9);
  EXPECT_GT(knots(2, 1), ee.z());
}

// A target that is itself inside the clearance is still reachable by a hop that
// gets no closer than the target does; otherwise every loop would lift away
// from a target the EE can never arrive at.
TEST(AdaptiveRepositionTest, ShortHopToTargetInsideClearanceGoesStraight) {
  const auto params = MakeParams();
  const auto options = MakeOptions();
  Scene s = MakeScene(RigidTransformd(Eigen::Vector3d(0.2, 0.2, 0.0)),
                      Eigen::Vector3d(0.002, 0.05, 0.1));
  const Eigen::Vector3d ee(0.212, 0.2, 0.03);     // 11 mm off the face
  const Eigen::Vector3d target(0.205, 0.2, 0.03);  // 4 mm off it

  EXPECT_TRUE(StraightHopIsClear(s.query(), s.ee_id, ee, target, kHopEERadius,
                                 params, options));
}

}  // namespace
}  // namespace systems
}  // namespace dairlib
