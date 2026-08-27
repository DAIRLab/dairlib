// Unit tests for the piecewise-linear repositioning knobs added for the 3D
// printer's slow z axis (see reposition_params.h / the "Faster piecewise-linear
// repositioning" plan): the adaptive cruise height in RepositionPiecewiseLinear,
// and the collision-check gating (ComputeRepositionClearance / the direct-
// diagonal route inside Reposition) exercised against a small SceneGraph.

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

}  // namespace
}  // namespace systems
}  // namespace dairlib
