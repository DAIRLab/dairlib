// ClearEEPlanOfFixedGeometries (reposition.h) against the cone demo's real
// ramp, built the way the controller builds its LCS scene.
//
// C3 has no EE-ramp contact, so its raw plans put the EE inside the ramp's
// walls.  Projected knot by knot, neighbouring knots inside a wall landed on
// its opposite faces and the printer drove straight through the wall between
// them while the compliant finger caught on it (2026-09-30/10-01 compliant
// sim).  These pin down the fix: the projection clamps to the workspace as it
// goes, so a knot that leaves a wall through its floor-level underside is not
// lifted back into it, and the path between knots is checked, not only the
// knots.
//
// Ramp pieces used below, in world mm (part numbers from new_ramp.urdf):
//   part 7, high side wall:  x 80-158, y 40-58, z 0-52
//   part 10, step side wall: x 158-208, a wedge whose 60 deg inner face leans
//                            out of the trough; at x 185 it spans y 40-51 at
//                            z 17 (the workspace floor) and tops out at z ~28
//   part 11, step floor:     x 158-208, y 58-83, z 0-7

#include <limits>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "examples/sampling_c3/generate_samples.h"
#include "examples/sampling_c3/parameter_headers/reposition_params.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_controller_params.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_options.h"
#include "examples/sampling_c3/parameter_headers/sampling_params.h"
#include "examples/sampling_c3/reposition.h"
#include "examples/sampling_c3/sampling_c3_utils.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/geometry_set.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"

namespace dairlib {
namespace systems {
namespace {

using drake::geometry::GeometryId;
using drake::geometry::GeometrySet;
using drake::geometry::QueryObject;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::systems::DiagramBuilder;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

constexpr double kEERadius = 0.010;
constexpr double kMargin = 0.002;      // workspace_margins
constexpr double kKnotMargin = 0.004;  // fixed_geometry_knot_margin
constexpr double kKnotClearance = kEERadius + kKnotMargin;
constexpr double kPathClearance = kEERadius + kMargin;

Eigen::VectorXd Row5(double a, double b, double c, double d, double e) {
  Eigen::VectorXd v(5);
  v << a, b, c, d, e;
  return v;
}

// The cone demo's workspace (cone/parameters/sampling_c3plus_options.yaml).
SamplingC3Options MakeOptions() {
  SamplingC3Options o{};
  o.workspace_limits = {Row5(1, 0, 0, 0.0, 0.35), Row5(0, 1, 0, 0.0, 0.35),
                        Row5(0, 0, 1, 0.015, 0.248)};
  o.workspace_margins = kMargin;
  return o;
}

Vector3d Mm(double x, double y, double z) { return 1e-3 * Vector3d(x, y, z); }

// The controller's LCS scene and its fixed geometry: every proximity geometry
// but the EE's and the cone's (sampling_based_c3_controller.cc).
class RampScene {
 public:
  RampScene() {
    const auto params = drake::yaml::LoadYamlFile<SamplingC3ControllerParams>(
        "examples/sampling_c3/three_d_printer/cone/parameters/"
        "sampling_c3_controller_params.yaml");
    auto pair = AddMultibodyPlantSceneGraph(&builder_, 0.0);
    plant_ = &pair.plant;
    AddLCSModelsTo3DPrinterPlant(plant_, &pair.scene_graph,
                                 params.object_models);
    plant_->Finalize();
    diagram_ = builder_.Build();
    diagram_context_ = diagram_->CreateDefaultContext();
    context_ =
        &diagram_->GetMutableSubsystemContext(*plant_, diagram_context_.get());

    const auto contact_pairs = BuildConeContactPairs(*plant_, params.base_names);
    std::unordered_set<GeometryId> excluded{contact_pairs.at(0).at(0).first()};
    for (const std::string& base_name : params.base_names) {
      for (const GeometryId& id : plant_->GetCollisionGeometriesForBody(
               plant_->GetBodyByName(base_name))) {
        excluded.insert(id);
      }
    }
    std::vector<GeometryId> fixed;
    for (const GeometryId& id : query_object().inspector().GetAllGeometryIds(
             drake::geometry::Role::kProximity)) {
      if (!excluded.count(id)) fixed.push_back(id);
    }
    fixed_ = GeometrySet(fixed);
  }

  const QueryObject<double>& query_object() const {
    return plant_->get_geometry_query_input_port().Eval<QueryObject<double>>(
        *context_);
  }

  // EE-centre signed distance to the nearest fixed geometry.
  double Distance(const Vector3d& p) const {
    double d = std::numeric_limits<double>::infinity();
    for (const auto& r :
         query_object().ComputeSignedDistanceGeometryToPoint(p, fixed_)) {
      d = std::min(d, r.distance);
    }
    return d;
  }

  // Smallest EE-centre distance along the plan's straight segments.
  double PathDistance(const MatrixXd& knots) const {
    double d = Distance(knots.col(0));
    for (int k = 1; k < knots.cols(); ++k) {
      for (int i = 1; i <= 100; ++i) {
        const double s = i / 100.0;
        d = std::min(d, Distance((1 - s) * knots.col(k - 1) +
                                 s * knots.col(k)));
      }
    }
    return d;
  }

  // The controller's fixed_obstacle_geometries_.
  const GeometrySet& fixed() const { return fixed_; }

  FixedGeometryPathCheck Clear(int num_exempt_knots, MatrixXd* knots) const {
    return ClearEEPlanOfFixedGeometries(query_object(), fixed_, kKnotClearance,
                                        kPathClearance, MakeOptions(),
                                        num_exempt_knots, knots);
  }

 private:
  DiagramBuilder<double> builder_;
  MultibodyPlant<double>* plant_{};
  std::unique_ptr<drake::systems::Diagram<double>> diagram_;
  std::unique_ptr<drake::systems::Context<double>> diagram_context_;
  drake::systems::Context<double>* context_{};
  GeometrySet fixed_;
};

MatrixXd Knots(const std::vector<Vector3d>& points) {
  MatrixXd knots(3, points.size());
  for (size_t i = 0; i < points.size(); ++i) knots.col(i) = points[i];
  return knots;
}

// The scene is what the comments above say it is.
TEST(FixedGeometryPathTest, RampPiecesAreWhereTheTestsExpect) {
  RampScene scene;
  // Inside the walls, the trough floor, and clear above the step floor.
  EXPECT_LT(scene.Distance(Mm(120, 49, 20)), 0.0);   // part 7
  EXPECT_LT(scene.Distance(Mm(185, 49, 20)), 0.0);   // part 10
  EXPECT_LT(scene.Distance(Mm(185, 70, 3)), 0.0);    // part 11
  EXPECT_NEAR(scene.Distance(Mm(185, 70, 21)), 0.014, 1e-3);
}

// A raw C3 knot low inside a wall (log 10_01/000017 at 183.8 s, C3 at z 6 mm)
// is nearest the wall's underside.  Projected out through it and only then
// clamped back up to the workspace floor, it landed 7 mm inside the wall.
TEST(FixedGeometryPathTest, KnotLowInsideAWallLeavesItSideways) {
  RampScene scene;
  MatrixXd knots = Knots({Mm(146, 46, 6)});
  scene.Clear(0, &knots);
  EXPECT_GE(scene.Distance(knots.col(0)), kKnotClearance - 1e-6);
  EXPECT_GE(knots(2, 0), 0.015 + kMargin - 1e-9);  // Within the workspace.
}

// Two neighbouring knots inside the same wall, nearer its opposite faces,
// project out of opposite faces: the path between them runs through the wall.
TEST(FixedGeometryPathTest, KnotsSplitAcrossAWallAreCaught) {
  RampScene scene;
  // Over the step floor, then low inside the step's side wall: first nearer
  // its inner face, then nearer its outer face.
  MatrixXd knots =
      Knots({Mm(185, 70, 21), Mm(185, 49, 17), Mm(185, 43, 17),
             Mm(185, 42, 17)});
  const FixedGeometryPathCheck check = scene.Clear(0, &knots);
  // Projection alone leaves every knot clear ...
  for (int k = 0; k < knots.cols(); ++k) {
    EXPECT_GE(scene.Distance(knots.col(k)), kKnotClearance - 1e-6);
  }
  // ... with the path through the wall between them.
  ASSERT_LT(scene.PathDistance(knots), 0.0);
  EXPECT_EQ(check.first_blocked_knot, 2);
  EXPECT_LT(check.blocked_distance, kPathClearance);
  // The hold point is on the trough side of the wall, and clear.
  EXPECT_GT(check.last_clear_point(1), 0.058);
  EXPECT_GE(scene.Distance(check.last_clear_point), kKnotClearance - 1e-6);

  HoldEEPlanFrom(check.first_blocked_knot, check.last_clear_point, &knots);
  EXPECT_GE(scene.PathDistance(knots), kPathClearance - 1e-4);
  // Holding stops short of the wall rather than turning back.
  EXPECT_LT((knots.col(3) - check.last_clear_point).norm(), 1e-12);
}

// A path that only skims the ramp is left alone, and clearing a cleared plan
// changes nothing.
TEST(FixedGeometryPathTest, ClearPathIsUntouchedAndClearingIsIdempotent) {
  RampScene scene;
  // Along the trough over the step floor, then up the wall's inner side.
  MatrixXd knots = Knots({Mm(205, 70, 25), Mm(195, 70, 25), Mm(185, 70, 25),
                          Mm(175, 72, 30), Mm(165, 74, 40)});
  const MatrixXd original = knots;
  const FixedGeometryPathCheck check = scene.Clear(0, &knots);
  EXPECT_EQ(check.first_blocked_knot, -1);
  EXPECT_TRUE(knots.isApprox(original, 1e-12));

  // A held plan, cleared again.
  MatrixXd held = Knots({Mm(185, 70, 21), Mm(185, 49, 17), Mm(185, 43, 17),
                         Mm(185, 42, 17)});
  const FixedGeometryPathCheck first = scene.Clear(0, &held);
  ASSERT_GE(first.first_blocked_knot, 0);
  HoldEEPlanFrom(first.first_blocked_knot, first.last_clear_point, &held);
  const MatrixXd once = held;
  EXPECT_EQ(scene.Clear(0, &held).first_blocked_knot, -1);
  EXPECT_TRUE(held.isApprox(once, 1e-12));
}

// A jam-unload leg retraces the way the EE came in, which can run back
// through a wall the bent finger carried the gantry across.  Its knots are
// left where they are, and the path after it may climb out of the wall.
TEST(FixedGeometryPathTest, ExemptLeadingKnotsAreNotMoved) {
  RampScene scene;
  // Back from the far side of the step's side wall to inside it, then on out
  // of its outer face.
  MatrixXd knots = Knots({Mm(185, 62, 21), Mm(185, 45, 17), Mm(185, 35, 17),
                          Mm(185, 25, 17)});
  const MatrixXd original = knots;
  ASSERT_LT(scene.Distance(original.col(1)), 0.0);
  const FixedGeometryPathCheck check = scene.Clear(2, &knots);
  EXPECT_TRUE(knots.leftCols(2).isApprox(original.leftCols(2), 1e-12));
  EXPECT_EQ(check.first_blocked_knot, -1);
  EXPECT_GE(scene.Distance(knots.col(2)), kKnotClearance - 1e-6);
}

// A point straight above the step floor whose EE surface clears it by @p gap.
Vector3d AboveStepFloor(const RampScene& scene, double gap) {
  Vector3d p = Mm(185, 70, 21);
  p(2) -= scene.Distance(p) - (kEERadius + gap);
  return p;
}

// The controller's repositioning loop, with the printer tracking perfectly:
// plan from x0 to the target, clear the plan's path, and start the next loop
// from the cleared plan's knot 1 (use_predicted_x0_repos).  Returns the loop on
// which Reposition() first reports the target reached, or -1.
int LoopsToFinish(const RampScene& scene, Vector3d x0, const Vector3d& target,
                  int max_loops) {
  constexpr int kNq = 10;  // 3 EE + 7 object (quat + xyz)
  constexpr int kNx = 19;  // + 3 EE vel + 6 object vel
  constexpr int kN = 10;
  constexpr double kDt = 0.075;  // planning_dt_position
  const auto params = drake::yaml::LoadYamlFile<SamplingC3RepositionParams>(
      "examples/sampling_c3/three_d_printer/printer_shared_parameters/"
      "reposition_params.yaml");
  for (int loop = 0; loop < max_loops; ++loop) {
    VectorXd x = VectorXd::Zero(kNx);
    x.head(3) = x0;
    x.segment(kNq - 3, 3) = Mm(230, 70, 25);  // The cone, out of the way.
    bool finished = false;
    MatrixXd knots = Reposition(kNq, kNx, kN, x, target, kDt,
                                /*is_doing_c3=*/false, finished, params,
                                MakeOptions(), /*query_object=*/nullptr,
                                GeometryId::get_new_id(), kEERadius);
    MatrixXd ee_knots = knots.topRows(3);
    if (scene.Clear(0, &ee_knots).first_blocked_knot >= 0) finished = false;
    if (finished) return loop;
    x0 = ee_knots.col(1);
  }
  return -1;
}

// Samples used to be accepted 2 mm off the ramp while plan knots are held
// 4 mm off it.  A repositioning target in between is never reached:  the EE
// parks 2 mm short, more than one 75 ms knot of the 15 mm/s z axis, so
// Reposition() never reports it finished and the controller repositions until
// something else changes (2026-10-01 compliant sims 20, 25 and 26: 46-129 s).
// Planning to the projected target arrives.
TEST(FixedGeometryPathTest, TargetInsideTheKnotClearanceIsReachedOnceProjected) {
  RampScene scene;
  const Vector3d target = AboveStepFloor(scene, 0.0021);
  ASSERT_NEAR(scene.Distance(target), kEERadius + 0.0021, 1e-6);
  const Vector3d start = target + Mm(0, 0, 15);

  // The stall, reproduced.
  EXPECT_EQ(LoopsToFinish(scene, start, target, 100), -1);

  Vector3d reachable = target;
  ProjectEEPositionOffFixedGeometries(scene.query_object(), scene.fixed(),
                                      kKnotClearance, MakeOptions(),
                                      &reachable);
  EXPECT_NEAR(scene.Distance(reachable), kKnotClearance, 1e-6);
  EXPECT_LT((reachable - target).norm(), 0.0021);
  const int loops = LoopsToFinish(scene, start, reachable, 100);
  EXPECT_GE(loops, 0);
  EXPECT_LE(loops, 20);  // 15 mm down at 15 mm/s is 13 loops.

  // A target already clear of the knot clearance is left alone.
  Vector3d clear = AboveStepFloor(scene, 0.006);
  const Vector3d clear_before = clear;
  ProjectEEPositionOffFixedGeometries(scene.query_object(), scene.fixed(),
                                      kKnotClearance, MakeOptions(), &clear);
  EXPECT_TRUE(clear.isApprox(clear_before, 1e-12));
}

// A jam escape that runs into the fixed scene is projected back onto its
// surface knot by knot, so the plan -- rebuilt from the same x0 every loop --
// never moves (2026-10-02 compliant sim 0: the gantry parked on the knot
// clearance above the step floor for 93 s, its escape pointing down).  Such a
// step is reported blocked; one along the surface or away from it is not.
TEST(FixedGeometryPathTest, AnEscapeIntoTheSceneIsBlocked) {
  RampScene scene;
  const Vector3d start = AboveStepFloor(scene, kKnotMargin);
  constexpr double kStep = 0.009;  // one 75 ms knot at 0.12 m/s
  const auto blocked = [&](const Vector3d& direction) {
    return RetreatIsBlockedByFixedGeometries(scene.query_object(),
                                             scene.fixed(), kKnotClearance,
                                             MakeOptions(), start, direction,
                                             kStep);
  };
  EXPECT_TRUE(blocked(Vector3d(0, 0, -1)));
  EXPECT_TRUE(blocked(Vector3d(0.2, 0, -1)));
  EXPECT_FALSE(blocked(Vector3d(1, 0, -0.1)));  // along the floor
  EXPECT_FALSE(blocked(Vector3d(0, 0, 1)));
  EXPECT_FALSE(blocked(Vector3d::Zero()));
}

// Samples are accepted no closer to the fixed scene than the plans' knots may
// go, so the controller never picks a target it cannot reach.
TEST(FixedGeometryPathTest, SamplesKeepTheKnotClearance) {
  RampScene scene;
  SamplingParams sampling_params{};
  sampling_params.avoid_sampling_within_fixed_environment_geometries = true;
  SamplingC3Options options = MakeOptions();
  options.robot_radius_limits = {0.0, 10.0};
  const auto acceptable = [&](double gap) {
    VectorXd sample = VectorXd::Zero(10);
    sample.head(3) = AboveStepFloor(scene, gap);
    return SampleIsAcceptable(sample, sampling_params, options,
                              MatrixXd::Zero(0, 10), scene.query_object(),
                              scene.fixed(), kEERadius);
  };

  // Path check off: knots are projected workspace_margins off at publish.
  EXPECT_DOUBLE_EQ(options.FixedGeometryKnotMargin(), kMargin);
  options.fixed_geometry_knot_margin = kKnotMargin;
  EXPECT_DOUBLE_EQ(options.FixedGeometryKnotMargin(), kMargin);
  EXPECT_FALSE(acceptable(0.0015));
  EXPECT_TRUE(acceptable(0.003));

  // Path check on, as in the cone demo.
  options.check_fixed_geometry_paths = true;
  EXPECT_DOUBLE_EQ(options.FixedGeometryKnotMargin(), kKnotMargin);
  EXPECT_FALSE(acceptable(0.0021));
  EXPECT_FALSE(acceptable(0.003));
  EXPECT_TRUE(acceptable(0.0045));
}

}  // namespace
}  // namespace systems
}  // namespace dairlib
