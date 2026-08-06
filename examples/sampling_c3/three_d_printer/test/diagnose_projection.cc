// Standalone diagnostic: query the three_d_printer LCS plant's fixed
// obstacle geometry (ground, ramp) the same way
// SamplingC3Controller::ProjectPlanAwayFromFixedGeometries does, for a handful
// of concrete points pulled from forensic analysis of 8/5/2026 simlog-000004's
// abrupt end-effector jump at t~42.99s.  This does not replay any simulation
// state -- the fixed geometries are world-welded, so only the query points
// matter.
//
// See /home/bibit/.claude/plans/i-am-seeing-some-precious-plum.md for the full
// evidence chain this is checking.

#include <iostream>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>
#include <drake/geometry/query_object.h>
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/diagram_builder.h>

#include "examples/sampling_c3/sampling_c3_utils.h"

using drake::geometry::GeometryId;
using drake::geometry::GeometrySet;
using drake::geometry::QueryObject;
using drake::geometry::Role;
using drake::geometry::SceneGraph;
using drake::geometry::Shape;
using drake::geometry::Sphere;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::systems::DiagramBuilder;
using Eigen::Vector3d;

namespace dairlib {
namespace {

// Mirrors SamplingC3Controller::ProjectPlanAwayFromFixedGeometries exactly
// (sampling_based_c3_controller.cc:2378-2405), but for a single point, with
// verbose per-iteration printing instead of silently overwriting knots.
Vector3d ProjectPointAwayFromFixedGeometriesVerbose(
    const Vector3d& p_in, const QueryObject<double>& query_object,
    const drake::geometry::SceneGraphInspector<double>& inspector,
    const GeometrySet& fixed_obstacle_geometries, double target_clearance) {
  constexpr int kMaxProjectionIterations = 3;
  Vector3d p = p_in;
  for (int iter = 0; iter < kMaxProjectionIterations; ++iter) {
    const auto& results = query_object.ComputeSignedDistanceGeometryToPoint(
        p, fixed_obstacle_geometries);
    double min_distance = std::numeric_limits<double>::infinity();
    Vector3d push_direction = Vector3d::Zero();
    GeometryId closest_id;
    for (const auto& result : results) {
      if (result.distance < min_distance) {
        min_distance = result.distance;
        push_direction = result.grad_W;
        closest_id = result.id_G;
      }
    }
    std::cout << "    iter " << iter << ": p=" << p.transpose()
              << "  num_results=" << results.size();
    if (std::isfinite(min_distance)) {
      std::string frame_name =
          inspector.GetName(inspector.GetFrameId(closest_id));
      std::cout << "  closest_geometry=" << inspector.GetName(closest_id)
                << " (frame=" << frame_name << ")"
                << "  distance=" << min_distance
                << "  push_direction=" << push_direction.transpose();
    } else {
      std::cout << "  (no finite distance found)";
    }
    std::cout << std::endl;

    if (!std::isfinite(min_distance) || min_distance >= target_clearance ||
        push_direction.norm() < 1e-9) {
      break;
    }
    Vector3d delta = (target_clearance - min_distance) * push_direction;
    std::cout << "      correction delta=" << delta.transpose()
              << "  |delta|=" << delta.norm() << std::endl;
    p += delta;
  }
  return p;
}

void Run() {
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  std::vector<std::string> object_models = {
      "examples/sampling_c3/urdf/cone/cone_controller.sdf"};
  AddLCSModelsTo3DPrinterPlant(&plant, &scene_graph, object_models);
  plant.Finalize();

  auto diagram = builder.Build();
  auto diagram_context = diagram->CreateDefaultContext();
  auto& plant_context =
      plant.GetMyMutableContextFromRoot(diagram_context.get());

  const auto& query_object =
      plant.get_geometry_query_input_port().template Eval<QueryObject<double>>(
          plant_context);
  const auto& inspector = query_object.inspector();

  // Replicate fixed_obstacle_geometries_ construction exactly
  // (sampling_based_c3_controller.cc:439-458): exclude the EE geometry and the
  // manipulated object's ("cone") geometries; everything else with a proximity
  // role is a fixed obstacle (ground, ramp).
  GeometryId ee_geometry_id = plant.GetCollisionGeometriesForBody(
      plant.GetBodyByName("end_effector_simple"))[0];
  std::unordered_set<GeometryId> excluded_geometry_ids{ee_geometry_id};
  for (const GeometryId& id :
       plant.GetCollisionGeometriesForBody(plant.GetBodyByName("cone"))) {
    excluded_geometry_ids.insert(id);
  }
  std::vector<GeometryId> fixed_geometry_ids;
  for (const GeometryId& id : inspector.GetAllGeometryIds(Role::kProximity)) {
    if (!excluded_geometry_ids.count(id)) {
      fixed_geometry_ids.push_back(id);
      std::cout << "Fixed obstacle geometry: " << inspector.GetName(id)
                << " (frame=" << inspector.GetName(inspector.GetFrameId(id))
                << ")" << std::endl;
    }
  }
  GeometrySet fixed_obstacle_geometries(fixed_geometry_ids);

  const Shape& ee_shape = inspector.GetShape(ee_geometry_id);
  const auto* ee_sphere = dynamic_cast<const Sphere*>(&ee_shape);
  double ee_radius = ee_sphere ? ee_sphere->radius() : 0.0;
  double workspace_margins = 0.002;  // sampling_c3plus_options.yaml
  double target_clearance = workspace_margins + ee_radius;
  std::cout << "\nee_radius=" << ee_radius
            << "  workspace_margins=" << workspace_margins
            << "  target_clearance=" << target_clearance << "\n"
            << std::endl;

  // Points pulled directly from 8/5/26 simlog-000004 forensic analysis around
  // the t~42.9-42.99s jump (see the plan doc for how these were derived).
  std::vector<std::pair<std::string, Vector3d>> points = {
      {"knot0 (sane, t=42.900)", Vector3d(0.1642, 0.1136, 0.1253)},
      {"p_expected (RepositionStraightLine reproduction for knot1)",
       Vector3d(0.1647, 0.1143, 0.1190)},
      {"p_published (actual corrupted knot1)", Vector3d(0.002, 0.12, 0.0423)},
      {"knot2 (sane, t=42.900)", Vector3d(0.1653, 0.1151, 0.1105)},
      {"prev_repositioning_target (SAMPLE_LOCATIONS idx1)",
       Vector3d(0.1707011793987627, 0.12302264233330215, 0.03365664289233885)},

      // 8/5/26 simlog-000005, t=72.108s: RepositionPiecewiseLinear branch,
      // published knot 1 corrupted to [0.002, 0.12033, 0.04231]; the
      // sane/expected value (from the clean diagnose_reposition_repro_pwl run)
      // is:
      {"[log5 t=72.108] p_expected sane knot 1 (RepositionPiecewiseLinear)",
       Vector3d(0.21756, 0.103748, 0.14057)},
      {"[log5 t=72.108] p_published corrupted knot 1",
       Vector3d(0.002, 0.12033, 0.04231)},

      // 8/5/26 simlog-000006, t=54.550s: published knot 0 corrupted to
      // [0.002, 0.11995, 0.04213]; knot 0 is always an exact passthrough of
      // x_lcs.head(3) in Reposition(), so the sane/expected value is x_lcs
      // itself, captured from C3_ACTUAL at that loop:
      {"[log6 t=54.550] p_expected sane knot 0 (=x_lcs.head(3))",
       Vector3d(0.19230206310749054, 0.11390551179647446, 0.13076630234718323)},
      {"[log6 t=54.550] p_published corrupted knot 0",
       Vector3d(0.002, 0.11995, 0.04213)},

      // 8/5/26 simlog-000007: the relocated sanity check (in
      // OutputTrajExecuteActor, after
      // ProjectPlanAwayFromFixedGeometries/ClampPlanToWorkspaceLimits) caught
      // this live, with a before/after snapshot -- these are the
      // "pre-projection" (sane, per the printed trajectory) values for the
      // one knot that got corrupted in each of 3 incidents.
      {"[log7 t=14.308] pre-projection knot 5 (sane per printout)",
       Vector3d(0.235081, 0.104662, 0.146632)},
      {"[log7 t=17.239] pre-projection knot 6 (sane per printout)",
       Vector3d(0.220063, 0.117877, 0.143363)},
      {"[log7 t=17.486] pre-projection knot 3 (sane per printout)",
       Vector3d(0.220063, 0.117877, 0.143264)},

      // 8/6/26 simlog-000000: New report (post ramp-mesh fix): 4 small (~0.03m)
      // within-call jump warnings, all with is_doing_c3_=1 (C3 mode, not
      // repositioning) and no hard-failure "implausible signed distance" throw.
      // Testing the pre-projection knot pairs straddling each flagged jump to
      // see the exact closest-geometry/distance for each and whether adjacent
      // knots land on different (now convex) ramp sub-parts.
      {"[t=28.118] pre-projection knot 5",
       Vector3d(0.169193, 0.104392, 0.0112628)},
      {"[t=28.118] pre-projection knot 6",
       Vector3d(0.167553, 0.101044, 0.0100985)},
      {"[t=28.176] pre-projection knot 3",
       Vector3d(0.169991, 0.10895, 0.0166562)},
      {"[t=28.176] pre-projection knot 4",
       Vector3d(0.16644, 0.10277, 0.0137469)},
      {"[t=37.290] pre-projection knot 5",
       Vector3d(0.173454, 0.155624, 0.00778082)},
      {"[t=37.290] pre-projection knot 6",
       Vector3d(0.169734, 0.159102, 0.00790744)},
      {"[t=37.326] pre-projection knot 6",
       Vector3d(0.17608, 0.157325, 0.00785182)},
      {"[t=37.326] pre-projection knot 7",
       Vector3d(0.174043, 0.159644, 0.00800828)},
  };

  for (const auto& [label, p] : points) {
    std::cout << "=== " << label << "  p_in=" << p.transpose()
              << " ===" << std::endl;
    Vector3d p_out = ProjectPointAwayFromFixedGeometriesVerbose(
        p, query_object, inspector, fixed_obstacle_geometries,
        target_clearance);
    std::cout << "  final p_out=" << p_out.transpose()
              << "  |p_out - p_in|=" << (p_out - p).norm() << std::endl;
    std::cout << std::endl;
  }
}

}  // namespace
}  // namespace dairlib

int main() {
  dairlib::Run();
  return 0;
}
