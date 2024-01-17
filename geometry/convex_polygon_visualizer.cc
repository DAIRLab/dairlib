#include "convex_polygon_visualizer.h"

// STL
#include <utility>

//drake
#include "drake/geometry/shape_specification.h"
#include "drake/geometry/rgba.h"


namespace dairlib::geometry{

using geometry::ConvexPolygon;
using geometry::ConvexPolygonSet;
using Eigen::Matrix3d;
using Eigen::Matrix3Xd;

ConvexPolygonVisualizer::ConvexPolygonVisualizer(
    std::shared_ptr<drake::geometry::Meshcat> meshcat,
    double update_period_sec) :
    meshcat_(std::move(meshcat)) {

  polygon_input_port_ = DeclareAbstractInputPort(
      "ConvexPolygonSet", drake::Value<ConvexPolygonSet>()
  ).get_index();

  n_polygons_idx_ = DeclareDiscreteState(1);

  if (update_period_sec > 0) {
    DeclarePeriodicUnrestrictedUpdateEvent(
        update_period_sec, 0., &ConvexPolygonVisualizer::UnrestrictedUpdate
    );
  } else {
    DeclareForcedUnrestrictedUpdateEvent(
        &ConvexPolygonVisualizer::UnrestrictedUpdate
    );
  }

}

void ConvexPolygonVisualizer::DrawPolygons(ConvexPolygonSet& foothold_set,
                                            int n_prev,
                                            const std::string& prefix) const {
  drake::geometry::Rgba green(1, 0, 0, 0.5);
  drake::geometry::Rgba black(0, 0, 0, 0);

  for (int i = 0; i < foothold_set.size(); i++) {
    auto polygon = foothold_set.polygons().at(i);
    const auto [verts, faces] = polygon.GetSurfaceMesh();
    Eigen::Matrix3Xd verts_line = verts;
    verts_line.rightCols<1>() = verts_line.leftCols<1>();

    meshcat_->SetTriangleMesh(
        prefix + make_polygon_path(i) + "top", verts, faces, green
    );
    meshcat_->SetLine(
        prefix + make_polygon_path(i) + "boundary", verts_line, 2.0, black
    );
  }
  for (int i = foothold_set.size(); i < n_prev; i++) {
    meshcat_->Delete(prefix + make_polygon_path(i) + "top");
    meshcat_->Delete(prefix + make_polygon_path(i) + "boundary");
  }
}

drake::systems::EventStatus ConvexPolygonVisualizer::UnrestrictedUpdate(
    const drake::systems::Context<double> &context,
    drake::systems::State<double> *state) const {


  auto polygons = EvalAbstractInput(
      context, polygon_input_port_)->get_value<ConvexPolygonSet>();

  int n_prev = state->get_discrete_state(n_polygons_idx_).get_value()(0);
  DrawPolygons(polygons, n_prev);
  state->get_mutable_discrete_state(n_polygons_idx_).set_value(
      Eigen::VectorXd::Constant(1, polygons.size())
  );
  return drake::systems::EventStatus::Succeeded();
}

}