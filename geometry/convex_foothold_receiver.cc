#include "convex_foothold_receiver.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "tf2_eigen/tf2_eigen.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::RowVector3d;
using drake::systems::Context;

using convex_plane_decomposition_msgs::PlanarRegion;
using convex_plane_decomposition_msgs::PlanarTerrain;
using convex_plane_decomposition_msgs::Point2d;

namespace dairlib::geometry {

namespace {
// For now, ignore any non-convexity and take the convex hull of each region,
// ignoring holes
ConvexFoothold MakeFootholdFromPlanarRegion(
    const convex_plane_decomposition_msgs::PlanarRegion &foothold) {
  int n = foothold.boundary.outer_boundary.points.size();
  MatrixXd verts = MatrixXd::Zero(2, n);
  for (int i = 0; i < n; i++) {
    const auto &pt = foothold.boundary.outer_boundary.points.at(i);
    verts.col(i)(0) = pt.x;
    verts.col(i)(1) = pt.y;
  }

  MatrixXd convex_hull = drake::geometry::optimization::VPolytope(verts)
      .GetMinimalRepresentation().vertices();

  // Append the first vertex to the end of the list to make the H
  // representation closed
  convex_hull.conservativeResize(2, convex_hull.cols() + 1);
  convex_hull.rightCols(1) = convex_hull.col(0);

  Eigen::Isometry3d X_WP;
  tf2::fromMsg(foothold.plane_parameters, X_WP);
  auto converted_foothold = ConvexFoothold();
  converted_foothold.SetContactPlane(X_WP.linear().col(2), X_WP.translation());

  for (int i = 0; i < convex_hull.cols() - 1; i++) {
    Vector3d p1 = X_WP.translation() +
        X_WP.linear().leftCols(2) * convex_hull.col(i);
    Vector3d p2 = X_WP.translation() +
        X_WP.linear().leftCols(2) * convex_hull.col(i + 1);
    converted_foothold.AddVertices(p2, p1);
  }
  return converted_foothold;
}
}

ConvexFootholdReceiver::ConvexFootholdReceiver() {
  PlanarTerrain terrain_msg;
  DeclareAbstractInputPort(
      "PlanarTerrainRosMsg", drake::Value<PlanarTerrain>(terrain_msg));
  DeclareAbstractOutputPort(
      "foolholds", &ConvexFootholdReceiver::CopyTerrain);
}

void ConvexFootholdReceiver::CopyTerrain(
    const drake::systems::Context<double> &context,
    ConvexFootholdSet* footholds) const {
  footholds->clear();
  const auto& planes =
      EvalAbstractInput(context, 0)->get_value<PlanarTerrain>();
  for (const auto& planar_region : planes.planarRegions) {
    footholds->append(MakeFootholdFromPlanarRegion(planar_region));
  }
}

}