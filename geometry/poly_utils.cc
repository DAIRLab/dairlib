#include <iostream>
#include "poly_utils.h"
#include "drake/geometry/optimization/iris.h"
#include "drake/geometry/optimization/point.h"


namespace dairlib::geometry {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::Vector3d;

using convex_plane_decomposition_msgs::PlanarRegion;
using convex_plane_decomposition_msgs::PlanarTerrain;
using convex_plane_decomposition_msgs::Point2d;
using convex_plane_decomposition_msgs::Polygon2d;
using drake::geometry::optimization::Iris;
using drake::geometry::optimization::IrisOptions;
using drake::geometry::optimization::Point;
using drake::geometry::optimization::VPolytope;
using drake::geometry::optimization::HPolyhedron;
using drake::geometry::optimization::ConvexSet;
using drake::geometry::optimization::ConvexSets;

ConvexFoothold MakeFootholdFromConvexHullOfPlanarRegion(
    const PlanarRegion &foothold) {
  Eigen::MatrixXd verts =
      GetVerticesAsMatrix2Xd(foothold.boundary.outer_boundary);
  VPolytope convex_hull = VPolytope(verts).GetMinimalRepresentation();
  Eigen::Isometry3d X_WP;
  tf2::fromMsg(foothold.plane_parameters, X_WP);
  return MakeFootholdFromConvexPolytope(convex_hull, X_WP);
}

ConvexFoothold MakeFootholdFromConvexPolytope(
    const VPolytope& poly2d, const Eigen::Isometry3d& plane_pose) {
  return MakeFootholdFromConvexPolytope(poly2d.vertices(), plane_pose);
}

ConvexFoothold MakeFootholdFromConvexPolytope(
    const MatrixXd& convex_poly2d, const Eigen::Isometry3d& plane_pose) {

  // Append the first vertex to the end of the list to make the H
  // representation closed
  MatrixXd convex_hull = convex_poly2d;
  convex_hull.conservativeResize(2, convex_hull.cols() + 1);
  convex_hull.rightCols(1) = convex_hull.col(0);

  auto foothold = ConvexFoothold();
  foothold.SetContactPlane(plane_pose.linear().col(2),
                           plane_pose.translation());

  for (int i = 0; i < convex_hull.cols() - 1; i++) {
    Vector3d p1 = plane_pose.translation() +
        plane_pose.linear().leftCols(2) * convex_hull.col(i);
    Vector3d p2 = plane_pose.translation() +
        plane_pose.linear().leftCols(2) * convex_hull.col(i + 1);
    foothold.AddVertices(p2, p1);
  }
  return foothold;
}

ConvexFoothold MakeFootholdFromInnerApproximationWithIRIS(
    const PlanarRegion& foothold) {

  IrisOptions opts;
  opts.require_sample_point_is_contained = true;
  opts.termination_threshold = 1e-1;

  Eigen::MatrixXd boundary_verts =
      GetVerticesAsMatrix2Xd(foothold.boundary.outer_boundary);
  VPolytope domain_v = VPolytope(boundary_verts).GetMinimalRepresentation();
  ConvexFoothold domain_f = MakeFootholdFromConvexPolytope(
      domain_v, Eigen::Isometry3d::Identity());
  const auto& [A, b] = domain_f.GetConstraintMatrices();

  HPolyhedron domain(A.leftCols<2>(), b);
  ConvexSets obstacles;
  for (int i = 0; i < boundary_verts.cols(); i++) {
    obstacles.push_back(
        drake::copyable_unique_ptr<ConvexSet>(
            std::make_unique<Point>(boundary_verts.col(i))
    ));
  }
  auto result = Iris(obstacles, centroid(boundary_verts), domain, opts);

  Eigen::Isometry3d X_WP;
  tf2::fromMsg(foothold.plane_parameters, X_WP);

  ConvexFoothold output;
  const MatrixXd& Aout = result.A();
  const VectorXd& bout = result.b();
  for(int i = 0; i < b.rows(); i++) {
    Vector3d a = Vector3d::Zero();
    a.head(2) = A.row(i).transpose();
    output.AddHalfspace(a, b.segment(i, 1));
  }
  output.SetContactPlane(Vector3d::UnitZ(), Vector3d::Zero());
  return output; // MakeFootholdFromConvexPolytope(VPolytope(domain), X_WP);
}

Eigen::MatrixXd GetVerticesAsMatrix2Xd(const Polygon2d &polygon) {
  int n = polygon.points.size();
  MatrixXd verts = MatrixXd::Zero(2, n);
  for (int i = 0; i < n; i++) {
    const auto &pt  = polygon.points.at(i);
    verts.col(i)(0) = pt.x;
    verts.col(i)(1) = pt.y;
  }
  return verts;
}

std::pair<MatrixXd, std::vector<MatrixXd>> GetPlanarBoundaryAndHolesFromRegion(
    const PlanarRegion &foothold) {

  std::vector<MatrixXd> holes{};
  for (const auto& hole: foothold.boundary.holes) {
    holes.push_back(
        GetVerticesAsMatrix2Xd(hole)
    );
  }
  return {
    GetVerticesAsMatrix2Xd(foothold.boundary.outer_boundary),
    holes
  };
}

VectorXd centroid (const MatrixXd& verts) {
  DRAKE_DEMAND(verts.size() > 0);
  const int n = verts.cols();
  const int m = verts.rows();
  VectorXd center = VectorXd::Zero(m);
  for(int i = 0; i < n; i++) {
    center += verts.col(i);
  }
  return (1.0 / static_cast<double>(n)) * center;
}


}