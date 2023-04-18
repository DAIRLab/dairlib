#pragma once

// dairlib includes
#include "geometry/convex_foothold_set.h"

// ros includes
#include "tf2_eigen/tf2_eigen.h"
#include "convex_plane_decomposition_msgs/Polygon2d.h"
#include "convex_plane_decomposition_msgs/PlanarTerrain.h"

// drake includes
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/geometry/optimization/hpolyhedron.h"

namespace dairlib::geometry {
// For now, ignore any non-convexity and take the convex hull of each region,
// ignoring holes
ConvexFoothold MakeFootholdFromConvexHullOfPlanarRegion(
    const convex_plane_decomposition_msgs::PlanarRegion &foothold);

void MaybeAddFootholdToSetFromRos(
    ConvexFootholdSet& footholds,
    const convex_plane_decomposition_msgs::PlanarRegion& foothold,
    double minimum_area = 0.2*0.2);

ConvexFoothold MakeFootholdFromInscribedConvexPolygon(
    const convex_plane_decomposition_msgs::PlanarRegion& foothold);

ConvexFoothold MakeFootholdFromInscribedConvexPolygon(
    const Eigen::MatrixXd& verts,
    const drake::geometry::optimization::VPolytope& convex_hull,
    const Eigen::Isometry3d& X_WP);

// Very quickly construct a closed convex HPolyhedron from a VPolytope which is
// known to be convex with its vertices sorted counterclockwise
// (i.e. returned from GetMinimalRepresentation())
drake::geometry::optimization::HPolyhedron
HPolyhedronFrom2dSortedConvexVPolytope(
    const drake::geometry::optimization::VPolytope& poly_in);

ConvexFoothold MakeFootholdFromConvexPolytope(
    const drake::geometry::optimization::VPolytope& convex_poly2d,
    const Eigen::Isometry3d& plane_pose);

ConvexFoothold MakeFootholdFromConvexPolytope(
    const Eigen::MatrixXd& convex_poly2d,
    const Eigen::Isometry3d& plane_pose);

Eigen::MatrixXd GetVerticesAsMatrix2Xd(
    const convex_plane_decomposition_msgs::Polygon2d &polygon);

std::pair<Eigen::MatrixXd, std::vector<Eigen::MatrixXd>>
GetPlanarBoundaryAndHolesFromRegion(
    const convex_plane_decomposition_msgs::PlanarRegion &foothold);

Eigen::VectorXd centroid(const Eigen::MatrixXd& verts);

inline bool vertex_in_poly(
    const Eigen::VectorXd& vert, const Eigen::MatrixXd& poly, double tol=1e-4) {
  DRAKE_DEMAND(vert.rows() == poly.rows());
  for (int i = 0; i < poly.cols(); i++) {
    if (vert.isApprox(poly.col(i), tol)) {return true;}
  }
  return false;
}


}