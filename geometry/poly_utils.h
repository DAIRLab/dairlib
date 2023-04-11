#pragma once

// dairlib includes
#include "geometry/convex_foothold_set.h"

// ros includes
#include "tf2_eigen/tf2_eigen.h"
#include "convex_plane_decomposition_msgs/Polygon2d.h"
#include "convex_plane_decomposition_msgs/PlanarTerrain.h"

// drake includes
#include "drake/geometry/optimization/vpolytope.h"

namespace dairlib::geometry {
// For now, ignore any non-convexity and take the convex hull of each region,
// ignoring holes
ConvexFoothold MakeFootholdFromConvexHullOfPlanarRegion(
    const convex_plane_decomposition_msgs::PlanarRegion &foothold);

ConvexFoothold MakeFootholdFromInnerApproximationWithIRIS(
    const convex_plane_decomposition_msgs::PlanarRegion& foothold);

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


}