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

// acd2d
#include "acd2d.h"

namespace dairlib::geometry {
// For now, ignore any non-convexity and take the convex hull of each region,
// ignoring holes
ConvexFoothold MakeFootholdFromConvexHullOfPlanarRegion(
    const convex_plane_decomposition_msgs::PlanarRegion &foothold);

// function to bind for testing entire polygon pipeline in python
std::vector<ConvexFoothold> ProcessTerrain2d(
    std::vector<std::pair<Eigen::MatrixXd, std::vector<Eigen::MatrixXd>>> terrain);

std::vector<ConvexFoothold> DecomposeTerrain(
    const convex_plane_decomposition_msgs::PlanarTerrain& terrain);
std::vector<ConvexFoothold> DecomposeTerrain(
    const convex_plane_decomposition_msgs::PlanarRegion& planar_region);

std::vector<Eigen::MatrixXd> TestAcd(const Eigen::MatrixXd& verts);

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
GetPlanarBoundaryAndHolesFromPolygonWithHoles2d(
    const convex_plane_decomposition_msgs::PolygonWithHoles2d &foothold);

Eigen::VectorXd centroid(const Eigen::MatrixXd& verts);

inline bool vertex_in_poly(
    const Eigen::VectorXd& vert, const Eigen::MatrixXd& poly, double tol=1e-4) {
  DRAKE_DEMAND(vert.rows() == poly.rows());
  for (int i = 0; i < poly.cols(); i++) {
    if (vert.isApprox(poly.col(i), tol)) {return true;}
  }
  return false;
}

inline bool is_degenerate(const Eigen::MatrixXd& verts) {
  int n = verts.cols();
  for (int i = 0; i < n; i++) {
    const auto& v = verts.col(i);
    for (int j = i+1; j < n;  j++) {
      const auto& p = verts.col(j);
      if ((v - p).squaredNorm() < 1e-10) {
        return true;
      }
    }
  }
  return false;
}

bool ValidateHoles(const Eigen::MatrixXd& boundary,
                   const std::vector<Eigen::MatrixXd>& holes);

Eigen::MatrixXd CleanOutline(const Eigen::MatrixXd& verts);

acd2d::cd_poly MakeAcdPoly(const Eigen::MatrixXd& verts,
                           acd2d::cd_databuffer& buf,
                           acd2d::cd_poly::POLYTYPE type = acd2d::cd_poly::POLYTYPE::POUT);
acd2d::cd_poly MakeAcdPoly(
    const convex_plane_decomposition_msgs::Polygon2d& poly2d,
    acd2d::cd_databuffer& buf,
    acd2d::cd_poly::POLYTYPE type  = acd2d::cd_poly::POLYTYPE::POUT);
acd2d::cd_polygon MakeAcdPolygon(
    const convex_plane_decomposition_msgs::PolygonWithHoles2d& poly2d,
    acd2d::cd_databuffer& buf);
acd2d::cd_polygon MakeAcdPolygon(
    const std::pair<Eigen::MatrixXd, std::vector<Eigen::MatrixXd>>& poly_with_holes,
    acd2d::cd_databuffer& buf);

Eigen::MatrixXd Acd2d2Eigen(const acd2d::cd_poly& poly);

}