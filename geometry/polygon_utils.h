#pragma once

// dairlib includes
#include "geometry/convex_polygon_set.h"

// ros includes
#ifdef DAIR_ROS_ON
#include "tf2_eigen/tf2_eigen.h"
#include "convex_plane_decomposition_msgs/Polygon2d.h"
#include "convex_plane_decomposition_msgs/PlanarTerrain.h"
#endif

// drake includes
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/geometry/optimization/hpolyhedron.h"

// acd2d
#include "acd2d.h"

namespace dairlib {
namespace geometry {

#ifdef DAIR_ROS_ON
/*!
 * Makes a convex polygon by simply taking the convex hull of a non-convex
 * planar region. Useful for debugging and visualizations but not recommended
 * for footstep planning
 */
ConvexPolygon GetConvexHullOfPlanarRegion(
    const convex_plane_decomposition_msgs::PlanarRegion &foothold);

/*!
 * Main method to decompose a set of planar regions into convex footholds
 * @param terrain the terrain to be processed
 * @param concavity_threshold the maximum concavity threshold to use for ACD
 * @return Convex decomposition of the terrrain
 */
std::vector<ConvexPolygon> DecomposeTerrain(
    const convex_plane_decomposition_msgs::PlanarTerrain &terrain,
    double concavity_threshold);

/*!
 * Method to decompose a single planar region into a set of ocnvex polygons
 * @param planar_region the planar region to decompose
 * @param concavity_threshold the concavity threshold to use for ACD
 * @return Convex decomposition of the terrain as a vector of Convex Footholds
 */
std::vector<ConvexPolygon> DecomposeRegion(
    const convex_plane_decomposition_msgs::PlanarRegion &planar_region,
    double concavity_threshold);

ConvexPolygon MakeFootholdFromInscribedConvexPolygon(
    const convex_plane_decomposition_msgs::PlanarRegion &foothold);

Eigen::MatrixXd GetVerticesAsMatrix2Xd(
    const convex_plane_decomposition_msgs::Polygon2d &polygon);

std::pair<Eigen::MatrixXd, std::vector<Eigen::MatrixXd>>
GetPlanarBoundaryAndHolesFromPolygonWithHoles2d(
    const convex_plane_decomposition_msgs::PolygonWithHoles2d &foothold);
acd2d::cd_poly MakeAcdPoly(
    const convex_plane_decomposition_msgs::Polygon2d &poly2d,
    acd2d::cd_databuffer &buf,
    acd2d::cd_poly::POLYTYPE type = acd2d::cd_poly::POLYTYPE::POUT);
acd2d::cd_polygon MakeAcdPolygon(
    const convex_plane_decomposition_msgs::PolygonWithHoles2d &poly2d,
    acd2d::cd_databuffer &buf);
double PolygonArea(const convex_plane_decomposition_msgs::Polygon2d &poly);


#endif

/*!
 * Decomposes every 2D polygon in terrain, represented as the pair
 * {boundary, holes}, where every column of boundary is a vertex on the boundary
 * and every matrix in holes is similarly a list of vertices
 * (with the opposite winding order). This method is bound in
 * pydairlib.geometry.polygon_utils to enable testing the entire pipeline from
 * python.
 */
std::vector<ConvexPolygon> ProcessTerrain2d(
    std::vector<
        std::pair<Eigen::MatrixXd, std::vector<Eigen::MatrixXd>>> terrain);

/*!
 * Perform approximate convex decomposition on a polygon with holes and return
 * the resulting components as lists of vertices.
 */
std::vector<Eigen::MatrixXd> GetAcdComponents(
    std::pair<Eigen::MatrixXd, std::vector<Eigen::MatrixXd>> planar_region);

std::vector<Eigen::MatrixXd> Acd(const Eigen::MatrixXd &verts,
                                 double concavity_threshold);


ConvexPolygon MakeFootholdFromInscribedConvexPolygon(
    const Eigen::MatrixXd &verts,
    const drake::geometry::optimization::VPolytope &convex_hull,
    const Eigen::Isometry3d &X_WP);

ConvexPolygon MakeFootholdFromConvexPolytope(
    const drake::geometry::optimization::VPolytope &convex_poly2d,
    const Eigen::Isometry3d &plane_pose);

ConvexPolygon MakeFootholdFromConvexPolytope(
    const Eigen::MatrixXd &convex_poly2d,
    const Eigen::Isometry3d &plane_pose);

Eigen::VectorXd centroid(const Eigen::MatrixXd &verts);

inline bool vertex_in_poly(
    const Eigen::VectorXd &vert,
    const Eigen::MatrixXd &poly,
    double tol = 1e-4) {
  DRAKE_DEMAND(vert.rows() == poly.rows());
  for (int i = 0; i < poly.cols(); i++) {
    if (vert.isApprox(poly.col(i), tol)) { return true; }
  }
  return false;
}

inline bool is_degenerate(const Eigen::MatrixXd &verts) {
  int n = verts.cols();
  for (int i = 0; i < n; i++) {
    const auto &v = verts.col(i);
    for (int j = i + 1; j < n; j++) {
      const auto &p = verts.col(j);
      if ((v - p).squaredNorm() < 1e-10) {
        return true;
      }
    }
  }
  return false;
}

bool ValidateHoles(const Eigen::MatrixXd &boundary,
                   const std::vector<Eigen::MatrixXd> &holes);

Eigen::MatrixXd CleanOutline(const Eigen::MatrixXd &verts);

acd2d::cd_poly MakeAcdPoly(const Eigen::MatrixXd &verts,
                           acd2d::cd_databuffer &buf,
                           acd2d::cd_poly::POLYTYPE type = acd2d::cd_poly::POLYTYPE::POUT);
acd2d::cd_polygon MakeAcdPolygon(
    const std::pair<Eigen::MatrixXd, std::vector<Eigen::MatrixXd>> &poly_with_holes,
    acd2d::cd_databuffer &buf);

Eigen::MatrixXd Acd2d2Eigen(const acd2d::cd_poly &poly);

/*!
 * Calculates the area of a 2D polygon using the shoelace formula
 * @param verts Matrix where each column is the next vertex of the polygon
 * @return the area of the polygon
 */
double PolygonArea(const Eigen::MatrixXd &verts);

} // namespace geometry
} // namespace dairlib
