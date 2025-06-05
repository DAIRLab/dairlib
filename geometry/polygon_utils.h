#pragma once

// dairlib includes
#include "geometry/convex_polygon_set.h"

// drake includes
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/geometry/optimization/hpolyhedron.h"

namespace dairlib {
namespace geometry {

/*!
 * Decomposes every 2D polygon in terrain, represented as the pair
 * {boundary, holes}, where every column of boundary is a vertex on the boundary
 * and every matrix in holes is similarly a list of vertices
 * (with the opposite winding order). This method is bound in
 * pydairlib.geometry.polygon_utils to enable testing the entire pipeline from
 * python.
 */
std::vector<ConvexPolygon> ProcessTerrain2d(
    const std::vector<std::pair<Eigen::MatrixXd, std::vector<Eigen::MatrixXd>>>& terrain,
    double convexity_thresh);

/*!
 * Perform approximate convex decomposition on a polygon with holes and return
 * the resulting components as lists of vertices.
 */
std::vector<Eigen::MatrixXd> GetAcdComponents(
    const std::vector<std::pair<Eigen::MatrixXd, std::vector<Eigen::MatrixXd>>>& terrain,
    double concavity_thresh=0.25);

/*!
 * Use the whittling algorithm to find an inscribed convex polygon for an input
 * polygon given by a list of vertices
 * @param verts vertices of the original non-convex polygon.
 * @param convex_hull convecx hull of the original non-convex polygon.
 * @return ConvexPolygon representing the inscribed convex polygon.
 */
ConvexPolygon MakeInscribedConvexPolygon(
    const Eigen::MatrixXd &verts,
    const drake::geometry::optimization::VPolytope &convex_hull);


} // namespace geometry
} // namespace dairlib
