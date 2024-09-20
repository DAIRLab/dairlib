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

/*!
 * Convenience struct to represent a polygon as a list of facets
 * for fast halfspace intersections.
 */
struct facet {
  Eigen::Vector2d a_;
  double b_;
  Eigen::Vector2d v0_;
  Eigen::Vector2d v1_;

  bool redundant(Eigen::Vector2d a,  double b) {
    return (a.dot(v0_) > b and a.dot(v1_) > b);
  }

  bool intersects(Eigen::Vector2d a, double b) {
    return (a.dot(v0_) > b or a.dot(v1_) > b) and not redundant(a, b);
  }

  // Should only be called when f0.intersects(a, b) and f1.intersects(a, b)
  static facet intersect(facet f0, facet f1, Eigen::Vector2d a, double b) {
    assert(f0.intersects(a, b) and f0.intersects(a, b));
    Eigen::Matrix2d A = Eigen::Matrix2d::Zero();
    A.row(0) = f0.a_.transpose();
    A.row(1) = a.transpose();
    Eigen::Vector2d v0 = A.inverse() * Eigen::Vector2d(f0.b_, b);
    A.row(0) = f1.a_.transpose();
    Eigen::Vector2d v1 = A.inverse() * Eigen::Vector2d(f1.b_, b);
    return {a, b, v0, v1};
  }
};

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
    const std::pair<Eigen::MatrixXd, std::vector<Eigen::MatrixXd>>& planar_region,
    double concavity_thresh=0.25);

ConvexPolygon MakeInscribedConvexPolygon(
    const Eigen::MatrixXd &verts,
    const drake::geometry::optimization::VPolytope &convex_hull);

ConvexPolygon MakeFootholdFromConvexPolygon(
    const Eigen::MatrixXd &vertices);

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
