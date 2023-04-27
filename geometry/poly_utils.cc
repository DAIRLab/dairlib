#include <iostream>
#include <algorithm>
#include <memory>
#include <chrono>
#include "poly_utils.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/gurobi_solver.h"


namespace dairlib::geometry {

using Eigen::MatrixXd;
using Eigen::Matrix2d;
using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using drake::Vector1d;
using convex_plane_decomposition_msgs::PlanarRegion;
using convex_plane_decomposition_msgs::PlanarTerrain;
using convex_plane_decomposition_msgs::Point2d;
using convex_plane_decomposition_msgs::Polygon2d;
using convex_plane_decomposition_msgs::PolygonWithHoles2d;
using drake::geometry::optimization::VPolytope;
using drake::geometry::optimization::HPolyhedron;
using drake::solvers::MathematicalProgram;
using drake::solvers::VectorXDecisionVariable;
using drake::solvers::OsqpSolver;


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


HPolyhedron HPolyhedronFrom2dSortedConvexVPolytope(const VPolytope& poly_in) {
  const auto& verts = poly_in.vertices();
  DRAKE_DEMAND(verts.rows() == 2);
  MatrixXd A = MatrixXd::Zero(verts.cols(), 2);
  VectorXd b = VectorXd::Zero(verts.cols());
  for (int i = 0; i < verts.cols(); i++) {
    VectorXd v = verts.col((i + 1) % verts.cols()) - verts.col(i);
    A(i, 0) = -v(1);
    A(i, 1) = v(0);
    A.row(i).normalize();
    b.segment<1>(i) = A.row(i) * verts.col(i);
  }
  return HPolyhedron(A, b);
}

namespace {
struct facet {
  Vector2d a_;
  double b_;
  Vector2d v0_;
  Vector2d v1_;

  bool redundant(Vector2d a,  double b) {
    return (a.dot(v0_) > b and a.dot(v1_) > b);
  }

  bool intersects(Vector2d a, double b) {
    return (a.dot(v0_) > b or a.dot(v1_) > b) and not redundant(a, b);
  }

  // Should only be called when f0.intersects(a, b) and f1.intersects(a, b)
  static facet intersect(facet f0, facet f1, Vector2d a, double b) {
    Matrix2d A = Matrix2d::Zero();
    A.row(0) = f0.a_.transpose();
    A.row(1) = a.transpose();
    Vector2d v0 = A.inverse() * Vector2d(f0.b_, b);
    A.row(0) = f1.a_.transpose();
    Vector2d v1 = A.inverse() * Vector2d(f1.b_, b);
    return {a, b, v0, v1};
  }
};

std::vector<facet> FacetsFrom2dSortedConvexVPolytope(const VPolytope& poly_in) {
  const auto& verts = poly_in.vertices();
  DRAKE_DEMAND(verts.rows() == 2);
  std::vector<facet> facets;
  const int n = verts.cols();
  facets.reserve(n);
  for (int i = 0; i < n; i++) {
    VectorXd v = verts.col((i + 1) % n) - verts.col(i);
    Vector2d a(-v(1), v(0));
    a.normalize();
    double b = a.dot(verts.col(i));
    facets.push_back({a, b, verts.col(i), verts.col((i + 1) % n)});
  }
  return facets;
}

bool contained(const std::vector<facet>& facets, const Vector2d& v) {
  for (const auto& f: facets) {
    if (f.a_.dot(v) >= f.b_) {
      return false;
    }
  }
  return true;
}

void insert(std::vector<facet>& facets, Vector2d& a, double b) {
  std::vector<int> idx_redundant;
  std::vector<int> idx_intersect;
  for (int i = 0; i < facets.size(); i++) {
    if (facets.at(i).redundant(a, b)) {
      idx_redundant.push_back(i);
    } else if (facets.at(i).intersects(a, b)) {
      idx_intersect.push_back(i);
    }
  }
  if (idx_redundant.empty()) {
    int delta = idx_intersect.back() - idx_intersect.front();
    DRAKE_DEMAND(delta == 1 or delta == facets.size() - 1);
  }
  DRAKE_DEMAND(idx_intersect.size() == 2);

  int ccw_facet_idx = 0;
  int cw_facet_idx = 0;
  if (a.dot(facets.at(idx_intersect.front()).v1_) > b) {
    ccw_facet_idx = idx_intersect.front();
    cw_facet_idx = idx_intersect.back();
  } else {
    ccw_facet_idx = idx_intersect.back();
    cw_facet_idx = idx_intersect.front();
  }

  auto new_facet = facet::intersect(
      facets.at(ccw_facet_idx), facets.at(cw_facet_idx), a, b);

  facets.at(ccw_facet_idx).v1_ = new_facet.v0_;
  facets.at(cw_facet_idx).v0_ = new_facet.v1_;

  int new_facet_idx = ccw_facet_idx + 1;
  facets.insert(facets.begin() + new_facet_idx, new_facet);
  cw_facet_idx += (cw_facet_idx > ccw_facet_idx);

  for (int i = 0; i < idx_redundant.size(); i++) {
    idx_redundant.at(i) += (idx_redundant.at(i) >= new_facet_idx);
  }

  for (const auto &i : {ccw_facet_idx, cw_facet_idx}) {
    if (vertex_in_poly(facets.at(i).v0_, facets.at(i).v1_, 1e-5)) {
      idx_redundant.push_back(i);
    }
  }
  std::sort(idx_redundant.begin(), idx_redundant.end());
  for (int i = 0; i < idx_redundant.size(); i++) {
    idx_redundant.at(i) -= i;
  }
  for (const auto &i : idx_redundant) {
    facets.erase(facets.begin() + i);
  }
}
  std::pair<Vector2d, double> SolveForBestApproximateSupport(
      Vector2d p, const std::vector<facet>& facets, const MatrixXd& all_verts) {
    MatrixXd points = MatrixXd::Zero(2, facets.size());
    for (int i = 0; i < facets.size(); i++) {
      points.col(i) = facets.at(i).v1_;
    }
    const int n = all_verts.cols();
    auto prog = MathematicalProgram();
    auto solver = OsqpSolver();

    auto a = prog.NewContinuousVariables(2);
    auto u = prog.NewContinuousVariables(n);

    prog.AddQuadraticErrorCost(MatrixXd::Identity(n,n), VectorXd::Zero(n), u);
    prog.AddBoundingBoxConstraint(0, std::numeric_limits<double>::infinity(), u);
    prog.AddLinearConstraint(
        (p - centroid(points)).transpose(),
        Vector1d::Constant(0.01),
        Vector1d::Constant(std::numeric_limits<double>::infinity()),
        a);
    MatrixXd A = MatrixXd::Zero(n, n+2);
    A.rightCols(n) = -MatrixXd::Identity(n, n);
    for (int i = 0; i < all_verts.cols(); i++) {
      A.block<1,2>(i,0) = (all_verts.col(i) - p).transpose();
    }
    prog.AddLinearConstraint(
        A,
        VectorXd::Constant(n, -std::numeric_limits<double>::infinity()),
        VectorXd::Zero(n),
        {a, u});
    auto result = solver.Solve(prog);

    DRAKE_DEMAND(result.is_success());

    Vector2d a_sol = result.GetSolution(a);
    a_sol.normalize();
    return {a_sol, a_sol.dot(p)};
  }

  MatrixXd get_vertices(const std::vector<facet>& f) {
    MatrixXd verts = MatrixXd::Zero(2, f.size());
    for (int i = 0; i < f.size(); i++) {
      verts.col(i) = f.at(i).v1_;
    }
    return verts;
  }
  ConvexFoothold MakeFootholdFromFacetList(
      std::vector<facet> f, const Eigen::Isometry3d& plane_pose) {
    return MakeFootholdFromConvexPolytope(
        get_vertices(f), plane_pose);
  }
}

void MaybeAddFootholdToSetFromRos(
    ConvexFootholdSet& footholds, const PlanarRegion& foothold,
    double minimum_area) {
  Eigen::MatrixXd verts = GetVerticesAsMatrix2Xd(
      foothold.boundary.outer_boundary);
  VPolytope convex_hull_v = VPolytope(verts).GetMinimalRepresentation();

  if (convex_hull_v.CalcVolume() < minimum_area) { return; }

  Eigen::Isometry3d X_WP;
  tf2::fromMsg(foothold.plane_parameters, X_WP);
  footholds.append(MakeFootholdFromInscribedConvexPolygon(
      verts, convex_hull_v, X_WP));
}

ConvexFoothold MakeFootholdFromInscribedConvexPolygon(
    const PlanarRegion& foothold) {

  MatrixXd verts = GetVerticesAsMatrix2Xd(
      foothold.boundary.outer_boundary);
  VPolytope convex_hull_v = VPolytope(verts).GetMinimalRepresentation();

  Eigen::Isometry3d X_WP;
  tf2::fromMsg(foothold.plane_parameters, X_WP);

  return MakeFootholdFromInscribedConvexPolygon(verts, convex_hull_v, X_WP);
}

std::vector<ConvexFoothold> ProcessTerrain2d(
    std::vector<std::pair<MatrixXd, std::vector<MatrixXd>>> terrain) {

  std::unique_ptr<acd2d::IConcavityMeasure> measure = std::make_unique<acd2d::ShortestPathMeasurement>();
  acd2d::cd_2d cd;
  for (const auto& planar_region : terrain) {
    auto poly = MakeAcdPolygon(planar_region);
    cd.addPolygon(poly);
  }
  std::vector<ConvexFoothold> footholds;
  for (const auto& poly_out : cd.getDoneList()) {
    MatrixXd verts = Acd2d2Eigen(poly_out.front());
    VPolytope convex_hull_v = VPolytope(verts).GetMinimalRepresentation();
    Eigen::Isometry3d X_WP = Eigen::Isometry3d::Identity();
    footholds.push_back(MakeFootholdFromInscribedConvexPolygon(verts, convex_hull_v, X_WP));
  }
  return footholds;
}

ConvexFoothold MakeFootholdFromInscribedConvexPolygon(
    const MatrixXd& verts,
    const drake::geometry::optimization::VPolytope& convex_hull_v,
    const Eigen::Isometry3d& X_WP) {

  // initialize the inscribed polygon as the convex hull
  auto facet_list = FacetsFrom2dSortedConvexVPolytope(convex_hull_v);
  auto verts_hull = convex_hull_v.vertices();

  // For each point which is on the interior of the polygon, P, find a
  // halfspace, H which contains as many vertices of P as possible
  // (approximated with a squared hinge loss). Then update P <- intersect(P, H)
  for (int i = 0; i < verts.cols(); i++) {
    if ((not vertex_in_poly(verts.col(i), verts_hull)) and
         contained(facet_list, verts.col(i))) {
      auto [a, b] = SolveForBestApproximateSupport(
          verts.col(i), facet_list, verts);
      insert(facet_list, a, b);
    }
  }
  return MakeFootholdFromFacetList(facet_list, X_WP);
}

MatrixXd GetVerticesAsMatrix2Xd(const Polygon2d &polygon) {
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

acd2d::cd_poly MakeAcdPoly(
    const MatrixXd& verts, acd2d::cd_poly::POLYTYPE type) {
  DRAKE_DEMAND(verts.rows() == 2);
  acd2d::cd_poly poly(type);
  poly.beginPoly();
  for(int i = 0; i < verts.cols(); i++) {
    const auto& v = verts.col(i);
    poly.addVertex(v(0), v(1));
  }
  poly.endPoly();
  return poly;
}

acd2d::cd_poly MakeAcdPoly(
    const Polygon2d& poly2d, acd2d::cd_poly::POLYTYPE type) {
  acd2d::cd_poly poly(type);
  poly.beginPoly();
  for(const auto& p : poly2d.points) {
    poly.addVertex(p.x, p.y);
  }
  poly.endPoly();
  return poly;
}

acd2d::cd_polygon MakeAcdPolygon(const PolygonWithHoles2d& poly2d) {
  acd2d::cd_polygon polygon{};
  polygon.push_back(MakeAcdPoly(poly2d.outer_boundary));
  for (const auto& h : poly2d.holes) {
    polygon.push_back(MakeAcdPoly(h, acd2d::cd_poly::POLYTYPE::PIN));
  }
  return polygon;
}

acd2d::cd_polygon MakeAcdPolygon(
    const std::pair<MatrixXd, std::vector<MatrixXd>>& poly2d) {
  acd2d::cd_polygon polygon{};
  polygon.push_back(MakeAcdPoly(poly2d.first, acd2d::cd_poly::POLYTYPE::POUT));
  for (const auto& h : poly2d.second) {
    polygon.push_back(MakeAcdPoly(h, acd2d::cd_poly::POLYTYPE::PIN));
  }
  return polygon;
}

std::vector<MatrixXd> TestAcd(const MatrixXd& verts) {
  auto acd_poly = MakeAcdPoly(verts);
  acd2d::cd_polygon polygon{};
  polygon.push_back(acd_poly);
  std::unique_ptr<acd2d::IConcavityMeasure> measure = std::make_unique<acd2d::ShortestPathMeasurement>();
  acd2d::cd_2d cd;
  cd.addPolygon(polygon);
  cd.decomposeAll(0.1, measure.get());

  std::vector<MatrixXd> polylist;
  for (const auto& poly_out : cd.getDoneList()) {
    polylist.push_back(Acd2d2Eigen(poly_out.front()));
  }
  return polylist;
}

MatrixXd Acd2d2Eigen(const acd2d::cd_poly& poly) {
  int n = poly.getSize();
  DRAKE_DEMAND(n > 2);
  MatrixXd verts = MatrixXd::Zero(2, n);
  auto ptr = poly.getHead();
  DRAKE_DEMAND(ptr != nullptr);
  int i = 0;
  do {
    verts.col(i)(0) = ptr->getPos().get()[0];
    verts.col(i)(1) = ptr->getPos().get()[1];
    i++;
    ptr = ptr->getNext();
  } while (ptr != poly.getHead());
  return verts;
}

}