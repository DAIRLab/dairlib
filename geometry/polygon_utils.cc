#include <iostream>
#include <algorithm>
#include <memory>
#include <chrono>
#include <cmath>

#include "polygon_utils.h"
#include "whittling_solver.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/gurobi_solver.h"


namespace dairlib::geometry {

using Eigen::MatrixXd;
using Eigen::Matrix2d;
using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using drake::Vector1d;

using drake::geometry::optimization::VPolytope;
using drake::geometry::optimization::HPolyhedron;
using drake::solvers::MathematicalProgram;
using drake::solvers::VectorXDecisionVariable;
using drake::solvers::OsqpSolver;

ConvexPolygon MakeFootholdFromConvexPolygon(const MatrixXd& convex_poly2d) {
  // Append the first vertex to the end of the list to make the H
  // representation closed
  MatrixXd convex_hull = convex_poly2d;
  convex_hull.conservativeResize(3, convex_hull.cols() + 1);
  convex_hull.bottomRows<1>().setZero();
  convex_hull.rightCols(1) = convex_hull.col(0);

  ConvexPolygon poly_out{};
  poly_out.SetPlane(Vector3d::UnitZ(), 0);

  for (int i = 0; i < convex_hull.cols() - 1; i++) {
    poly_out.AddVertices(convex_hull.col(i), convex_hull.col(i+1));
  }
  return poly_out;
}


namespace {
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

double distance_to_boundary(
    const std::vector<facet>& facets, const Vector2d& v) {
  double min_distance = std::numeric_limits<double>::max();
  for (const auto& f : facets) {
    double d = f.b_ - f.a_.dot(v);
    if (d <= 0){
      return -1;
    }
    if (d <= min_distance) {
      min_distance = d;
    }
  }
  return min_distance;
}

 Vector2d normal_of_closest_face(
    const std::vector<facet>& facets, const Vector2d& v) {
  DRAKE_DEMAND(contained(facets, v));
  double min_distance = std::numeric_limits<double>::max();
  Vector2d normal = Vector2d::Zero();
   for (const auto& f : facets) {
     double d = f.b_ - f.a_.dot(v);
     if (d <= min_distance) {
       min_distance = d;
       normal = f.a_;
     }
   }

   DRAKE_DEMAND(not normal.cwiseEqual(Vector2d::Zero()).all());
   return normal;
}

vector<Vector2d> get_sorted_interior_points(
    const std::vector<facet>& facets, const MatrixXd& verts) {
  typedef std::pair<int, double> idx_dist;

  std::vector<idx_dist> vert_distances;
  for (int j = 0; j < verts.cols(); j++) {
    vert_distances.push_back({j, distance_to_boundary(facets, verts.col(j))});
  }

  std::sort(vert_distances.begin(), vert_distances.end(),
            [](const idx_dist& p1, const idx_dist& p2) {
    return p1.second > p2.second;
  });

  int i = 0;
  std::vector<Vector2d> verts_sorted;
  while (i < vert_distances.size() and vert_distances.at(i).second > 0) {
    verts_sorted.push_back(verts.col(vert_distances[i].first));
    ++i;
  }
  return verts_sorted;
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
  } else if (idx_intersect.empty()) {
    DRAKE_DEMAND(idx_redundant.size() == 0);
    return;
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

  DRAKE_DEMAND(not new_facet.v0_.hasNaN());
  DRAKE_DEMAND(not new_facet.v1_.hasNaN());

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

std::pair<Vector2d, double> GetBestSupportAsRayToBoundary(
    Vector2d p, const std::vector<facet>& facets ) {
  Vector2d a =  normal_of_closest_face(facets, p);
  double b = a.dot(p);
  return {a, b};
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

  DRAKE_DEMAND(not a_sol.hasNaN());

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

ConvexPolygon MakeFootholdFromFacetList(const std::vector<facet>& f) {
  ConvexPolygon poly;
  poly.SetPlane(Vector3d::UnitZ(), Vector3d::Zero());
  for (const auto& facet : f) {
    Vector3d face = Vector3d::Zero();
    face.head<2>() = facet.a_;
    poly.AddHalfspace(face, VectorXd::Constant(1, facet.b_));
  }
  return poly;
}
}

ConvexPolygon MakeInscribedConvexPolygon(
    const MatrixXd& verts,
    const drake::geometry::optimization::VPolytope& convex_hull_v) {

  // initialize the inscribed polygon as the convex hull
  std::vector<facet> facets = FacetsFrom2dSortedConvexVPolytope(convex_hull_v);
  std::vector<facet> facet_hull = facets;

  auto verts_sorted = get_sorted_interior_points(facets, verts);

  WhittlingSolver solver;

  while (not verts_sorted.empty()) {
    // To avoid numerical issues, ignore violations of less than 1mm
    if (distance_to_boundary(facets, verts_sorted.front()) < 0.001) {
      break;
    }

//    auto start = std::chrono::high_resolution_clock::now();
//    auto [a, b] = GetBestSupportAsRayToBoundary(
//        verts_sorted.front(), facet_hull);

//    auto [a, b] = SolveForBestApproximateSupport(
//        verts_sorted.front(), facets, verts);

    auto [a, b] = solver.SolveForBestCut(
        verts_sorted.front(),
        verts,
        normal_of_closest_face(facets, verts_sorted.front())
    );

//    auto end = std::chrono::high_resolution_clock::now();
    insert(facets, a, b);
    verts_sorted = get_sorted_interior_points(facets, verts);
  }
  return MakeFootholdFromFacetList(facets);
}

std::vector<MatrixXd> GetAcdComponents(
    const std::pair<MatrixXd, std::vector<MatrixXd>>& planar_region,
    double convexity_thresh) {
  std::unique_ptr<acd2d::IConcavityMeasure> measure =
      std::make_unique<acd2d::StraightLineMeasurement>();
  acd2d::cd_2d cd;

  acd2d::cd_polygon poly;
  if (ValidateHoles(planar_region.first, planar_region.second)) {
    poly = MakeAcdPolygon(planar_region, cd.buf());
  } else {
    poly = MakeAcdPolygon({planar_region.first, {}}, cd.buf());
  }
  cd.addPolygon(poly);

  std::vector<MatrixXd> polylist;
  for (const auto& poly_out : cd.getDoneList()) {
    auto poly_eigen = Acd2d2Eigen(poly_out.front());
    // Only push back valid polygons (3 or more vertices)
    if (poly_eigen.cols() > 2) {
      polylist.push_back(poly_eigen);
    }
  }
  return polylist;
}

std::vector<ConvexPolygon> ProcessTerrain2d(
    const std::vector<std::pair<MatrixXd, std::vector<MatrixXd>>>& terrain,
    double convexity_thresh) {
  std::unique_ptr<acd2d::IConcavityMeasure> measure = std::make_unique<acd2d::StraightLineMeasurement>();
  acd2d::cd_2d cd;
  for (const auto& planar_region : terrain) {
    if (is_degenerate(planar_region.first)) {
      continue;
    }
    double area = PolygonArea(planar_region.first);
    if (area < 0.05) {
      continue;
    }

    acd2d::cd_polygon poly;
    if (ValidateHoles(planar_region.first, planar_region.second)) {
      poly = MakeAcdPolygon(planar_region, cd.buf());
    } else {
      poly = MakeAcdPolygon({planar_region.first, {}}, cd.buf());
    }
    cd.addPolygon(poly);
  }

  try {
    cd.maybe_decomposeAll(convexity_thresh, measure.get());
  } catch (const std::exception& e) {
    std::cout << e.what() << std::endl;
    return {};
  }
  std::vector<ConvexPolygon> footholds;

  for (const auto& poly_out : cd.getDoneList()) {
    MatrixXd verts = Acd2d2Eigen(poly_out.front());
    if (verts.cols() < 3) {
      continue;
    }
    VPolytope convex_hull_v = VPolytope(verts).GetMinimalRepresentation();
    footholds.push_back(MakeInscribedConvexPolygon(verts, convex_hull_v));
  }

  return footholds;
}

VectorXd centroid (const MatrixXd& verts) {
  DRAKE_DEMAND(verts.size() > 0);
  const int n = verts.cols();
  const int m = verts.rows();
  double sum_weight = 0;
  VectorXd center = VectorXd::Zero(m);
  for(int i = 0; i < n; i++) {
    int idx_prev = (i - 1 < 0) ? n - 1 : i - 1;
    int idx_next = (i + 1) % n;
    double w = (verts.col(i) - verts.col(idx_prev)).norm() +
               (verts.col(i) - verts.col(idx_next)).norm();
    center += w * verts.col(i);
    sum_weight += w;
  }
  return (1.0 / sum_weight) * center;
}

acd2d::cd_poly MakeAcdPoly(
    const MatrixXd& verts,
    acd2d::cd_databuffer& buf, acd2d::cd_poly::POLYTYPE type) {
  DRAKE_DEMAND(verts.rows() == 2);
  acd2d::cd_poly poly(type);
  poly.beginPoly();
  for(int i = 0; i < verts.cols(); i++) {
    const auto& v = verts.col(i);
    poly.addVertex(buf, v(0), v(1));
  }
  poly.endPoly();
  return poly;
}

acd2d::cd_polygon MakeAcdPolygon(
    const std::pair<MatrixXd, std::vector<MatrixXd>>& poly2d,
    acd2d::cd_databuffer& buf) {
  acd2d::cd_polygon polygon{};
  polygon.push_back(MakeAcdPoly(
      poly2d.first, buf, acd2d::cd_poly::POLYTYPE::POUT));
  for (const auto& h : poly2d.second) {
    polygon.push_back(MakeAcdPoly(h, buf, acd2d::cd_poly::POLYTYPE::PIN));
  }
  return polygon;
}

MatrixXd Acd2d2Eigen(const acd2d::cd_poly& poly) {
  int n = poly.getSize();
  if (n < 3) {
    return MatrixXd::Zero(2, 0);
  }
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

bool ValidateHoles(const MatrixXd& boundary, const std::vector<MatrixXd>& holes) {
  for (const auto& hole : holes) {
    auto v = VPolytope(hole).GetMinimalRepresentation();
    auto facets = FacetsFrom2dSortedConvexVPolytope(v);
    for (int i  = 0; i < boundary.cols(); i++) {
      if (contained(facets, boundary.col(i))) {
        return false;
      }
    }
  }
  return true;
}

double PolygonArea(const MatrixXd& verts) {
  // https://en.wikipedia.org/wiki/Shoelace_formula
  double a_sum = 0;
  int n = verts.cols();

  // Note: This should also be correct for c = 0, if we really need that speedup
  Vector2d c = centroid(verts);
  for (int i = 0; i < n; i++) {
    Vector2d p = verts.col(i) - c;
    Vector2d q = verts.col((i+1) % n) - c;
    a_sum += p(0) * q(1) - p(1) * q(0);
  }
  return 0.5 * a_sum;
}


}