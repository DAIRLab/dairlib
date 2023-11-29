#include <limits>
#include <chrono>
#include <iostream>
#include "convex_polygon_set.h"
#include "common/eigen_utils.h"

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/osqp_solver.h"

using Eigen::Matrix3d;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::Vector2d;
using Eigen::Matrix2d;
using Eigen::Matrix;

using std::numeric_limits;

namespace dairlib{
namespace geometry{

ConvexPolygonSet ConvexPolygonSet::GetSubsetCloseToPoint(
    const Vector3d &query_pt, double threshold) const {

  drake::solvers::MathematicalProgram prog;
  std::vector<drake::solvers::VectorXDecisionVariable> pp;

  for (int i = 0; i < set_.size(); i++) {
    auto p = prog.NewContinuousVariables(3);
    const auto& [Aeq, beq] = set_.at(i).GetEqualityConstraintMatrices();
    const auto& [A, b] = set_.at(i).GetConstraintMatrices();
    prog.AddLinearEqualityConstraint(Aeq, beq, p);
    prog.AddLinearConstraint(
        A, -numeric_limits<double>::infinity()*VectorXd::Ones(A.rows()), b, p);
    prog.AddQuadraticErrorCost(Matrix3d::Identity(), query_pt, p);
    pp.push_back(p);
  }

  drake::solvers::OsqpSolver solver;
  const auto result = solver.Solve(prog);
  DRAKE_ASSERT(result.is_success());

  ConvexPolygonSet close;
  for (int i = 0; i < set_.size(); i++) {
    auto p = result.GetSolution(pp.at(i));
    if ((query_pt - p).norm() < threshold) {
      close.append(set_.at(i));
    }
  }
  return close;
}


ConvexPolygonSet ConvexPolygonSet::GetSubsetInForwardLookingCone(
    const Vector3d &query_pt, double cone_angle) const {
  Matrix2d Aq;
  Aq << cos(cone_angle), sin(cone_angle), cos(cone_angle), -sin(cone_angle);
  Vector2d bq = Aq * query_pt.head<2>();
  Matrix<double, 2, 4> pq;
  pq.leftCols<2>() = Matrix2d::Identity();
  pq.rightCols<2>() = -Matrix2d::Identity();
  Matrix<double, 4, 4> Q = pq.transpose() * pq;

  drake::solvers::MathematicalProgram prog;
  std::vector<drake::solvers::VectorXDecisionVariable> pp;
  std::vector<drake::solvers::VectorXDecisionVariable> qq;
  for (int i = 0; i < set_.size(); i++) {
    auto p = prog.NewContinuousVariables(2);
    auto q = prog.NewContinuousVariables(2);
    const auto& [A, b] = set_.at(i).GetConstraintMatrices();
    prog.AddLinearConstraint(
        A.leftCols<2>(),
        VectorXd::Constant(A.rows(), -numeric_limits<double>::infinity()),
        b, p);
    prog.AddLinearConstraint(
        Aq, bq, Vector2d::Constant(std::numeric_limits<double>::infinity()), q);
    prog.AddQuadraticCost(Q, Vector4d::Zero(), {p, q});
    pp.push_back(p);
    qq.push_back(q);
  }

  drake::solvers::OsqpSolver solver;
  const auto result = solver.Solve(prog);
  DRAKE_ASSERT(result.is_success());

  ConvexPolygonSet ret;
  for (int i = 0; i < set_.size(); i++) {
    auto p = result.GetSolution(pp.at(i));
    auto q = result.GetSolution(qq.at(i));
    if ((p - q).norm() < 1e-3) {
      ret.append(set_.at(i));
    }
  }
  return ret;
}

void ConvexPolygonSet::ReExpressInNewFrame(const Matrix3d &R_WF) {
  for (auto& f: set_) {
    f.ReExpressInNewFrame(R_WF);
  }
}

void ConvexPolygonSet::CopyToLcm(lcmt_foothold_set *set) const {
  set->footholds.clear();
  set->n = this->size();
  for (const auto& polygon : set_) {
    lcmt_convex_foothold foothold_lcm;
    const auto& [Aeq, beq] = polygon.GetEqualityConstraintMatrices();
    const auto& [A, b] = polygon.GetConstraintMatrices();
    for (int i = 0; i < 3; i++) {
      foothold_lcm.Aeq[i] = Aeq(i);
    }
    foothold_lcm.beq = beq(0);
    foothold_lcm.nfaces = A.rows();
    foothold_lcm.A.clear();
    for (int i = 0; i < A.rows(); i++) {
      foothold_lcm.A.push_back(
          CopyVectorXdToStdVector(A.block(i, 0, 1, 3).transpose())
      );
    }
    foothold_lcm.b = CopyVectorXdToStdVector(b);
    set->footholds.push_back(foothold_lcm);
  }
}

ConvexPolygonSet ConvexPolygonSet::CopyFromLcm(const lcmt_foothold_set& set_lcm) {
  ConvexPolygonSet set{};
  for (const auto& foothold_lcm : set_lcm.footholds) {
    ConvexPolygon foothold;
    const Vector3d Aeq = Vector3d::Map(foothold_lcm.Aeq);
    foothold.SetPlane(Aeq, foothold_lcm.beq);
    for (int i = 0; i < foothold_lcm.nfaces; i++) {
      foothold.AddHalfspace(
          Vector3d::Map(foothold_lcm.A.at(i).data()),
          VectorXd::Constant(1, foothold_lcm.b.at(i))
      );
    }
    set.append(foothold);
  }
  return set;
}

double  ConvexPolygonSet::CalcHeightOfPoint(const Eigen::Vector3d& point) const {
  double zmax = -std::numeric_limits<double>::infinity();
  for (const auto& poly : set_) {
    if (not poly.PointViolatesInequalities(point)) {
      const auto& [A, b] = poly.GetEqualityConstraintMatrices();
      double z = (b - A.leftCols<2>() * point.head<2>())(0);
      zmax = (z > zmax) ? z : zmax;
    }
  }
  return zmax;
}


bool ConvexPolygonSet::Feasible2d(const Vector3d& pt, double tol) const {
  return std::any_of(
      set_.begin(), set_.end(),
      [&tol, &pt](const ConvexPolygon& f) {
        return f.Get2dViolation(pt) < tol;
      });
}

}
}