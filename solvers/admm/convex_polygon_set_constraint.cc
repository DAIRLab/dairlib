#include "convex_polygon_set_constraint.h"

namespace dairlib::solvers {

using Eigen::Vector3d;
using geometry::ConvexPolygonSet;
static constexpr double kInf = std::numeric_limits<double>::infinity();

ConvexPolygonSetConstraint::ConvexPolygonSetConstraint(
    const ConvexPolygonSet& polygons)
    : SetMembershipConstraint(3, 3, Vector3d::Zero(), Vector3d::Zero()){
  set_ = polygons;
}

void ConvexPolygonSetConstraint::ProjectToFeasibleSet(
    const Eigen::Ref<const drake::VectorX<double>> &x,
    drake::VectorX<double> *y) const {
  const auto [proj, _] = set_.ProjectPointToPolygonSet(x);
  *y = proj;
}

std::tuple<Eigen::MatrixXd, Eigen::VectorXd, Eigen::VectorXd>
ConvexPolygonSetConstraint::CalcClosestConvexRestrictionToQP(
    const Eigen::Ref<const drake::VectorX<double>>& x) const {
  const auto [_, poly] = set_.ProjectPointToPolygonSet(x);

  const auto& [Aeq, beq] = poly.GetEqualityConstraintMatrices();
  const auto& [A, b] = poly.GetConstraintMatrices();

  Eigen::MatrixXd Aout = Eigen::MatrixXd(A.rows() + Aeq.rows(), A.cols());
  Aout.topRows(Aeq.rows()) = Aeq;
  Aout.bottomRows(A.rows()) = A;

  Eigen::VectorXd lb = Eigen::VectorXd::Constant(Aout.rows(), -kInf);
  lb.head(beq.rows()) = beq;

  Eigen::VectorXd ub = Eigen::VectorXd::Constant(Aout.rows(), 0);
  ub.head(beq.rows()) = beq;
  ub.tail(b.rows()) = b;

  return {Aout, lb, ub};
}

}