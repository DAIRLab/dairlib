#include "convex_polygon_set_constraint.h"

namespace dairlib::solvers {

using Eigen::Vector3d;
using geometry::ConvexPolygonSet;

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

std::unique_ptr<SetMembershipConstraint>
ConvexPolygonSetConstraint::RestrictionToConvexComponentClosestTo(
    const Eigen::Ref<const drake::VectorX<double>> &x) const {
  auto [_, poly] = set_.ProjectPointToPolygonSet(x);
  return std::make_unique<ConvexPolygonSetConstraint>(
      ConvexPolygonSet({poly}));
}

}