#pragma once

#include "set_membership_constraint.h"
#include "geometry/convex_polygon_set.h"

namespace dairlib::solvers {

class ConvexPolygonSetConstraint : public SetMembershipConstraint {
 public:
  explicit ConvexPolygonSetConstraint(
      const geometry::ConvexPolygonSet& polygons);

  void ProjectToFeasibleSet(
      const Eigen::Ref<const drake::VectorX<double>>& x,
      drake::VectorX<double>* y) const override;

  void UpdatePolygons(const geometry::ConvexPolygonSet& polygons) {
    set_ = polygons;
  }

  std::unique_ptr<SetMembershipConstraint>
  RestrictionToConvexComponentClosestTo(
      const Eigen::Ref<const drake::VectorX<double>>& x) const override;

 private:
  geometry::ConvexPolygonSet set_;
};

}