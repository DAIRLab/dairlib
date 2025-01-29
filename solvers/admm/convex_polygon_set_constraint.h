#pragma once

#include "set_membership_constraint.h"
#include "solvers/osqp_wrapper.h"
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
    BuildProjectionProg();
  }

  std::tuple<Eigen::MatrixXd, Eigen::VectorXd, Eigen::VectorXd>
  CalcClosestConvexRestrictionToQP(
      const Eigen::Ref<const drake::VectorX<double>>& x) const override;

 private:

  std::pair<Eigen::Vector3d, geometry::ConvexPolygon>
      DoProjection(const Eigen::Vector3d& x) const;

  void BuildProjectionProg();

  geometry::ConvexPolygonSet set_;
  mutable solvers::OsqpWrapper osqp_;
  drake::solvers::MathematicalProgram projection_prog_;
  QPData projection_qp_;
  std::vector<drake::solvers::VectorXDecisionVariable> pp_{};
};

}