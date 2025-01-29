#include "drake/solvers/osqp_solver.h"
#include "convex_polygon_set_constraint.h"

namespace dairlib::solvers {

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using geometry::ConvexPolygon;
using geometry::ConvexPolygonSet;
static constexpr double kInf = std::numeric_limits<double>::infinity();
using drake::solvers::MathematicalProgramResult;


ConvexPolygonSetConstraint::ConvexPolygonSetConstraint(
    const ConvexPolygonSet& polygons)
    : SetMembershipConstraint(3, 3, Vector3d::Zero(), Vector3d::Zero()){
  set_ = polygons;
  projection_prog_.SetSolverOption(
      drake::solvers::OsqpSolver::id(), "scale", 0);
  BuildProjectionProg();
}

void ConvexPolygonSetConstraint::ProjectToFeasibleSet(
    const Eigen::Ref<const drake::VectorX<double>> &x,
    drake::VectorX<double> *y) const {
  const auto [proj, _] = this->DoProjection(x);
  *y = proj;
}

std::tuple<Eigen::MatrixXd, Eigen::VectorXd, Eigen::VectorXd>
ConvexPolygonSetConstraint::CalcClosestConvexRestrictionToQP(
    const Eigen::Ref<const drake::VectorX<double>>& x) const {
  const auto [_, poly] = this->DoProjection(x);

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

void ConvexPolygonSetConstraint::BuildProjectionProg() {
  for (const auto& binding : projection_prog_.GetAllCosts()) {
    projection_prog_.RemoveCost(binding);
  }
  for (const auto& binding : projection_prog_.GetAllConstraints()) {
    projection_prog_.RemoveConstraint(binding);
  }
  for (int i = 0; i < pp_.size(); ++i) {
    for (int j = 0; j < 3; ++j) {
      projection_prog_.RemoveDecisionVariable(pp_.at(i)(j));
    }
  }
  pp_.clear();

  for (int i = 0; i < set_.polygons().size(); i++) {
    auto p = projection_prog_.NewContinuousVariables(3);
    const auto& [Aeq, beq] = set_.polygons().at(i).GetEqualityConstraintMatrices();
    const auto& [A, b] = set_.polygons().at(i).GetConstraintMatrices();
    projection_prog_.AddLinearEqualityConstraint(Aeq, beq, p);
    projection_prog_.AddLinearConstraint(
        A, VectorXd::Constant (A.rows(), -kInf), b, p);
    projection_prog_.AddQuadraticErrorCost(Matrix3d::Identity(), Vector3d::Zero(), p);
    pp_.push_back(p);
  }

  projection_qp_ = QPData::ToQPData(projection_prog_);
  if (set_.size() > 0) {
    osqp_.InitializeSolver(projection_qp_, projection_prog_.solver_options());
  }
}

std::pair<Vector3d, ConvexPolygon> ConvexPolygonSetConstraint::DoProjection(
    const Vector3d &x) const {

  for (auto& binding : projection_prog_.quadratic_costs()) {
    for (int i = 0; i < 3; ++i) {
      binding.evaluator()->update_linear_coefficient_entry(i, -2.0 * x(i));
    }
  }
  MathematicalProgramResult result = osqp_.Solve(projection_prog_, false);

  Vector3d sol = x;
  ConvexPolygon active_poly;
  double min_dist = std::numeric_limits<double>::infinity();
  for (int i = 0; i < set_.size(); i++) {
    auto p = result.GetSolution(pp_.at(i));
    double dist = (x - p).norm();
    if (dist < min_dist) {
      min_dist = dist;
      sol = p;
      active_poly = set_.polygons().at(i);
    }
  }
  return {sol, active_poly};
}

}