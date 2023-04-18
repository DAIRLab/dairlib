#include <limits>
#include <chrono>
#include <iostream>
#include "convex_foothold_set.h"
#include "common/eigen_utils.h"

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/osqp_solver.h"

using Eigen::Matrix3d;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Vector2d;

using std::numeric_limits;

namespace dairlib{
namespace geometry{

ConvexFootholdSet ConvexFootholdSet::GetSubsetCloseToPoint(
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

  ConvexFootholdSet close;
  for (int i = 0; i < set_.size(); i++) {
    auto p = result.GetSolution(pp.at(i));
    if ((query_pt - p).norm() < threshold) {
      close.append(set_.at(i));
    }
  }
  return close;
}

void ConvexFootholdSet::ReExpressInNewFrame(const Matrix3d &R_WF) {
  for (auto& f: set_) {
    f.ReExpressInNewFrame(R_WF);
  }
}

void ConvexFootholdSet::CopyToLcm(lcmt_foothold_set *set) const {
  set->footholds.clear();
  set->n = this->size();
  for (const auto& foothold : set_) {
    lcmt_convex_foothold foothold_lcm;
    const auto& [Aeq, beq] = foothold.GetEqualityConstraintMatrices();
    const auto& [A, b] = foothold.GetConstraintMatrices();
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

ConvexFootholdSet ConvexFootholdSet::CopyFromLcm(const lcmt_foothold_set& set_lcm) {
  ConvexFootholdSet set{};
  for (const auto& foothold_lcm : set_lcm.footholds) {
    ConvexFoothold foothold;
    const Vector3d Aeq = Vector3d::Map(foothold_lcm.Aeq);
    foothold.SetContactPlane(Aeq, foothold_lcm.beq);
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

}
}