#include <limits>

#include "convex_foothold_set.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/osqp_solver.h"
#include "include/_usr_include_eigen3/Eigen/Core"

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

}
}