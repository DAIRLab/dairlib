#include "solvers/admm/convex_polygon_set_constraint.h"
#include "solvers/admm/ncqp_solver.h"
#include <iostream>

namespace dairlib::solvers {

using Eigen::Vector3d;

using drake::solvers::Binding;
using drake::solvers::Constraint;

int DoMain() {

  geometry::ConvexPolygon square;
  geometry::ConvexPolygon shifted_square;
  square.SetPlane(Vector3d::UnitZ(), Vector3d::Zero());
  square.AddFace(Vector3d::UnitX(), Vector3d::UnitX());
  square.AddFace(Vector3d::UnitY(), Vector3d::UnitY());
  square.AddFace(-Vector3d::UnitX(), -Vector3d::UnitX());
  square.AddFace(-Vector3d::UnitY(), -Vector3d::UnitY());

  Vector3d s = 1.2 * Vector3d::UnitX() + Vector3d::UnitY();
  shifted_square.SetPlane(Vector3d::UnitZ(), Vector3d::Zero());
  shifted_square.AddFace(Vector3d::UnitX(), s + Vector3d::UnitX());
  shifted_square.AddFace(Vector3d::UnitY(), s + Vector3d::UnitY());
  shifted_square.AddFace(-Vector3d::UnitX(), s - Vector3d::UnitX());
  shifted_square.AddFace(-Vector3d::UnitY(), s - Vector3d::UnitY());

  geometry::ConvexPolygonSet set({square, shifted_square});

  drake::solvers::MathematicalProgram prog;
  auto p = prog.NewContinuousVariables(3, "p");

  prog.AddQuadraticErrorCost(
      1.0 * Eigen::Matrix3d::Identity(),
      2.0 * Vector3d::UnitY(),
      p
  );

  std::vector<Binding<Constraint>> set_constraints;

  auto square_constraint = std::make_shared<ConvexPolygonSetConstraint>(set);

  Eigen::VectorXd p_star_proj;
  square_constraint->ProjectToFeasibleSet(
      2.0 * Vector3d::UnitY(),
      &p_star_proj
  );

  std::cout << "Projection: " << p_star_proj.transpose() << std::endl;

  Eigen::VectorXd y;
  prog.GetAllCosts().front().evaluator()->Eval(p_star_proj, &y);
  std::cout << "optimal cost: " << y(0) << std::endl;

  set_constraints.push_back(prog.AddConstraint(square_constraint, p));

  drake::solvers::SolverOptions opts;
  NCQPSolver solver(opts, opts, ADMMParams());
  auto sol = solver.Solve(prog, set_constraints);

  std::cout << sol << std::endl;

  return 0;
}

}

int main(int argc, char**argv) {
  return dairlib::solvers::DoMain();
}