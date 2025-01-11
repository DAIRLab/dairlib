//
// Created by brian on 1/11/25.
//

#include "ncqp_solver.h"

namespace dairlib::solvers {

using drake::solvers::Binding;
using drake::solvers::Constraint;
using drake::solvers::MathematicalProgram;

NCQPSolution NCQPSolver::Solve(
    const MathematicalProgram &qp,
    const std::vector<Binding<Constraint>>& constraints ) const {

  NCQPSolution sol(qp.num_vars());
  sol.x = qp.initial_guess();
  sol.z = qp.initial_guess();

  std::unique_ptr<MathematicalProgram> work_qp = qp.Clone();




}


}