/* clang-format off to disable clang-format-includes */
#include "solvers/fast_osqp_solver.h"
/* clang-format on */

#include "drake/common/never_destroyed.h"
#include "drake/solvers/mathematical_program.h"

// This file contains implementations that are common to both the available and
// unavailable flavor of this class.

using drake::solvers::MathematicalProgram;
using drake::solvers::ProgramAttributes;
using drake::solvers::ProgramAttribute;
using drake::solvers::SolverId;

namespace dairlib {
namespace solvers {

FastOsqpSolver::FastOsqpSolver()
    : SolverBase(&id, &is_available, &is_enabled,
                 &ProgramAttributesSatisfied) {}

FastOsqpSolver::~FastOsqpSolver() = default;

SolverId FastOsqpSolver::id() {
  static const drake::never_destroyed<SolverId> singleton{"OSQP"};
  return singleton.access();
}

bool FastOsqpSolver::is_enabled() { return true; }

bool FastOsqpSolver::ProgramAttributesSatisfied(const drake::solvers::MathematicalProgram& prog) {
  static const drake::never_destroyed<ProgramAttributes> solver_capabilities(
      std::initializer_list<ProgramAttribute>{
          ProgramAttribute::kLinearCost, ProgramAttribute::kQuadraticCost,
          ProgramAttribute::kLinearConstraint,
          ProgramAttribute::kLinearEqualityConstraint});
  return AreRequiredAttributesSupported(prog.required_capabilities(),
                                        solver_capabilities.access()) &&
      prog.required_capabilities().count(ProgramAttribute::kQuadraticCost) >
          0;
}

}  // namespace solvers
}  // namespace drake
