#pragma once

#include <osqp.h>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/solver_base.h"

namespace dairlib {
namespace solvers {
/**
 * This class is a slight modification of Drake's OsqpSolver that saves the
 * osqp_settings_ and workspace_ along with warm starting the solve.
 * The OSQP solver details after calling Solve() function. The user can call
 * MathematicalProgramResult::get_solver_details<OsqpSolver>() to obtain the
 * details.
 */

class FastOsqpSolver final : public drake::solvers::SolverBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FastOsqpSolver)

  /// Type of details stored in MathematicalProgramResult.
  using Details = drake::solvers::OsqpSolverDetails;

  FastOsqpSolver();
  ~FastOsqpSolver() final;

  /// @name Static versions of the instance methods with similar names.
  //@{
  static drake::solvers::SolverId id();
  static bool is_available();
  static bool is_enabled();
  static bool ProgramAttributesSatisfied(
      const drake::solvers::MathematicalProgram&);
  //@}

  void InitializeSolver(const drake::solvers::MathematicalProgram&,
                        const drake::solvers::SolverOptions&);

  // A using-declaration adds these methods into our class's Doxygen.
  using SolverBase::Solve;

 private:
  void DoSolve(const drake::solvers::MathematicalProgram&,
               const Eigen::VectorXd&, const drake::solvers::SolverOptions&,
               drake::solvers::MathematicalProgramResult*) const final;

  OSQPData* osqp_data_;
  OSQPSettings* osqp_settings_;
  OSQPWorkspace* workspace_;
};
}  // namespace solvers
}  // namespace dairlib
