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
  /// Solver will automatically reenable warm starting after a successful solve
  void DisableWarmStart() const {
    osqp_settings_->warm_start = false;
    warm_start_ = false;
    is_init_ = false;
  }
  /// Solver will automatically reenable warm starting after a successful solve
  void EnableWarmStart() const {
    osqp_settings_->warm_start = true;
    warm_start_ = true;
  }

  bool IsInitialized()const{
    return is_init_;
  }

  // A using-declaration adds these methods into our class's Doxygen.
  using SolverBase::Solve;

 private:
  void DoSolve(const drake::solvers::MathematicalProgram&,
               const Eigen::VectorXd&, const drake::solvers::SolverOptions&,
               drake::solvers::MathematicalProgramResult*) const final;

  OSQPData* osqp_data_;
  mutable OSQPSettings* osqp_settings_;
  mutable OSQPWorkspace* workspace_;
  mutable bool warm_start_ = true;
  mutable bool is_init_ = false;
};
}  // namespace solvers
}  // namespace dairlib
