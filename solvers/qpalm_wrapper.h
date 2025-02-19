#pragma once

#include "qp_data.h"
#include "drake/solvers/solver_options.h"
#include "drake/solvers/mathematical_program_result.h"


namespace dairlib {
namespace solvers {

class QpalmWrapper {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(QpalmWrapper);

  QpalmWrapper();
  ~QpalmWrapper();

  // Initialize the solver with problem data and solver options
  void InitializeSolver(QPData& qp, const drake::solvers::SolverOptions& options);

  // Warm starting controls
  void DisableWarmStart() const;
  void EnableWarmStart() const;

  // Set warm start values for primal and dual variables
  void WarmStart(const Eigen::VectorXd& primal, const Eigen::VectorXd& dual);

  // Check if solver is initialized
  bool IsInitialized() const;

  // Main solve methods
  void Solve(QPData& qp, QPResult& result, bool has_matrix_update = true) const;

  drake::solvers::MathematicalProgramResult Solve(
      const drake::solvers::MathematicalProgram& prog,
      bool has_matrix_update = true) const;

  // Clean up allocated resources
  void FreeProblemData();

 private:
  // The implementation class need to be hidden in the .cc file to avoid
  // including QPALM's headers anywhere else, since they contain many conflicting
  // typedefs and macros
  class Impl;
  std::unique_ptr<Impl> pimpl_;
};

}  // namespace solvers
}  // namespace dairlib