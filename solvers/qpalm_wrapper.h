#pragma once

#include "drake/solvers/solver_options.h"
#include "drake/solvers/mathematical_program_result.h"

// qpalm includes have to come after drake bc qpalm defines a mod
// macro that interferes with fmt :(
#include "qpalm.hpp"
#include "qp_data.h"


namespace dairlib {
namespace solvers {

class QpalmWrapper {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(QpalmWrapper);

  QpalmWrapper() = default;
  ~QpalmWrapper();

  // Initialize the solver with problem data and solver options
  void InitializeSolver(QPData& qp, const drake::solvers::SolverOptions& options);

  // Warm starting controls
  void DisableWarmStart() const {
    if (qpalm_settings_) qpalm_settings_->warm_start = false;
    warm_start_ = false;
  }

  void EnableWarmStart() const {
    if (qpalm_settings_) qpalm_settings_->warm_start = true;
    warm_start_ = true;
  }

  // Set warm start values for primal and dual variables
  void WarmStart(const Eigen::VectorXd& primal, const Eigen::VectorXd& dual);

  // Check if solver is initialized
  bool IsInitialized() const {
    return is_init_;
  }

  // Main solve methods
  void Solve(QPData& qp, QPResult& result, bool has_matrix_update = true) const;

  drake::solvers::MathematicalProgramResult Solve(
      const drake::solvers::MathematicalProgram& prog,
      bool has_matrix_update = true) const;

  // Clean up allocated resources
  void FreeProblemData();

 private:
  // QPALM data structures
  mutable QPALMData* qpalm_data_ = nullptr;
  mutable QPALMSettings* qpalm_settings_ = nullptr;
  mutable QPALMWorkspace* qpalm_work_ = nullptr;

  // Problem matrices in CSC format (compressed sparse column)
  mutable qpalm::ladel_sparse_matrix_ptr P_csc_ = nullptr;
  mutable qpalm::ladel_sparse_matrix_ptr A_csc_ = nullptr;

  // Problem vectors
  mutable std::vector<c_float> l_;  // Lower bounds
  mutable std::vector<c_float> u_;  // Upper bounds
  mutable std::vector<c_float> q_;  // Linear cost vector

  // Solver state
  mutable bool warm_start_ = true;
  mutable bool is_init_ = false;
};

}  // namespace solvers
}  // namespace dairlib