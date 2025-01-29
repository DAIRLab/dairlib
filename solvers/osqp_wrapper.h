
#pragma once

#include <osqp.h>

#include "qp_data.h"
#include "drake/solvers/solver_options.h"
#include "drake/solvers/mathematical_program_result.h"

namespace dairlib {
namespace solvers {

class OsqpWrapper {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OsqpWrapper);

  OsqpWrapper() = default;
  ~OsqpWrapper();

  void InitializeSolver(QPData &qp,
                        const drake::solvers::SolverOptions &);

  /// Solver will automatically reenable warm starting after a successful solve
  void DisableWarmStart() const {
    osqp_settings_->warm_start = false;
    warm_start_ = false;
  }
  /// Solver will automatically reenable warm starting after a successful solve
  void EnableWarmStart() const {
    osqp_settings_->warm_start = true;
    warm_start_ = true;
  }

  void WarmStart(const Eigen::VectorXd &primal, const Eigen::VectorXd &dual);

  bool IsInitialized() const { return is_init_; }
  void Solve(QPData &qp, QPResult &result, bool has_matrix_update=true) const;
  drake::solvers::MathematicalProgramResult Solve(
      const drake::solvers::MathematicalProgram& prog,
      bool has_matrix_update = true) const;

  void FreeProblemData();

 private:

  OSQPData *osqp_data_ = nullptr;
  mutable csc *P_csc_ = nullptr;
  mutable csc *A_csc_ = nullptr;
  mutable std::vector<c_float> l_;
  mutable std::vector<c_float> u_;
  mutable std::vector<c_float> q_;

  mutable OSQPSettings *osqp_settings_ = nullptr;
  mutable OSQPWorkspace *workspace_ = nullptr;
  mutable bool warm_start_ = true;
  mutable bool is_init_ = false;

};

}
}