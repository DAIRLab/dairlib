#pragma once

#include "fcc_qp.hpp"
#include "drake/common/drake_copyable.h"
#include "drake/solvers/solver_base.h"
#include "drake/solvers/osqp_solver.h"


namespace dairlib {
namespace solvers {

class FCCQPSolver final : public drake::solvers::SolverBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FCCQPSolver)

  using Details = fcc_qp::FCCQPDetails;

  FCCQPSolver();
  ~FCCQPSolver() final;

  static inline drake::solvers::SolverId id() {
    static const drake::never_destroyed<drake::solvers::SolverId>
        singleton{"FCCQP"};
    return singleton.access();
  }
  static inline bool is_enabled() {return true;}
  static inline bool is_available() {return true;}

  static inline bool is_convex_qp(
      const drake::solvers::MathematicalProgram& prog) {
    return drake::solvers::OsqpSolver::ProgramAttributesSatisfied(prog);
  }
  static inline bool ProgramAttributesSatisfied(
      const drake::solvers::MathematicalProgram& prog) {
    return is_convex_qp(prog) and prog.linear_constraints().empty();
  };

  static inline std::string UnsatisfiedProgramAttributes(
      const drake::solvers::MathematicalProgram& prog) {
    if (not is_convex_qp(prog)) {
      return drake::solvers::OsqpSolver::UnsatisfiedProgramAttributes(prog);
    }
    if (not prog.linear_constraints().empty()) {
      return "FCCQP does not support general linear inequalities";
    }
    return "";
  };

  void InitializeSolver(const drake::solvers::MathematicalProgram&,
                        const drake::solvers::SolverOptions&, int, int,
                        const std::vector<double>&);

  void UpdateFrictionCoefficients(const std::vector<double>& friction_coeffs) {
    DRAKE_DEMAND(is_initialized());
    DRAKE_DEMAND(friction_coeffs.size() == friction_coeffs_.size());
    friction_coeffs_ = friction_coeffs;
  }

  [[nodiscard]] bool is_initialized() const {return fcc_qp_ != nullptr;}

 private:
  void DoSolve(const drake::solvers::MathematicalProgram&,
               const Eigen::VectorXd&, const drake::solvers::SolverOptions&,
               drake::solvers::MathematicalProgramResult*) const final;

  std::unique_ptr<fcc_qp::FCCQP> fcc_qp_ = nullptr;
  std::vector<double> friction_coeffs_{};
};

}
}

