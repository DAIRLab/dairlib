#pragma once
#include "alip_mpc.h"
#include "drake/solvers/gurobi_solver.h"

namespace dairlib::systems::controllers {

class AlipMIQP : public AlipMPC {
 public:
  AlipMIQP(double m,
           double H,
           int nknots,
           alip_utils::ResetDiscretization reset_discretization) :
           AlipMPC(m, H, nknots, reset_discretization){
    DRAKE_DEMAND(drake::solvers::GurobiSolver::is_enabled());
  }

 AlipMIQP(double m,
          double H,
          int nknots,
          alip_utils::ResetDiscretization reset_discretization,
          int nmodes) : AlipMIQP(m, H, nknots, reset_discretization) {
    for (int i = 0; i < nmodes; i++) {
      this->AddMode();
    }
  }

  void AddMode() final;
  void Build(const drake::solvers::SolverOptions &solver_options);
  void Build();

 private:
  void MakeFootholdConstraints(drake::solvers::MathematicalProgram& prog);
  void SolveOCProblemAsIs() final;
  void UpdateInitialGuess() final;
  void UpdateInitialGuess(
      const Eigen::Vector3d& p0, const Eigen::Vector4d& x0) final;
  drake::solvers::GurobiSolver solver_{};

};

}
