#pragma once
#include "alip_mpc.h"

namespace dairlib::systems::controllers {

class AlipMultiQP final : public AlipMPC {
 public:

  /// Constructor takes a nominal robot mass and walking height
  AlipMultiQP(double m,
              double H,
              int nknots,
              alip_utils::ResetDiscretization reset_discretization)
      : AlipMPC(m, H, nknots, reset_discretization) {};

  AlipMultiQP(double m, double H, int nk,
              alip_utils::ResetDiscretization reset_discretization, int nmodes)
            : AlipMultiQP(m, H, nk, reset_discretization) {
    for (int i = 0; i < nmodes; i++) {
      this->AddMode();
    }
  }

  void AddMode() final;
  void MakeFootholdConstraints();
  void Build(const drake::solvers::SolverOptions &solver_options) final;
  void Build() final;

 private:

  // per solve updates and solve steps
  void SolveOCProblemAsIs() final;
  void UpdateInitialGuess() final;
  void UpdateInitialGuess(const Eigen::Vector3d &p0, const Eigen::Vector4d &x0) final;

  // Per QP updates
  void UpdateFootholdConstraints(std::vector<int> foothold_idxs);
  void UpdateIndividualFootholdConstraint(int idx_mode, int idx_foothold);
  drake::solvers::OsqpSolver solver_;

};
}
