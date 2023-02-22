#pragma once
#include <array>
#include <chrono>
#include "alip_mpc.h"

namespace dairlib::systems::controllers {

using solvers::NonlinearConstraint;

using drake::solvers::Binding;
using drake::solvers::OsqpSolver;
using drake::solvers::MosekSolver;
using drake::solvers::QuadraticCost;
using drake::solvers::DecisionVariable;
using drake::solvers::LinearConstraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::BoundingBoxConstraint;
using drake::solvers::VectorXDecisionVariable;
using drake::solvers::LinearEqualityConstraint;

using std::vector;

class AlipMINLP final : public AlipMPC {
 public:

  /// Constructor takes a nominal robot mass and walking height
  AlipMINLP(double m,
            double H,
            int nknots,
            alip_utils::ResetDiscretization reset_discretization)
      : AlipMPC(m, H, nknots, reset_discretization) {};

  AlipMINLP(double m, double H, int nk,
            alip_utils::ResetDiscretization reset_discretization, int nmodes)
            : AlipMINLP(m, H, nk, reset_discretization) {
    for (int i = 0; i < nmodes; i++) {
      this->AddMode();
    }
  }

  void AddMode() final;
  void MakeFootholdConstraints() final;
  void Build(const drake::solvers::SolverOptions &solver_options) final;
  void Build() final;

 private:

  // per solve updates and solve steps
  void SolveOCProblemAsIs() final;
  void UpdateInitialGuess() final;
  void UpdateInitialGuess(const Eigen::Vector3d &p0, const Eigen::Vector4d &x0) final;

  // Per QP updates
  void UpdateFootholdConstraints(vector<int> foothold_idxs);
  void UpdateIndividualFootholdConstraint(int idx_mode, int idx_foothold);
  drake::solvers::OsqpSolver solver_;

};
}
