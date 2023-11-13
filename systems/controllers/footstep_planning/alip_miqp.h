#pragma once
#include "alip_mpc.h"
#include "drake/solvers/gurobi_solver.h"
#include "solvers/optimization_utils.h"

namespace dairlib::systems::controllers {

using drake::solvers::Binding;
using drake::solvers::LinearEqualityConstraint;
using drake::solvers::GurobiSolver;
using drake::solvers::VectorXDecisionVariable;

class AlipMIQP final : public AlipMPC {
 public:
  AlipMIQP(double m,
           double H,
           int nknots,
           alip_utils::ResetDiscretization reset_discretization) :
           AlipMPC(m, H, nknots, reset_discretization){
    DRAKE_DEMAND(drake::solvers::GurobiSolver::is_available());
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

  std::vector<geometry::ConvexPolygon> GetFootholdSequence() const;
  void AddMode() final;
  void Build(const drake::solvers::SolverOptions &solver_options) final;
  void Build() final;

  std::vector<Eigen::VectorXd> GetFootholdSelection() const {
    std::vector<Eigen::VectorXd> zz{};
    for (const auto& z : zz_) {
      zz.push_back(solution_.first.GetSolution(z));
    }
    return zz;
  }

  Eigen::Vector3d SnapFootstepToTopFoothold(const Eigen::Vector3d& p) const;
  bool success() const {return success_;}
 private:
  bool success_ = false;
  static constexpr int kMaxFootholds = 20;
  std::vector<VectorXDecisionVariable> zz_{};
  std::vector<Binding<LinearEqualityConstraint>> integer_sum_constraints_;
  std::vector<std::vector<solvers::LinearBigMConstraint>> foothold_constraints_;
  std::vector<std::vector<solvers::LinearBigMEqualityConstraint>> foothold_equality_constraints_;
  void MakeFootholdConstraints();
  void UpdateFootholdConstraints();
  void SolveOCProblemAsIs() final;
  void UpdateInitialGuess() final;
  void UpdateInitialGuess(
      const Eigen::Vector3d& p0, const Eigen::Vector4d& x0) final;
  drake::solvers::GurobiSolver solver_{};

};
}
