#include "alip_miqp.h"
#include "solvers/optimization_utils.h"

namespace dairlib::systems::controllers {

using Eigen::VectorXd;
using drake::solvers::GurobiSolver;
using solvers::AddBigMInequalityConstraint;
using solvers::AddBigMEqualityConstraint;

void AlipMIQP::AddMode() {
  std::string nm = std::to_string(nmodes_);
  pp_.push_back(prog_->NewContinuousVariables(np_, "pp_" + nm));
  xx_.push_back(prog_->NewContinuousVariables(nx_ * nknots_, "xx_" + nm));
  uu_.push_back(prog_->NewContinuousVariables(nu_ * (nknots_ - 1), "uu_" + nm));
  nmodes_ += 1;
}

void AlipMIQP::Build(const drake::solvers::SolverOptions& options) {
  prog_->SetSolverOptions(options);
  Build();
}

void AlipMIQP::Build() {
  DRAKE_ASSERT(td_.size() == nmodes_);
  tt_ = VectorXd::Zero(nmodes_);
  for (int i = 0; i < nmodes_; i++) {
    tt_(i) = td_.at(i);
  }
  MakeNoCrossoverConstraint();
  MakeResetConstraints();
  MakeDynamicsConstraints();
  MakeWorkspaceConstraints();
  MakeInputBoundConstaints();
  MakeNextFootstepReachabilityConstraint();
  MakeInitialStateConstraint();
  MakeInitialFootstepConstraint();
  built_ = true;
}

void AlipMIQP::MakeFootholdConstraints(MathematicalProgram& prog) {
  double bigM = 2.0;
  for (int j = 1; j < nmodes_; j++) {
    auto z = prog.NewBinaryVariables(footholds_.size(),
                                     "foothold_selection_" + std::to_string(j));
    prog.AddLinearEqualityConstraint(
        Eigen::RowVectorXd::Ones(z.size()), drake::Vector1d::Ones(), z);
    for (int i = 0; i < footholds_.size(); i++) {
      const auto&[A, b] = footholds_.at(i).GetConstraintMatrices();
      const auto&[A_eq, b_eq] = footholds_.at(i).GetEqualityConstraintMatrices();
      AddBigMEqualityConstraint(prog, A_eq, b_eq, bigM, pp_.at(j), z(i));
      AddBigMInequalityConstraint(prog, A, b, bigM, pp_.at(j), z(i));
    }
  }
}

void AlipMIQP::SolveOCProblemAsIs() {
  drake::copyable_unique_ptr<MathematicalProgram> prog_tmp(prog_);
  MakeFootholdConstraints(*prog_tmp);
  const auto& result = solver_.Solve(*prog_tmp);
  if (result.is_success()) {
    solution_.first = result;
    solution_.second = ExtractDynamicsConstraintDual(result);
  }
}

}