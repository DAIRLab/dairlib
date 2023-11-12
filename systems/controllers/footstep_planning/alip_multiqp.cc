#include <algorithm>
#include "alip_multiqp.h"

#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/solve.h"
#include "drake/solvers/osqp_solver.h"

namespace dairlib::systems::controllers {

using std::pair;
using std::vector;
using std::to_string;
using std::numeric_limits;

using Eigen::Matrix;
using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::MatrixXd;
using Eigen::Matrix4d;
using Eigen::Matrix3d;

using drake::VectorX;
using drake::Vector4;
using drake::AutoDiffXd;
using drake::AutoDiffVecXd;
using drake::math::InitializeAutoDiff;

using drake::solvers::Solve;
using drake::solvers::Binding;
using drake::solvers::OsqpSolver;
using drake::solvers::QuadraticCost;
using drake::solvers::DecisionVariable;
using drake::solvers::LinearConstraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::BoundingBoxConstraint;
using drake::solvers::VectorXDecisionVariable;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::LinearEqualityConstraint;

using geometry::kMaxFootholdFaces;

void AlipMultiQP::SolveOCProblemAsIs() {

  solve_time_.start_ = std::chrono::steady_clock::now();
  solve_time_.solve_time_ = 0;
  mode_sequnces_ = GetPossibleModeSequences();
  vector<pair<MathematicalProgramResult, vector<VectorXd>>> solutions;

  if (mode_sequnces_.empty()) {
    const auto sol = solver_.Solve(*prog_);
    auto dual_solutions =
        sol.is_success() ? ExtractDynamicsConstraintDual(sol) :
        vector<VectorXd>(nmodes_, VectorXd::Zero((nknots_ - 1) * nx_));
    solutions.push_back({sol, dual_solutions}); // NOLINT
    solve_time_.solve_time_ = sol.get_solver_details<OsqpSolver>().solve_time;
  } else {
    for (auto &seq : mode_sequnces_) {
      UpdateFootholdConstraints(seq);
      const auto sol = solver_.Solve(*prog_);
      auto dual_solutions = sol.is_success() ?
          ExtractDynamicsConstraintDual(sol) :
          vector<VectorXd>(nmodes_, VectorXd::Zero((nknots_ - 1) * nx_));
      solutions.push_back({sol, dual_solutions}); // NOLINT
      solve_time_.solve_time_ +=
          sol.get_solver_details<OsqpSolver>().solve_time;
    }
  }
  std::sort(
      solutions.begin(), solutions.end(),
      [](
          const pair<MathematicalProgramResult, vector<VectorXd>> &lhs,
          const pair<MathematicalProgramResult, vector<VectorXd>> &rhs) {
        return lhs.first.get_optimal_cost() < rhs.first.get_optimal_cost() &&
            lhs.first.is_success();
      });

  if (solutions.front().first.is_success()) {
    solution_ = solutions.front();
  } else {
    drake::log()->error("solve failed with code {}",
                        solutions.front().first.get_solution_result()
    );
  }
  if (std::isnan(solution_.first.get_optimal_cost())) {
    drake::log()->error(
        "NaNs slipped through unnoticed. Dumping the program:\n{}",
        prog_->to_string()
    );
  }
  solve_time_.finish_ = std::chrono::steady_clock::now();
}

void AlipMultiQP::Build(const drake::solvers::SolverOptions &options) {
  prog_->SetSolverOptions(options);
  Build();
}

void AlipMultiQP::Build() {
  DRAKE_ASSERT(td_.size() == nmodes_);
  tt_ = VectorXd::Zero(nmodes_);
  for (int i = 0; i < nmodes_; i++) {
    tt_(i) = td_.at(i);
  }
  MakeFootholdConstraints();
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

void AlipMultiQP::AddMode() {
  std::string nm = to_string(nmodes_);
  pp_.push_back(prog_->NewContinuousVariables(np_, "pp_" + nm));
  xx_.push_back(prog_->NewContinuousVariables(nx_ * nknots_, "xx_" + nm));
  uu_.push_back(prog_->NewContinuousVariables(nu_ * (nknots_ - 1), "uu_" + nm));
  nmodes_ += 1;
}

void AlipMultiQP::MakeFootholdConstraints() {
  for (int n = 0; n < nmodes_; n++) {
    footstep_c_.push_back( // If you get a "use emplace_back" suggestion here,
        {                  // clang-tidy is wrong.
            prog_->AddLinearConstraint(
                Matrix<double, kMaxFootholdFaces, 3>::Zero(),
                VectorXd::Zero(kMaxFootholdFaces),
                VectorXd::Zero(kMaxFootholdFaces),
                pp_.at(n)),
            prog_->AddLinearEqualityConstraint(
                Eigen::RowVector3d::Zero(),
                Eigen::VectorXd::Zero(1),
                pp_.at(n))
        }
    );
  }
}

void AlipMultiQP::UpdateIndividualFootholdConstraint(
    int idx_mode, int idx_foothold) {
  const auto &[A, b] = footholds_.at(idx_foothold).GetConstraintMatrices();
  const auto &[A_eq, b_eq] =
      footholds_.at(idx_foothold).GetEqualityConstraintMatrices();

  Matrix<double, kMaxFootholdFaces, 3> bigA =
      Matrix<double, kMaxFootholdFaces, 3>::Zero();
  bigA.topRows(A.rows()) = A;
  VectorXd bigb = VectorXd::Zero(kMaxFootholdFaces);
  bigb.head(b.rows()) = b;

  footstep_c_.at(idx_mode).first.evaluator()->UpdateCoefficients(
      bigA,
      -numeric_limits<double>::infinity() * VectorXd::Ones(bigb.rows()),
      bigb
  );
  footstep_c_.at(idx_mode).second.evaluator()->UpdateCoefficients(A_eq, b_eq);
}

void AlipMultiQP::UpdateFootholdConstraints(vector<int> foothold_idxs) {
  DRAKE_ASSERT(foothold_idxs.size() == nmodes_);
  for (int i = 0; i < foothold_idxs.size(); i++) {
    UpdateIndividualFootholdConstraint(i + 1, foothold_idxs.at(i));
  }
}

void AlipMultiQP::UpdateInitialGuess(const Eigen::Vector3d &p0,
                                     const Eigen::Vector4d &x0) {
  // Update state initial guess
  vector<VectorXd> xg = xd_;

  // Set the initial guess for the current mode based on limited time
  VectorXd xx = VectorXd(nx_ * nknots_);
  xx.head<4>() = x0;
  Matrix4d Ad = alip_utils::CalcAd(H_, m_, tt_(0) / (nknots_ - 1));
  for (int i = 1; i < nknots_; i++) {
    GetStateAtKnot(xx, i) = Ad * GetStateAtKnot(xx, i - 1);
  }
  xg.front() = xx;

  for (int n = 0; n < nmodes_; n++) {
    for (int k = 0; k < nknots_; k++) {
      prog_->SetInitialGuess(
          GetStateAtKnot(xx_.at(n), k),
          GetStateAtKnot(xg.at(n), k));
    }
  }
  Vector3d ptemp = p0;
  prog_->SetInitialGuess(pp_.front(), p0);
  for (int n = 1; n < nmodes_; n++) {
    Vector2d p1 = (xd_.at(n - 1).tail<4>() - xd_.at(n).head<4>()).head<2>()
        + ptemp.head<2>();
    prog_->SetInitialGuess(pp_.at(n).head<2>(), p1);
    ptemp.head<2>() = p1;
  }
}

void AlipMultiQP::UpdateInitialGuess() {
  DRAKE_DEMAND(
      solution_.first.is_success() ||
          solution_.first.get_solution_result() ==
              drake::solvers::kIterationLimit);
  prog_->SetInitialGuessForAllVariables(solution_.first.GetSolution());
}
}