#include <algorithm>
#include <iostream>

#include "alip_minlp.h"
#include "solvers/optimization_utils.h"

#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/solve.h"
#include "drake/solvers/osqp_solver.h"

namespace dairlib::systems::controllers{

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

AlipDynamicsConstraint::AlipDynamicsConstraint(double m, double H, double n) :
    NonlinearConstraint<AutoDiffXd>(
        4, 10, Vector4d::Zero(), Vector4d::Zero(), "dynamics"),
    m_(m), H_(H), n_(n-1) {

  A_ = alip_utils::CalcA(H_, m_);
  A_inv_ = A_.inverse();
  B_ = Vector4d::UnitW();

}

void AlipDynamicsConstraint::EvaluateConstraint(
    const Eigen::Ref<const drake::VectorX<AutoDiffXd>> &x,
          drake::VectorX<AutoDiffXd>* y) const {
  VectorXd xd = drake::math::ExtractValue(x);
  Vector4d x0 = xd.head(4);
  VectorXd u0 = xd.segment(4, 1);
  Vector4d x1 = xd.segment(5, 4);
  double t = xd(xd.size() -1) / n_;

  Matrix4d Ad = alip_utils::CalcAd(H_, m_, t);
  Vector4d Bd = A_inv_ * (Ad - Matrix4d::Identity()) * B_;

  VectorXd y0 = Ad * x0 + Bd * u0 - x1;
  MatrixXd dy = MatrixXd::Zero(4, 10);
  dy.block(0, 0, 4, 4) = Ad;
  dy.col(4) = Bd;
  dy.block(0, 5, 4, 4) = -Matrix4d::Identity();
  dy.rightCols(1) = (1 / n_) *  (A_ * Ad * x0 + Ad * B_ * u0);
  *y = InitializeAutoDiff(y0, dy);
}

Matrix4d AlipDynamicsConstraint::Ad(double t) {
  return (A_ * t / n_).exp();
}

void AlipMINLP::AddTrackingCost(const vector<Eigen::VectorXd> &xd,
                                const Matrix4d &Q, const Eigen::MatrixXd& Qf) {
  DRAKE_DEMAND(xd.size() == nmodes_);
  for (int i = 1; i < nmodes_; i++){
    DRAKE_DEMAND(xd.at(i).size() == nx_ * nknots_.at(i));
    vector<Binding<QuadraticCost>> QQ;
    for (int k = 0; k < nknots_.at(i); k++){
      QQ.push_back(
          prog_->AddQuadraticErrorCost(
              Q,
              GetStateAtKnot(xd.at(i), k),
              GetStateAtKnot(xx_.at(i), k)));
    }
    tracking_costs_.push_back(QQ);
  }
  xd_ = xd;
  Q_ = Q;
  Qf_ = Qf;
  MakeTerminalCost();
}

void AlipMINLP::MakeTerminalCost(){
  terminal_cost_ = prog_->AddQuadraticCost(
      2 * Qf_, -2 * Qf_ * xd_.back().tail<4>(),
          xx_.back().tail<4>()).evaluator();
}

void AlipMINLP::AddInputCost(double R) {
  for (int i = 0; i < nmodes_; i++){
    int n = nknots_.at(i) - 1;
    input_costs_.push_back(
        prog_->AddQuadraticCost(
            R * MatrixXd::Identity(n, n),
            VectorXd::Zero(n),
            uu_.at(i))
        );
  }
  R_ = R;
}

void AlipMINLP::ActivateInitialTimeEqualityConstraint(double t) {
  DRAKE_DEMAND(built_);
  tt_(0) = t;
}

void AlipMINLP::UpdateMaximumCurrentStanceTime(double tmax) {
  tmax_.at(0) = tmax;
}

void AlipMINLP::Build(const drake::solvers::SolverOptions& options) {
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

  prog_->SetSolverOptions(options);
  built_ = true;
}

void AlipMINLP::AddMode(int n_knots) {
  std::string nm = to_string(nmodes_);
  pp_.push_back(prog_->NewContinuousVariables(np_, "pp_" + nm));
  xx_.push_back(prog_->NewContinuousVariables(nx_ * n_knots, "xx_" + nm));
  uu_.push_back(prog_->NewContinuousVariables(nu_ * (n_knots - 1), "uu_" + nm));
  nmodes_ += 1;
  nknots_.push_back(n_knots);
}

void AlipMINLP::UpdateTrackingCost(const vector<VectorXd>& xd) {
  for(int n = 0; n < nmodes_ - 1 ; n++) {
    for (int k = 0; k < nknots_.at(n); k++) {
      tracking_costs_.at(n).at(k).evaluator()->
          UpdateCoefficients( 2.0*Q_,
                             -2.0*Q_ * GetStateAtKnot(xd.at(n+1), k));
    }
  }
  terminal_cost_->UpdateCoefficients(2.0 * Qf_,
                                     -2.0 * Qf_ * xd.back().tail<4>());
  xd_ = xd;
}

void AlipMINLP::UpdateIndividualFootholdConstraint(
    int idx_mode, int idx_foothold) {
  const auto& [A, b] = footholds_.at(idx_foothold).GetConstraintMatrices();
  const auto& [A_eq, b_eq] = footholds_.at(idx_foothold).GetEqualityConstraintMatrices();

  Matrix<double, kMaxFootholdFaces, 3> bigA =
      Matrix<double, kMaxFootholdFaces, 3>::Zero();
  bigA.topRows(A.rows()) = A;
  VectorXd bigb = VectorXd::Zero(kMaxFootholdFaces);
  bigb.head(b.rows()) = b;

  footstep_c_.at(idx_mode).first.evaluator()->UpdateCoefficients(
      bigA,
      -numeric_limits<double>::infinity()*VectorXd::Ones(bigb.rows()),
      bigb
  );
  footstep_c_.at(idx_mode).second.evaluator()->UpdateCoefficients(A_eq, b_eq);
}

void AlipMINLP::MakeCapturePointConstraint(int foothold_idx) {
  const auto& [A, b] = footholds_.at(foothold_idx).GetConstraintMatrices();
  Matrix<double, 2, 4> Acp = Matrix<double, 2, 4>::Zero();
  Acp.leftCols(2) = Eigen::Matrix2d::Identity();
  Acp(0,3) = -1.0 / (m_ * sqrt(9.81 * H_));
  Acp(1,2) = -Acp(0,3);
  MatrixXd A_constraint = A.leftCols<2>() * Acp;

  capture_point_contstraint_ =
      std::make_shared<Binding<LinearConstraint>>(
      prog_->AddLinearConstraint(
          A_constraint,
          -numeric_limits<double>::infinity() * VectorXd::Ones(A_constraint.rows()),
          b, {pp_.back().head(2), xx_.back().tail(2)}
      ));
}

void AlipMINLP::UpdateFootholdConstraints(vector<int> foothold_idxs) {
  DRAKE_ASSERT(foothold_idxs.size() == nmodes_);
  for (int i = 0; i < foothold_idxs.size(); i++) {
    UpdateIndividualFootholdConstraint(i+1, foothold_idxs.at(i));
  }
}

void AlipMINLP::CalcResetMap(Eigen::Matrix<double, 4, 12> *Aeq) const {
  Matrix<double, 4, 8> A = alip_utils::CalcResetMap(H_, m_, Tds_);
  Aeq->topLeftCorner<4, 4>() = A.topLeftCorner<4, 4>();
  Aeq->topRightCorner<4, 4>() = A.topRightCorner<4, 4>();
  Aeq->block<4, 4>(0, 4) = -Matrix4d::Identity();
}

void AlipMINLP::MakeResetConstraints() {
  Matrix<double, 4, 12> A_eq = Matrix<double, 4, 12>::Zero();
  CalcResetMap(&A_eq);
  for (int i = 0; i < (nmodes_ - 1); i++) {
    reset_map_c_.push_back(
        prog_->AddLinearEqualityConstraint(
            A_eq, Vector4d::Zero(),
            {xx_.at(i).tail<4>(), xx_.at(i+1).head<4>(),
             pp_.at(i).head<2>(), pp_.at(i+1).head<2>()}));
    reset_map_c_.back().evaluator()->set_description("reset map constraint");
  }
}

void AlipMINLP::MakeFootholdConstraints() {
  for (int n = 0; n < nmodes_; n++) {
    footstep_c_.push_back(
        {
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

void AlipMINLP::MakeDynamicsConstraints() {
  for (int i = 0; i < nmodes_; i++) {
    vector<Binding<LinearEqualityConstraint>> dyn_c_this_mode;
    double t = tt_(i) / (nknots_.at(i) - 1);
    Matrix4d Ad = alip_utils::CalcAd(H_, m_, t);
    Vector4d Bd = alip_utils::CalcA(H_, m_).inverse() * (Ad - Matrix4d::Identity()) * Vector4d::UnitW();

    Matrix<double, 4, 9> Adyn = Matrix<double, 4, 9>::Zero();
    Adyn.leftCols<4>() = Ad;
    Adyn.col(4) = Bd;
    Adyn.rightCols<4>() = -Matrix4d::Identity();

    for (int k = 0; k < nknots_.at(i) - 1; k++) {
      dyn_c_this_mode.push_back(
          prog_->AddLinearEqualityConstraint(
              Adyn, Vector4d::Zero(),
              {GetStateAtKnot(xx_.at(i), k),
               GetInputAtKnot(uu_.at(i), k),
               GetStateAtKnot(xx_.at(i), k+1)}));
      dyn_c_this_mode.back().evaluator()->
      set_description("dynamics" + std::to_string(i) + "_" + std::to_string(k));
    }
    dynamics_c_.push_back(dyn_c_this_mode);
  }
}

void AlipMINLP::MakeInitialFootstepConstraint() {
  initial_foot_c_ = prog_->AddLinearEqualityConstraint(
      Matrix3d::Identity(), Vector3d::Zero(), pp_.at(0)).evaluator();
  initial_foot_c_->set_description("initial_footstep");
}

void AlipMINLP::MakeInitialStateConstraint() {
  initial_state_c_ = prog_->AddLinearEqualityConstraint(
      Matrix4d::Identity(),
      Vector4d::Zero(),
      xx_.front().head<4>()
  ).evaluator();

  initial_state_c_->set_description("initial_state");
}

void AlipMINLP::MakeInputBoundConstaints() {
  DRAKE_DEMAND(umax_ >= 0 && !uu_.empty());
  int i = 0;
  for (const auto& vec : uu_) {
      input_bounds_c_.push_back(
          prog_->AddBoundingBoxConstraint(-umax_, umax_, vec)
      );
      i++;
      input_bounds_c_.back().evaluator()->set_description(
          "input bounds" + std::to_string(i));
  }
}

void AlipMINLP::MakeWorkspaceConstraints() {
  for (int n = 1; n < nmodes_; n++) {
    for (int k = 0; k < nknots_.at(n); k++) {
      prog_->AddLinearConstraint(
          MatrixXd::Identity(2, 2),
          Vector2d(-0.8, -0.75),
          Vector2d(0.8, 0.75),
          GetStateAtKnot(xx_.at(n), k).head<2>());
    }
  }
}

void AlipMINLP::MakeNextFootstepReachabilityConstraint() {
  next_step_reach_c_fixed_ = prog_->AddLinearConstraint(
      MatrixXd::Zero(1,3),
      -numeric_limits<double>::infinity()*VectorXd::Ones(1),
      VectorXd::Zero(1), pp_.at(1))
  .evaluator();
  next_step_reach_c_fixed_->set_description("next footstep workspace constraint");
}

void AlipMINLP::MakeNoCrossoverConstraint() {
  for (int i = 0; i < nmodes_ - 1 ; i++) {
    int stance = i % 2 == 0 ? -1 : 1;
    Eigen::RowVector2d A = {stance, -stance};
    no_crossover_constraint_.push_back(
        prog_->AddLinearConstraint(
            A, -numeric_limits<double>::infinity(), -0.04,
            {pp_.at(i).segment(1,1), pp_.at(i+1).segment(1,1)})
    );
    no_crossover_constraint_.back().evaluator()->set_description(
        "no crossover constraint" + std::to_string(i));
  }
}

void AlipMINLP::UpdateInitialGuess(const Eigen::Vector3d &p0,
                                   const Eigen::Vector4d &x0) {
  // Update state initial guess
  vector<VectorXd> xg = xd_;

  // Set the initial guess for the current mode based on limited time
  VectorXd xx = VectorXd (nx_ * nknots_.front());
  xx.head<4>() = x0;
  Matrix4d Ad = alip_utils::CalcAd(H_, m_, tt_(0) / (nknots_.front() - 1));
  for (int i = 1; i < nknots_.front(); i++) {
    GetStateAtKnot(xx, i) = Ad * GetStateAtKnot(xx, i-1);
  }
  xg.front() = xx;

  for (int n = 0; n < nmodes_; n++) {
    for (int k = 0; k < nknots_.at(n); k++) {
      prog_->SetInitialGuess(
          GetStateAtKnot(xx_.at(n), k),
          GetStateAtKnot(xg.at(n), k));
    }
  }
  Vector3d ptemp = p0;
  prog_->SetInitialGuess(pp_.front(), p0);
  for(int n = 1; n < nmodes_; n++) {
    Vector2d p1 = (xd_.at(n-1).tail<4>() - xd_.at(n).head<4>()).head<2>() + ptemp.head<2>();
    prog_->SetInitialGuess(pp_.at(n).head<2>(), p1);
    ptemp.head<2>() = p1;
  }
}

void AlipMINLP::UpdateInitialGuess() {
  DRAKE_DEMAND(
      solution_.first.is_success() ||
      solution_.first.get_solution_result() ==
          drake::solvers::kIterationLimit);
  prog_->SetInitialGuessForAllVariables(solution_.first.GetSolution());
}

void AlipMINLP::UpdateNextFootstepReachabilityConstraint(const geometry::ConvexFoothold &workspace) {

  const auto& [Af, bf] = workspace.GetConstraintMatrices();
  double neg_inf  = -numeric_limits<double>::infinity();
  next_step_reach_c_fixed_->UpdateCoefficients(
      Af, neg_inf * VectorXd::Ones(bf.rows()), bf);
}

void AlipMINLP::UpdateNoCrossoverConstraint() {
  for (auto& constraint: no_crossover_constraint_) {
    auto A = constraint.evaluator()->GetDenseA();
    A *= -1;
    constraint.evaluator()->UpdateCoefficients(
        A,
        -numeric_limits<double>::infinity()*VectorXd::Ones(1),
        VectorXd::Zero(1));
  }
}

void AlipMINLP::UpdateModeTiming(bool take_sqp_step) {
  tt_(0) = td_.front();
  if (take_sqp_step) {
    UpdateTimingGradientStep();
  }
  UpdateDynamicsConstraints();
}

void AlipMINLP::UpdateModeTimingsOnTouchdown() {
  for (int i = 0; i < nmodes_ - 1; i++){
    tt_(i) = tt_(i+1);
  }
  tt_.tail(1)(0) = td_.back();
}

void AlipMINLP::UpdateTimingGradientStep() {
  Matrix4d A = alip_utils::CalcA(H_, m_);
  Vector4d B = Vector4d::UnitW();

  for (int n = 0; n < nmodes_; n++) {
    double dLdt_n = 0;
    Matrix4d Ad = alip_utils::CalcAd(H_, m_, tt_(n) / (nknots_.at(n) - 1));
    for (int k = 0; k < nknots_.at(n) - 1; k++) {
      Vector4d nu = GetStateAtKnot(solution_.second.at(n), k);
      Vector4d x = solution_.first.GetSolution(GetStateAtKnot(xx_.at(n), k));
      VectorXd u = solution_.first.GetSolution(GetInputAtKnot(uu_.at(n), k));
      dLdt_n += (1.0 / (nknots_.at(n) - 1)) * nu.dot(A * Ad * x + Ad * B * u);
    }
    double tnew = tt_(n) - 1e-7 * dLdt_n;
    tt_(n) =  std::isnan(tnew) ? tt_(n)  : std::clamp(tnew, tmin_.at(n), tmax_.at(n));
  }
}

void AlipMINLP::UpdateDynamicsConstraints() {
  for (int n = 0; n < nmodes_; n++) {
    int nk =  nknots_.at(n) - 1;
    double t = tt_(n) / nk;
    Matrix4d Ad = alip_utils::CalcAd(H_, m_, t);
    Vector4d Bd = alip_utils::CalcBd(H_, m_, t);
    Matrix<double, 4, 9> Adyn = Matrix<double, 4, 9>::Zero();
    Adyn.leftCols<4>() = Ad;
    Adyn.col(4) = Bd;
    Adyn.rightCols<4>() = -Matrix4d::Identity();
    Vector4d bdyn = Vector4d::Zero();
    for (int k = 0; k < nk; k++) {
      dynamics_c_.at(n).at(k).evaluator()->UpdateCoefficients(Adyn, bdyn);
    }
  }
}

void AlipMINLP::SolveOCProblemAsIs() {
  mode_sequnces_ = GetPossibleModeSequences();
  vector<pair<MathematicalProgramResult, vector<VectorXd>>> solutions;
  if (mode_sequnces_.empty()) {
    vector<VectorXd> dual_solutions;
    const auto sol = solver_.Solve(*prog_);
    for (int n = 0; n < nmodes_; n++) {
      VectorXd duals = VectorXd::Zero((nknots_.at(n) - 1) * nx_);
      if (sol.is_success()) {
        for (int k = 0; k < nknots_.at(n) - 1; k++) {
          GetStateAtKnot(duals, k) =
              sol.GetDualSolution<LinearEqualityConstraint>(
                  dynamics_c_.at(n).at(k));
        }
      }
      dual_solutions.push_back(duals);
    }
    solutions.push_back({sol, dual_solutions}); // NOLINT
  } else {
    for (auto& seq: mode_sequnces_) {
      UpdateFootholdConstraints(seq);
      vector<VectorXd> dual_solutions;
      const auto sol = solver_.Solve(*prog_);

      for (int n = 0; n < nmodes_; n++) {
       VectorXd duals = VectorXd::Zero((nknots_.at(n) - 1) * nx_);
        if (sol.is_success()) {
          for (int k = 0; k < nknots_.at(n) - 1; k++) {
            GetStateAtKnot(duals, k) =
                sol.GetDualSolution<LinearEqualityConstraint>(
                    dynamics_c_.at(n).at(k));
          }
        }
        dual_solutions.push_back(duals);
      }
      solutions.push_back({sol, dual_solutions}); // NOLINT
    }
  }

  std::sort(
      solutions.begin(), solutions.end(),
      [](
    const pair<MathematicalProgramResult, vector<VectorXd>>& lhs,
    const pair<MathematicalProgramResult, vector<VectorXd>>& rhs){
    return lhs.first.get_optimal_cost() < rhs.first.get_optimal_cost() && lhs.first.is_success();
  });

  if (solutions.front().first.is_success()) {
    solution_ = solutions.front();
  } else {
    std::cout << "solve failed with code " <<
              solutions.front().first.get_solution_result() << std::endl;
  }
  if (std::isnan(solution_.first.get_optimal_cost())) {
    std::cout << "NaNs slipped through unnoticed. Dumping all constraints\n";
    const auto& constraints = prog_->GetAllConstraints();
    solvers::print_constraint(constraints);
  }
}

void AlipMINLP::CalcOptimalFootstepPlan(const Eigen::Vector4d &x,
                                        const Eigen::Vector3d &p,
                                        bool warmstart) {
  DRAKE_DEMAND(built_);
  if (warmstart &&
     (solution_.first.is_success() ||
     solution_.first.get_solution_result() == drake::solvers::kIterationLimit)){
    UpdateInitialGuess();
  } else {
    UpdateInitialGuess(p, x);
  }
  initial_state_c_->UpdateCoefficients(Matrix4d::Identity(), x);
  initial_foot_c_->UpdateCoefficients(Matrix3d::Identity(), p);
  SolveOCProblemAsIs();
}

vector<VectorXd> AlipMINLP::MakeXdesTrajForVdes(
    const Vector2d& vdes, double step_width, double Ts, double nk,
    alip_utils::Stance stance) const {
  double omega = sqrt(9.81 / H_);
  double frac = 1 / (m_ * H_ * omega);
  double Ly = H_ * m_ * vdes(0);
  double Lx = H_ * m_ * omega * step_width * tanh(omega * Ts / 2);
  double dLx = - H_ * m_ * vdes(1);
  Matrix4d Ad = alip_utils::CalcAd(H_, m_, (Ts / (nk-1)));
  vector<VectorXd> xx;

  double stance_sign = stance == alip_utils::Stance::kLeft ? -1.0 : 1.0;
  Eigen::MatrixPower<Matrix4d> APow(Ad);
  for (int i = 0; i < nmodes_; i++) {
    double s = stance_sign * ((i % 2 == 0) ? 1.0 : -1.0);
    Vector4d x0(
        -frac * tanh(omega * Ts /2) * Ly,
        s / 2 * step_width,
        s / 2 * Lx + dLx,
        Ly);
    VectorXd x = VectorXd::Zero(nx_ * nknots_.at(i));
    for (int k = 0; k < nk; k++) {
      GetStateAtKnot(x, k) = APow(k) * x0;
    }
    xx.push_back(x);
  }
  return xx;
}

VectorXd AlipMINLP::MakeXdesTrajForCurrentStep(
    const Vector2d &vdes, double t_current, double t_remain,
    double Ts, double step_width, alip_utils::Stance stance, double nk) const {

  double omega = sqrt(9.81 / H_);
  double frac = 1 / (m_ * H_ * omega);
  double Ly = H_ * m_ * vdes(0);
  double Lx = H_ * m_ * omega * step_width * tanh(omega * Ts / 2);
  double dLx = -H_ * m_ * vdes(1);

  Matrix4d Ad0 = alip_utils::CalcAd(H_, m_, t_current);
  Matrix4d Ad = alip_utils::CalcAd(H_, m_, t_remain / (nk-1));
  Eigen::MatrixPower<Matrix4d> APow(Ad);

  double stance_sign = stance == alip_utils::Stance::kLeft ? -1.0 : 1.0;

  Vector4d x0(
      -frac * tanh(omega * Ts /2) * Ly,
      stance_sign / 2 * step_width,
      stance_sign / 2 * Lx + dLx,
      Ly);
  x0 = Ad0 * x0;
  VectorXd x = VectorXd::Zero(nx_ * nknots_.front());
  for (int k = 0; k < nk; k++) {
    GetStateAtKnot(x, k) = APow(k) * x0;
  }
  return x;
}

vector<vector<int>> AlipMINLP::GetPossibleModeSequences() {
  return alip_utils::cartesian_product(footholds_.size(), nmodes_ - 1);
}

double AlipMINLP::solve_time() const {
  return solution_.first.get_solver_details<OsqpSolver>().run_time;
};

vector<Vector3d> AlipMINLP::GetFootstepSolution() const {
  vector<Vector3d> pp;
  for (auto& p : pp_){
    pp.emplace_back(solution_.first.GetSolution(p));
  }
  return pp;
}

vector<Vector3d> AlipMINLP::GetFootstepGuess() const {
  vector<Vector3d> pp;
  for (auto& p : pp_){
    pp.emplace_back(prog_->GetInitialGuess(p));
  }
  return pp;
}

vector<VectorXd> AlipMINLP::GetStateSolution() const {
  vector<VectorXd> xx;
  for(auto& x : xx_) {
    xx.push_back(solution_.first.GetSolution(x));
  }
  return xx;
}

vector<VectorXd> AlipMINLP::GetStateGuess() const {
  vector<VectorXd> xx;
  for(auto& x : xx_) {
    xx.push_back(prog_->GetInitialGuess(x));
  }
  return xx;
}

vector<VectorXd> AlipMINLP::GetInputSolution() const {
  vector<VectorXd> uu;
  for(auto& u : uu_) {
    uu.push_back(solution_.first.GetSolution(u));
  }
  return uu;
}

vector<VectorXd> AlipMINLP::GetInputGuess() const {
  vector<VectorXd> uu;
  for(auto& u : uu_) {
    uu.push_back(prog_->GetInitialGuess(u));
  }
  return uu;
}

VectorXd AlipMINLP::GetTimingSolution() const {
  VectorXd tt = VectorXd::Zero(nmodes_);
  return tt_;
}

VectorXd AlipMINLP::GetTimingGuess() const {
  return tt_;
}

VectorXd AlipMINLP::GetDesiredTiming() const {
  const VectorXd tt = Eigen::Map<const VectorXd>(td_.data(), td_.size());
  return tt; // NOLINT(performance-no-automatic-move)
}
}