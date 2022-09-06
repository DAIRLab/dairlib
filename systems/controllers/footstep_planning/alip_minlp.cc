#include <algorithm>

#include "alip_minlp.h"
#include "alip_utils.h"

#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/solve.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/nlopt_solver.h"
#include "drake/solvers/osqp_solver.h"

namespace dairlib::systems::controllers{

using std::to_string;
using std::vector;
using std::numeric_limits;

using Eigen::Matrix;
using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::MatrixXd;
using Eigen::Matrix4d;
using Eigen::Matrix3d;

using drake::AutoDiffXd;
using drake::AutoDiffVecXd;
using drake::math::InitializeAutoDiff;

using drake::solvers::Solve;
using drake::solvers::Binding;
using drake::solvers::IpoptSolver;
using drake::solvers::NloptSolver;
using drake::solvers::SnoptSolver;
using drake::solvers::QuadraticCost;
using drake::solvers::DecisionVariable;
using drake::solvers::LinearConstraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::BoundingBoxConstraint;
using drake::solvers::VectorXDecisionVariable;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::LinearEqualityConstraint;

using drake::VectorX;
using drake::Vector4;

AlipDynamicsConstraint::AlipDynamicsConstraint(double m, double H, double n) :
    NonlinearConstraint<AutoDiffXd>(
        4, 10, Vector4d::Zero(), Vector4d::Zero(), "dynamics"),
    m_(m), H_(H), n_(n) {

  A_ = alip_utils::CalcA(H_, m_);
  A_inv_ = A_.inverse();
  B_ = Vector4d(0, 0, 0, 1);

}

void AlipDynamicsConstraint::EvaluateConstraint(
    const Eigen::Ref<const drake::VectorX<AutoDiffXd>> &x,
          drake::VectorX<AutoDiffXd>* y) const {
  VectorXd xd = drake::math::ExtractValue(x);
  Vector4d x0 = xd.head(4);
  VectorXd u0 = xd.segment(4, 1);
  Vector4d x1 = xd.segment(5, 4);
  double t = xd(xd.size() -1) / n_;

  Matrix4d Ad = (t * A_).exp();
  Vector4d Bd = A_inv_ * (Ad - Matrix4d::Identity()) * B_;

  VectorXd y0 = Ad * x0 + Bd * u0 - x1;
  MatrixXd dy = MatrixXd::Zero(4, 10);
  dy.block(0, 0, 4, 4) = Ad;
  dy.col(4) = Bd;
  dy.block(0, 5, 4, 4) = -Matrix4d::Identity();
  dy.rightCols(1) = (1 / n_) *  (A_ * Ad * x0 + Ad * B_ * u0);
//  std::cout << dy << "\n" << std::endl;
  *y = InitializeAutoDiff(y0, dy);
}

Matrix4d AlipDynamicsConstraint::Ad(double t) {
  return (A_ * t / n_).exp();
}

void AlipMINLP::AddTrackingCost(const vector<vector<Eigen::Vector4d>> &xd,
                                const Matrix4d &Q) {
  DRAKE_DEMAND(xd.size() == nmodes_);
  for (int i = 0; i < nmodes_; i++){
    DRAKE_DEMAND(xd.at(i).size() == nknots_.at(i));
    vector<Binding<QuadraticCost>> QQ;
    for (int k = 0; k < nknots_.at(i); k++){
      QQ.push_back(prog_->AddQuadraticErrorCost(Q, xd.at(i).at(k), xx_.at(i).at(k)));
    }
    tracking_costs_.push_back(QQ);
  }
  xd_ = xd;
  Q_ = Q;
}

void AlipMINLP::MakeTerminalCost(){
  terminal_cost_ = prog_->AddQuadraticCost(
      20.0*Q_, -20.0*Q_ *xd_.back().back(), xx_.back().back()).evaluator();
}

void AlipMINLP::AddInputCost(double R) {
  for (int i = 0; i < nmodes_; i++){
    vector<Binding<QuadraticCost>> RR;
    for (int k = 0; k < nknots_.at(i) -1; k++){
      RR.push_back(
          prog_->AddQuadraticCost(
              R*MatrixXd::Identity(nu_, nu_),
              VectorXd::Zero(nu_), uu_.at(i).at(k)));
    }
    tracking_costs_.push_back(RR);
  }
  R_ = R;
}

void AlipMINLP::ActivateInitialTimeConstraint(double t) {
  DRAKE_DEMAND(built_ && initial_time_c_);
  initial_time_c_->UpdateCoefficients(MatrixXd::Ones(1,1), t * VectorXd::Ones(1));
  ts_bounds_c_.front().evaluator()->UpdateLowerBound(VectorXd::Zero(1));
}

void AlipMINLP::DeactivateInitialTimeConstraint() {
  if (initial_time_c_) {
    initial_time_c_->UpdateCoefficients(MatrixXd::Zero(1,1), VectorXd::Zero(1));
  }
  ts_bounds_c_.front().evaluator()->UpdateLowerBound(tmin_ * VectorXd::Ones(1));
}

void AlipMINLP::Build() {
  mode_sequnces_ = GetPossibleModeSequences();
  for (int i = 0; i < nmodes_; i++) {
    tt_.push_back(prog_->NewContinuousVariables(1, "t"));
  }
  MakeDynamicsConstraints();
  MakeResetConstraints();
  MakeTimingBoundsConstraint();
  MakeInitialStateConstraint();
  MakeInitialFootstepConstraint();
  MakeInitialTimeConstraint();
  MakeTerminalCost();
  built_ = true;
}

void AlipMINLP::AddMode(int nk) {
  pp_.push_back(prog_->NewContinuousVariables(np_, "pp_" + to_string(nmodes_)));
  vector<VectorXDecisionVariable> xknots;
  vector<VectorXDecisionVariable> uknots;
  for (int i = 0; i < nk; i++) {
    xknots.push_back(prog_->NewContinuousVariables(
        nx_, "xx_" + to_string(nmodes_) + "_" + to_string(i) ));
  }
  for (int i = 0; i < (nk -1) ; i++) {
    uknots.push_back(prog_->NewContinuousVariables(
        nu_, "uu_" + to_string(nmodes_) + "_" + to_string(i)));
  }
  nmodes_ += 1;
  nknots_.push_back(nk);
  xx_.push_back(xknots);
  uu_.push_back(uknots);
}

void AlipMINLP::UpdateTrackingCost(const vector<vector<Vector4d>> &xd) {
  for(int n = 0; n < nmodes_; n++) {
    for (int k = 0; k < nknots_.at(n); k++) {
      tracking_costs_.at(n).at(k).evaluator()->
          UpdateCoefficients(2.0*Q_, -2.0*Q_ * xd.at(n).at(k));
    }
  }
  terminal_cost_->UpdateCoefficients(20.0 * Q_, -20.0 * Q_ * xd.back().back());
  xd_ = xd;
}

void AlipMINLP::MakeIndividualFootholdConstraint(int idx_mode,
                                                 int idx_foothold) {
  const auto& [A, b] = footholds_.at(idx_foothold).GetConstraintMatrices();
  const auto& [A_eq, b_eq] = footholds_.at(idx_foothold).GetEqualityConstraintMatrices();
  footstep_c_.push_back( // NOLINT(modernize-use-emplace)
      {prog_->AddLinearConstraint(
          A, -numeric_limits<double>::infinity()*VectorXd::Ones(b.rows()),
          b, pp_.at(idx_mode)),
       prog_->AddLinearEqualityConstraint(
           A_eq, b_eq, pp_.at(idx_mode)
           )}
      );
}

void AlipMINLP::MakeFootstepConstraints(vector<int> foothold_idxs) {
  DRAKE_ASSERT(foothold_idxs.size() == nmodes_);
  for (int i = 0; i < foothold_idxs.size(); i++) {
    MakeIndividualFootholdConstraint(i+1, foothold_idxs.at(i));
  }
}

void AlipMINLP::MakeResetConstraints() {
  Matrix<double, 4, 12> A_eq;
  A_eq.topLeftCorner<4, 4>() = Matrix4d::Identity();
  A_eq.block<4, 4>(0, 4) = -Matrix4d::Identity();
  A_eq.block<2, 2>(0, 8) = Eigen::Matrix2d::Identity();
  A_eq.block<2, 2>(0, 10) = -Eigen::Matrix2d::Identity();
  for (int i = 0; i < (nmodes_ - 1); i++) {
    reset_map_c_.push_back(
        prog_->AddLinearEqualityConstraint(
            A_eq, Vector4d::Zero(),
            {xx_.at(i).back(), xx_.at(i+1).front(),
             pp_.at(i).head<2>(), pp_.at(i+1).head<2>()}));
  }
}

void AlipMINLP::MakeDynamicsConstraints() {
  dynamics_evaluator_ = std::make_shared<AlipDynamicsConstraint>(
      m_, H_, nknots_.front() - 1.0);
  std::unordered_map<int, double> constraint_scaling;
  constraint_scaling.insert({2, 1.0 /10.0});
  constraint_scaling.insert({3, 1.0/20.0});
  dynamics_evaluator_->SetConstraintScaling(constraint_scaling);
  for (int i = 0; i < nmodes_; i++) {
    vector<Binding<drake::solvers::Constraint>> dyn_c_this_mode{};
    for (int k = 0; k < nknots_.at(i)-1; k++) {
      dyn_c_this_mode.push_back(
          prog_->AddConstraint(
              dynamics_evaluator_,
              {xx_.at(i).at(k), uu_.at(i).at(k), xx_.at(i).at(k+1), tt_.at(i)}));
    }
    dynamics_c_.push_back(dyn_c_this_mode);
  }
}

void AlipMINLP::MakeInitialFootstepConstraint() {
  initial_foot_c_ = prog_->AddLinearEqualityConstraint(
      Matrix3d::Identity(), Vector3d::Zero(), pp_.at(0)).evaluator();
}

void AlipMINLP::MakeInitialStateConstraint() {
  initial_state_c_ = prog_->AddLinearEqualityConstraint(
      Matrix4d::Identity(), Vector4d::Zero(), xx_.at(0).at(0)).evaluator();
}

void AlipMINLP::MakeTimingBoundsConstraint() {
  DRAKE_DEMAND(tmin_ > 0);
  for (int i = 0; i < nmodes_; i++) {
    ts_bounds_c_.push_back(
      prog_->AddBoundingBoxConstraint(
          tmin_ * VectorXd::Ones(1), VectorXd::Ones(1), tt_.at(i)));
  }
}

void AlipMINLP::MakeInitialTimeConstraint() {
  initial_time_c_ = prog_->AddLinearEqualityConstraint(
      MatrixXd::Zero(1, 1), VectorXd::Zero(1), tt_.front()
      ).evaluator();
}

void AlipMINLP::ClearFootholdConstraints() {
  for (auto & i : footstep_c_) {
    prog_->RemoveConstraint(i.first);
    prog_->RemoveConstraint(i.second);
  }
  footstep_c_.clear();
}

void AlipMINLP::UpdateInitialGuess(const Eigen::Vector3d &p0,
                                   const Eigen::Vector4d &x0) {
  // Update state initial guess
  DRAKE_DEMAND(!td_.empty());

    std::vector<Vector4d> xx;
    xx.push_back(x0);
    Matrix4d Ad = dynamics_evaluator_->Ad(td_.front());
    for (int i = 1; i < nknots_.front(); i++) {
      xx.push_back(Ad * xx.at(i-1));
    }
    xd_.front() = xx;
  for(int n = 0; n < nmodes_; n++) {
    for (int k = 0; k < nknots_.at(n); k++) {
      prog_->SetInitialGuess(xx_.at(n).at(k), xd_.at(n).at(k));
    }
    prog_->SetInitialGuess(tt_.at(n), td_.at(n) * VectorXd::Ones(1));
  }
  Vector3d ptemp = p0;
  for(int n = 1; n < nmodes_; n++) {
    Vector2d p1 = (xd_.at(n-1).back() - xd_.at(n).front()).head<2>() + ptemp.head<2>();
    prog_->SetInitialGuess(pp_.at(n).head<2>(), p1);
    ptemp.head<2>() = p1;
  }
}

void AlipMINLP::UpdateInitialGuess() {
  DRAKE_DEMAND(solutions_.front().is_success());
  prog_->SetInitialGuessForAllVariables(solutions_.front().GetSolution());
}

void AlipMINLP::SolveOCProblemAsIs() {
  auto solver = SnoptSolver();
//  prog_->SetSolverOption(IpoptSolver::id(), "print_level", 0);
  prog_->SetSolverOption(SnoptSolver::id(), "Major Iterations Limit", 1);
  prog_->SetSolverOption(SnoptSolver::id(), "Major feasibility tolerance", 1e-5);
  prog_->SetSolverOption(SnoptSolver::id(), "Major optimality tolerance", 1e-5);
  prog_->SetSolverOption(SnoptSolver::id(), "Print file", "../snopt_alip.out");

  if (mode_sequnces_.empty()) {
    solutions_.push_back(solver.Solve(*prog_));
  } else {
    for (auto& seq: mode_sequnces_) {
      MakeFootstepConstraints(seq);
      solutions_.push_back(solver.Solve(*prog_));
      ClearFootholdConstraints();
    }
  }
  std::sort(
      solutions_.begin(), solutions_.end(),
      [](const MathematicalProgramResult& lhs, const MathematicalProgramResult& rhs){
    return lhs.get_optimal_cost() < rhs.get_optimal_cost();
  });
}

void AlipMINLP::CalcOptimalFootstepPlan(const Eigen::Vector4d &x,
                                        const Eigen::Vector3d &p,
                                        bool warmstart) {
  DRAKE_DEMAND(built_);
  if (warmstart && !solutions_.empty() && solutions_.front().is_success()) {
    UpdateInitialGuess();
  } else {
    UpdateInitialGuess(p, x);
  }
  dynamics_evaluator_->set_H(H_);
  dynamics_evaluator_->set_m(m_);
  initial_state_c_->UpdateCoefficients(Matrix4d::Identity(), x);
  initial_foot_c_->UpdateCoefficients(Matrix3d::Identity(), p);
  SolveOCProblemAsIs();
}

vector<vector<Vector4d>> AlipMINLP::MakeXdesTrajForVdes(
    const Vector2d& vdes, double step_width, double Ts, double nk,
    int stance) const {
  double omega = sqrt(9.81 / H_);
  double frac = 1 / (m_ * H_ * omega);
  double Ly = H_ * m_ * vdes(0);
  double Lx = H_ * m_ * omega * step_width * atanh(omega * Ts / 2);
  double dLx = - H_ * m_ * vdes(1);
  Matrix4d Ad = ((Ts / (nk-1)) * alip_utils::CalcA(H_, m_)).exp();
  vector<vector<Vector4d>> xx;

  Eigen::MatrixPower<Matrix4d> APow(Ad);
  for (int i = 0; i < nmodes_; i++) {
    double s = stance * ((i % 2 == 0) ? 1.0 : -1.0);
    Vector4d x0(
        -frac * tanh(omega * Ts /2) * Ly,
        s / 2 * step_width,
        s / 2 * Lx + dLx,
        Ly);
    vector<Vector4d> x;
    for (int k = 0; k < nk; k++) {
      x.emplace_back(APow(k) * x0);
    }
    xx.push_back(x);
  }
  return xx;
}

vector<Vector4d> AlipMINLP::MakeXdesTrajForCurrentStep(
    const Vector2d &vdes, double t_current, double t_remain,
    double Ts, double step_width, int stance, double nk) const {

  double omega = sqrt(9.81 / H_);
  double frac = 1 / (m_ * H_ * omega);
  double Ly = H_ * m_ * vdes(0);
  double Lx = H_ * m_ * omega * step_width * atanh(omega * Ts / 2);
  double dLx = - H_ * m_ * vdes(1);

  MatrixXd Ad0 = (t_current * alip_utils::CalcA(H_, m_)).exp();
  Matrix4d Ad = ((t_remain / (nk-1)) * alip_utils::CalcA(H_, m_)).exp();
  Eigen::MatrixPower<Matrix4d> APow(Ad);

  Vector4d x0(
      -frac * tanh(omega * Ts /2) * Ly,
      static_cast<double>(stance) / 2 * step_width,
      static_cast<double>(stance) / 2 * Lx + dLx,
      Ly);
  x0 = Ad0 * x0;
  vector<Vector4d> x;
  for (int k = 0; k < nk; k++) {
    x.emplace_back(APow(k) * x0);
  }
  return x;
}

vector<vector<int>> AlipMINLP::GetPossibleModeSequences() {
  return cartesian_product(footholds_.size(), nmodes_ - 1);
}

vector<Vector3d> AlipMINLP::GetFootstepSolution() const {
  vector<Vector3d> pp;
  for (auto& p : pp_){
    pp.emplace_back(solutions_.front().GetSolution(p));
  }
  return pp;
}

vector<vector<Vector4d>> AlipMINLP::GetStateSolution() const {
  vector<vector<Vector4d>> xx;
  for(auto& x : xx_) {
    vector<Vector4d> xknots;
    for (auto& knot : x) {
      xknots.emplace_back(solutions_.front().GetSolution(knot));
    }
    xx.push_back(xknots);
  }
  return xx;
}

vector<vector<VectorXd>> AlipMINLP::GetInputSolution() const {
  vector<vector<VectorXd>> uu;
  for(auto& u : uu_) {
    vector<VectorXd> uknots;
    for (auto& knot : u) {
      uknots.push_back(solutions_.front().GetSolution(knot));
    }
    uu.push_back(uknots);
  }
  return uu;
}

VectorXd AlipMINLP::GetTimingSolution() const {
  VectorXd tt = VectorXd::Zero(nmodes_);
  for (int i = 0; i < nmodes_; i++) {
    tt.segment(i, 1) = solutions_.front().GetSolution(tt_.at(i));
  }
  return tt;
}

}