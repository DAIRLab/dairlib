#include <algorithm>

#include "alip_minlp.h"
#include "alip_utils.h"

#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/solve.h"

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
using drake::solvers::QuadraticCost;
using drake::solvers::DecisionVariable;
using drake::solvers::LinearConstraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::BoundingBoxConstraint;
using drake::solvers::VectorXDecisionVariable;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::LinearEqualityConstraint;

AlipDynamicsConstraint::AlipDynamicsConstraint(double m, double H) :
    NonlinearConstraint<AutoDiffXd>(
        4, 10, Vector4d::Zero(), Vector4d::Zero(), "dynamics"), m_(m), H_(H) {
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
  double t = xd(xd.size() -1);

  Matrix4d Ad = (t * A_).exp();
  Vector4d Bd = A_inv_ * (Ad - Matrix4d::Identity()) * B_;

  VectorXd y0 = Ad * x0 + Bd * u0 - x1;
  MatrixXd dy = MatrixXd::Zero(4, 10);
  dy.block(0, 0, 4, 4) = Ad;
  dy.col(4) = Bd;
  dy.block(0, 5, 4, 4) = -Matrix4d::Identity();
  dy.rightCols(1) = A_ * (Ad * x0 + A_ * Ad * B_ * u0);
  *y = InitializeAutoDiff(y0, dy);
}

//// Expensive copy constructor, but it must be done!
AlipMINLP::AlipMINLP(const AlipMINLP &rhs) {
  *this = rhs;
}

AlipMINLP& AlipMINLP::operator=(const AlipMINLP &rhs) {
  if (this != &rhs) {
    m_ = rhs.m_;
    H_ = rhs.H_;
    AddFootholds(rhs.footholds_);
    for (int i = 0; i < rhs.nmodes_; i++) {
      AddMode(rhs.nknots_.at(i));
    }
    if (!rhs.xd_.empty()) {
      AddTrackingCost(rhs.xd_, rhs.Q_);
    }
    if (rhs.R_ > 0) {
      AddInputCost(rhs.R_);
    }
    if (rhs.built_) {
      Build();
    }
  }
  return *this;
}

void AlipMINLP::AddTrackingCost(const vector<vector<Eigen::Vector4d>> &xd,
                                const Matrix4d &Q) {
  for (int i = 0; i < nmodes_; i++){
    vector<Binding<QuadraticCost>> QQ;
    for (int k = 0; k < nknots_.at(i); k++){
      QQ.push_back(prog_->AddQuadraticErrorCost(Q, xd.at(i).at(k), xx_.at(i).at(k)));
    }
    tracking_costs_.push_back(QQ);
  }
  xd_ = xd;
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
  A_eq.block<2, 2>(0, 10) = Eigen::Matrix2d::Identity();
  for (int i = 0; i < (nmodes_ - 1); i++) {
    reset_map_c_.push_back(
        prog_->AddLinearEqualityConstraint(
            A_eq, Vector4d::Zero(),
            {xx_.at(i).back(), xx_.at(i+1).front(),
             pp_.at(i).head<2>(), pp_.at(i+1).head<2>()}));
  }
}

void AlipMINLP::MakeDynamicsConstraints() {
  auto dynamics_constraint = std::make_shared<AlipDynamicsConstraint>(m_, H_);
  for (int i = 0; i < nmodes_; i++) {
    vector<Binding<drake::solvers::Constraint>> dyn_c_this_mode{};
    for (int k = 0; k < nknots_.at(i)-1; k++) {
      dyn_c_this_mode.push_back(
          prog_->AddConstraint(
              dynamics_constraint,
              {xx_.at(i).at(k), uu_.at(i).at(k), xx_.at(i).at(k+1), tt_.at(i)}));
    }
    dynamics_c_.push_back(dyn_c_this_mode);
  }
}

void AlipMINLP::MakeInitialFootstepConstraint() {
  initial_foot_c_ = prog_->AddLinearEqualityConstraint(
      Matrix3d::Identity(), Vector3d::Zero(), pp_.at(0)).evaluator().get();
}

void AlipMINLP::MakeInitialStateConstraint() {
  initial_state_c_ = prog_->AddLinearEqualityConstraint(
      Matrix4d::Identity(), Vector4d::Zero(), xx_.at(0).at(0)).evaluator().get();
}

void AlipMINLP::MakeTimingBoundsConstraint() {
  for (int i = 0; i < nmodes_; i++) {
    ts_bounds_c_.push_back(
      prog_->AddBoundingBoxConstraint(
          VectorXd::Zero(1), VectorXd::Ones(1), tt_.at(i)));
  }
}

void AlipMINLP::ClearFootholdConstraints() {
  for (auto & i : footstep_c_) {
    prog_->RemoveConstraint(i.first);
    prog_->RemoveConstraint(i.second);
  }
  footstep_c_.clear();
}

void AlipMINLP::SolveOCProblemAsIs() {
  for (auto& seq: mode_sequnces_) {
    MakeFootstepConstraints(seq);
    solutions_.push_back(Solve(*prog_));
    ClearFootholdConstraints();
  }
  std::sort(
      solutions_.begin(), solutions_.end(),
      [](const MathematicalProgramResult& lhs, const MathematicalProgramResult& rhs){
    return lhs.get_optimal_cost() < rhs.get_optimal_cost();
  });
}

void AlipMINLP::CalcOptimalFootstepPlan(const Eigen::Vector4d &x,
                                        const Eigen::Vector3d &p) {
  DRAKE_DEMAND(built_);
  initial_state_c_->UpdateCoefficients(Matrix4d::Identity(), x);
  initial_foot_c_->UpdateCoefficients(Matrix3d::Identity(), p);
  SolveOCProblemAsIs();
}

vector<vector<Vector4d>> AlipMINLP::MakeXdesTrajForVdes(
    const Vector2d& vdes, double step_width, double Ts, double nk) const {
  double omega = sqrt(9.81 / H_);
  double frac = 1 / (m_ * H_ * omega);
  double Ly = H_ * m_ * vdes(0);
  double Lx = H_ * m_ * omega * step_width * atanh(omega * Ts / 2);
  double dLx = - H_ * m_ * vdes(1);
  Matrix4d Ad = ((Ts / (nk-1)) * alip_utils::CalcA(H_, m_)).exp();
  vector<vector<Vector4d>> xx;

  Eigen::MatrixPower<Matrix4d> APow(Ad);
  for (int i = 0; i < nmodes_; i++) {
    double s = (i % 2 == 0) ? 1.0 : -1.0;
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