#include <algorithm>
#include <iostream>

#include "alip_mpc.h"
#include "drake/solvers/solve.h"

namespace dairlib::systems::controllers{

using std::pair;
using std::vector;
using std::to_string;
using std::numeric_limits;


using Eigen::Matrix;

template <typename T, size_t N>
using Vector = Matrix<T, N, 1>;

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
using drake::solvers::QuadraticCost;
using drake::solvers::DecisionVariable;
using drake::solvers::LinearConstraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::BoundingBoxConstraint;
using drake::solvers::VectorXDecisionVariable;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::LinearEqualityConstraint;
using drake::solvers::kIterationLimit;

using geometry::kMaxFootholdFaces;

void AlipMPC::CalcOptimalFootstepPlan(const Eigen::Vector4d &x,
                                      const Eigen::Vector3d &p,
                                      bool warmstart) {
  DRAKE_DEMAND(built_);
  if (warmstart &&
     (solution_.first.is_success() ||
      solution_.first.get_solution_result() == kIterationLimit)){
    UpdateInitialGuess();
    prog_->SetInitialGuess(xx_.front().head<4>(), x);
    prog_->SetInitialGuess(pp_.front(), p);
  } else {
    UpdateInitialGuess(p, x);
  }
  initial_state_c_->UpdateCoefficients(Matrix4d::Identity(), x);
  initial_foot_c_->UpdateCoefficients(Matrix3d::Identity(), p);
  SolveOCProblemAsIs();
}

void AlipMPC::Build(const drake::solvers::SolverOptions& options) {
  prog_->SetSolverOptions(options);
  Build();
}

void AlipMPC::AddTrackingCost(const vector<Eigen::VectorXd> &xd,
                                const Matrix4d &Q, const Eigen::MatrixXd& Qf) {
  DRAKE_DEMAND(xd.size() == nmodes_);
  DRAKE_DEMAND(Tds_ > 0);
  DRAKE_DEMAND(not td_.empty());
  for (int i = 1; i < nmodes_; i++){
    DRAKE_DEMAND(xd.at(i).size() == nx_ * nknots_);
    vector<Binding<QuadraticCost>> QQ;
    for (int k = 0; k < nknots_ - 1 ; k++){
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
  Qf_ = 0.5 * alip_utils::SolveDareTwoStep(
      Q, H_, m_, td_.back(), Tds_, nknots_, reset_discretization_);
  MakeTerminalCost();
}

void AlipMPC::AddFootholdRegularization(const MatrixXd &W_footstep_reg) {
  DRAKE_DEMAND(W_footstep_reg.rows() == 3);
  DRAKE_DEMAND(W_footstep_reg.cols() == 3);
  DRAKE_DEMAND(nmodes_ >= 2);
  W_footstep_reg_ = W_footstep_reg;
  footstep_position_regularization_ = prog_->AddQuadraticCost(
      MatrixXd::Zero(6,6),
      VectorXd::Zero(6),
      {pp_.at(0), pp_.at(1)}
    ).evaluator();
}

vector<VectorXd> AlipMPC::ExtractDynamicsConstraintDual(
    const drake::solvers::MathematicalProgramResult& sol) {
  DRAKE_ASSERT(sol.is_success());
  vector<VectorXd> dual_solutions(nmodes_, VectorXd::Zero((nknots_ - 1) * nx_));

  // Dual solution isn't well-defined for an MIQP.
  // If not using OSQP, we probably have integer variables, so just return zeros
  if (sol.get_solver_id() != drake::solvers::OsqpSolver::id()) {
    return dual_solutions;
  }

  for (int n = 0; n < nmodes_; n++) {
    auto& duals = dual_solutions.at(n);
    for (int k = 0; k < nknots_ - 1; k++) {
      GetStateAtKnot(duals, k) = sol.GetDualSolution<LinearEqualityConstraint>(
          dynamics_c_.at(n).at(k));
    }
  }
  return dual_solutions;
}

void AlipMPC::UpdateFootholdRegularization(double scale, const Vector3d &pST_SW) {
  Matrix<double, 3, 6> A = Matrix<double, 3, 6>::Zero();
  A.leftCols<3>() = -Matrix3d::Identity();
  A.rightCols<3>() = Matrix3d::Identity();
  Matrix<double, 6, 6> Q = 2.0 * scale * A.transpose() * W_footstep_reg_ * A;
  Vector<double, 6> b = -2.0 * scale * A.transpose() * W_footstep_reg_ * pST_SW;
  double c = scale * pST_SW.transpose() * W_footstep_reg_ * pST_SW;
  footstep_position_regularization_->UpdateCoefficients(Q, b, c);
}

void AlipMPC::AddInputCost(double R) {
  for (int i = 0; i < nmodes_; i++){
    int n = nknots_ - 1;
    input_costs_.push_back(
        prog_->AddQuadraticCost(
            R * MatrixXd::Identity(n, n),
            VectorXd::Zero(n),
            uu_.at(i))
    );
  }
  R_ = R;
}

void AlipMPC::MakeTerminalCost(){
  Matrix<double, 8, 8> Qf = Matrix<double, 8, 8>::Zero();
  Qf.topLeftCorner<4,4>() = 0.5 * Qf_;
  Qf.bottomRightCorner<4,4>() = Qf_;
  Vector<double, 8> xd = Vector<double, 8>::Zero();
  xd.head<4>() = xd_.at(nmodes_ - 2).tail<4>();
  xd.tail<4>() = xd_.back().tail<4>();
  terminal_cost_ = prog_->AddQuadraticCost(
      2 * Qf, -2 * Qf * xd,
      {xx_.at(nmodes_ - 2).tail<4>(), xx_.back().tail<4>()}).evaluator();
}

void AlipMPC::MakeCapturePointConstraint(int foothold_idx) {
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

void AlipMPC::MakeResetConstraints() {
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

void AlipMPC::MakeDynamicsConstraints() {
  for (int i = 0; i < nmodes_; i++) {
    vector<Binding<LinearEqualityConstraint>> dyn_c_this_mode;
    double t = tt_(i) / (nknots_ - 1);
    Matrix4d Ad = alip_utils::CalcAd(H_, m_, t);
    Vector4d Bd = alip_utils::CalcA(H_, m_).inverse() * (Ad - Matrix4d::Identity()) * Vector4d::UnitW();

    Matrix<double, 4, 9> Adyn = Matrix<double, 4, 9>::Zero();
    Adyn.leftCols<4>() = Ad;
    Adyn.col(4) = Bd;
    Adyn.rightCols<4>() = -Matrix4d::Identity();

    for (int k = 0; k < nknots_ - 1; k++) {
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


double AlipMPC::GetTerminalCost() const {
  VectorXd res = VectorXd::Zero(1);
  Vector<double, 8> x = Vector<double, 8>::Zero();
  x.head<4>() = solution_.first.GetSolution(xx_.at(nmodes_ - 2).tail<4>());
  x.tail<4>() = solution_.first.GetSolution(xx_.back().tail<4>());
  terminal_cost_->Eval(x,&res);
  return res(0);
}

double AlipMPC::GetRunningCost() const {
  VectorXd res = VectorXd::Zero(1);
  double cost = 0;
  for (int i = 1; i < nmodes_; i++) {
    auto xx = solution_.first.GetSolution(xx_.at(i));
    for (int k = 0; k < nknots_ - 1; k++) {
      tracking_costs_.at(i-1).at(k).evaluator()->Eval(
          GetStateAtKnot(xx, k), &res);
      cost += res(0);
    }
  }
  return cost;
}

double AlipMPC::GetInputCost() const {
  VectorXd res = VectorXd::Zero(1);
  double cost = 0;
  for (int i = 0; i < nmodes_; i++) {
    input_costs_.at(i).evaluator()->Eval(
        solution_.first.GetSolution(uu_.at(i)), &res);
    cost += res(0);
  }
  return cost;
}

double AlipMPC::GetSoftConstraintCost() const {
  VectorXd res = VectorXd::Zero(1);
  double cost = 0;
  for (int i = 1; i < nmodes_; i++) {
    auto ee = solution_.first.GetSolution(ee_.at(i-1));
    for (int k = 0; k < nknots_; k++) {
      soft_constraint_cost_.at(i-1).at(k).evaluator()->Eval(
          ee.segment<2>(2*k), &res);
      cost += res(0);
    }
  }
  return cost;
}

void AlipMPC::MakeInitialFootstepConstraint() {
  initial_foot_c_ = prog_->AddLinearEqualityConstraint(
      Matrix3d::Identity(), Vector3d::Zero(), pp_.at(0)).evaluator();
  initial_foot_c_->set_description("initial_footstep");
}

void AlipMPC::MakeInitialStateConstraint() {
  initial_state_c_ = prog_->AddLinearEqualityConstraint(
      Matrix4d::Identity(),
      Vector4d::Zero(),
      xx_.front().head<4>()
  ).evaluator();

  initial_state_c_->set_description("initial_state");
}

void AlipMPC::MakeInputBoundConstaints() {
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

void AlipMPC::MakeWorkspaceConstraints() {
  Vector2d bb(xlim_, ylim_);

  MatrixXd I1 = MatrixXd::Zero(2, 4);
  MatrixXd I2 = MatrixXd::Zero(2, 4);
  I1.leftCols(2) = MatrixXd::Identity(2, 2);
  I1.rightCols(2) = -MatrixXd::Identity(2, 2);
  I2.leftCols(2) = -MatrixXd::Identity(2, 2);
  I2.rightCols(2) = -MatrixXd::Identity(2, 2);

  for (int n = 1; n < nmodes_; n++) {
    vector<Binding<QuadraticCost>> soft_constraint_this_mode;
    ee_.push_back(prog_->NewContinuousVariables(2 * nknots_));
    for (int k = 0; k < nknots_; k++) {
      const auto& ee = ee_.at(n-1).segment(2*k, 2);
      soft_constraint_this_mode.push_back(
          prog_->AddQuadraticErrorCost(50000 * MatrixXd::Identity(2,2), bb, ee)
      );
      prog_->AddLinearConstraint(
          I1,
          -numeric_limits<double>::infinity() * Vector2d::Ones(),
          Vector2d::Zero(),
          {GetStateAtKnot(xx_.at(n), k).head<2>(), ee});
      prog_->AddLinearConstraint(
          I2,
          -numeric_limits<double>::infinity() * Vector2d::Ones(),
          Vector2d::Zero(),
          {GetStateAtKnot(xx_.at(n), k).head<2>(), ee});
    }
    soft_constraint_cost_.push_back(soft_constraint_this_mode);
  }
}

void AlipMPC::MakeNextFootstepReachabilityConstraint() {
  next_step_reach_c_fixed_ = prog_->AddLinearConstraint(
          MatrixXd::Zero(1,3),
          -numeric_limits<double>::infinity()*VectorXd::Ones(1),
          VectorXd::Zero(1), pp_.at(1))
      .evaluator();
  next_step_reach_c_fixed_->set_description("next footstep workspace constraint");
}

void AlipMPC::MakeNoCrossoverConstraint() {
  for (int i = 0; i < nmodes_ - 1 ; i++) {
    int stance = i % 2 == 0 ? -1 : 1;
    Eigen::RowVector2d A = {stance, -stance};
    no_crossover_constraint_.push_back(
        prog_->AddLinearConstraint(
            A, -numeric_limits<double>::infinity(), -0.05,
            {pp_.at(i).segment(1,1), pp_.at(i+1).segment(1,1)})
    );
    no_crossover_constraint_.back().evaluator()->set_description(
        "no crossover constraint" + std::to_string(i));
  }
}

vector<VectorXd> AlipMPC::MakeXdesTrajForVdes(
    const Vector2d& vdes, double step_width, double Ts, int nk,
    alip_utils::Stance stance) const {
  alip_utils::AlipGaitParams params;
  params.height = H_;
  params.mass = m_;
  params.single_stance_duration = Ts;
  params.double_stance_duration = Tds_;
  params.stance_width = step_width;
  params.desired_velocity = vdes;
  params.reset_discretization_method = reset_discretization_;
  params.initial_stance_foot = stance;
  return alip_utils::MakePeriodicAlipGaitTrajectory(params, nmodes(), nk);
}

void AlipMPC::ActivateInitialTimeEqualityConstraint(double t) {
  DRAKE_DEMAND(built_);
  tt_(0) = t;
}

void AlipMPC::UpdateMaximumCurrentStanceTime(double tmax) {
  tmax_.at(0) = tmax;
}

void AlipMPC::UpdateTrackingCost(const vector<VectorXd>& xd) {
  for(int n = 0; n < nmodes_ - 1 ; n++) {
    for (int k = 0; k < nknots_ - 1; k++) {
      tracking_costs_.at(n).at(k).evaluator()->
          UpdateCoefficients( 2.0*Q_,
                              -2.0*Q_ * GetStateAtKnot(xd.at(n+1), k));
    }
  }
  Matrix<double, 8, 8> Qf = Matrix<double, 8, 8>::Zero();
  Qf.topLeftCorner<4,4>() = 0.5 * Qf_;
  Qf.bottomRightCorner<4,4>() = Qf_;
  Vector<double, 8> x = Vector<double, 8>::Zero();
  x.head<4>() = xd.at(nmodes_ - 2).tail<4>();
  x.tail<4>() = xd.back().tail<4>();
  terminal_cost_->UpdateCoefficients(2.0 * Qf,
                                     -2.0 * Qf * x);
  xd_ = xd;
}

void AlipMPC::CalcResetMap(Eigen::Matrix<double, 4, 12> *Aeq) const {
  Matrix<double, 4, 8> A =
      alip_utils::CalcResetMap(H_, m_, Tds_, reset_discretization_);
  Aeq->topLeftCorner<4, 4>() = A.topLeftCorner<4, 4>();
  Aeq->topRightCorner<4, 4>() = A.topRightCorner<4, 4>();
  Aeq->block<4, 4>(0, 4) = -Matrix4d::Identity();
}

void AlipMPC::UpdateInitialGuess(const Eigen::Vector3d &p0,
                                   const Eigen::Vector4d &x0) {
  // Update state initial guess
  vector<VectorXd> xg = xd_;

  // Set the initial guess for the current mode based on limited time
  VectorXd xx = VectorXd (nx_ * nknots_);
  xx.head<4>() = x0;
  Matrix4d Ad = alip_utils::CalcAd(H_, m_, tt_(0) / (nknots_ - 1));
  for (int i = 1; i < nknots_; i++) {
    GetStateAtKnot(xx, i) = Ad * GetStateAtKnot(xx, i-1);
  }
  xg.front() = xx;

  for (int n = 0; n < nmodes_; n++) {
    for (int k = 0; k < nknots_ ; k++) {
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

void AlipMPC::UpdateInitialGuess() {
  DRAKE_DEMAND(
      solution_.first.is_success() ||
          solution_.first.get_solution_result() ==
              drake::solvers::kIterationLimit);
  prog_->SetInitialGuessForAllVariables(solution_.first.GetSolution());
}

void AlipMPC::UpdateNextFootstepReachabilityConstraint(const geometry::ConvexPolygon &workspace) {

  const auto& [Af, bf] = workspace.GetConstraintMatrices();
  double neg_inf  = -numeric_limits<double>::infinity();
  next_step_reach_c_fixed_->UpdateCoefficients(
      Af, neg_inf * VectorXd::Ones(bf.rows()), bf);
}

void AlipMPC::UpdateNoCrossoverConstraint() {
  for (auto& constraint: no_crossover_constraint_) {
    auto A = constraint.evaluator()->GetDenseA();
    A *= -1;
    constraint.evaluator()->UpdateCoefficients(
        A,
        -numeric_limits<double>::infinity()*VectorXd::Ones(1),
        VectorXd::Zero(1));
  }
}

void AlipMPC::UpdateModeTiming(bool take_sqp_step) {
  tt_(0) = td_.front();
  if (take_sqp_step) {
    UpdateTimingGradientStep();
  }
  UpdateDynamicsConstraints();
}

void AlipMPC::UpdateModeTimingsOnTouchdown() {
  for (int i = 0; i < nmodes_ - 1; i++){
    tt_(i) = tt_(i+1);
  }
  tt_.tail(1)(0) = td_.back();
}

void AlipMPC::UpdateTimingGradientStep() {
  Matrix4d A = alip_utils::CalcA(H_, m_);
  Vector4d B = Vector4d::UnitW();

  for (int n = 0; n < nmodes_; n++) {
    double dLdt_n = 0;
    Matrix4d Ad = alip_utils::CalcAd(H_, m_, tt_(n) / (nknots_ - 1));
    for (int k = 0; k < nknots_ - 1; k++) {
      Vector4d nu = GetStateAtKnot(solution_.second.at(n), k);
      Vector4d x = solution_.first.GetSolution(GetStateAtKnot(xx_.at(n), k));
      VectorXd u = solution_.first.GetSolution(GetInputAtKnot(uu_.at(n), k));
      dLdt_n += (1.0 / (nknots_ - 1)) * nu.dot(A * Ad * x + Ad * B * u);
    }
    double dt = std::clamp(1e-4 * dLdt_n, -0.02, 0.02);
    double tnew = tt_(n) - dt;
    tt_(n) =  std::isnan(tnew) ? tt_(n)  : std::clamp(tnew, tmin_.at(n), tmax_.at(n));
  }
}

void AlipMPC::UpdateDynamicsConstraints() {
  for (int n = 0; n < nmodes_; n++) {
    int nk =  nknots_ - 1;
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

vector<vector<int>> AlipMPC::GetPossibleModeSequences() {
  unsigned int range = footholds_.size();
  int sets = nmodes_ - 1;
  auto products = std::vector<std::vector<int>>();
  for (int i = 0; i < pow(range, sets); i++) {
    products.emplace_back(std::vector<int>(sets, 0));
  }
  auto counter = std::vector<int>(sets, 0); // array of zeroes
  for (auto &product : products) {
    product = counter;

    // counter increment and wrapping/carry over
    counter.back()++;
    for (size_t i = counter.size() - 1; i != 0; i--) {
      if (counter[i] == range) {
        counter[i] = 0;
        counter[i - 1]++;
      } else break;
    }
  }
  return products;
}

vector<Vector3d> AlipMPC::GetFootstepSolution() const {
  vector<Vector3d> pp;
  for (auto& p : pp_){
    pp.emplace_back(solution_.first.GetSolution(p));
  }
  return pp;
}

vector<Vector3d> AlipMPC::GetFootstepGuess() const {
  vector<Vector3d> pp;
  for (auto& p : pp_){
    pp.emplace_back(prog_->GetInitialGuess(p));
  }
  return pp;
}

vector<VectorXd> AlipMPC::GetStateSolution() const {
  vector<VectorXd> xx;
  for(auto& x : xx_) {
    xx.push_back(solution_.first.GetSolution(x));
  }
  return xx;
}

vector<VectorXd> AlipMPC::GetStateGuess() const {
  vector<VectorXd> xx;
  for(auto& x : xx_) {
    xx.push_back(prog_->GetInitialGuess(x));
  }
  return xx;
}

vector<VectorXd> AlipMPC::GetInputSolution() const {
  vector<VectorXd> uu;
  for(auto& u : uu_) {
    uu.push_back(solution_.first.GetSolution(u));
  }
  return uu;
}

vector<VectorXd> AlipMPC::GetInputGuess() const {
  vector<VectorXd> uu;
  for(auto& u : uu_) {
    uu.push_back(prog_->GetInitialGuess(u));
  }
  return uu;
}

VectorXd AlipMPC::GetTimingSolution() const {
  VectorXd tt = VectorXd::Zero(nmodes_);
  return tt_;
}

VectorXd AlipMPC::GetTimingGuess() const {
  return tt_;
}

VectorXd AlipMPC::GetTimingDesired() const {
  const VectorXd tt = Eigen::Map<const VectorXd>(td_.data(), td_.size());
  return tt; // NOLINT(performance-no-automatic-move)
}
}