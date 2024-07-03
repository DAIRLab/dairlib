#include "cf_mpfc.h"
#include "common/eigen_utils.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"

namespace dairlib {
namespace systems {
namespace controllers {

using alip_utils::CalcAd;
using alip_utils::Stance;
using alip_utils::AlipGaitParams;

using std::vector;
using drake::solvers::Binding;
using drake::solvers::QuadraticCost;
using drake::solvers::DecisionVariable;
using drake::solvers::LinearConstraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::BoundingBoxConstraint;
using drake::solvers::VectorXDecisionVariable;
using drake::systems::controllers::DiscreteTimeLinearQuadraticRegulator;

using drake::solvers::LinearEqualityConstraint;
using drake::Vector6d;

using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::Matrix4d;
using Eigen::Matrix3d;
using Eigen::Matrix2d;

using Eigen::VectorXd;
using Eigen::Vector4d;
using Eigen::Vector3d;
using Eigen::Vector2d;

namespace {
constexpr double kInfinity = std::numeric_limits<double>::infinity();
constexpr int AlipDim = 4;
constexpr int FootstepDim = 3;
constexpr int ComplexDim = 6;
}

CFMPFC::CFMPFC(cf_mpfc_params params) : params_(params) {
  ValidateParams();
  const auto [A, B] = AlipStepToStepDynamics(
      params_.gait_params.height,
      params_.gait_params.mass,
      params_.gait_params.single_stance_duration,
      params_.gait_params.double_stance_duration,
      params_.gait_params.reset_discretization_method
  );
  A_ = A;
  B_ = B;
  auto lqr_result = DiscreteTimeLinearQuadraticRegulator(
      A_, B_, params_.Q, params_.R.topLeftCorner<2,2>());
  lqr_K_ = lqr_result.K;
  lqr_S_ = lqr_result.S;
  std::cout << "lqr s: \n" << lqr_S_ << std::endl;

  MakeMPCVariables();
  MakeMPCCosts();
  MakeFootstepConstraints();
  MakeComplexInputConstraints();
  MakeStateConstraints();
  MakeDynamicsConstraints();
  MakeInitialConditionsConstraints();
  Check();
}

cf_mpfc_solution CFMPFC::Solve(
    const Vector6d &x, const Eigen::Vector3d &p, double t,
    const Eigen::Vector2d &vdes, alip_utils::Stance stance,
    const cf_mpfc_solution &prev_sol) {
  auto start = std::chrono::steady_clock::now();

  UpdateInitialConditions(x, p);
  UpdateCrossoverConstraint(stance);
  UpdateFootstepCost(vdes, stance);
  UpdateTrackingCost(vdes, stance);
  UpdateComplexModelCosts(vdes, stance);

  std::vector<Vector6d> xc;
  std::vector<Vector2d> uu(params_.nknots, Vector2d::Zero());

  bool use_prev_sol = prev_sol.success and prev_sol.init and prev_sol.stance == stance;

  // TODO (@Brian-Acosta) need to re-project rotation matrix to SO(3) before
  //   Linearizing (or consider alternate rotation representation)
  for (int i = 0; i < params_.nknots; ++i) {
    if (use_prev_sol) {
      xc.push_back(prev_sol.xc.at(i));
      uu.at(i) = prev_sol.uu.at(i);
    } else {
      xc.push_back(x);
    }
  }
  if (use_prev_sol) {
    xc.front() = x;
  }

  Vector3d p_post = use_prev_sol ? prev_sol.pp.at(1) :
                                   p + CalcS2SLQRInput(x, vdes, t, stance);

  UpdateComplexDynamicsConstraints(xc, uu,  t);
  UpdateModelSwitchConstraint(xc.back(), p, p_post);

  auto solver_start = std::chrono::steady_clock::now();
  auto result = solver_.Solve(*prog_, std::nullopt, params_.solver_options);

  if (not result.is_success()) {
    std::cout << "solve failed with code "
              << result.get_solution_result() << std::endl;
  }
  auto solver_end = std::chrono::steady_clock::now();

  cf_mpfc_solution mpfc_solution;

  for (int i = 0; i < params_.nknots; ++i) {
    mpfc_solution.xc.push_back(result.GetSolution(xc_.at(i)));
    mpfc_solution.uu.push_back(result.GetSolution(uu_.at(i)));
  }
  for (int i = 0; i < params_.nmodes; ++i) {
    mpfc_solution.pp.push_back(result.GetSolution(pp_.at(i)));
  }
  for (int i = 0; i < params_.nmodes - 1; ++i) {
    mpfc_solution.xx.push_back(result.GetSolution(xx_.at(i)));
  }
  for (int i = 0; i < params_.nmodes - 2; ++i) {
    mpfc_solution.ee.push_back(result.GetSolution(ee_.at(i)));
  }
  mpfc_solution.xi = result.GetSolution(xi_);
  mpfc_solution.stance = stance;
  mpfc_solution.desired_velocity = vdes;
  mpfc_solution.success = result.is_success();
  mpfc_solution.solution_result = result.get_solution_result();

  auto solution_details =
      result.get_solver_details<drake::solvers::GurobiSolver>();

  auto end = std::chrono::steady_clock::now();

  std::chrono::duration<double> total_time = end - start;
  std::chrono::duration<double> solve_time = solver_end - solver_start;

  mpfc_solution.optimizer_time = solution_details.optimizer_time;
  mpfc_solution.total_time = total_time.count();
  mpfc_solution.init = result.is_success();
  
  return mpfc_solution;
}

void CFMPFC::MakeMPCVariables() {
  DRAKE_DEMAND(prog_->num_vars() == 0);

  for (int i = 0; i < params_.nknots; ++i) {
    xc_.push_back(prog_->NewContinuousVariables(ComplexDim,
                                                "xc" + std::to_string(i)));
    uu_.push_back(prog_->NewContinuousVariables(2,
                                                "ff" + std::to_string(i)));
  }
  for (int i = 0; i < params_.nmodes; ++i) {
    pp_.push_back(prog_->NewContinuousVariables(3, "pp" + std::to_string(i)));
  }
  xi_ = prog_->NewContinuousVariables(4, "xi");
  for (int i = 0; i < params_.nmodes - 1; ++i) {
    xx_.push_back(prog_->NewContinuousVariables(4,
                                                "xx" + std::to_string(i + 1)));
  }
  for (int i = 0; i < params_.nmodes - 2; ++i) {
    ee_.push_back(prog_->NewContinuousVariables(1,
                                                "ee" + std::to_string(i + 1)));
  }
}

void CFMPFC::MakeMPCCosts() {
  DRAKE_DEMAND(complex_state_cost_.empty());
  DRAKE_DEMAND(complex_input_cost_.empty());
  DRAKE_DEMAND(tracking_cost_.empty());
  DRAKE_DEMAND(footstep_cost_.empty());
  DRAKE_DEMAND(soft_constraint_cost_.empty());
  DRAKE_DEMAND(terminal_cost_ == nullptr);

  for (int i = 0; i < params_.nknots; ++i) {
    // state and input cost for first model phase
    // TODO (@Brian-Acosta) After initial prototype, make sure to include
    //  the appropriate CoM height cost and CoM Vel cost to incorporate
    //  footstep height change
    complex_state_cost_.push_back(
        prog_->AddQuadraticCost(
            MatrixXd::Identity(ComplexDim, ComplexDim), VectorXd::Zero(ComplexDim), xc_.at(i)
        ));
    complex_input_cost_.push_back(
        prog_->AddQuadraticCost(
            MatrixXd::Identity(2,2),
            VectorXd::Zero(2),
            uu_.at(i)
        ));
  }
  for (int i = 0; i < params_.nmodes - 2; ++i) {
    // ALIP state cost, not including final cost
    tracking_cost_.push_back(
        prog_->AddQuadraticCost(
            Matrix4d::Identity(), Vector4d::Zero(), xx_.at(i)
        ));
  }
  for (int i = 0; i < params_.nmodes - 1; ++i) {
    footstep_cost_.push_back(
        prog_->AddQuadraticCost(
            Matrix4d::Identity(), Vector4d::Zero(),
            {pp_.at(i).head<2>(), pp_.at(i + 1).head<2>()}
        ));
  }

  for (int i = 0; i < params_.nmodes - 2; ++i) {
    soft_constraint_cost_.push_back(
        prog_->AddQuadraticCost(
            2 * params_.soft_constraint_cost * MatrixXd::Identity(1, 1),
            VectorXd::Zero(1),
            ee_.at(i)
        ));
  }
  terminal_cost_ = prog_->AddQuadraticCost(
      Matrix4d::Identity(), Vector4d::Zero(), 0, xx_.back()
  ).evaluator();

  complex_state_final_cost_ = prog_->AddQuadraticCost(
      MatrixXd::Identity(6, 6), Vector6d::Zero(), 0, xc_.back()
  ).evaluator();

  // build cost matrices
  alip_utils::MakeAlipStepToStepCostMatrices(
      params_.gait_params, params_.Q, params_.Qf,
      Q_proj_, Q_proj_f_,
      g_proj_p1_, g_proj_p2_,
      p2o_premul_, projection_to_p2o_complement_,
      p2o_orthogonal_complement_, p2o_basis_
  );
}

void CFMPFC::MakeFootstepConstraints() {
//  constexpr double bigM = 20.0;
  DRAKE_DEMAND(reachability_c_.empty());
  DRAKE_DEMAND(no_crossover_c_.empty());

  Matrix<double, 2, 4> A_reach = Matrix<double, 2, 4>::Zero();
  A_reach.leftCols<2>() = -Matrix2d::Identity();
  A_reach.rightCols<2>() = Matrix2d::Identity();

  for (int i = 0; i < params_.nmodes - 1; ++i) {
    reachability_c_.push_back(
        prog_->AddLinearConstraint(
            A_reach,
            -2*params_.com_pos_bound,
            2*params_.com_pos_bound,
            {pp_.at(i).head<2>(), pp_.at(i + 1).head<2>()}
        ));
    no_crossover_c_.push_back(
        prog_->AddLinearConstraint(
            MatrixXd::Ones(1, 2),
            VectorXd::Constant(1, -kInfinity),
            VectorXd::Constant(1, kInfinity),
            {pp_.at(i).segment(1, 1), pp_.at(i + 1).segment(1, 1)}
        ));
    prog_->AddLinearEqualityConstraint(pp_.at(i+1)(2) == 0);
  }
  // TODO (@Brian-Acosta) MIQP foothold constraints
}

void CFMPFC::MakeComplexInputConstraints() {
  DRAKE_DEMAND(complex_input_constraints_.empty());
  for (const auto& u : uu_) {
    double a_max = std::min(9.81, params_.input_bounds(1));
    double l = params_.foot_length / 2.0;
    Matrix2d A = Matrix2d::Zero();
    Vector2d lb = Vector2d::Constant(- params_.gait_params.mass * a_max);
    Vector2d ub = Vector2d::Constant(kInfinity);
    A(0, 1) = params_.gait_params.mass;
    A(1, 1) = params_.gait_params.mass;
    A(0, 0) = 1 / l;
    A(1, 0) = -1 / l;
    complex_input_constraints_.push_back(
        prog_->AddLinearConstraint(A, lb, ub,u));
    complex_input_bounds_.push_back(
        prog_->AddBoundingBoxConstraint(
            -params_.input_bounds, params_.input_bounds, u));
  }
}

void CFMPFC::MakeStateConstraints() {
  DRAKE_DEMAND(workspace_c_.empty());

  Vector4d state_bound;
  state_bound.head<2>() = params_.com_pos_bound;
  state_bound.tail<2>() = params_.gait_params.mass *
      params_.gait_params.height * params_.com_vel_bound;

  Matrix4d Ad_inv = CalcAd(
      params_.gait_params.height,
      params_.gait_params.mass,
      params_.gait_params.single_stance_duration
  ).inverse();

  MatrixXd A_ws(4 * AlipDim, AlipDim + 1);

  A_ws.setZero();
  A_ws.block<4,4>(0, 0) = Matrix4d::Identity();
  A_ws.block<4,4>(4, 0) = Ad_inv;
  A_ws.block<4,4>(8, 0) = Matrix4d::Identity();
  A_ws.block<4,4>(12, 0) = Ad_inv;
  A_ws.topRightCorner<8,1>() = VectorXd::Ones(8);
  A_ws.bottomRightCorner<8,1>() = -VectorXd::Ones(8);

  VectorXd lb = VectorXd::Constant(4 * AlipDim, -kInfinity);
  lb.head<8>() = stack<double>({-state_bound, -state_bound});
  VectorXd ub = VectorXd::Constant(4 * AlipDim, kInfinity);
  ub.tail<8>() = stack<double>({state_bound, state_bound});


  for (int i = 0; i < params_.nmodes - 2; ++i) {
    workspace_c_.push_back(
        prog_->AddLinearConstraint(A_ws, lb, ub, {xx_.at(i+1), ee_.at(i)})
    );
    //std::cout << workspace_c_.at(i).ToLatex() << std::endl;
  }
}

void CFMPFC::MakeDynamicsConstraints() {
  DRAKE_DEMAND(alip_dynamics_c_ == nullptr);
  DRAKE_DEMAND(complex_dynamics_c_ == nullptr);

  MatrixXd M(
      AlipDim * (params_.nmodes - 2), (AlipDim + FootstepDim) * (params_.nmodes - 1));
  M.setZero();
  for (int i = 0; i < params_.nmodes - 2; ++i) {
    M.block<4,4>(AlipDim * i, AlipDim * i) = A_;
    M.block<4,4>(AlipDim * i, AlipDim * (i + 1)) = -Matrix4d::Identity();
    M.block<4,2>(AlipDim * i, AlipDim * (params_.nmodes - 1) + FootstepDim * (i+1)) = B_;
    M.block<4,2>(AlipDim * i, AlipDim * (params_.nmodes - 1) + FootstepDim * i) = - B_;
  }
  // first footstep is not part of the ALIP dynamics
  vector<VectorXDecisionVariable> pp;
  for (int i = 1; i < pp_.size(); ++i) {
    pp.push_back(pp_.at(i));
  }
  alip_dynamics_c_ = prog_->AddLinearEqualityConstraint(
      M, VectorXd::Zero(AlipDim * (params_.nmodes - 2)), {stack(xx_), stack(pp)}
  ).evaluator();


  MatrixXd A(
      ComplexDim * (params_.nknots - 1), params_.nknots * (ComplexDim + 2));
  A.setIdentity();

  complex_dynamics_c_ = prog_->AddLinearEqualityConstraint(
      A, VectorXd::Zero(ComplexDim * (params_.nknots - 1)), {stack(xc_), stack(uu_)}
  ).evaluator();

}

void CFMPFC::MakeInitialConditionsConstraints() {
  DRAKE_DEMAND(initial_state_c_ == nullptr);
  DRAKE_DEMAND(initial_foot_c_ == nullptr);
  DRAKE_DEMAND(initial_alip_state_c_ == nullptr);
  DRAKE_DEMAND(model_switch_c_ == nullptr);

  initial_foot_c_ = prog_->AddLinearEqualityConstraint(
      Matrix3d::Identity(), Vector3d::Zero(), pp_.front()
  ).evaluator();

  initial_state_c_ = prog_->AddLinearEqualityConstraint(
      Eigen::MatrixXd::Identity(ComplexDim, ComplexDim),
      VectorXd ::Zero(ComplexDim),
      xc_.front()
  ).evaluator();

  MatrixXd M = MatrixXd::Zero(AlipDim, 2*AlipDim);
  M.leftCols<AlipDim>() = A_;
  M.rightCols<AlipDim>() = -Matrix4d::Identity();
  initial_alip_state_c_ = prog_->AddLinearEqualityConstraint(
      M, Vector4d::Zero(), {xi_, xx_.front()}
  ).evaluator();

  model_switch_c_ = prog_->AddLinearEqualityConstraint(
      MatrixXd::Identity(AlipDim, ComplexDim + FootstepDim + AlipDim),
      Vector4d::Zero(),
      {xc_.back(), pp_.at(1), xi_}
  ).evaluator();
}

void CFMPFC::UpdateInitialConditions(const Vector6d& x, const Eigen::Vector3d& p) {
  initial_state_c_->UpdateCoefficients(
      Matrix<double, 6, 6>::Identity(), x);
  initial_foot_c_->UpdateCoefficients(Matrix3d::Identity(), p);
}

void CFMPFC::UpdateCrossoverConstraint(Stance stance) {
  double s = (stance == Stance::kLeft) ? 1.0 : -1.0;
  for (auto& c : no_crossover_c_) {
    c.evaluator()->UpdateCoefficients(
        Eigen::RowVector2d(-s, s),
        VectorXd::Constant(1, -kInfinity),
        VectorXd::Constant(1, -0.04)
    );
    s *= -1.0;
  }
}

void CFMPFC::UpdateComplexDynamicsConstraints(
    const vector<Vector6d>& xc, const vector<Eigen::Vector2d> &uu, double t) {

  double h = t / (params_.nknots - 1);

  // Dynamics constraint order: x0, x1, ... xn, f0, ... fn-1
  MatrixXd A_dyn = complex_dynamics_c_->GetDenseA();
  A_dyn.setZero();
  VectorXd b_dyn = complex_dynamics_c_->lower_bound();
  b_dyn.setZero();

  MatrixXd A;
  MatrixXd B;
  VectorXd c;

  for (int i = 0; i < params_.nknots - 1; ++i) {
    nonlinear_pendulum::LinearizeTrapezoidalCollocationConstraint(
        h, xc.at(i), xc.at(i+1), uu.at(i), uu.at(i+1),
        params_.gait_params.mass, A, B, c);


    A_dyn.block<6,  2 * 6>(6*i, 6*i) = A;
    A_dyn.block<6, 2 * 2>(6 * i, 6 * params_.nknots + 2 * i) = B;
    b_dyn.segment<6>(6 * i) = c;
  }
//  std::cout << A_dyn << std::endl;
  complex_dynamics_c_->UpdateCoefficients(A_dyn, b_dyn);
}

void CFMPFC::UpdateModelSwitchConstraint(
    const Vector6d& xc, const Eigen::Vector3d &p_pre, const Eigen::Vector3d &p_post) {
  MatrixXd Ax;
  MatrixXd Bp;
  Vector4d b;
  nonlinear_pendulum::LinearizeALIPReset(xc, p_pre, p_post, params_.gait_params.mass, Ax, Bp, b);
  // xi = Ax (x - xc) + Bp(p - p_post) + b
  // Ax * x + Bp * p - xi = Ax * xc + Bp * p_post - b
  MatrixXd A_eq = model_switch_c_->GetDenseA();
  A_eq.leftCols<6>() = Ax;
  A_eq.middleCols<FootstepDim>(6) = Bp;
  A_eq.rightCols<AlipDim>() = -Matrix4d::Identity();

  Vector4d b_eq = Ax * xc + Bp * p_post - b;
  model_switch_c_->UpdateCoefficients(A_eq, b_eq);
}

void CFMPFC::UpdateComplexModelCosts(const Eigen::Vector2d &vdes, alip_utils::Stance stance) {
  Vector6d xd = Vector6d::Zero();
  xd(2) = params_.gait_params.height;
  Vector2d ud = Vector2d::Zero();

  MatrixXd Qx = params_.Qc;
  MatrixXd Qu = params_.Rc;

  for (int i = 0; i < params_.nknots; ++i) {
    double m = (i == 0 or i == params_.nknots -1) ? 1 : 2;
    complex_state_cost_.at(i).evaluator()->UpdateCoefficients(
        m * Qx, -m * Qx * xd, 0.5 * m * xd.transpose() * Qx * xd, true);
    complex_input_cost_.at(i).evaluator()->UpdateCoefficients(
        m * Qu, -m * Qu * ud, 0.5 * m * ud.transpose() * Qu * ud, true);
  }
  complex_state_final_cost_->UpdateCoefficients(
       Qx, -Qx * xd, 0.5 * xd.transpose() * Qx * xd, true);
}

void CFMPFC::UpdateFootstepCost(const Vector2d &vdes, Stance stance) {
  AlipGaitParams gait_params = params_.gait_params;
  gait_params.desired_velocity = vdes;
  gait_params.initial_stance_foot = stance;
  const auto ud = alip_utils::MakeP2Orbit(gait_params);

  Matrix<double, 2, 4> r;
  r.leftCols<2>() = -Matrix2d::Identity();
  r.rightCols<2>() = Matrix2d::Identity();
  Matrix2d R = params_.R.topLeftCorner<2,2>();

  Matrix4d Q = 2 * r.transpose() * R * r;

  for (int i = 0; i < params_.nmodes - 1; ++i) {
    Vector4d b = - 2 * r.transpose() * R * ud[i % 2];
    footstep_cost_.at(i).evaluator()->UpdateCoefficients(Q, b, 0, true);
  }
}

void CFMPFC::UpdateTrackingCost(const Vector2d &vdes, Stance stance) {
  if (params_.tracking_cost_type ==
      alip_utils::AlipTrackingCostType::kVelocity) {
    UpdateTrackingCostVelocity(vdes);
    UpdateTerminalCostVelocity(vdes);
  } else {
    UpdateTrackingCostGait(vdes, stance);
    UpdateTerminalCostGait(vdes, stance);
  }
}

void CFMPFC::UpdateTrackingCostVelocity(const Vector2d &vdes) {
  for (int i = 1; i < params_.nmodes - 1; ++i) {
    const Matrix<double, 4, 2>& vdes_mul = i % 2 == 0 ?
        -2 * Q_proj_ * g_proj_p1_ : -2 * Q_proj_ * g_proj_p2_;
    tracking_cost_.at(i-1).evaluator()->UpdateCoefficients(
        2 * Q_proj_, vdes_mul * vdes, 0, true);
  }
}

void CFMPFC::UpdateTerminalCostVelocity(const Vector2d &vdes) {
  const Matrix<double, 4, 2>& vdes_mul =
      params_.nmodes % 2 == 0 ?
          -2 * Q_proj_f_ * g_proj_p1_ : -2 * Q_proj_f_ * g_proj_p2_;
  Matrix4d Qf = 2 * Q_proj_f_;
  Vector4d bf = vdes_mul * vdes;
  terminal_cost_->UpdateCoefficients(Qf, bf, 0, true);
}

void CFMPFC::UpdateTrackingCostGait(const Vector2d &vdes, Stance stance) {
  AlipGaitParams gait_params = params_.gait_params;
  gait_params.desired_velocity = vdes;
  gait_params.initial_stance_foot = stance;

  const auto& [x0, x1] = alip_utils::MakePeriodicAlipGait(gait_params);
  for (int i = 1; i < params_.nmodes - 1; ++i) {
    const Vector4d& xd = i % 2 == 0 ? x0: x1;
    tracking_cost_.at(i-1).evaluator()->UpdateCoefficients(
        2 * params_.Q, -2*params_.Q * xd, 0, true);
  }

}

void CFMPFC::UpdateTerminalCostGait(const Eigen::Vector2d &vdes,
                                         alip_utils::Stance stance) {
  AlipGaitParams gait_params = params_.gait_params;
  gait_params.desired_velocity = vdes;
  gait_params.initial_stance_foot = stance;

  const auto& [x0, x1] = alip_utils::MakePeriodicAlipGait(gait_params);

  const Vector4d& xd = params_.nmodes % 2 == 0 ? x1 : x0;
  terminal_cost_->UpdateCoefficients(
      2* params_.Qf, -2 * params_.Qf * xd, 0, true);
}

Vector3d CFMPFC::CalcS2SLQRInput(
    const Vector6d &x, const Vector2d &vdes, double t,
    alip_utils::Stance stance) const {

  AlipGaitParams gait_params = params_.gait_params;
  gait_params.desired_velocity = vdes;
  gait_params.initial_stance_foot = stance;

  const auto [x0, x1] = alip_utils::MakePeriodicAlipGait(gait_params);
  const auto ud = alip_utils::MakeP2Orbit(gait_params).front();
  Vector4d x_alip = nonlinear_pendulum::CalcAlipStateAtTouchdown(
      x, gait_params.mass, t);
  Vector3d footstep = Vector3d::Zero();
  footstep.head<2>() = ud - lqr_K_ * (x_alip - x0);
  return footstep;
}

}
}
}