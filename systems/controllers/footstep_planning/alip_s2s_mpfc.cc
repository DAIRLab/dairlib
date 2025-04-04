#include "alip_s2s_mpfc.h"
#include <algorithm>
#include <iostream>
#include <chrono>

#include "common/eigen_utils.h"
#include "solvers/admm/ncqp_solver.h"
#include "solvers/admm/convex_polygon_set_constraint.h"
#include "drake/solvers/osqp_solver.h"

namespace dairlib::systems::controllers{

using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::Matrix4d;
using Eigen::Matrix3d;
using Eigen::Matrix2d;
using Eigen::VectorXd;
using Eigen::Vector4d;
using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::RowVectorXd;
using Eigen::RowVector3d;

using alip_utils::Stance;
using alip_utils::CalcA;
using alip_utils::CalcAd;
using alip_utils::AlipGaitParams;
using alip_utils::AlipStepToStepDynamics;

using geometry::ConvexPolygonSet;

lcmt_alip_s2s_mpfc_input alip_s2s_mpfc_input::ToLcm(double timestamp) const {
  lcmt_alip_s2s_mpfc_input msg{};

  msg.utime = timestamp * 1e6;

  for (int i = 0; i < 4; i++) {
    msg.x[i] = x(i);
  }
  for (int i = 0; i < 3; i++) {
    msg.p[i] = p(i);
    msg.p_prev_stance[i] = p_prev_stance(i);
  }
  for (int i = 0; i < 2; i++) {
    msg.vdes[i] = vdes(i);
  }

  msg.t = t;
  msg.tmin = tmin;
  msg.tmax = tmax;
  msg.stance = static_cast<int32_t>(stance);
  footholds.CopyToLcm(&msg.footholds);

  return msg;
}

alip_s2s_mpfc_input alip_s2s_mpfc_input::FromLcm(
    const lcmt_alip_s2s_mpfc_input& msg) {
  alip_s2s_mpfc_input ret;
  for (int i = 0; i < 4; i++) {
    ret.x(i) = msg.x[i];
  }
  for (int i = 0; i < 3; i++) {
    ret.p(i) = msg.p[i];
    ret.p_prev_stance(i) = msg.p_prev_stance[i];
  }
  for (int i = 0; i < 2; i++) {
    ret.vdes(i) = msg.vdes[i];
  }

  ret.t = msg.t;
  ret.tmin = msg.tmin;
  ret.tmax = msg.tmax;
  ret.stance = static_cast<alip_utils::Stance>(msg.stance);
  ret.footholds = geometry::ConvexPolygonSet::CopyFromLcm(msg.footholds);
  return ret;
}

static constexpr double kInfinity = std::numeric_limits<double>::infinity();

AlipS2SMPFC::AlipS2SMPFC(alip_s2s_mpfc_params params) :
  params_(params) {

  if (not params.miqp) {
    ncqp_solver_ = solvers::NCQPSolver(params.ncqp_solver_options_path);
  }

  ValidateParams();
  MakeMPCVariables();
  MakeMPCCosts();
  MakeInputConstraints();
  MakeStateConstraints();
  MakeDynamicsConstraint();
  MakeInitialConditionsConstraints();
  Check();
}

alip_s2s_mpfc_solution AlipS2SMPFC::SolveFromInput(
    const alip_s2s_mpfc_input &input) {
  return Solve(
      input.x,
      input.p,
      input.t,
      input.tmin,
      input.tmax,
      input.vdes,
      input.stance,
      input.footholds,
      input.p_prev_stance
  );
}

alip_s2s_mpfc_solution AlipS2SMPFC::Solve(
    const Vector4d &x, const Vector3d &p, double t, double tmin, double tmax,
    const Vector2d& vdes, Stance stance, const ConvexPolygonSet& footholds,
    const Vector3d& p_prev_stance) {

  auto start = std::chrono::steady_clock::now();

  UpdateInitialConditions(x, p, t, tmin, tmax);
  UpdateCrossoverConstraint(stance);
  UpdateFootholdConstraints(footholds);
  UpdateInputCost(vdes, stance);
  UpdateTrackingCost(vdes, stance);
  UpdateTimeRegularization(t);
  UpdateTrustRegionConstraint(t, p_prev_stance);

  drake::solvers::MathematicalProgramResult result;

  auto solver_start = std::chrono::steady_clock::now();

  if (params_.miqp) {
    result = solver_.Solve(*prog_, std::nullopt, params_.solver_options);
  } else {
    solvers::QPResult solution = ncqp_solver_.Solve(*prog_, foothold_set_c_);
    result.set_decision_variable_index(prog_->decision_variable_index());
    result.set_x_val(solution.x);
    result.set_solution_result(solution.solution_result);
  }
  auto solver_end = std::chrono::steady_clock::now();

  alip_s2s_mpfc_solution mpfc_solution;
  mpfc_solution.success = result.is_success();
  mpfc_solution.solution_result = result.get_solution_result();

  mpfc_solution.pp.clear();
  mpfc_solution.xx.clear();
  mpfc_solution.ee.clear();
  mpfc_solution.mu.clear();

  for (int i = 0; i < params_.nmodes; ++i) {
    mpfc_solution.xx.push_back(result.GetSolution(xx_.at(i)));
    mpfc_solution.pp.push_back(result.GetSolution(pp_.at(i)));
  }
  for (int i = 0; i < params_.nmodes - 1; ++i) {
    mpfc_solution.ee.push_back(result.GetSolution(ee_.at(i)));
    mpfc_solution.mu.push_back(params_.miqp ?
        result.GetSolution(mu_.at(i)) : VectorXd::Zero(kMaxFootholds));
    mpfc_solution.mu.back()(0) = 1;
  }

  mpfc_solution.t_nom = t;
  mpfc_solution.t_sol = result.GetSolution(tau_)(0);
  mpfc_solution.u_sol = result.GetSolution(u_)(0);
  mpfc_solution.desired_velocity = vdes;

  auto end = std::chrono::steady_clock::now();

  std::chrono::duration<double> total_time = end - start;
  std::chrono::duration<double> solve_time = solver_end - solver_start;

  mpfc_solution.total_time = total_time.count();
  mpfc_solution.input_footholds = footholds;

  // assign costs
  mpfc_solution.total_cost = result.get_optimal_cost();
  mpfc_solution.footstep_cost = 0;
  mpfc_solution.state_cost = 0;
  mpfc_solution.soft_constraint_cost = 0;

  for (const auto& c : input_cost_) {
    mpfc_solution.footstep_cost += result.EvalBinding(c)(0);
  }
  for (const auto& c : tracking_cost_) {
    mpfc_solution.state_cost += result.EvalBinding(c)(0);
  }
  for (const auto& c : soft_constraint_cost_) {
    mpfc_solution.soft_constraint_cost += result.EvalBinding(c)(0);
  }
  VectorXd y = VectorXd::Zero(1);
  time_regularization_->Eval(result.GetSolution(tau_), &y);
  mpfc_solution.time_reg_cost = y(0);

  ankle_torque_regularization_->Eval(result.GetSolution(u_), &y);
  mpfc_solution.input_reg_cost = y(0);

  terminal_cost_->Eval(result.GetSolution(xx_.back()), &y);
  mpfc_solution.final_cost = y(0);

  return mpfc_solution;
}

void AlipS2SMPFC::MakeMPCVariables() {
  tau_ = prog_->NewContinuousVariables(1, "t0");
  u_ = prog_->NewContinuousVariables(1, "u0");
  for (int i = 0; i < params_.nmodes; ++i) {
    std::string mode = std::to_string(i);
    xx_.push_back(prog_->NewContinuousVariables(nx_, "xx_" + mode));
    pp_.push_back(prog_->NewContinuousVariables(np_, "pp_" + mode));
  }
  for (int i = 0; i < params_.nmodes - 1; ++i) {
    std::string mode = std::to_string(i+1);
    ee_.push_back(prog_->NewContinuousVariables(1, "ee_" + mode));
    if (params_.miqp) {
      mu_.push_back(prog_->NewBinaryVariables(kMaxFootholds, "mu_" + mode));
    }
  }
}

void AlipS2SMPFC::MakeMPCCosts() {
  for (int i = 0; i < params_.nmodes - 1; ++i) {
    tracking_cost_.push_back(
        prog_->AddQuadraticCost(
            Matrix4d::Identity(), Vector4d::Zero(), xx_.at(i)
        ));
    input_cost_.push_back(
        prog_->AddQuadraticCost(
            Matrix4d::Identity(), Vector4d::Zero(),
            {pp_.at(i).head<2>(), pp_.at(i+1).head<2>()}
        ));
    soft_constraint_cost_.push_back(
        prog_->AddQuadraticCost(
            2 * params_.soft_constraint_cost * MatrixXd::Identity(1,1),
            VectorXd::Zero(1),
            ee_.at(i)
        ));
  }

  terminal_cost_ = prog_->AddQuadraticCost(
      Matrix4d::Identity(), Vector4d::Zero(), 0, xx_.back()
  ).evaluator();

  time_regularization_ = prog_->AddQuadraticCost(
     MatrixXd::Identity(1,1), VectorXd::Zero(1), tau_
  ).evaluator();

  ankle_torque_regularization_ = prog_->AddQuadraticCost(
      params_.ankle_torque_regularization * MatrixXd::Identity(1,1),
      VectorXd::Zero(1), u_
  ).evaluator();

  // build cost matrices
  alip_utils::MakeProjectionToP2Orbit(
      params_.gait_params, PI_0_, PI_1_, g_0_, g_1_);
  PIs_ = {PI_0_, PI_1_};
  gs_ = {g_0_, g_1_};
}

void AlipS2SMPFC::MakeInputConstraints() {

  Matrix<double, 3, 8> A_reach = Matrix<double, 3, 8>::Zero();
  A_reach.topLeftCorner<2,2>() = -Matrix2d ::Identity();
  A_reach.block<3, 3>(0, 2) = -Matrix3d::Identity();
  A_reach.block<3, 3>(0, 5) = Matrix3d::Identity();
  Vector3d lb_reach = Vector3d::Zero();
  Vector3d ub_reach = Vector3d::Zero();
  lb_reach.head<2>() = -params_.com_pos_bound;
  ub_reach.head<2>() = params_.com_pos_bound;
  lb_reach(2) = -0.25;
  ub_reach(2) = 0.25;
  for (int i = 0; i < params_.nmodes - 1; ++i) {
    reachability_c_.push_back(
        prog_->AddLinearConstraint(
            A_reach,
            lb_reach,
            ub_reach,
            {xx_.at(i).head<2>(), pp_.at(i), pp_.at(i+1)}
        ));
    no_crossover_c_.push_back(
        prog_->AddLinearConstraint(
            MatrixXd::Ones(1, 2),
            VectorXd::Constant(1, -kInfinity),
            VectorXd::Constant(1, kInfinity),
            {pp_.at(i).segment(1,1), pp_.at(i+1).segment(1,1)}
    ));
  }

  if (params_.miqp) {
    MakeMIQPFootholdConstraints();
  } else {
    MakeNCQPFootholdConstraints();
  }

  MatrixXd A_trust = MatrixXd::Zero(3, 6);
  A_trust.leftCols<3>().setIdentity();
  A_trust.rightCols<3>().setIdentity();
  A_trust.rightCols<3>() *= -1;
  trust_region_ = prog_->AddLinearConstraint(
      A_trust, -Vector3d::Constant(kInfinity), Vector3d::Constant(kInfinity),
      {pp_.at(1), pp_.at(0)}
  ).evaluator();
}

void AlipS2SMPFC::MakeMIQPFootholdConstraints() {
  DRAKE_DEMAND(params_.miqp);

  constexpr double bigM = 20.0;
  for (int i = 0; i < params_.nmodes - 1; ++i) {
    vector<LinearBigMConstraint> tmp;
    vector<LinearBigMEqualityConstraint> tmp_eq;
    for (size_t j = 0; j < kMaxFootholds; ++j) {
      tmp.push_back(
          LinearBigMConstraint(
              *prog_,
              RowVector3d::UnitX(),
              VectorXd::Zero(1),
              bigM,
              pp_.at(i+1),
              mu_.at(i)(j)
          ));
      tmp_eq.push_back(
          LinearBigMEqualityConstraint(
              *prog_,
              RowVector3d::UnitZ(),
              VectorXd::Zero(1),
              bigM,
              pp_.at(i+1),
              mu_.at(i)(j)
          ));
    }
    footstep_c_.push_back(tmp);
    footstep_c_eq_.push_back(tmp_eq);
    for (auto& clist: footstep_c_) {
      for (auto& c: clist) {
        c.deactivate();
      }
    }
    for (auto& clist: footstep_c_eq_) {
      for (auto& c: clist) {
        c.deactivate();
      }
    }
  }

  footstep_choice_c_ = prog_->AddLinearEqualityConstraint(
      BlockDiagonalRepeat<double>(RowVectorXd::Ones(kMaxFootholds), mu_.size()),
      VectorXd::Ones(mu_.size()),
      stack(mu_)
  ).evaluator();
}

void AlipS2SMPFC::MakeNCQPFootholdConstraints() {
  DRAKE_DEMAND(not params_.miqp);
  auto evaluator = std::make_shared<solvers::ConvexPolygonSetConstraint>(
      ConvexPolygonSet());
  for (int i = 0 ; i < params_.nmodes - 1; ++i) {
    foothold_set_c_.push_back(
        prog_->AddConstraint(evaluator, pp_.at(i+1))
    );
  }
}

void AlipS2SMPFC::MakeStateConstraints() {
  Vector4d state_bound;

  Eigen::Matrix2d flipper = Eigen::Matrix2d::Zero();
  flipper(0, 1) = 1;
  flipper(1, 0) = 1;

  state_bound.head<2>() = params_.com_pos_bound;
  state_bound.tail<2>() = params_.gait_params.mass *
      params_.gait_params.height * flipper * params_.com_vel_bound;

  Matrix4d Ad_inv = CalcAd(
      params_.gait_params.height,
      params_.gait_params.mass,
      params_.gait_params.single_stance_duration
  ).inverse();

  MatrixXd A_ws(4 * nx_, nx_ + 1);

  A_ws.setZero();
  A_ws.block<4,4>(0, 0) = Matrix4d::Identity();
  A_ws.block<4,4>(4, 0) = Ad_inv;
  A_ws.block<4,4>(8, 0) = Matrix4d::Identity();
  A_ws.block<4,4>(12, 0) = Ad_inv;
  A_ws.topRightCorner<8,1>() = VectorXd::Ones(8);
  A_ws.bottomRightCorner<8,1>() = -VectorXd::Ones(8);

  VectorXd lb = VectorXd::Constant(4 * nx_, -kInfinity);
  lb.head<8>() = stack<double>({-state_bound, -state_bound});
  VectorXd ub = VectorXd::Constant(4 * nx_, kInfinity);
  ub.tail<8>() = stack<double>({state_bound, state_bound});


  for (int i = 0; i < params_.nmodes - 1; ++i) {
    workspace_c_.push_back(
        prog_->AddLinearConstraint(A_ws, lb, ub, {xx_.at(i+1), ee_.at(i)})
    );
  }
}

void AlipS2SMPFC::MakeDynamicsConstraint() {
  alip_utils::AddS2SDynamicsConstraints(
      params_.gait_params, xx_, pp_, prog_.get_mutable());
}

void AlipS2SMPFC::MakeInitialConditionsConstraints() {

  initial_foot_c_ = prog_->AddLinearEqualityConstraint(
      Matrix3d::Identity(), Vector3d::Zero(), pp_.front()
  ).evaluator();

  initial_state_c_ = prog_->AddLinearEqualityConstraint(
      Eigen::MatrixXd::Identity(4, 6), Vector4d::Zero(), {xx_.front(), tau_, u_}
  ).evaluator();

  initial_time_constraint_ = prog_->AddBoundingBoxConstraint(
      params_.tmin, params_.tmax, tau_
  ).evaluator();

  ankle_torque_bounds_ = prog_->AddBoundingBoxConstraint(-1, 1, u_).evaluator();
}

void AlipS2SMPFC::UpdateInitialConditions(
    const Eigen::Vector4d &x, const Eigen::Vector3d &p,
    double t, double tmin, double tmax) {

  initial_foot_c_->UpdateCoefficients(Matrix3d::Identity(), p);

  Matrix4d A = CalcA(params_.gait_params.height, params_.gait_params.mass);
  Matrix4d Ad = CalcAd(
      params_.gait_params.height, params_.gait_params.mass, t);
  Vector4d Bd = alip_utils::CalcBd(
      params_.gait_params.height, params_.gait_params.mass, t);
  Eigen::Matrix<double, 4, 5> A_t = Eigen::Matrix<double, 4, 5>::Zero();

  // linear approximation of exp(At) = exp(A t_*) + A exp(A t_*) * (t  -t*)
  Vector4d c = (Ad - A * Ad * t) * x;
  A_t.leftCols<4>() = Matrix4d::Identity();
  A_t.rightCols<1>() = - A * Ad * x;

  Eigen::Matrix<double, 4, 6> A_dyn = Eigen::Matrix<double, 4, 6>::Zero();
  A_dyn.leftCols<5>() = A_t;
  A_dyn.rightCols<1>() = -Bd;

  initial_state_c_->UpdateCoefficients(A_dyn, c);

  initial_time_constraint_->UpdateLowerBound(
      Eigen::VectorXd::Constant(1,  tmin)
  );
  initial_time_constraint_->UpdateUpperBound(
      Eigen::VectorXd::Constant(1, std::max(tmin, tmax))
  );
  ankle_torque_bounds_->UpdateLowerBound(
      Eigen::VectorXd::Constant(1, -params_.umax)
  );
  ankle_torque_bounds_->UpdateUpperBound(
      Eigen::VectorXd::Constant(1, params_.umax)
  );
}

void AlipS2SMPFC::UpdateCrossoverConstraint(Stance stance) {
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

void AlipS2SMPFC::UpdateFootholdConstraints(
    const geometry::ConvexPolygonSet &footholds) {
  if (params_.miqp) {
    UpdateMIQPFootholdConstraints(footholds);
  } else {
    dynamic_cast<solvers::ConvexPolygonSetConstraint*>(
        foothold_set_c_.front().evaluator().get())->UpdatePolygons(footholds);
  }
}

void AlipS2SMPFC::UpdateMIQPFootholdConstraints(
    const ConvexPolygonSet& footholds) {
  const auto& polys = footholds.polygons();
  size_t n = std::min(kMaxFootholds, footholds.size());

  RowVectorXd choice_constraint = RowVectorXd::Zero(kMaxFootholds);
  choice_constraint.leftCols(n) = RowVectorXd::Ones(n);

  footstep_choice_c_->UpdateCoefficients(
      BlockDiagonalRepeat<double>(choice_constraint, mu_.size()),
      VectorXd::Ones(mu_.size())
  );

  for (auto& c : footstep_c_) {
    for (size_t i = 0; i < n; ++i) {
      const auto& [A, b] = polys.at(i).GetConstraintMatrices();
      c.at(i).UpdateCoefficients(A, b);
    }
    for (size_t i = n; i < kMaxFootholds; ++i) {
      c.at(i).deactivate();
    }
  }
  for (auto& c : footstep_c_eq_) {
    for (size_t i = 0; i < n; ++i) {
      const auto& [A, b] = polys.at(i).GetEqualityConstraintMatrices();
      c.at(i).UpdateCoefficients(A, b);
    }
    for (size_t i = n; i < kMaxFootholds; ++i) {
      c.at(i).deactivate();
    }
  }
}

void AlipS2SMPFC::UpdateInputCost(const Vector2d &vdes, Stance stance) {
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
    input_cost_.at(i).evaluator()->UpdateCoefficients(
        Q, b, 0, true // we know it's convex
    );
  }
}

void AlipS2SMPFC::UpdateTrackingCost(const Vector2d &vdes, Stance stance) {
  if (params_.tracking_cost_type ==
      alip_utils::AlipTrackingCostType::kVelocity) {
    UpdateTrackingCostVelocity(vdes, stance);
    UpdateTerminalCostVelocity(vdes, stance);
  } else {
    UpdateTrackingCostGait(vdes, stance);
    UpdateTerminalCostGait(vdes, stance);
  }
}

void AlipS2SMPFC::UpdateTrackingCostVelocity(const Vector2d &vdes, Stance stance) {
  int start_period = stance == Stance::kLeft ? 0 : 1;
  for (int i = 0; i < params_.nmodes - 1; ++i) {
    const Matrix4d& PI = PIs_.at((start_period + i) % 2);
    const Matrix<double, 4, 2>& g = gs_.at((start_period + i) % 2);
    Matrix4d Q = PI.transpose() * params_.Q * PI;
    Vector4d q = -2 * Q * g * vdes;
    Q += 1e-5 * Matrix4d::Identity();
    tracking_cost_.at(i).evaluator()->UpdateCoefficients(
        2 * Q, q, 0, true // we know it's convex
    );
  }
}

void AlipS2SMPFC::UpdateTerminalCostVelocity(const Vector2d &vdes, Stance stance) {
  int final_period = stance == Stance::kLeft ? params_.nmodes - 1 : params_.nmodes;
  const Matrix4d& PI = PIs_.at(final_period % 2);
  const Matrix<double, 4, 2>& g = gs_.at((final_period) % 2);
  Matrix4d Q = PI.transpose() * params_.Qf * PI;
  Vector4d q = -2 * Q * g * vdes;
  Q += 1e-5 * Matrix4d::Identity();
  terminal_cost_->UpdateCoefficients(Q, q, 0, true);
}

void AlipS2SMPFC::UpdateTrackingCostGait(const Vector2d &vdes, Stance stance) {
  AlipGaitParams gait_params = params_.gait_params;
  gait_params.desired_velocity = vdes;
  gait_params.initial_stance_foot = stance;

  const auto& [x0, x1] = alip_utils::MakePeriodicAlipGait(gait_params);
  for (int i = 0; i < params_.nmodes - 1; ++i) {
    const Vector4d& xd = i % 2 == 0 ? x0: x1;
    tracking_cost_.at(i).evaluator()->UpdateCoefficients(
        2 * params_.Q, -2*params_.Q * xd, 0, true // we know it's convex
    );
  }

}

void AlipS2SMPFC::UpdateTerminalCostGait(const Eigen::Vector2d &vdes,
                                         alip_utils::Stance stance) {
  AlipGaitParams gait_params = params_.gait_params;
  gait_params.desired_velocity = vdes;
  gait_params.initial_stance_foot = stance;

  const auto& [x0, x1] = alip_utils::MakePeriodicAlipGait(gait_params);

  const Vector4d& xd = params_.nmodes % 2 == 0 ? x1 : x0;
  terminal_cost_->UpdateCoefficients(
      2* params_.Qf, -2 * params_.Qf * xd, 0, true);

}

void AlipS2SMPFC::UpdateTrustRegionConstraint(double t, const Vector3d& p) {
  double bound_size = t <= params_.tmin ? std::max(t, 0.04) : kInfinity;
  Vector3d bound = Vector3d::Constant(bound_size);
  trust_region_->UpdateLowerBound(p - bound);
  trust_region_->UpdateUpperBound(p + bound);
}


void AlipS2SMPFC::UpdateTimeRegularization(double t) {
  time_regularization_->UpdateCoefficients(
      2 * params_.time_regularization * MatrixXd::Identity(1,1),
      -2 * params_.time_regularization * VectorXd::Constant(1, t)
  );
}

}