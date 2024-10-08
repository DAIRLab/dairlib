#include "alip_s2s_mpfc.h"
#include <algorithm>
#include <iostream>
#include <chrono>
#include "common/eigen_utils.h"

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

static constexpr double kInfinity = std::numeric_limits<double>::infinity();

AlipS2SMPFC::AlipS2SMPFC(alip_s2s_mpfc_params params) : params_(params){
  ValidateParams();
  const auto[A, B] = AlipStepToStepDynamics(
      params_.gait_params.height,
      params_.gait_params.mass,
      params_.gait_params.single_stance_duration,
      params_.gait_params.double_stance_duration,
      params_.gait_params.reset_discretization_method
  );
  A_ = A;
  B_ = B;
  MakeMPCVariables();
  MakeMPCCosts();
  MakeInputConstraints();
  MakeStateConstraints();
  MakeDynamicsConstraint();
  MakeInitialConditionsConstraints();
  Check();
}

alip_s2s_mpfc_solution AlipS2SMPFC::Solve(
    const Vector4d &x,const Vector3d &p, double t, double tmin, double tmax,
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

  auto solver_start = std::chrono::steady_clock::now();

  auto result = solver_.Solve(*prog_, std::nullopt, params_.solver_options);

  auto solver_end = std::chrono::steady_clock::now();

  if (not result.is_success()) {
    std::cout << "Solve failed wth code "
              << result.get_solution_result() << std::endl;
    std::cout << "x: " << x.transpose() << std::endl;
    std::cout << "t: " << t << std::endl;
  }

  alip_s2s_mpfc_solution mpfc_solution;

  mpfc_solution.success = result.is_success();
  mpfc_solution.solution_result = result.get_solution_result();

  auto solution_details =
      result.get_solver_details<drake::solvers::GurobiSolver>();

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
    mpfc_solution.mu.push_back(result.GetSolution(mu_.at(i)));
  }

  mpfc_solution.t_nom = t;
  mpfc_solution.t_sol = result.GetSolution(tau_)(0);
  mpfc_solution.u_sol = result.GetSolution(u_)(0);
  mpfc_solution.desired_velocity = vdes;

  auto end = std::chrono::steady_clock::now();

  std::chrono::duration<double> total_time = end - start;
  std::chrono::duration<double> solve_time = solver_end - solver_start;

  mpfc_solution.optimizer_time = solution_details.optimizer_time;
  mpfc_solution.total_time = total_time.count();
  mpfc_solution.input_footholds = footholds;

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
    mu_.push_back(prog_->NewBinaryVariables(kMaxFootholds, "mu_" + mode));
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
  alip_utils::MakeAlipStepToStepCostMatrices(
      params_.gait_params, params_.Q, params_.Qf,
      Q_proj_, Q_proj_f_,
      g_proj_p1_, g_proj_p2_,
      p2o_premul_, projection_to_p2o_complement_,
      p2o_orthogonal_complement_, p2o_basis_
  );
}

void AlipS2SMPFC::MakeInputConstraints() {
  constexpr double bigM = 20.0;

  Matrix<double, 2, 6> A_reach = Matrix<double, 2, 6>::Zero();
  A_reach.leftCols<2>() = -Matrix2d ::Identity();
  A_reach.block<2, 2>(0, 2) = -Matrix2d::Identity();
  A_reach.rightCols<2>() = Matrix2d::Identity();
  for (int i = 0; i < params_.nmodes - 1; ++i) {
    reachability_c_.push_back(
        prog_->AddLinearConstraint(
            A_reach,
            -params_.com_pos_bound,
            params_.com_pos_bound,
            {xx_.at(i).head<2>(), pp_.at(i).head<2>(), pp_.at(i+1).head<2>()}
        ));
    no_crossover_c_.push_back(
        prog_->AddLinearConstraint(
            MatrixXd::Ones(1, 2),
            VectorXd::Constant(1, -kInfinity),
            VectorXd::Constant(1, kInfinity),
            {pp_.at(i).segment(1,1), pp_.at(i+1).segment(1,1)}
        ));
    vector<LinearBigMConstraint> tmp;
    vector<LinearBigMEqualityConstraint> tmp_eq;
    for (int j = 0; j < kMaxFootholds; ++j) {
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

  MatrixXd A_trust = MatrixXd::Zero(3, 6);
  A_trust.leftCols<3>().setIdentity();
  A_trust.rightCols<3>().setIdentity();
  A_trust.rightCols<3>() *= -1;
  trust_region_ = prog_->AddLinearConstraint(
      A_trust, -Vector3d::Constant(kInfinity), Vector3d::Constant(kInfinity),
      {pp_.at(1), pp_.at(0)}
  ).evaluator();
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
    //std::cout << workspace_c_.at(i).ToLatex() << std::endl;
  }
}

void AlipS2SMPFC::MakeDynamicsConstraint() {
  MatrixXd M(nx_ * (params_.nmodes - 1), (nx_ + np_) * params_.nmodes);
  M.setZero();
  for (int i = 0; i < params_.nmodes - 1; ++i) {
    M.block<4,4>(nx_ * i, nx_ * i) = A_;
    M.block<4,4>(nx_ * i, nx_ * (i + 1)) = -Matrix4d::Identity();
    M.block<4,2>(nx_ * i, nx_ * params_.nmodes + np_ * (i+1)) = B_;
    M.block<4,2>(nx_ * i, nx_ * params_.nmodes + np_ * i) = - B_;
  }
  dynamics_c_ = prog_->AddLinearEqualityConstraint(
      M, VectorXd::Zero(nx_ * (params_.nmodes - 1)), {stack(xx_), stack(pp_)}
  ).evaluator();
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
      params_.gait_params.height, params_.gait_params.mass, tmin);
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
      Eigen::VectorXd::Constant(1, tmax)
  );
  ankle_torque_bounds_->UpdateLowerBound(
      Eigen::VectorXd::Constant(1, -params_.umax * t)
  );
  ankle_torque_bounds_->UpdateUpperBound(
      Eigen::VectorXd::Constant(1, params_.umax * t)
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

void AlipS2SMPFC::UpdateFootholdConstraints(const ConvexPolygonSet &footholds) {
  const auto& polys = footholds.polygons();
  int n = std::min(kMaxFootholds, footholds.size());

  RowVectorXd choice_constraint = RowVectorXd::Zero(kMaxFootholds);
  choice_constraint.leftCols(n) = RowVectorXd::Ones(n);

  footstep_choice_c_->UpdateCoefficients(
      BlockDiagonalRepeat<double>(choice_constraint, mu_.size()),
      VectorXd::Ones(mu_.size())
  );

  for (auto& c : footstep_c_) {
    for (int i = 0; i < n; ++i) {
      const auto& [A, b] = polys.at(i).GetConstraintMatrices();
      c.at(i).UpdateCoefficients(A, b);
    }
    for (int i = n; i < kMaxFootholds; ++i) {
      c.at(i).deactivate();
    }
  }
  for (auto& c : footstep_c_eq_) {
    for (int i = 0; i < n; ++i) {
      const auto& [A, b] = polys.at(i).GetEqualityConstraintMatrices();
      c.at(i).UpdateCoefficients(A, b);
    }
    for (int i = n; i < kMaxFootholds; ++i) {
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
    UpdateTrackingCostVelocity(vdes);
    UpdateTerminalCostVelocity(vdes);
  } else {
    UpdateTrackingCostGait(vdes, stance);
    UpdateTerminalCostGait(vdes, stance);
  }
}

void AlipS2SMPFC::UpdateTrackingCostVelocity(const Vector2d &vdes) {
  for (int i = 0; i < params_.nmodes - 1; ++i) {
    const Matrix<double, 4, 2>& vdes_mul = i % 2 == 0 ?
        -2 * Q_proj_ * g_proj_p1_ : -2 * Q_proj_ * g_proj_p2_;
    tracking_cost_.at(i).evaluator()->UpdateCoefficients(
        2 * Q_proj_, vdes_mul * vdes, 0, true // we know it's convex
    );
  }
}

void AlipS2SMPFC::UpdateTerminalCostVelocity(const Vector2d &vdes) {
  const Matrix<double, 4, 2>& vdes_mul = params_.nmodes % 2 == 0 ?
      -2 * Q_proj_f_ * g_proj_p1_ : -2 * Q_proj_f_ * g_proj_p2_;
  Matrix4d Qf = 2 * Q_proj_f_;
  Vector4d bf = vdes_mul * vdes;
  terminal_cost_->UpdateCoefficients(Qf, bf, 0, true);
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
};


void AlipS2SMPFC::UpdateTimeRegularization(double t) {
  time_regularization_->UpdateCoefficients(
      2 * params_.time_regularization * MatrixXd::Identity(1,1),
      -2 * params_.time_regularization * VectorXd::Constant(1, t)
  );
}

}