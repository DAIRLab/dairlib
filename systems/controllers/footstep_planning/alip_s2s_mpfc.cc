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
    const Vector4d &x,const Vector3d &p, double t, const Vector2d& vdes,
    Stance stance, const ConvexPolygonSet& footholds) {

  auto start = std::chrono::steady_clock::now();

  UpdateInitialConditions(x, p, t);
  UpdateCrossoverConstraint(stance);
  UpdateFootholdConstraints(footholds);
  UpdateInputCost(vdes, stance);
  UpdateTrackingCost(vdes);
  UpdateTerminalCost(vdes);
  UpdateTimeRegularization(t);

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

  auto end = std::chrono::steady_clock::now();

  std::chrono::duration<double> total_time = end - start;
  std::chrono::duration<double> solve_time = solver_end - solver_start;

  mpfc_solution.optimizer_time = solve_time.count();
  mpfc_solution.total_time = total_time.count();
  mpfc_solution.input_footholds = footholds;
  mpfc_solution.desired_velocity = vdes;

  return mpfc_solution;
}

void AlipS2SMPFC::MakeMPCVariables() {
  tau_ = prog_->NewContinuousVariables(1, "t0");
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
            params_.soft_constraint_cost * MatrixXd::Identity(1,1),
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

  // build cost matrices
  p2o_premul_ = (Matrix4d::Identity() - A_ * A_).inverse();
  p2o_basis_ = p2o_premul_ * (A_* B_ - B_);
  p2o_orthogonal_complement_ = Eigen::FullPivLU<Eigen::Matrix<double, 2, 4>>(
          p2o_basis_.transpose()
  ).kernel().transpose();
  p2o_cost_hessian_ =
      p2o_orthogonal_complement_.transpose() * p2o_orthogonal_complement_;
  p2o_cost_gradient_factor_p1_ = -2 * p2o_cost_hessian_ * p2o_premul_ * B_;
  p2o_cost_gradient_factor_p2_ = -2 * p2o_cost_hessian_ * A_ * p2o_premul_ * B_;
}

void AlipS2SMPFC::MakeInputConstraints() {
  constexpr double bigM = 20.0;

  for (int i = 0; i < params_.nmodes - 1; ++i) {
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

}

void AlipS2SMPFC::MakeStateConstraints() {
  Vector4d state_bound;
  state_bound.head<2>() = params_.com_pos_bound;
  state_bound.tail<2>() = params_.gait_params.mass *
      params_.gait_params.height * params_.com_vel_bound;
  MatrixXd A_ws(2 * nx_, nx_ + 1);
  A_ws.setZero();

  VectorXd lb = VectorXd::Constant(8, -kInfinity);
  lb.tail<4>() = -state_bound;

  VectorXd ub = VectorXd::Constant(8, kInfinity);
  ub.tail<4>() = state_bound;

  A_ws.topLeftCorner<4,4>() = Matrix4d::Identity();
  A_ws.bottomLeftCorner<4,4>() = Matrix4d::Identity();
  A_ws.topRightCorner<4,1>() = -Vector4d::Ones();
  A_ws.bottomRightCorner<4,1>() = Vector4d::Ones();

  for (int i = 0; i < params_.nmodes - 1; ++i) {
    workspace_c_.push_back(
        prog_->AddLinearConstraint(A_ws, lb, ub, {xx_.at(i+1), ee_.at(i)})
    );
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
      Eigen::MatrixXd::Identity(4, 5), Vector4d::Zero(), {xx_.front(), tau_}
  ).evaluator();

  initial_time_constraint_ = prog_->AddBoundingBoxConstraint(
      params_.tmin, params_.tmax, tau_
  ).evaluator();

}

void AlipS2SMPFC::UpdateInitialConditions(
    const Eigen::Vector4d &x, const Eigen::Vector3d &p, double t) {

  initial_foot_c_->UpdateCoefficients(Matrix3d::Identity(), p);

  Matrix4d Ad = CalcAd(
      params_.gait_params.height, params_.gait_params.mass, t);
  Eigen::Matrix<double, 4, 5> A = Eigen::Matrix<double, 4, 5>::Zero();

  A.leftCols<4>() = Matrix4d::Identity();
  Vector4d c = Ad * x;

  if (t > params_.tmin) {
    // linear approximation of exp(At) = exp(A t_*) + A exp(A t_*) * (t  -t*)
    A.rightCols<1>() = - A_ * Ad * c;
    c = (Ad - A_ * Ad * t) * x;

    initial_time_constraint_->UpdateLowerBound(
        Eigen::VectorXd::Constant(1,  params_.tmin)
    );
    initial_time_constraint_->UpdateUpperBound(
        Eigen::VectorXd::Constant(1, params_.tmax)
    );
  } else {
    initial_time_constraint_->UpdateLowerBound(
        Eigen::VectorXd::Constant(1, t)
    );
    initial_time_constraint_->UpdateUpperBound(
        Eigen::VectorXd::Constant(1, t)
    );
  }

  initial_state_c_->UpdateCoefficients(A, c);
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
  const auto ud = MakeP2Orbit(gait_params);

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

void AlipS2SMPFC::UpdateTrackingCost(const Vector2d &vdes) {
  for (int i = 0; i < params_.nmodes - 1; ++i) {
    const Matrix<double, 4, 2>& vdes_mul = i % 2 == 0 ?
        p2o_cost_gradient_factor_p1_ : p2o_cost_gradient_factor_p2_;
    tracking_cost_.at(i).evaluator()->UpdateCoefficients(
        2 * p2o_cost_hessian_, vdes_mul * vdes, 0, true // we know it's convex
    );
  }
}

void AlipS2SMPFC::UpdateTerminalCost(const Vector2d &vdes) {
  const Matrix<double, 4, 2>& vdes_mul = params_.nmodes % 2 == 0 ?
      p2o_cost_gradient_factor_p1_ : p2o_cost_gradient_factor_p2_;

  Matrix4d Qf = 200.0 * p2o_cost_hessian_;
  Vector4d bf = 100.0 * vdes_mul * vdes;

  terminal_cost_->UpdateCoefficients(Qf, bf, 0, true);
}

void AlipS2SMPFC::UpdateTimeRegularization(double t) {
  if (t < params_.tmin) {
    time_regularization_->UpdateCoefficients(
        MatrixXd::Zero(1,1), VectorXd::Zero(1)
    );
  } else {
    time_regularization_->UpdateCoefficients(
        2 * params_.time_regularization * MatrixXd::Identity(1,1),
        -2 * params_.time_regularization * VectorXd::Constant(1, t)
    );
  }
}

std::vector<Eigen::Vector2d> AlipS2SMPFC::MakeP2Orbit(
    AlipGaitParams gait_params) {

  double s = gait_params.initial_stance_foot == Stance::kLeft ? -1 : 1;
  Vector2d u0 = Vector2d::Zero();
  u0(0) = gait_params.desired_velocity(0) * (
      gait_params.single_stance_duration +
      gait_params.double_stance_duration
  );
  u0(1) = gait_params.stance_width * s + gait_params.desired_velocity(1) * (
      gait_params.single_stance_duration +
      gait_params.double_stance_duration
  );
  Vector2d u1 = u0;
  u1(1) = - 2 * (s * gait_params.stance_width);
  return {u0, u1};
}

}