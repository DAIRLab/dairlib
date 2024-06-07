#include "cf_mpfc.h"
#include "common/eigen_utils.h"

namespace dairlib {
namespace systems {
namespace controllers {

using alip_utils::CalcAd;
using alip_utils::Stance;
using alip_utils::AlipGaitParams;

using cf_mpfc_utils::SrbDim;

using std::vector;
using drake::solvers::Binding;
using drake::solvers::QuadraticCost;
using drake::solvers::DecisionVariable;
using drake::solvers::LinearConstraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::BoundingBoxConstraint;
using drake::solvers::VectorXDecisionVariable;

using drake::solvers::LinearEqualityConstraint;
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
  MakeMPCVariables();
  MakeMPCCosts();
  MakeFootstepConstraints();
  MakeFrictionConeConstraints();
  MakeStateConstraints();
  MakeDynamicsConstraints();
  MakeInitialConditionsConstraints();
  Check();
}

cf_mpfc_solution CFMPFC::Solve(
    const CentroidalState<double> &x, const Eigen::Vector3d &p, double t,
    const Eigen::Vector2d &vdes, alip_utils::Stance stance,
    const Eigen::Matrix3d &I, const cf_mpfc_solution &prev_sol) {
  auto start = std::chrono::steady_clock::now();

  UpdateInitialConditions(x, p);
  UpdateCrossoverConstraint(stance);
  UpdateFootstepCost(vdes, stance);
  UpdateTrackingCost(vdes, stance);
  UpdateSRBDCosts(vdes, stance);

  std::vector<CentroidalState<double>> xc;
  std::vector<VectorXd> ff;

  bool use_prev_sol = prev_sol.init and prev_sol.stance == stance;

  for (int i = 0; i < params_.nknots - 1; ++i) {
    if (use_prev_sol) {
      xc.push_back(prev_sol.xc.at(i));
      ff.push_back(prev_sol.ff.at(i));
    } else {
      Vector3d model_force = -x.segment<3>(cf_mpfc_utils::com_idx) *
          (9.81 * params_.gait_params.mass / x(cf_mpfc_utils::com_idx+2));
      vector<VectorXd> forces;
      for (int i = 0; i < params_.contacts_in_stance_frame.size(); ++i) {
        forces.push_back(model_force);
      }
      xc.push_back(x);
      ff.push_back(stack(forces));
    }
  }
  if (use_prev_sol) {
    xc.push_back(prev_sol.xc.back());
    xc.front() = x;
  } else {
    xc.push_back(x);
  }

  double s = stance == Stance::kLeft ? -1 : 1;
  Vector3d p_post = use_prev_sol ? prev_sol.pp.at(1) : p + s * params_.gait_params.stance_width * Vector3d::UnitY();

  UpdateSRBDynamicsConstraint(xc, ff, I, t);
  UpdateModelSwitchConstraint(x, p, p_post, I);

  auto solver_start = std::chrono::steady_clock::now();
  auto result = solver_.Solve(*prog_, std::nullopt, params_.solver_options);
  auto solver_end = std::chrono::steady_clock::now();


  cf_mpfc_solution mpfc_solution;
  mpfc_solution.success = result.is_success();
  mpfc_solution.solution_result = result.get_solution_result();

  auto solution_details =
      result.get_solver_details<drake::solvers::GurobiSolver>();

  auto end = std::chrono::steady_clock::now();

  std::chrono::duration<double> total_time = end - start;
  std::chrono::duration<double> solve_time = solver_end - solver_start;

  mpfc_solution.optimizer_time = solution_details.optimizer_time;
  mpfc_solution.total_time = total_time.count();
  
  return mpfc_solution;
}

void CFMPFC::MakeMPCVariables() {
  DRAKE_DEMAND(prog_->num_vars() == 0);

  int nc = params_.contacts_in_stance_frame.size();
  for (int i = 0; i < params_.nknots; ++i) {
    xc_.push_back(prog_->NewContinuousVariables(SrbDim,
                                                "xc" + std::to_string(i)));
  }
  // TODO (@Brian-Acosta) move this up to add another force variable
  //  when changing from ZOH to trapezoidal collocation
  for (int i = 0; i < params_.nknots - 1; ++i) {
    ff_.push_back(prog_->NewContinuousVariables(3 * nc,
                                                "ff" + std::to_string(i)));
  }
  for (int i = 0; i < params_.nmodes; ++i) {
    pp_.push_back(prog_->NewContinuousVariables(3, "pp" + std::to_string(i)));
  }
  xi_ = prog_->NewContinuousVariables(4, "xi");
  for (int i = 0; i < params_.nmodes - 1; ++i) {
    xx_.push_back(prog_->NewContinuousVariables(4,
                                                "xx" + std::to_string(i + 1)));
    ee_.push_back(prog_->NewContinuousVariables(1,
                                                "ee" + std::to_string(i + 1)));
  }
}

void CFMPFC::MakeMPCCosts() {
  DRAKE_DEMAND(centroidal_state_cost_.empty());
  DRAKE_DEMAND(centroidal_input_cost_.empty());
  DRAKE_DEMAND(tracking_cost_.empty());
  DRAKE_DEMAND(footstep_cost_.empty());
  DRAKE_DEMAND(soft_constraint_cost_.empty());
  DRAKE_DEMAND(terminal_cost_ == nullptr);

  int nc = params_.contacts_in_stance_frame.size();
  for (int i = 0; i < params_.nknots - 1; ++i) {
    // state and input cost for first model phase
    // TODO (@Brian-Acosta) After initial prototype, make sure to include
    //  the appropriate CoM height cost and CoM Vel cost to incorporate
    //  footstep height change
    centroidal_state_cost_.push_back(
        prog_->AddQuadraticCost(
            MatrixXd::Identity(SrbDim, SrbDim),
            VectorXd::Zero(SrbDim), xc_.at(i+1)
        ));
    centroidal_input_cost_.push_back(
        prog_->AddQuadraticCost(
            MatrixXd::Identity(FootstepDim * nc, FootstepDim * nc),
            VectorXd::Zero(FootstepDim * nc),
            ff_.at(i)
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
  Matrix<double, 2, 6> A_reach = Matrix<double, 2, 6>::Zero();
  A_reach.leftCols<2>() = -Matrix2d::Identity();
  A_reach.block<2, 2>(0, 2) = -Matrix2d::Identity();
  A_reach.rightCols<2>() = Matrix2d::Identity();
  for (int i = 0; i < params_.nmodes - 1; ++i) {
    VectorXDecisionVariable x = i == 0 ?
        xc_.back().segment<2>(cf_mpfc_utils::com_idx) : xx_.at(i-1).head<2>();
    reachability_c_.push_back(
        prog_->AddLinearConstraint(
            A_reach,
            -params_.com_pos_bound,
            params_.com_pos_bound,
            {x, pp_.at(i).head<2>(), pp_.at(i + 1).head<2>()}
        ));
    no_crossover_c_.push_back(
        prog_->AddLinearConstraint(
            MatrixXd::Ones(1, 2),
            VectorXd::Constant(1, -kInfinity),
            VectorXd::Constant(1, kInfinity),
            {pp_.at(i).segment(1, 1), pp_.at(i + 1).segment(1, 1)}
        ));
  }
  // TODO (@Brian-Acosta) MIQP foothold constraints
}

void CFMPFC::MakeFrictionConeConstraints() {
  DRAKE_DEMAND(fcone_constraints_.empty());

  // Pyramidal friction cone
  MatrixXd A = MatrixXd(5, 3);
  double mu = params_.mu;
  A << -1, 0, mu, 0, -1, mu, 1, 0, mu, 0, 1, mu, 0, 0, 1;
  for (const auto& f : ff_) {
    for (int i = 0; i < params_.contacts_in_stance_frame.size(); ++i) {
      fcone_constraints_.push_back(
          prog_->AddLinearConstraint(
              A, VectorXd::Zero(5),
              VectorXd::Constant(5, std::numeric_limits<double>::infinity()),
              f.segment<3>(3*i)));
    }
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


  for (int i = 0; i < params_.nmodes - 1; ++i) {
    workspace_c_.push_back(
        prog_->AddLinearConstraint(A_ws, lb, ub, {xx_.at(i), ee_.at(i)})
    );
    //std::cout << workspace_c_.at(i).ToLatex() << std::endl;
  }
}

void CFMPFC::MakeDynamicsConstraints() {
  DRAKE_DEMAND(alip_dynamics_c_ == nullptr);
  DRAKE_DEMAND(srb_dynamics_c_ == nullptr);

  MatrixXd M(
      AlipDim * (params_.nmodes - 2), (AlipDim + FootstepDim) * (params_.nmodes - 1));
  M.setZero();
  for (int i = 0; i < params_.nmodes - 2; ++i) {
    M.block<4,4>(AlipDim * i, AlipDim * i) = A_;
    M.block<4,4>(AlipDim * i, AlipDim * (i + 1)) = -Matrix4d::Identity();
    M.block<4,2>(AlipDim * i, AlipDim * params_.nmodes + FootstepDim * (i+1)) = B_;
    M.block<4,2>(AlipDim * i, AlipDim * params_.nmodes + FootstepDim * i) = - B_;
  }
  // first footstep is not part of the ALIP dynamics
  vector<VectorXDecisionVariable> pp;
  for (int i = 1; i < pp_.size(); ++i) {
    pp.push_back(pp_.at(i));
  }
  alip_dynamics_c_ = prog_->AddLinearEqualityConstraint(
      M, VectorXd::Zero(AlipDim * (params_.nmodes - 2)), {stack(xx_), stack(pp)}
  ).evaluator();


  int nc = params_.contacts_in_stance_frame.size();
  MatrixXd A(
      SrbDim * (params_.nknots - 1),
      SrbDim * params_.nknots + FootstepDim * nc * (params_.nknots - 1));
  A.setIdentity();

  srb_dynamics_c_ = prog_->AddLinearEqualityConstraint(
      A, VectorXd::Zero(SrbDim * (params_.nknots - 1)), {stack(xc_), stack(ff_)}
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
      Eigen::MatrixXd::Identity(SrbDim, SrbDim),
      CentroidalState<double>::Zero(),
      xc_.front()
  ).evaluator();

  MatrixXd M = MatrixXd::Zero(AlipDim, 2*AlipDim);
  M.leftCols<AlipDim>() = A_;
  M.rightCols<AlipDim>() = -Matrix4d::Identity();
  initial_alip_state_c_ = prog_->AddLinearEqualityConstraint(
      M, Vector4d::Zero(), {xi_, xx_.front()}
  ).evaluator();

  model_switch_c_ = prog_->AddLinearEqualityConstraint(
      MatrixXd::Identity(AlipDim, SrbDim + FootstepDim + AlipDim),
      Vector4d::Zero(),
      {xc_.back(), pp_.at(1), xi_}
  ).evaluator();
}

void CFMPFC::UpdateInitialConditions(
    const CentroidalState<double>& x, const Eigen::Vector3d& p) {

  initial_state_c_->UpdateCoefficients(
      Matrix<double, SrbDim, SrbDim>::Identity(), x);
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

void CFMPFC::UpdateSRBDynamicsConstraint(
    const vector<CentroidalState<double>>& xc, const vector<VectorXd>& ff,
    const Matrix3d &I, double t) {

  double h = t / (params_.nknots - 1);

  // Dynamics constraint order: x0, x1, ... xn, f0, ... fn-1
  MatrixXd A_dyn = srb_dynamics_c_->GetDenseA();
  A_dyn.setIdentity();
  VectorXd b_dyn = srb_dynamics_c_->lower_bound();
  b_dyn.setZero();

  MatrixXd A;
  MatrixXd Bp;
  MatrixXd Bf;
  VectorXd c;

  for (int i = 0; i < params_.nmodes - 1; ++i) {
    cf_mpfc_utils::LinearizeSRBDynamics(
        xc.at(i),
        params_.contacts_in_stance_frame,
        unstack<double, 3>(ff.at(i)),
        I, params_.gait_params.mass,
        A, Bp, Bf, c
    );

    // xdot = A (x - xk*) + B(f - fk*) + c
    // x_k+1 - x_k = h * A * (x_k - xk*) + h * B * (f_k - fk*) + h * c
    // (h * A + I) x_k - x_k+1 + (h * B) * f_k = h * (B * fk* + A * xk* - c)
    int nc = Bf.cols();
    A_dyn.block<SrbDim, SrbDim>(SrbDim*i, SrbDim*i) += A * h;
    A_dyn.block<SrbDim, SrbDim>(SrbDim*i, SrbDim * (i+1)) =
        -Matrix<double, SrbDim, SrbDim>::Identity();
    A_dyn.block(SrbDim * i, SrbDim * params_.nknots + nc * i, SrbDim, nc) = h * Bf;
    b_dyn.segment<SrbDim>(SrbDim * i) = h * (A * xc.at(i) + Bf * ff.at(i) - c);
  };
  srb_dynamics_c_->UpdateCoefficients(A_dyn, b_dyn);
}

void CFMPFC::UpdateModelSwitchConstraint(
    CentroidalState<double> xc, const Eigen::Vector3d &p_pre,
    const Eigen::Vector3d &p_post, const Matrix3d &I) {
  MatrixXd Ax;
  MatrixXd Bp;
  Vector4d b;
  cf_mpfc_utils::LinearizeReset(xc, p_pre, p_post, I, params_.gait_params.mass, Ax, Bp, b);
  // xi = Ax (x - xc) + Bp(p - p_post) + b
  // Ax * x + Bp * p - xi = Ax * xc + Bp * p_post - b
  MatrixXd A_eq = model_switch_c_->GetDenseA();
  A_eq.leftCols<SrbDim>() = Ax;
  A_eq.middleCols<FootstepDim>(SrbDim) = Bp;
  A_eq.rightCols<AlipDim>() = -Matrix4d::Identity();

  Vector4d b_eq = Ax * xc + Bp * p_post - b;
  model_switch_c_->UpdateCoefficients(A_eq, b_eq);
}

void CFMPFC::UpdateSRBDCosts(const Vector2d &vdes, alip_utils::Stance stance) {
  CentroidalState<double> xd = CentroidalState<double>::Zero();
  xd(0) = 1;
  xd(4) = 1;
  xd(8) = 1;
  xd(cf_mpfc_utils::com_idx + 2) = params_.gait_params.height;
  xd.segment<2>(cf_mpfc_utils::com_dot_idx) = vdes;

  int nc = params_.contacts_in_stance_frame.size();
  Vector3d fd = (9.81 * params_.gait_params.mass / nc) * Vector3d::UnitZ();

  MatrixXd Qx = MatrixXd::Identity(SrbDim, SrbDim);
  MatrixXd Qf = 0.001 * MatrixXd::Identity(3 * nc, 3 * nc);

  for (int i = 0; i < params_.nknots - 1; ++i) {
    centroidal_state_cost_.at(i).evaluator()->UpdateCoefficients(
        2 * Qx, -2 * Qx * xd, xd.transpose() * Qx * xd, true
    );
    centroidal_input_cost_.at(i).evaluator()->UpdateCoefficients(
        2 * Qf, -2 * Qf * fd, fd.transpose() * Qf * fd, true
    );
  }
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
    footstep_cost_.at(i).evaluator()->UpdateCoefficients(
        Q, b, 0, true // we know it's convex
    );
  }
}

void CFMPFC::UpdateTrackingCost(const Vector2d &vdes, Stance stance) {
  if (params_.tracking_cost_type == alip_utils::kVelocity) {
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
        2 * Q_proj_, vdes_mul * vdes, 0, true // we know it's convex
    );
  }
}

void CFMPFC::UpdateTerminalCostVelocity(const Vector2d &vdes) {
  const Matrix<double, 4, 2>& vdes_mul = params_.nmodes % 2 == 0 ?
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
        2 * params_.Q, -2*params_.Q * xd, 0, true // we know it's convex
    );
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


}
}
}