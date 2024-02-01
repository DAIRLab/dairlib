#include "alip_s2s_mpfc.h"
#include <algorithm>
#include <iostream>
#include "common/eigen_utils.h"

namespace dairlib::systems::controllers{

using Eigen::MatrixXd;
using Eigen::Matrix4d;
using Eigen::VectorXd;
using Eigen::Vector4d;
using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::RowVectorXd;
using Eigen::RowVector3d;

static constexpr double kInfinity = std::numeric_limits<double>::infinity();

void AlipS2SMPFC::MakeMPCVariables() {
  t0_ = prog_->NewContinuousVariables(1, "exp(wt)");
  for (int i = 0; i < params_.nmodes; ++i) {
    std::string mode = std::to_string(i);
    xx_.push_back(prog_->NewContinuousVariables(nx_, "xx_" + mode));
    pp_.push_back(prog_->NewContinuousVariables(np_, "pp_" + mode));
  }
  for (int i = 1; i < params_.nmodes; ++i) {
    std::string mode = std::to_string(i);
    ee_.push_back(prog_->NewContinuousVariables(1, "ee_" + mode));
    mu_.push_back(prog_->NewBinaryVariables(kMaxFootholds, "mu_" + mode));
  }
}

void AlipS2SMPFC::MakeMPCCosts() {
  for (int i = 0; i < params_.nmodes - 1; ++i) {
    tracking_cost_.push_back(
        prog_->AddQuadraticCost(
            Matrix4d::Identity(), Vector4d::Zero(),xx_.at(i)
        ));
    input_cost_.push_back(
        prog_->AddQuadraticCost(
            Matrix4d::Identity(), Vector4d::Zero(),xx_.at(i)
        ));
    soft_constraint_cost_.push_back(
        prog_->AddQuadraticCost(
            params_.soft_constraint_cost * MatrixXd::Identity(1,1),
            VectorXd::Zero(1),
            ee_.at(i)
        ));
  }
}

void AlipS2SMPFC::MakeInputConstraints() {
  constexpr double bigM = 20.0;

  for (int i = 0; i < params_.nmodes - 1; ++i) {
    no_crossover_c_.push_back(
        prog_->AddLinearConstraint(
            MatrixXd::Zero(1, 2),
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
              RowVector3d::Zero(),
              VectorXd::Zero(1),
              bigM,
              pp_.at(i+1),
              mu_.at(i)(j)
          ));
      tmp_eq.push_back(
          LinearBigMEqualityConstraint(
              *prog_,
              RowVector3d::Zero(),
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

  VectorXd ub = VectorXd::Constant(8, -kInfinity);
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
  const auto[A, B] = alip_utils::AlipStepToStepDynamics(
        params_.gait_params.height,
        params_.gait_params.mass,
        params_.gait_params.single_stance_duration,
        params_.gait_params.double_stance_duration,
        params_.gait_params.reset_discretization_method
  );

  MatrixXd M(nx_ * (params_.nmodes - 1), (nx_ + np_) * params_.nmodes);
  M.setZero();

  for (int i = 0; i < params_.nmodes - 1; ++i) {
    M.block<4,4>(nx_ * i, nx_ * i) = A;
    M.block<4,4>(nx_ * i, nx_ * (i + 1)) = -Matrix4d::Identity();
    M.block<4,2>(nx_ * i, nx_ * params_.nmodes + np_ * i) = B;
  }
}

}