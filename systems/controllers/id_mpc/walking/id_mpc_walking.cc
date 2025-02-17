#include "id_mpc_walking.h"
#include "systems/controllers/id_mpc/costs/relative_position_cost.h"

namespace dairlib::systems::controllers::id_mpc {

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

using geometry::ConvexPolygon;
using geometry::ConvexPolygonSet;
using solvers::ConvexPolygonSetConstraint;

IDMPCWalking::IDMPCWalking(
    IDMPCParams params, std::unique_ptr<ConstrainedDynamicsInfo> dynamics,
    GaitParams gait_params) :
    mpc_(params, std::move(dynamics)), params_(gait_params) {

  MakeFootsteps();
  MakeSwingTrajCosts();
  MakeGroundConstraints();
  MakeFootLevelingCosts();
}

void IDMPCWalking::UpdateProblemData(
    const MPCReference &reference, const VectorXd &initial_state) {
  mpc_.UpdateProblemData(reference, initial_state);
  UpdateFootstepConstraints(reference.touchdown_ee_names_,
                            reference.touchdown_ee_points_);

}

void IDMPCWalking::UpdateFootstepConstraints(
    const std::vector<std::string> &foot_names,
    const std::vector<Vector3d> &contact_points) {
  for (int i = 1; i < params_.mpc_N + 1;  ++i) {
    td_constraints_.at(i - 1)->set_point(
        foot_names.at(i), contact_points.at(i));
  }
}

void IDMPCWalking::SetFootstepInitialGuess(const std::vector<Vector3d>& pp) {
  DRAKE_DEMAND(pp.size() == pp_.size());
  for (size_t i = 0; i < pp.size(); ++i) {
    mpc_.get_prog().SetInitialGuess(pp_.at(i), pp.at(i));
  }
}

void IDMPCWalking::UpdateFootstepLocationsInStackedVariables(
    const std::vector<Vector3d>& pp, VectorXd *z) const {
  for (size_t i = 0; i < pp_.size(); ++i) {
    mpc_.SetDecisionVariableValue(pp_.at(i), pp.at(i), z);
  }
}

void IDMPCWalking::MakeFootsteps() {
  auto& prog = mpc_.get_prog();

  int intervals = std::round((params_.t_ss + params_.t_ds) / params_.mpc_dt);
  int num_touchdowns = (params_.mpc_N / intervals);

  // unless the number of intervals between touchdowns evenly divides the MPC
  // horizon, we need to add an extra touchdown event
  if (num_touchdowns * intervals > params_.mpc_N) {
    ++num_touchdowns;
  }

  DRAKE_DEMAND(params_.footstep_horizon >= num_touchdowns);

  for (int i = 0; i < params_.footstep_horizon; ++i) {
    pp_.push_back(prog.NewContinuousVariables(3, "p_" + std::to_string(i)));
  }

  // Make the touchdown constraints, noting that the initial footstep should
  // just be set by FK to the current stance foot (so we skip it here)
  for (int i = 1; i <= params_.mpc_N; ++i) {
    int step_idx = i / intervals + 1;
    auto pos_constraint = std::make_shared<PointPositionConstraint<AutoDiffXd>>(
        mpc_.dynamics(), "", Vector3d::Zero());
    prog.AddConstraint(
        pos_constraint, {mpc_.position_vars(i), pp_.at(step_idx)});
    td_constraints_.push_back(pos_constraint);
  }
}

void IDMPCWalking::MakeGroundConstraints() {
  auto terrain = ConvexPolygonSet({ConvexPolygon::MakeFlatGround()});
  MatrixXd A = MatrixXd::Identity(1,1);
  for (size_t i = 1; i < pp_.size(); ++i) {
    mpc_.get_prog().AddLinearConstraint(
        A, VectorXd::Zero(1), VectorXd::Constant(1, 0.01), pp_[i].tail<1>());
    footholds_.push_back(
        std::make_shared<ConvexPolygonSetConstraint>(terrain));
    foothold_bindings_.push_back(
        mpc_.get_prog().AddConstraint(footholds_.back(), pp_.at(i)));
  }
}

void IDMPCWalking::MakeSwingTrajCosts() {
  mpc_.AddTaskCost<RelativePositionCost>(
      "swing_foot", params_.foot_pos_W, Vector3d::Zero(),
      mpc().dynamics().get_plant(),
      params_.right_foot_body_name, params_.left_foot_body_name,
      Vector3d::Zero(), Vector3d::Zero());
}

void IDMPCWalking::MakeFootLevelingCosts() {
  Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
  Q(2,2) = params_.foot_pos_W(2,2);
  mpc_.AddTaskCost<RelativePositionCost>(
      "foot_level_left", Q, Vector3d::Zero(),
      mpc().dynamics().get_plant(),
      params_.left_foot_body_name, params_.left_foot_body_name,
      params_.foot_rear, params_.foot_front);
  mpc_.AddTaskCost<RelativePositionCost>(
      "foot_level_right", Q, Vector3d::Zero(),
      mpc().dynamics().get_plant(),
      params_.right_foot_body_name, params_.right_foot_body_name,
      params_.foot_rear, params_.foot_front);
}

solvers::NCQPSolver::SetMembershipConstraints
IDMPCWalking::GetFootholdConstraints(const VectorXd &z) {
  for (size_t i = 0; i < footholds_.size(); ++i) {
    footholds_.at(i)->SetShift(
        mpc().GetDecisionVariableValue(pp_.at(i + 1), z));
  }
  return solvers::NCQPSolver::ExtractSetMembershipConstraints(
      mpc_.get_prog(), foothold_bindings_);
}

}