#include "id_mpc_walking.h"

namespace dairlib::systems::controllers::id_mpc {

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

IDMPCWalking::IDMPCWalking(
    IDMPCParams params, std::unique_ptr<ConstrainedDynamicsInfo> dynamics,
    GaitParams gait_params) :
    mpc_(params, std::move(dynamics)), params_(gait_params) {

  MakeFootsteps();
  MakeGroundConstraints();
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
  MatrixXd A = MatrixXd::Identity(1,1);
  for (size_t i = 1; i < pp_.size(); ++i) {
    mpc_.get_prog().AddLinearConstraint(
        A, VectorXd::Zero(1), VectorXd::Constant(1, 0.01), pp_[i].tail<1>());
  }
}

}