#include "id_mpc_walking.h"
#include "solvers/sqp/relative_position_cost.h"
#include "systems/controllers/footstep_planning/alip_utils.h"

namespace dairlib::systems::controllers::id_mpc {

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

using geometry::ConvexPolygon;
using geometry::ConvexPolygonSet;
using solvers::sqp::RelativePositionCost;
using solvers::ConvexPolygonSetConstraint;

IDMPCWalking::IDMPCWalking(
    IDMPCParams params, std::unique_ptr<ConstrainedDynamicsInfo> dynamics,
    GaitParams gait_params) :
    mpc_(params, std::move(dynamics)), params_(gait_params) {

  MakeFootsteps();
  MakeSwingTrajCosts();
  MakeGroundConstraints();
  MakeFootLevelingCosts();
  MakeALIPTerms();
}

void IDMPCWalking::UpdateProblemData(
    const MPCReference &reference, const VectorXd &initial_state,
    const VectorXd& prev_sol,
    const geometry::ConvexPolygonSet& footholds) {
  mpc_.UpdateProblemData(reference, initial_state, prev_sol);
  UpdateFootstepConstraints(reference.touchdown_ee_names_,
                            reference.touchdown_ee_points_);
  for (auto& foothold: footholds_) {
    foothold->UpdatePolygons(footholds);
  }
  UpdateALIPTerms(reference);
}

void IDMPCWalking::UpdateFootstepConstraints(
    const std::vector<std::string> &foot_names,
    const std::vector<Vector3d> &contact_points) {
  for (int i = 2; i < params_.mpc_N + 1;  ++i) {
    td_constraints_.at(i - 2)->set_point(
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

  DRAKE_DEMAND(params_.footstep_horizon * intervals > params_.mpc_N);
  DRAKE_DEMAND(params_.mpc_N % intervals == 0);

  for (int i = 0; i < params_.footstep_horizon; ++i) {
    pp_.push_back(prog.NewContinuousVariables(3, "p_" + std::to_string(i)));
  }

  // Make the touchdown constraints, noting that adding the constraint to the
  // first 2 timesteps would make the problem overconstrained
  for (int i = 2; i <= params_.mpc_N; ++i) {
    int step_idx = i / intervals ;
    auto pos_constraint = std::make_shared<PointPositionConstraint<AutoDiffXd>>(
        mpc_.dynamics(), "", Vector3d::Zero());
    prog.AddConstraint(
        pos_constraint, {mpc_.position_vars(i), pp_.at(step_idx)});
    td_constraints_.push_back(pos_constraint);
  }
}

void IDMPCWalking::MakeALIPTerms() {
  auto& prog = mpc_.get_prog();

  int intervals = std::round((params_.t_ss + params_.t_ds) / params_.mpc_dt);
  int non_alip_footsteps = params_.mpc_N / intervals - 1;
  int num_alips = params_.footstep_horizon - non_alip_footsteps;

  a0_ = prog.NewContinuousVariables(4, "a0");
  prog.SetInitialGuess(a0_, VectorXd::Zero(4));
  for (int i = 0; i < num_alips; ++i) {
    xa_.push_back(prog.NewContinuousVariables(4, "xa_" + std::to_string(i)));
    prog.SetInitialGuess(xa_.back(), VectorXd::Zero(4));
  }
  alip_mapping_constraint_ = std::make_shared<ALIPMappingConstraint>(
      dynamics());

  prog.AddConstraint(
      alip_mapping_constraint_,
      {
        mpc_.position_vars(params_.mpc_N),
        mpc_.velocity_vars(params_.mpc_N),
        a0_
      });

  MatrixXd A_stance = MatrixXd::Identity(4, 8);
  initial_s2s_state_constraint_ = prog.AddLinearEqualityConstraint(
      A_stance, VectorXd::Zero(4), {a0_, xa_.front()}
  ).evaluator().get();

  std::vector<drake::solvers::VectorXDecisionVariable> pp_tmp;
  for (int i = non_alip_footsteps; i < params_.footstep_horizon; ++i) {
    pp_tmp.push_back(pp_.at(i));
  }
  alip_utils::AlipGaitParams alip_params;
  alip_params.height = params_.pelvis_height - 0.1;
  alip_params.double_stance_duration = params_.t_ds;
  alip_params.single_stance_duration = params_.t_ss;
  alip_params.reset_discretization_method =
      alip_utils::ResetDiscretization::kFOH;
  auto ctx = dynamics().get_plant().CreateDefaultContext();
  alip_params.mass = dynamics().get_plant().CalcTotalMass(*ctx);
  alip_utils::AddS2SDynamicsConstraints(
    alip_params, xa_, pp_tmp, &mutable_mpc().get_prog()
  );

  for (int i = 0; i < num_alips; ++i) {
    auto state_cost = std::make_shared<solvers::sqp::SqpQuadraticCost>(
        Eigen::Matrix4d::Identity(), Eigen::Vector4d::Zero(), 0);
    prog.AddCost(state_cost, xa_.at(i));
    alip_state_costs_.push_back(state_cost);
  }
  for (int i = 0; i < num_alips - 1; ++i) {
    auto footstep_cost = std::make_shared<solvers::sqp::SqpQuadraticCost>(
        Eigen::Matrix4d::Identity(), Eigen::Vector4d::Zero(), 0);
    prog.AddCost(
        footstep_cost,
        {pp_tmp.at(i).head<2>(), pp_tmp.at(i+1).head<2>()});
    alip_footstep_costs_.push_back(footstep_cost);
  }

}

void IDMPCWalking::UpdateALIPTerms(const MPCReference &reference) {
  // find the touchdown event closest to the end of the horizon
  // (non-inclusive) and make that foot the stance foot for alip
  double t_final = reference.knot_times_.back();
  double t_prev_impact = 0;
  for (int i = params_.mpc_N - 1; i >= 0; --i) {
    if (not reference.touchdown_ee_names_.at(i).empty()) {
      alip_mapping_constraint_->set_contact_point(
          reference.touchdown_ee_names_.at(i),
          reference.touchdown_ee_points_.at(i)
      );
      t_prev_impact = reference.knot_times_.at(i);
      break;
    }
  }
}

std::vector<VectorXd> IDMPCWalking::get_footstep_solutions(
    const Eigen::VectorXd& z) const {
  std::vector<VectorXd> sol;
  for (const auto& p : pp_) {
    sol.push_back(mpc_.GetDecisionVariableValue(p, z));
  }
  return sol;
}

void IDMPCWalking::MakeGroundConstraints() {
  auto terrain = ConvexPolygonSet({ConvexPolygon::MakeFlatGround()});
  for (size_t i = 0; i < pp_.size(); ++i) {
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
      Vector3d::Zero(), Vector3d::Zero(),
      "swing_foot");
}

void IDMPCWalking::MakeFootLevelingCosts() {
  Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
  Q(2,2) = 10 * params_.foot_pos_W(2,2);
  mpc_.AddTaskCost<RelativePositionCost>(
      "foot_level_left", Q, Vector3d::Zero(),
      mpc().dynamics().get_plant(),
      params_.left_foot_body_name, params_.left_foot_body_name,
      params_.foot_rear, params_.foot_front,
      "foot_level_left");
  mpc_.AddTaskCost<RelativePositionCost>(
      "foot_level_right", Q, Vector3d::Zero(),
      mpc().dynamics().get_plant(),
      params_.right_foot_body_name, params_.right_foot_body_name,
      params_.foot_rear, params_.foot_front,
      "foot_level_left");
}

solvers::NCQPSolver::SetMembershipConstraints
IDMPCWalking::GetFootholdConstraints(const VectorXd &z) {
  if (footholds_.empty()) {
    return {{}, {}};
  }
  for (size_t i = 0; i < footholds_.size(); ++i) {
    footholds_.at(i)->SetShift(
        mpc().GetDecisionVariableValue(pp_.at(i), z));
  }
  return solvers::NCQPSolver::ExtractSetMembershipConstraints(
      mpc_.get_prog(), foothold_bindings_);
}

}