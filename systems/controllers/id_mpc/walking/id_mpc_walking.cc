
#include "id_mpc_walking.h"
#include "solvers/sqp/relative_position_cost.h"

namespace dairlib::systems::controllers::id_mpc {

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;

using alip_utils::Stance;

using geometry::ConvexPolygon;
using geometry::ConvexPolygonSet;
using solvers::sqp::TerrainSDFCost;
using solvers::sqp::RelativePositionCost;
using solvers::ConvexPolygonSetConstraint;

static constexpr double kInfinity = std::numeric_limits<double>::infinity();


IDMPCWalking::IDMPCWalking(
    IDMPCParams params, std::unique_ptr<ConstrainedDynamicsInfo> dynamics,
    GaitParams gait_params) :
    mpc_(params, std::move(dynamics)), params_(gait_params) {

  MakeFlatTerrain();
  MakeFootsteps();
  MakeSwingTrajCosts();
  MakeSwingTrajCostsSDF();
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
  for (int i = 0; i < pp_.size(); ++i) {
    footstep_hyst_costs_.at(i)->UpdateReference(
        mpc_.GetDecisionVariableValue(pp_.at(i), prev_sol));
  }
  // shuffle the footstep hysteresis costs on touchdown
  if (not reference.touchdown_ee_names_to_update_.empty()) {
    for (int i = 0; i < pp_.size() - 1; ++i) {
      footstep_hyst_costs_.at(i)->UpdateReference(
          mpc_.GetDecisionVariableValue(pp_.at(i+1), prev_sol));
    }
  }
  UpdateSwingTrajCostsSDF(reference);
  //  UpdateTerrainCBF(reference, initial_state);
}

void IDMPCWalking::UpdateFootstepConstraints(
    const std::vector<std::string> &foot_names,
    const std::vector<Vector3d> &contact_points) {
  for (int i = 2; i < params_.mpc_N + 1;  ++i) {
    td_constraints_.at(i - 2)->set_point(
        foot_names.at(i), contact_points.at(i));
  }
}

void IDMPCWalking::MakeFlatTerrain() {
  double zlen = 0.1;
  drake::math::RigidTransformd X(Vector3d(0.0, 0.0, -0.5 * zlen));

  std::vector<std::pair<drake::math::RigidTransformd, Vector3d>> boxes =
      {{X, Vector3d(100.0, 100.0, zlen)}};
  boxy_terrain_ = std::make_unique<multibody::BoxSet>(boxes);
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
    Eigen::Matrix3d W_hyst = 0.25 * Eigen::Matrix3d::Identity();
    W_hyst(2,2) = 0.01;
    auto step_hyst_cost =
        std::make_shared<solvers::sqp::QuadraticErrorCost<double>>(
            W_hyst, Vector3d::Zero());
    prog.AddCost(step_hyst_cost, pp_.at(i));
    footstep_hyst_costs_.push_back(step_hyst_cost);
  }

  // TODO (@Brian-Acosta) This crossover constraint is implemented in the world
  //  frame as written. Ideally we would have only a collision constraint for
  //  the whole-body knot points, and only a local frame crossover constraint
  //  for the ALIP knot points
  for (size_t i = 0; i < pp_.size() - 1; ++i) {
    no_crossover_c_.push_back(
        prog.AddLinearConstraint(
            MatrixXd::Ones(1, 2),
            VectorXd::Constant(1, -kInfinity),
            VectorXd::Constant(1, kInfinity),
            {pp_.at(i).segment(1,1), pp_.at(i+1).segment(1,1)}
        ));
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
  alip_mass_ = alip_params.mass;
  alip_height_ = alip_params.height;
  alip_utils::AddS2SDynamicsConstraints(
    alip_params, xa_, pp_tmp, &mutable_mpc().get_prog()
  );

  // Setup ALIP costs
  Qa_ = 4.0 * Matrix4d::Identity();
  Qaf_ = 10.0 * Matrix4d::Identity();
  Ra_ = 15.0 * Eigen::Matrix2d::Identity();

  Matrix4d PI0;
  Matrix4d PI1;
  Eigen::Matrix<double, 4, 2> g0;
  Eigen::Matrix<double, 4, 2> g1;
  alip_utils::MakeProjectionToP2Orbit(
      alip_params, PI0, PI1, g0, g1);
  PIs_ = {PI0, PI1};
  gs_ = {g0, g1};
  MakeALIPCosts(num_alips, pp_tmp);

}

void IDMPCWalking::MakeALIPCosts(
    int num_alips,
    const std::vector<drake::solvers::VectorXDecisionVariable>& pp_tmp) {
  auto& prog = mpc_.get_prog();
  for (int i = 0; i < num_alips; ++i) {
    auto state_cost = std::make_shared<solvers::sqp::SqpQuadraticCost>(
        Matrix4d::Identity(), Vector4d::Zero(), 0);
    prog.AddCost(state_cost, xa_.at(i));
    alip_state_costs_.push_back(state_cost);
  }
  for (int i = 0; i < num_alips - 1; ++i) {
    auto footstep_cost = std::make_shared<solvers::sqp::SqpQuadraticCost>(
        Matrix4d::Identity(), Vector4d::Zero(), 0);
    prog.AddCost(
        footstep_cost,
        {pp_tmp.at(i).head<2>(), pp_tmp.at(i+1).head<2>()});
    alip_footstep_costs_.push_back(footstep_cost);
  }

}

namespace {
double horizon_blend(int i, int N) {
  double x = static_cast<double>(i) - static_cast<double>(N) / 2.0;
  return 0.5 * (1 + std::tanh(-0.25 * x));
}

double swing_profile(double phase) {
  double sin_pi_phase = std::sin(M_PI * phase);
  return 1.18 * sin_pi_phase / sqrt(sin_pi_phase * sin_pi_phase + 0.4);
}
}

void IDMPCWalking::MakeSwingTrajCostsSDF() {
  DRAKE_DEMAND(boxy_terrain_ != nullptr);
  auto& prog = mutable_mpc().get_prog();
  for (int i = 0; i <= params_.mpc_N; ++i) {
    double s = (i == 0 || i == params_.mpc_N) ? 0.5 : 1;
    double w = s * horizon_blend(i, params_.mpc_N);
    auto sdf_cost = std::make_shared<TerrainSDFCost>(
        w * params_.mpc_dt * params_.foot_pos_W.bottomRightCorner<1,1>(),
        VectorXd::Zero(1), dynamics().get_plant(), "toe_left",
        Vector3d::Zero(), boxy_terrain_.get()
    );
    terrain_sdf_costs_.push_back(sdf_cost);
    prog.AddCost(terrain_sdf_costs_.back(), mpc_.position_vars(i));
  }
}

void IDMPCWalking::UpdateSwingTrajCostsSDF(const MPCReference &mpc_reference) {
  for (int i = 0; i <= params_.mpc_N; ++i) {
    double clearance = params_.step_height *
        swing_profile(mpc_reference.single_stance_phase_.at(i));
    const std::string foot =
        mpc_reference.active_contacts_.at(i).front() == "toe_left_front" ?
        "toe_right" : "toe_left";
    terrain_sdf_costs_.at(i)->UpdateReference(VectorXd::Constant(1, clearance));
    terrain_sdf_costs_.at(i)->set_frame(foot);
    terrain_sdf_costs_.at(i)->set_front_point(params_.foot_front);
    terrain_sdf_costs_.at(i)->set_rear_point(params_.foot_rear);
  }
}

void IDMPCWalking::UpdateTerrainCBF(
    const MPCReference &reference, const VectorXd &initial_state) {

  double a =  std::sin(M_PI * reference.single_stance_phase_.front());
  double dt = reference.knot_times_[1] - reference.knot_times_[0];

  int nv = dynamics().nv();
  MatrixXd Adyn = MatrixXd::Zero(nv, 3 * nv);

  Adyn.leftCols(nv) = dt * MatrixXd::Identity(nv, nv);
  Adyn.middleCols(nv, nv) = -MatrixXd::Identity(nv, nv);
  Adyn.rightCols(nv) = MatrixXd::Identity(nv, nv);


  if (a > 0.5) {
    initial_dv_constraint_->UpdateCoefficients(Adyn, VectorXd::Zero(nv));
    const std::string foot =
        reference.active_contacts_.front().front() == "toe_left_front" ?
        "toe_right" : "toe_left";
    const Vector3d& point = params_.foot_front;
    auto [A, b] = terrain_cbf_->GetConstraintMatrices(
        foot, point, initial_state.head(dynamics().nq()),
        initial_state.tail(dynamics().nv()), 10.0, 10.0, dt);

    std::cout << "A: " << A << std::endl;
    std::cout << "b: " << b << std::endl;
    terrain_cbf_constraint_->UpdateCoefficients(
        A, b, VectorXd::Constant(1, kInfinity));

  } else {
    initial_dv_constraint_->UpdateCoefficients(
        MatrixXd::Zero(nv, 3 * nv), VectorXd::Zero(nv));
    terrain_cbf_constraint_->UpdateCoefficients(
        MatrixXd::Zero(1, dynamics().nv()),
        VectorXd::Zero(1),
        VectorXd::Zero(1));
  }
}


void IDMPCWalking::UpdateALIPTerms(const MPCReference &reference) {
  // find the touchdown event closest to the end of the horizon
  // (non-inclusive) and make that foot the stance foot for alip
  double t_final = reference.knot_times_.back();
  double t_prev_impact = 0;
  Stance stance;
  for (int i = params_.mpc_N - 1; i >= 0; --i) {
    if (not reference.touchdown_ee_names_.at(i).empty()) {
      alip_mapping_constraint_->set_contact_point(
          reference.touchdown_ee_names_.at(i),
          reference.touchdown_ee_points_.at(i)
      );
      t_prev_impact = reference.knot_times_.at(i);
      if (reference.touchdown_ee_names_.at(i) == "toe_left") {
        stance = Stance::kLeft;
      } else {
        stance = Stance::kRight;
      }
      break;
    }
  }
  double t_remain = (params_.t_ss + params_.t_ds) - (t_final - t_prev_impact);
  if (not reference.touchdown_ee_names_.back().empty()) {
    t_remain = 0;
  }
  MatrixXd A_dyn = MatrixXd::Zero(4, 8);
  MatrixXd Ad = alip_utils::CalcAd(alip_height_, alip_mass_, t_remain);
  A_dyn.leftCols(4) = Ad;
  A_dyn.rightCols(4) = -MatrixXd::Identity(4, 4);
  initial_s2s_state_constraint_->UpdateCoefficients(A_dyn, VectorXd::Zero(4));
  UpdateALIPCosts(reference.vdes_, stance);

  // get the current stance (not the initial ALIP stance) and use it to update
  // the crossover constraint
  int i = 0;
  while (reference.touchdown_ee_names_.at(i).empty()) {
    ++i;
  }
  Stance curr_stance = reference.touchdown_ee_names_.at(i) == "toe_left" ?
      Stance::kRight : Stance::kLeft;
  UpdateCrossoverConstraint(curr_stance);
}

std::vector<VectorXd> IDMPCWalking::get_footstep_solutions(
    const Eigen::VectorXd& z) const {
  std::vector<VectorXd> sol;
  for (const auto& p : pp_) {
    sol.push_back(mpc_.GetDecisionVariableValue(p, z));
  }
  return sol;
}

void IDMPCWalking::UpdateALIPCosts(const Eigen::Vector2d& vdes,
                                   const Stance& stance) {
  // state costs
  int start_period = stance == Stance::kLeft ? 0 : 1;
  for (size_t i = 0; i < xa_.size() - 1; ++i) {
    const Matrix4d& PI = PIs_.at((start_period + i) % 2);
    const Eigen::Matrix<double, 4, 2>& g = gs_.at((start_period + i) % 2);
    Matrix4d Q = PI.transpose() * Qa_ * PI;
    Vector4d q = -2 * Q * g * vdes;
    Q += 1e-5 * Matrix4d::Identity();
    alip_state_costs_.at(i)->UpdateCoefficients(2.0 * Q, q, 0);
  }

  int final_period = stance == Stance::kLeft ? xa_.size() - 1 : xa_.size();
  const Matrix4d& PI = PIs_.at(final_period % 2);
  const Eigen::Matrix<double, 4, 2>& g = gs_.at((final_period) % 2);
  Matrix4d Q = PI.transpose() * Qaf_ * PI;
  Vector4d q = -2 * Q * g * vdes;
  Q += 1e-5 * Matrix4d::Identity();
  alip_state_costs_.back()->UpdateCoefficients(Q, q, 0);

  // footstep costs
  alip_utils::AlipGaitParams gait_params;
  gait_params.height = alip_height_;
  gait_params.double_stance_duration = params_.t_ds;
  gait_params.single_stance_duration = params_.t_ss;
  gait_params.reset_discretization_method =
      alip_utils::ResetDiscretization::kFOH;
  gait_params.desired_velocity = vdes;
  gait_params.initial_stance_foot = stance;
  gait_params.stance_width = params_.stance_width;
  gait_params.mass = alip_mass_;
  const auto ud = alip_utils::MakeP2Orbit(gait_params);

  Eigen::Matrix<double, 2, 4> r;
  r.leftCols<2>() = -Eigen::Matrix2d::Identity();
  r.rightCols<2>() = Eigen::Matrix2d::Identity();

  Matrix4d Qr = 2 * r.transpose() * Ra_ * r;
  for (size_t i = 0; i < alip_footstep_costs_.size(); ++i) {
    Vector4d b = - 2 * r.transpose() * Ra_ * ud[i % 2];
    alip_footstep_costs_.at(i)->UpdateCoefficients(Qr, b, 0);
  }
}


void IDMPCWalking::UpdateCrossoverConstraint(Stance stance) {
  double s = (stance == Stance::kLeft) ? -1.0 : 1.0;
  for (auto& c : no_crossover_c_) {
    c.evaluator()->UpdateCoefficients(
        Eigen::RowVector2d(-s, s),
        VectorXd::Constant(1, -kInfinity),
        VectorXd::Constant(1, -0.04)
    );
    s *= -1.0;
  }
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
  Eigen::MatrixXd W = params_.foot_pos_W;
  W(2,2) = 0.1;
  mpc_.AddTaskCost<RelativePositionCost>(
      "swing_foot", W, Vector3d::Zero(), mpc().dynamics().get_plant(),
      params_.right_foot_body_name, params_.left_foot_body_name,
      Vector3d::Zero(), Vector3d::Zero(),
      "swing_foot");
}

void IDMPCWalking::MakeFootLevelingCosts() {
  Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
  Q(2,2) = params_.foot_pos_W(2,2);
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