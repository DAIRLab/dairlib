#include <numeric>
#include "id_mpc.h"
#include "common/eigen_utils.h"
#include "solvers/sqp/orientation_error_cost.h"
#include "solvers/sqp/sqp_utils.h"

namespace dairlib::systems::controllers::id_mpc {

using Eigen::MatrixXd;
using Eigen::VectorXd;

using solvers::NonlinearConstraint;
using solvers::QPData;
using solvers::AppendQuadraticCost;
using solvers::AppendLinearConstraint;

using drake::solvers::MathematicalProgramResult;
using drake::solvers::MathematicalProgram;
using drake::solvers::VectorXDecisionVariable;
using drake::solvers::LinearConstraint;

using drake::AutoDiffXd;
using drake::AutoDiffVecXd;
using drake::math::ExtractValue;
using drake::math::ExtractGradient;
using drake::math::InitializeAutoDiff;

using solvers::sqp::QuadraticErrorCost;
using solvers::sqp::OrientationErrorCost;
using solvers::sqp::ParseCostsToSQP;
using solvers::sqp::ParseConstraintsToSQP;

constexpr double kInfinity = std::numeric_limits<double>::infinity();

IDMPC::IDMPC(IDMPCParams params, std::unique_ptr<ConstrainedDynamicsInfo>
    dynamics) : params_(params), dynamics_(std::move(dynamics)) {

  DRAKE_DEMAND(dynamics_ != nullptr);
  DRAKE_DEMAND(params_.N > 0);
  DRAKE_DEMAND(params_.dt > 0);
  DRAKE_DEMAND(params_.Wq.rows() == dynamics_->nq());
  DRAKE_DEMAND(params_.Wq.cols() == dynamics_->nq());
  DRAKE_DEMAND(params_.Wv.rows() == dynamics_->nv());
  DRAKE_DEMAND(params_.Wv.cols() == dynamics_->nv());
  DRAKE_DEMAND(params_.Wu.rows() == dynamics_->nu());
  DRAKE_DEMAND(params_.Wu.cols() == dynamics_->nu());
  DRAKE_DEMAND(params_.Wlambda.rows() == dynamics_->nh() + dynamics_->nc());
  DRAKE_DEMAND(params_.Wlambda.cols() == dynamics_->nh() + dynamics_->nc());

  // MakeKnotPoints has to be called first
  // to create the decision variables and KnotPoints
  // for the remaining objects to use
  MakeKnotPoints();
  MakeMPCCosts();
  MakeForceLimits();
  MakeCollocationConstraints();
  MakeKinematicConstraints();

  initial_state_constraint_ = prog_.AddLinearEqualityConstraint(
      MatrixXd::Identity(dynamics_->nx(), dynamics_->nx()),
      VectorXd::Zero(dynamics_->nx()),
      knot_point_vars_.front().head(dynamics_->nx())).evaluator().get();

  num_constraints_ = 0;
  for (const auto& binding : prog_.GetAllConstraints()) {
    num_constraints_ += binding.evaluator()->num_constraints();
  }
}

void IDMPC::UpdateProblemData(const MPCReference &reference,
                              const VectorXd &initial_state,
                              const Eigen::VectorXd& prev_sol) {
  UpdateInitialState(initial_state);
  timeline_.set_time_vector(reference.knot_times_);
  for (size_t i = 0; i < params_.N + 1; ++i) {
    UpdateActiveContacts(i, reference.active_contacts_.at(i));
  }
  for (size_t i = 0; i < params_.N; ++i) {
    UpdateFrictionCone(i, reference.active_contacts_.at(i));
  }
  UpdateCosts(reference);
  smoothness_cost_->UpdateReference(prev_sol.head(smoothness_cost_->num_vars()));
}

void IDMPC::UpdateFrictionCone(
    int knot_index, const std::vector<std::string> &active_contacts) {
  for (const auto& c : dynamics_->contacts()) {
    if (std::find(active_contacts.begin(), active_contacts.end(), c) !=
        active_contacts.end()) {
      MatrixXd A = MatrixXd(4, 3);
      A << -1, 0, params_.mu,
            0, -1, params_.mu,
            1, 0, params_.mu,
            0, 1, params_.mu;
      contact_force_limits_.at(c).at(knot_index)->UpdateCoefficients(
          A, VectorXd::Zero(4), VectorXd::Constant(4, kInfinity));
    } else {
      contact_force_limits_.at(c).at(knot_index)->UpdateCoefficients(
          MatrixXd::Identity(4, 3), VectorXd::Zero(4), VectorXd::Zero(4));
    }
  }
}

void IDMPC::UpdateInitialState(const Eigen::VectorXd &x) {
  DRAKE_ASSERT(x.size() == dynamics_->nx());
  initial_state_constraint_->UpdateCoefficients(
      MatrixXd::Identity(dynamics_->nx(), dynamics_->nx()), x);
}

void IDMPC::UpdateActiveContacts(
    int knot_index, std::vector<std::string> contacts) {
  timeline_.knot_states.at(knot_index).UpdateActiveContacts(contacts);
}

void IDMPC::UpdateCosts(const MPCReference &reference) {
  reference_manager_.UpdateReference(
      "q", reference.q_traj_, reference.knot_times_);
  reference_manager_.UpdateReference(
      "quat", reference.quat_traj_, reference.knot_times_);
  reference_manager_.UpdateReference(
      "v", reference.v_traj_, reference.knot_times_);
  reference_manager_.UpdateReference(
      "u", reference.u_traj_, reference.knot_times_);
  reference_manager_.UpdateReference(
      "lambda", reference.lambda_traj_, reference.knot_times_);

  for (const auto& [name, traj] : reference.task_space_trajs_) {
    reference_manager_.UpdateReference(name, traj, reference.knot_times_);
  }
}

void IDMPC::SetDecisionVariableValue(
    const VectorXDecisionVariable &var, const VectorXd& value,
    VectorXd *z) const {
  const auto& indices = prog_.FindDecisionVariableIndices(var);
  DRAKE_DEMAND(indices.size() == value.rows());
  for (int i = 0; i < value.rows(); ++i) {
    (*z)(indices[i]) = value(i);
  }
}

VectorXd IDMPC::GetDecisionVariableValue(
    const VectorXDecisionVariable &var, const VectorXd &z) const {
  const auto& indices = prog_.FindDecisionVariableIndices(var);
  VectorXd x = VectorXd::Zero(indices.size());
  for (int i = 0; i < x.rows(); ++i) {
    x(i) = z(indices[i]);
  }
  return x;
}

void IDMPC::MakeForceLimits() {
  for (const auto& c : dynamics_->contacts()) {
    std::vector<LinearConstraint*> evaluators;
    for (int i = 0; i < params_.N ; ++i) {
      evaluators.push_back(
        prog_.AddLinearConstraint(
            MatrixXd::Zero(4, 3),
            VectorXd::Zero(4),
            VectorXd::Zero(4),
            dynamics_->select_contact_force_from_lambda(c, lambda_vars(i))
        ).evaluator().get());
    }
    contact_force_limits_.insert({c, evaluators});
  }
}

// TODO (Brian-Acosta) this assumes floating base plant when constructing the
//  Orientation cost
void IDMPC::MakeMPCCosts() {
  reference_manager_ = ReferenceManager<double>(params_.N, params_.dt);
  reference_manager_.AddRunningStateCost<QuadraticErrorCost<double>>(
      "q", params_.Wq, VectorXd::Zero(dynamics_->nq()));
  reference_manager_.AddRunningStateCost<OrientationErrorCost<double>>(
      "quat", params_.Wrot, Eigen::Vector4d::UnitX());
  reference_manager_.AddRunningStateCost<QuadraticErrorCost<double>>(
      "v", params_.Wv, VectorXd::Zero(dynamics_->nv()));
  reference_manager_.AddRunningInputCost(
      "u", params_.Wu, VectorXd::Zero(dynamics_->nu()));
  reference_manager_.AddRunningInputCost(
      "lambda", params_.Wlambda, VectorXd::Zero(dynamics_->nlambda()));

  reference_manager_.AddTerminalStateCost<QuadraticErrorCost<double>>(
      "q", params_.Wq_final, VectorXd::Zero(dynamics_->nq()));
  reference_manager_.AddTerminalStateCost<OrientationErrorCost<double>>(
      "quat", params_.Wrot_final, Eigen::Vector4d::UnitX());
  reference_manager_.AddTerminalStateCost<QuadraticErrorCost<double>>(
      "v", params_.Wv_final, VectorXd::Zero(dynamics_->nv()));

  for (int i = 0; i < params_.N + 1; ++i) {
    prog_.AddCost(
        reference_manager_.GetEvaluator("q", i),
        position_vars(i));
    prog_.AddCost(
        reference_manager_.GetEvaluator("quat", i),
        position_vars(i).head<4>());
    prog_.AddCost(
        reference_manager_.GetEvaluator("v", i),
        velocity_vars(i));
  }
  for (int i = 0; i < params_.N; ++i) {
    prog_.AddCost(
        reference_manager_.GetEvaluator("lambda", i),
        lambda_vars(i));
  }
  for (int i = 0; i < params_.num_full_torque_knots; ++i) {
    prog_.AddCost(
        reference_manager_.GetEvaluator("u", i),
        input_vars(i));
  }
  prog_.AddCost(
      reference_manager_.GetTerminalEvaluator("q"),
      position_vars(params_.N));
  prog_.AddCost(
      reference_manager_.GetTerminalEvaluator("quat"),
      position_vars(params_.N).head<4>());
  prog_.AddCost(
      reference_manager_.GetTerminalEvaluator("v"),
      velocity_vars(params_.N));

  smoothness_cost_ = std::make_shared<QuadraticErrorCost<double>>(
      1e-8 * MatrixXd::Identity(prog_.num_vars(), prog_.num_vars()),
      VectorXd::Zero(prog_.num_vars())
  );

  prog_.AddCost(smoothness_cost_, prog_.decision_variables());
}

void IDMPC::MakeKnotPoints() {
  std::vector<double> breaks;
  for (int i = 0; i < params_.N + 1; ++i) {
    // Base config with no constraint handling
    auto cfg = knot_config{
      i,
      i == params_.N,
      i < params_.num_full_torque_knots,
      {}, {}
    };
    if (i > 0) {
      cfg.active_constraint_dot_indices.resize(
          dynamics_->nh() + dynamics_->nc_active(), 0);
      std::iota(cfg.active_constraint_dot_indices.begin(),
                cfg.active_constraint_dot_indices.end(), 0);
    }

    auto knot = std::make_unique<KnotPoint>(*dynamics_, cfg);
    knot_point_vars_.push_back(
        prog_.NewContinuousVariables(knot->total_variables()));
    timeline_.knot_states.push_back(KnotPointState(*dynamics_));
    timeline_.knots.push_back(std::move(knot));
    breaks.push_back(params_.dt * i);
  }
  timeline_.set_time_vector(breaks);
}

void IDMPC::MakeCollocationConstraints() {
  for (int i = 0; i < params_.N; ++i) {
    auto collocation_constraint =
        std::make_shared<CollocationConstraint<double>>(
            *timeline_.knots.at(i),
            *timeline_.knots.at(i+1),
            &timeline_.knot_states.at(i),
            &timeline_.knot_states.at(i+1)
        );
    prog_.AddConstraint(
      collocation_constraint,
      {knot_point_vars_.at(i), knot_point_vars_.at(i+1).head(dynamics_->nx())});
  }
}

void IDMPC::MakeKinematicConstraints() {
  for (int i = 0; i < params_.N; ++i) {
    auto kinematic_constraint = std::make_shared<KinematicConstraint<double>>(
        *timeline_.knots.at(i+1), &timeline_.knot_states.at(i+1));
    prog_.AddConstraint(
      kinematic_constraint,
      timeline_.knots.at(i+1)->get_x(knot_point_vars_.at(i+1)));
  }
}

void IDMPC::ConstructSQPProgram(const VectorXd &x, QPData* qp) const {
  DRAKE_DEMAND(qp != nullptr);
  DRAKE_DEMAND(x.rows() == prog_.num_vars());

  // Reset problem size data
  qp->num_vars = prog_.num_vars();
  ParseCostsToSQP(x, prog_, qp);
  ParseConstraintsToSQP(x, prog_, qp);
}

void IDMPC::ProjectToQuaternionConstraint(Eigen::VectorXd *x) const {
  DRAKE_DEMAND(x != nullptr);
  for (int i = 0; i < params_.N + 1; ++i) {
    auto w = position_vars(i)(0);
    int start_idx = prog_.FindDecisionVariableIndex(w);
    x->segment<4>(start_idx) = x->segment<4>(start_idx).normalized();
  }
}

double IDMPC::EvaluateCost(const Eigen::VectorXd &x) const {
  double cost = 0;
  for (const auto& binding: prog_.GetAllCosts()) {
    VectorXd y = VectorXd::Zero(1);
    const auto& v = binding.variables();
    const auto& indices = prog_.FindDecisionVariableIndices(v);

    VectorXd xval = VectorXd::Zero(v.rows());
    for (int i = 0; i < v.rows(); ++i) {
      xval(i) = x(indices[i]);
    }
    binding.evaluator()->Eval(xval, &y);
    cost += y(0);
  }
  return cost;
}

double IDMPC::EvaluateConstraintViolation(const Eigen::VectorXd &x) const {
  VectorXd lb_viol = VectorXd::Zero(num_constraints());
  VectorXd ub_viol = VectorXd::Zero(num_constraints());
  int start = 0;
  for (const auto& binding : prog_.GetAllConstraints()) {
    const auto &v = binding.variables();
    const auto &indices = prog_.FindDecisionVariableIndices(v);
    VectorXd xval = VectorXd::Zero(v.rows());
    for (int i = 0; i < v.rows(); ++i) {
      xval(i) = x(indices[i]);
    }
    VectorXd y;
    binding.evaluator()->Eval(xval, &y);
    const int n = binding.evaluator()->num_constraints();
    const VectorXd& lb = binding.evaluator()->lower_bound();
    const VectorXd& ub = binding.evaluator()->upper_bound();
    lb_viol.segment(start, n) = (lb - y).cwiseMax(0);
    ub_viol.segment(start, n) = (y - ub).cwiseMax(0);
  }
  return params_.dt * lb_viol.cwiseMax(ub_viol).norm();
}

LcmTrajectory IDMPC::GetSolutionAsLcmTrajectory(const MathematicalProgramResult &result) const {
  LcmTrajectory::Trajectory q("q", dynamics_->nq(), params_.N + 1);
  LcmTrajectory::Trajectory v("v", dynamics_->nv(), params_.N + 1);
  LcmTrajectory::Trajectory u("u", dynamics_->nu(), params_.num_full_torque_knots);
  LcmTrajectory::Trajectory lambda("lambda",dynamics_->nc() + dynamics_->nh(),
                                   params_.N);

  for (int i = 0; i < params_.N + 1; ++i) {
    q.time_vector(i) = timeline_.breaks().at(i);
    v.time_vector(i) = timeline_.breaks().at(i);
    q.datapoints.col(i) = result.GetSolution(position_vars(i));
    v.datapoints.col(i) = result.GetSolution(velocity_vars(i));
  }

  for (int i = 0; i < params_.N; ++i) {
    lambda.time_vector(i) = timeline_.breaks().at(i);
    lambda.datapoints.col(i) = result.GetSolution(lambda_vars(i));
  }

  for (int i = 0; i < params_.num_full_torque_knots; ++i) {
    u.time_vector(i) = timeline_.breaks().at(i);
    u.datapoints.col(i) = result.GetSolution(input_vars(i));
  }

  return LcmTrajectory(
      {q, v, u, lambda}, {"q", "v", "u", "lambda"}, "", "", false);
}
}