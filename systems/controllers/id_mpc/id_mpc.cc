#include <numeric>
#include "id_mpc.h"
#include "common/eigen_utils.h"

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

using drake::AutoDiffVecXd;
using drake::math::ExtractValue;
using drake::math::ExtractGradient;
using drake::math::InitializeAutoDiff;

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
  MakeUnitQuaternionConstraints();

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
                              const VectorXd &initial_state) {
  UpdateInitialState(initial_state);
  timeline_.set_time_vector(reference.knot_times_);
  for (size_t i = 0; i < params_.N + 1; ++i) {
    UpdateActiveContacts(i, reference.active_contacts_.at(i));
  }
  for (size_t i = 0; i < params_.N; ++i) {
    UpdateFrictionCone(i, reference.active_contacts_.at(i));
  }
  UpdateCosts(reference);
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

// TODO (@Brian-Acosta) correctly implement orientation cost
void IDMPC::UpdateCosts(const MPCReference &reference) {
  reference_manager_.UpdateReference(
      "q", reference.q_traj_, reference.knot_times_);
  reference_manager_.UpdateReference(
      "v", reference.v_traj_, reference.knot_times_);
  reference_manager_.UpdateReference(
      "u", reference.u_traj_, reference.knot_times_);
  reference_manager_.UpdateReference(
      "lambda", reference.lambda_traj_, reference.knot_times_);
}

void IDMPC::MakeUnitQuaternionConstraints() {
  unit_quat_ = std::make_shared<QuaternionNormConstraint<AutoDiffXd>>();

  for (auto index: dynamics_->get_plant().GetFloatingBaseBodies()) {
    const auto& body = dynamics_->get_plant().get_body(index);
    DRAKE_DEMAND(body.has_quaternion_dofs());
    for (int i = 0; i <= params_.N; ++i) {
      nonlin_constraints_.push_back(
      prog_.AddConstraint(
          unit_quat_,
          this->position_vars(i).segment(body.floating_positions_start(), 4)));
    }
  }
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

void IDMPC::MakeMPCCosts() {
  reference_manager_ = ReferenceManager<double>(params_.N, params_.dt);
  reference_manager_.AddRunningStateCost<QuadraticErrorCost<double>>(
      "q", params_.Wq, VectorXd::Zero(dynamics_->nq()));
  reference_manager_.AddRunningStateCost<QuadraticErrorCost<double>>(
      "v", params_.Wv, VectorXd::Zero(dynamics_->nv()));
  reference_manager_.AddRunningStateCost<QuadraticErrorCost<double>>(
      "u", params_.Wu, VectorXd::Zero(dynamics_->nu()));
  reference_manager_.AddRunningStateCost<QuadraticErrorCost<double>>(
      "lambda", params_.Wlambda, VectorXd::Zero(dynamics_->nlambda()));

  for (int i = 0; i < params_.N + 1; ++i) {
    prog_.AddCost(
        reference_manager_.GetEvaluator("q", i),
        position_vars(i));
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
//      cfg.active_constraint_indices.resize(dynamics_->nh(), 0);
//      std::iota(cfg.active_constraint_indices.begin(),
//                cfg.active_constraint_indices.end(), 0);
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
    nonlin_constraints_.push_back(
        prog_.AddConstraint(
            collocation_constraint,
            {knot_point_vars_.at(i),
             knot_point_vars_.at(i+1).head(dynamics_->nx())}));
  }
}

void IDMPC::MakeKinematicConstraints() {
  for (int i = 0; i < params_.N; ++i) {
    auto kinematic_constraint = std::make_shared<KinematicConstraint<double>>(
        *timeline_.knots.at(i+1), &timeline_.knot_states.at(i+1));
    nonlin_constraints_.push_back(
        prog_.AddConstraint(
            kinematic_constraint,
            timeline_.knots.at(i+1)->get_x(knot_point_vars_.at(i+1))));
  }
}

void IDMPC::ConstructSQPProgram(const VectorXd &x, QPData& qp) const {
  DRAKE_DEMAND(x.rows() == prog_.num_vars());

  // Reset problem size data
  qp.num_vars = prog_.num_vars();

  ParseCostsToSQP(x, qp);
  ParseConstraintsToSQP(x, qp);
}


// TODO (@Brian-Acosta) add support for L2Norm costs
void IDMPC::ParseCostsToSQP(const VectorXd& x, QPData &qp) const {

  qp.g = VectorXd::Zero(prog_.num_vars());

  std::vector<Eigen::Triplet<double>> cost_triplets;
  cost_triplets.reserve(prog_.num_vars() * params_.N);

  for (const auto& binding : prog_.GetAllCosts()) {
    const auto& v = binding.variables();
    const auto& indices = prog_.FindDecisionVariableIndices(v);

    VectorXd xval = VectorXd::Zero(v.rows());
    for (int i = 0; i < v.rows(); ++i) {
      xval(i) = x(indices[i]);
    }
    GaussNewtonApproximation cost_data;
    if (dynamic_cast<NonlinearLeastSquaresCost<AutoDiffXd>*>(binding.evaluator().get())) {
      cost_data = dynamic_cast<NonlinearLeastSquaresCost<AutoDiffXd>*>(
          binding.evaluator().get()
      )->CalcGaussNewtonApproximation(xval);
    } else if (dynamic_cast<NonlinearLeastSquaresCost<double>*>(binding.evaluator().get())) {
      cost_data = dynamic_cast<NonlinearLeastSquaresCost<double>*>(
          binding.evaluator().get()
      )->CalcGaussNewtonApproximation(xval);
    } else {
      throw std::runtime_error("IDMPC only supports NonlinearLeastSquares costs "
                               "To ensure that the SQP gauss newton "
                               "approximation is properly implemented");
    }
    AppendQuadraticCost(indices, cost_data.H, cost_data.g, cost_data.c,
                        cost_triplets, qp.g, &qp.c, cost_data.diagonal_hessian);
  }
  qp.H.resize(prog_.num_vars(), prog_.num_vars());
  qp.H.setFromTriplets(cost_triplets.begin(), cost_triplets.end());
}


// TODO (@Brian-Acosta)
//  Can maybe make this more efficient by handling constraints differently
//   Depending on their type. Also likely want to denote true equality vs
//   inequality constraints
void IDMPC::ParseConstraintsToSQP(const VectorXd& x, QPData &qp) const {

  qp.num_eq = 0;
  qp.num_ineq = 0;

  std::vector<Eigen::Triplet<double>> inequality_triplets;

  for (const auto& binding : prog_.GetAllConstraints()) {
    const auto& v = binding.variables();
    const auto& indices = prog_.FindDecisionVariableIndices(v);
    VectorXd xval = VectorXd::Zero(v.rows());

    for (int i = 0; i < v.rows(); ++i) {
      xval(i) = x(indices[i]);
    }

    AutoDiffVecXd xval_ad = InitializeAutoDiff(xval);
    AutoDiffVecXd y_ad;
    binding.evaluator()->Eval(xval_ad, &y_ad);
    MatrixXd dydx = ExtractGradient(y_ad);
    VectorXd yval = ExtractValue(y_ad);
    // SQP constraint: lb <= y(x) <= ub -->  lb <= dydx * dx + y* <= ub
    // subtract y* from the above equation
    VectorXd lb = binding.evaluator()->lower_bound() - yval;
    VectorXd ub = binding.evaluator()->upper_bound() - yval;
    AppendLinearConstraint(
        indices, qp.num_ineq, dydx, lb, ub, inequality_triplets,
        qp.lb, qp.ub
    );
    qp.num_ineq += binding.evaluator()->num_constraints();
  }
  qp.A.resize(qp.num_ineq, x.rows());
  qp.A.setFromTriplets(inequality_triplets.begin(), inequality_triplets.end());
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