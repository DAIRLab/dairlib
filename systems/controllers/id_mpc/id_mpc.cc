#include <numeric>
#include "id_mpc.h"
#include "common/eigen_utils.h"

namespace dairlib::systems::controllers::id_mpc {

using Eigen::MatrixXd;
using Eigen::VectorXd;

using solvers::NonlinearConstraint;

using drake::solvers::MathematicalProgramResult;
using drake::solvers::MathematicalProgram;
using drake::solvers::VectorXDecisionVariable;
using drake::AutoDiffVecXd;
using drake::math::ExtractValue;
using drake::math::ExtractGradient;
using drake::math::InitializeAutoDiff;


IDMPC::IDMPC(IDMPCParams params, std::unique_ptr<ConstrainedDynamicsInfo>
    dynamics) : params_(params), dynamics_(std::move(dynamics)) {

  DRAKE_DEMAND(dynamics_ != nullptr);
  DRAKE_DEMAND(params_.N > 0);
  DRAKE_DEMAND(params_.dt > 0);

  MakeKnotPoints();
  MakeCollocationConstraints();
  MakeKinematicConstraints();
  AddUnitQuaternionConstraintToAllFloatingBodies();

  initial_state_constraint_ = prog_.AddLinearEqualityConstraint(
      MatrixXd::Identity(dynamics_->nx(), dynamics_->nx()),
      VectorXd::Zero(dynamics_->nx()),
      knot_point_vars_.front().head(dynamics_->nx())).evaluator().get();
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

void IDMPC::AddUnitQuaternionConstraintToAllFloatingBodies() {
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

void IDMPC::MakeKnotPoints() {
  std::vector<double> breaks;
  for (int i = 0; i < params_.N + 1; ++i) {
    // Base config with no constraint handling
    auto cfg = knot_config{
      i,
      i == params_.N,
      i >= params_.num_full_torque_knots,
      {}, {}
    };
    if (i > 0) {
      cfg.active_constraint_indices.resize(dynamics_->nh(), 0);
      std::iota(cfg.active_constraint_indices.begin(),
                cfg.active_constraint_indices.end(), 0);
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
        prog_.AddConstraint(kinematic_constraint, knot_point_vars_.at(i+1)));
  }
}

void IDMPC::ConstructSQPProgram(const VectorXd &x, QPData& qp) const {
  DRAKE_DEMAND(x.rows() == prog_.num_vars());

  // Reset problem size data
  qp.num_vars = prog_.num_vars();

  ParseCostsToQP(x, qp);
  ParseConstraintsToQP(x, qp);
}

void IDMPC::ParseCostsToQP(const VectorXd& x, QPData &qp) const {

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
    } else if (dynamic_cast<drake::solvers::QuadraticCost*>(binding.evaluator().get())) {
      const auto evaluator = dynamic_cast<drake::solvers::QuadraticCost*>(
          binding.evaluator().get());
      cost_data.H = evaluator->Q();
      cost_data.g = evaluator->b();
      cost_data.c = evaluator->c();
    } else {
      throw std::runtime_error("Unsupported cost type has been added to IDMPC");
    }
    for (int j = 0; j < cost_data.H.cols(); ++j) {
      for (int i = 0; i < cost_data.H.rows(); ++i) {
        int r = indices[i];
        int c = indices[j];
        cost_triplets.emplace_back(r, c, cost_data.H(i,j));
      }
    }
    for (int i = 0; i < cost_data.g.rows(); ++i) {
      qp.g(indices[i]) += cost_data.g(i);
    }
    qp.c += cost_data.c;
  }
  qp.H.resize(prog_.num_vars(), prog_.num_vars());
  qp.H.setFromTriplets(cost_triplets.begin(), cost_triplets.end());
}


// TODO (@Brian-Acosta)
//  Can maybe make this more efficient by handling constraints differently
//   Depending on their type. Also likely want to denote true equality vs
//   inequality constraints
void IDMPC::ParseConstraintsToQP(const VectorXd& x, QPData &qp) const {

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
    int rows = binding.evaluator()->num_constraints();
    qp.lb.conservativeResize(std::max(qp.lb.rows(), qp.num_ineq + rows));
    qp.ub.conservativeResize(std::max(qp.ub.rows(), qp.num_ineq + rows));

    qp.lb.segment(qp.num_ineq, rows) = binding.evaluator()->lower_bound() - yval;
    qp.ub.segment(qp.num_ineq, rows) = binding.evaluator()->upper_bound() - yval;
    for (int j = 0; j < dydx.cols(); ++j) {
      for (int i = 0; i < dydx.rows(); ++i) {
        int row = qp.num_ineq + i;
        int col = indices[j];
        inequality_triplets.emplace_back(row, col, dydx(i, j));
      }
    }
    qp.num_ineq += binding.evaluator()->num_constraints();
  }
  qp.A.resize(qp.num_ineq, x.rows());
  qp.A.setFromTriplets(inequality_triplets.begin(), inequality_triplets.end());
}

LcmTrajectory IDMPC::GetSolutionAsLcmTrajectory(const MathematicalProgramResult &result) const {

  LcmTrajectory::Trajectory q("q", dynamics_->nq(), params_.N + 1);
  LcmTrajectory::Trajectory v("v", dynamics_->nv(), params_.N + 1);
  LcmTrajectory::Trajectory u("u", dynamics_->nu(), params_.N);
  LcmTrajectory::Trajectory lambda("lambda",dynamics_->n_constraint_total(),
                                   params_.N);

  for (int i = 0; i < params_.N + 1; ++i) {
    q.time_vector(i) = timeline_.breaks().at(i);
    v.time_vector(i) = timeline_.breaks().at(i);
    q.datapoints.col(i) = result.GetSolution(position_vars(i));
    v.datapoints.col(i) = result.GetSolution(velocity_vars(i));
  }

  for (int i = 0; i < params_.N; ++i) {
    u.time_vector(i) = timeline_.breaks().at(i);
    lambda.time_vector(i) = timeline_.breaks().at(i);
    u.datapoints.col(i) = result.GetSolution(input_vars(i));
    lambda.datapoints.col(i) = stack<double>(
        {result.GetSolution(lambda_h_vars(i)),
         result.GetSolution(lambda_c_vars(i))}
    );
  }

  return LcmTrajectory(
      {q, v, u, lambda}, {"q", "v", "u", "lambda"}, "", "", false);
}
}