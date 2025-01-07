#include "id_mpc.h"
#include "common/eigen_utils.h"

namespace dairlib::systems::controllers::id_mpc {

using Eigen::MatrixXd;
using Eigen::VectorXd;

using solvers::NonlinearConstraint;

using drake::solvers::MathematicalProgramResult;
using drake::solvers::MathematicalProgram;
using drake::solvers::VectorXDecisionVariable;


IDMPC::IDMPC(IDMPCParams params, std::unique_ptr<ConstrainedDynamicsInfo>
    dynamics) : params_(params), dynamics_(std::move(dynamics)) {

  DRAKE_DEMAND(dynamics_ != nullptr);
  DRAKE_DEMAND(params_.N > 0);
  DRAKE_DEMAND(params_.dt > 0);

  for (int i = 0; i < params_.N + 1; ++i) {
    timeline_.breaks.push_back(i * params_.dt);
    timeline_.knots.push_back(KnotPointState(*dynamics_));
    knot_point_vars_.push_back(
        prog_.NewContinuousVariables(dynamics_->variable_count()));
  }

  AddUnitQuaternionConstraintToAllFloatingBodies();

  // TODO (@Birn-Acosta) split dynamics constraint into pairwise constraints on
  //  knot points for sparsity
  dynamics_constraint_ =
      std::make_shared<CollocationConstraint<double>>(&timeline_);

  nonlin_constraints_.push_back(
       prog_.AddConstraint(dynamics_constraint_, stack(knot_point_vars_)));

  for (int i = 0; i < params_.N; ++i) {
    auto kinematic_constraint = std::make_shared<KinematicConstraint<double>>(
        &timeline_.knots.at(i+1));
    nonlin_constraints_.push_back(
         prog_.AddConstraint(kinematic_constraint, knot_point_vars_.at(i+1)));
  }

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
  timeline_.knots.at(knot_index).UpdateActiveContacts(contacts);
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

void IDMPC::ConstructSQPProgram(const VectorXd &x, QPData& qp) const {
  DRAKE_DEMAND(x.rows() == prog_.num_vars());
  for (const auto& binding : prog_.GetAllCosts()) {
    if (dynamic_cast<NonlinearLeastSquaresCost<AutoDiffXd>*>(binding.evaluator().get())) {

    } else if (dynamic_cast<NonlinearLeastSquaresCost<double>*>(binding.evaluator().get())) {

    } else if (dynamic_cast<drake::solvers::QuadraticCost*>(binding.evaluator().get())) {

    } else {
      throw std::runtime_error("Unsupported cost type has been added to IDMPC");
    }
  }

  for (const auto& binding : prog_.GetAllConstraints()) {
    if (dynamic_cast<NonlinearConstraint<AutoDiffXd>*>(binding.evaluator().get())) {

    } else if (dynamic_cast<NonlinearConstraint<double>*>(binding.evaluator().get())) {

    } else if (dynamic_cast<drake::solvers::LinearEqualityConstraint*>(binding.evaluator().get())) {

    } else if (dynamic_cast<drake::solvers::LinearConstraint*>(binding.evaluator().get())) {

    } else {
      throw std::runtime_error("Unsupported constraint type has been added to IDMPC");
    }

  }
}

LcmTrajectory IDMPC::GetSolutionAsLcmTrajectory(const MathematicalProgramResult &result) const {

  LcmTrajectory::Trajectory q("q", dynamics_->nq(), params_.N + 1);
  LcmTrajectory::Trajectory v("v", dynamics_->nv(), params_.N + 1);
  LcmTrajectory::Trajectory u("u", dynamics_->nu(), params_.N);
  LcmTrajectory::Trajectory lambda("lambda",dynamics_->n_constraint_total(),
                                   params_.N);

  for (int i = 0; i < params_.N + 1; ++i) {
    q.time_vector(i) = timeline_.breaks.at(i);
    v.time_vector(i) = timeline_.breaks.at(i);
    q.datapoints.col(i) = result.GetSolution(position_vars(i));
    v.datapoints.col(i) = result.GetSolution(velocity_vars(i));
  }

  for (int i = 0; i < params_.N; ++i) {
    u.time_vector(i) = timeline_.breaks.at(i);
    lambda.time_vector(i) = timeline_.breaks.at(i);
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