#include "id_mpc.h"

namespace dairlib::systems::controllers::id_mpc {

using Eigen::MatrixXd;
using Eigen::VectorXd;

IDMPC::IDMPC(IDMPCParams params, std::unique_ptr<ConstrainedDynamicsInfo>
    dynamics) : params_(params), dynamics_(std::move(dynamics)) {

  DRAKE_DEMAND(dynamics_ != nullptr);
  DRAKE_DEMAND(params_.N > 0);
  DRAKE_DEMAND(params_.dt > 0);

  for (int i = 0; i < params_.N + 1; ++i) {
    knot_point_work_.push_back(KnotPointState(*dynamics_));
    knot_point_vars_.push_back(
        prog_.NewContinuousVariables(dynamics_->variable_count()));
  }

  for (int i = 0; i < params_.N; ++i) {
    auto collocation_constraint =
        std::make_shared<TrapezoidalCollocationConstraint<double>>(
            &knot_point_work_.at(i), &knot_point_work_.at(i + 1), params_.dt);
    dynamics_constraints_.push_back(
        prog_.AddConstraint(
            collocation_constraint,
            {knot_point_vars_.at(i), knot_point_vars_.at(i + 1)}));
    auto kinematic_constraint =
        std::make_shared<KinematicConstraint<double>>(
        &knot_point_work_.at(i+1));
    kinematic_constraints_.push_back(
        prog_.AddConstraint(kinematic_constraint, knot_point_vars_.at(i+1)));
  }

  initial_state_constraint_ = prog_.AddLinearEqualityConstraint(
      MatrixXd::Identity(dynamics_->nx(), dynamics_->nx()),
      VectorXd::Zero(dynamics_->nx()),
      knot_point_vars_.front().head(dynamics_->nx()));
}

void IDMPC::SetInitialState(const Eigen::VectorXd &x) {
  initial_state_constraint_.evaluator()->UpdateCoefficients(
      MatrixXd::Identity(dynamics_->nx(), dynamics_->nx()), x);
}

void IDMPC::SetActiveContacts(
    int knot_index, std::vector<std::string> contacts) {
  knot_point_work_.at(knot_index).UpdateActiveContacts(contacts);
}

}