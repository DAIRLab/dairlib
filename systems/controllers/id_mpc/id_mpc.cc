#include "id_mpc.h"
#include "common/eigen_utils.h"

namespace dairlib::systems::controllers::id_mpc {

using Eigen::MatrixXd;
using Eigen::VectorXd;

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

  dynamics_constraint_ =
      std::make_shared<CollocationConstraint<double>>(&timeline_);

  prog_.AddConstraint(dynamics_constraint_, stack(knot_point_vars_));

  for (int i = 0; i < params_.N; ++i) {
    auto kinematic_constraint =
        std::make_shared<KinematicConstraint<double>>(
        &timeline_.knots.at(i+1));
    kinematic_constraints_.push_back(
        prog_.AddConstraint(kinematic_constraint, knot_point_vars_.at(i+1)));
  }

  initial_state_constraint_ = prog_.AddLinearEqualityConstraint(
      MatrixXd::Identity(dynamics_->nx(), dynamics_->nx()),
      VectorXd::Zero(dynamics_->nx()),
      knot_point_vars_.front().head(dynamics_->nx())).evaluator().get();
}

void IDMPC::SetInitialState(const Eigen::VectorXd &x) {
  DRAKE_ASSERT(x.size() == dynamics_->nx());
  initial_state_constraint_->UpdateCoefficients(
      MatrixXd::Identity(dynamics_->nx(), dynamics_->nx()), x);
}

void IDMPC::SetActiveContacts(
    int knot_index, std::vector<std::string> contacts) {
  timeline_.knots.at(knot_index).UpdateActiveContacts(contacts);
}

}