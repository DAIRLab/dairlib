#include "multibody/multibody_solvers.h"

namespace dairlib {
namespace multibody {

using std::make_unique;
using std::logic_error;
using std::string;

using drake::AutoDiffXd;
using drake::AutoDiffVecXd;
using drake::math::autoDiffToValueMatrix;
using drake::math::autoDiffToGradientMatrix;
using drake::math::DiscardGradient;
using drake::math::initializeAutoDiff;
using drake::solvers::Constraint;
using drake::symbolic::Variable;
using drake::symbolic::Expression;
using drake::VectorX;
using Eigen::VectorXd;

PositionConstraint::PositionConstraint(const RigidBodyTree<double>& tree,
                                       const string& description)
    : Constraint(tree.getNumPositionConstraints(), tree.get_num_positions(),
                 VectorXd::Zero(tree.getNumPositionConstraints()),
                 VectorXd::Zero(tree.getNumPositionConstraints()), description),
      tree_(tree) {}

void PositionConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& q,
                                Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  Eval(drake::math::initializeAutoDiff(q), &y_t);
  *y = autoDiffToValueMatrix(y_t);
}

void PositionConstraint::DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& q,
                                drake::AutoDiffVecXd* y) const {
  const AutoDiffVecXd q_t = q.head(tree_.get_num_positions());
  KinematicsCache<AutoDiffXd> k_cache = tree_.doKinematics(q_t);
  *y = tree_.positionConstraints(k_cache);
}

void PositionConstraint::DoEval(
    const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>& x,
    drake::VectorX<drake::symbolic::Expression>* y) const {
  throw logic_error("PositionConstraint does not support symbolic evaluation.");
}

FixedPointConstraint::FixedPointConstraint(const RigidBodyTree<double>& tree,
                                           ContactInfo contact_info,
                                           const string& description)
    : Constraint(tree.get_num_velocities(),
                 tree.get_num_positions() + tree.get_num_actuators() +
                     tree.getNumPositionConstraints() +
                     contact_info.idxA.size(),
                 VectorXd::Zero(tree.get_num_velocities()),
                 VectorXd::Zero(tree.get_num_velocities()), description),
      tree_(tree),
      contact_info_(contact_info),
      num_positions_(tree.get_num_positions()),
      num_velocities_(tree.get_num_velocities()),
      num_efforts_(tree.get_num_actuators()),
      num_position_forces_(tree.getNumPositionConstraints()),
      num_contact_forces_(contact_info.idxA.size()),
      num_forces_(tree.getNumPositionConstraints() + contact_info.idxA.size()) {
  contact_toolkit_ =
      make_unique<ContactToolkit<AutoDiffXd>>(tree, contact_info);
}

void FixedPointConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& q_u_l, Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  Eval(initializeAutoDiff(q_u_l), &y_t);
  *y = autoDiffToValueMatrix(y_t);
}

void FixedPointConstraint::DoEval(
    const Eigen::Ref<const drake::AutoDiffVecXd>& q_u_l,
    drake::AutoDiffVecXd* y) const {
  // Verifying the size of the input vector
  DRAKE_DEMAND(q_u_l.size() == num_positions_ + num_efforts_ + num_forces_);

  // Extracting the components
  const AutoDiffVecXd q = q_u_l.head(num_positions_);
  const AutoDiffVecXd u = q_u_l.segment(num_positions_, num_efforts_);
  const AutoDiffVecXd lambda = q_u_l.tail(num_forces_);

  AutoDiffVecXd x = VectorXd::Zero(num_positions_ + num_velocities_)
                     .template cast<AutoDiffXd>();
  x.head(num_positions_) = q;

  *y = contact_toolkit_->CalcMVDot(x, u, lambda);
}

void FixedPointConstraint::DoEval(
    const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>& q_u_l,
    drake::VectorX<drake::symbolic::Expression>* y) const {
  throw logic_error(
      "FixedPointConstraint does not support symbolic evaluation.");
}

}  // namespace multibody
}  // namespace dairlib
