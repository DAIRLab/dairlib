#include "multibody/multibody_solvers.h"

namespace dairlib {
namespace multibody {

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

// FixedPointConstraint::FixedPointConstraint(const RigidBodyTree<double>& tree,
//                                           const int num_forces,
//                                           const string& description)
//    : Constraint(
//          tree.get_num_velocities(),
//          tree.get_num_positions() + tree.get_num_actuators() + num_forces,
//          VectorXd::Zero(tree.get_num_velocities()),
//          VectorXd::Zero(tree.get_num_velocities()), description),
//      tree_(tree),
//      num_forces_(num_forces) {}
//
// void FixedPointConstraint::DoEval(
//    const Eigen::Ref<const Eigen::VectorXd>& q_u_l, Eigen::VectorXd* y) const
//    {
//  AutoDiffVecXd y_t;
//  Eval(initializeAutoDiff(q_u_l), &y_t);
//  *y = autoDiffToValueMatrix(y_t);
//}

}  // namespace multibody
}  // namespace dairlib
