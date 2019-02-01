#include "multibody/multibody_solvers.h"

namespace dairlib {
namespace multibody {

using std::string;

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

FixedPointConstraint::FixedPointConstraint(const RigidBodyTree<double>& tree,
                                           const int num_forces,
                                           const string& description)
    : Constraint(
          tree.get_num_velocities(),
          tree.get_num_positions() + tree.get_num_actuators() + num_forces,
          VectorXd::Zero(tree.get_num_velocities()),
          VectorXd::Zero(tree.get_num_velocities()), description),
      tree_(tree),
      num_forces_(num_forces) {}

void FixedPointConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& q_u_l,
    Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  Eval(initializeAutoDiff(q_u_l), &y_t);
  *y = autoDiffToValueMatrix(y_t);
}

}  // namespace multibody
}  // namespace dairlib
