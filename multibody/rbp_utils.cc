#include "rbp_utils.h"

using drake::systems::RigidBodyPlant;
using drake::VectorX;
using drake::MatrixX;
using drake::AutoDiffXd;

using Eigen::VectorXd;
using Eigen::MatrixXd;


namespace dairlib {
namespace multibody {
namespace utils {


VectorX<AutoDiffXd> CalcMVdot(RigidBodyPlant<double>* plant,
                              VectorX<AutoDiffXd> q,
                              VectorX<AutoDiffXd> v,
                              VectorX<AutoDiffXd> u, 
                              VectorX<AutoDiffXd> lambda) {

  const RigidBodyTree<double>& tree = plant->get_rigid_body_tree();
  auto k_cache = tree.doKinematics(q, v);
  const MatrixX<AutoDiffXd> M = tree.massMatrix(k_cache);

  const typename RigidBodyTree<AutoDiffXd>::BodyToWrenchMap no_external_wrenches;

  VectorX<AutoDiffXd> C =
    tree.dynamicsBiasTerm(k_cache, no_external_wrenches);
  MatrixX<AutoDiffXd> J =
    tree.positionConstraintsJacobian(k_cache);
  VectorX<AutoDiffXd> m_vdot =
    -C + tree.B*u + J.transpose()*lambda;

  return m_vdot;
}


VectorX<double> CalcMVdot(RigidBodyPlant<double>* plant,
                              VectorX<double> q,
                              VectorX<double> v,
                              VectorX<double> u, 
                              VectorX<double> lambda) {

  const RigidBodyTree<double>& tree = plant->get_rigid_body_tree();
  auto k_cache = tree.doKinematics(q, v);
  const MatrixX<double> M = tree.massMatrix(k_cache);

  const typename RigidBodyTree<double>::BodyToWrenchMap no_external_wrenches;

  VectorX<double> C =
    tree.dynamicsBiasTerm(k_cache, no_external_wrenches);
  MatrixX<double> J =
    tree.positionConstraintsJacobian(k_cache);
  VectorX<double> m_vdot =
    -C + tree.B*u + J.transpose()*lambda;

  return m_vdot;
}


VectorXd CalcTimeDerivativesUsingLambda(RigidBodyPlant<double>* plant, 
                                        VectorXd x,
                                        VectorXd u,
                                        VectorXd lambda) {

  const RigidBodyTree<double>& tree = plant->get_rigid_body_tree();
  const int num_positions = tree.get_num_positions();
  const int num_velocities = tree.get_num_velocities();
  const int num_actuators = tree.get_num_actuators();

  VectorXd q = x.topRows(num_positions);
  VectorXd v = x.bottomRows(num_velocities);

  auto k_cache = tree.doKinematics(q, v);
  const MatrixXd M = tree.massMatrix(k_cache);
  const typename RigidBodyTree<double>::BodyToWrenchMap no_external_wrenches;

  VectorXd right_hand_side = 
    -tree.dynamicsBiasTerm(k_cache, no_external_wrenches);

  if (num_actuators > 0) {
    right_hand_side += tree.B * u;
  }

  {
    for (auto const& b : tree.get_bodies()) {
      if (!b->has_parent_body()) continue;
      auto const& joint = b->getJoint();
      if (joint.get_num_positions() == 1 && joint.get_num_velocities() == 1) {
        const double limit_force = 
          plant->JointLimitForce(joint, q(b->get_position_start_index()),
                                  v(b->get_velocity_start_index()));
        right_hand_side(b->get_velocity_start_index()) += limit_force;
      }
    }
  }

  auto J = tree.positionConstraintsJacobian(k_cache);
  right_hand_side += J.transpose()*lambda;

  VectorXd vdot =
    M.completeOrthogonalDecomposition().solve(right_hand_side);

  VectorXd x_dot(plant->get_num_states());
  x_dot << tree.transformVelocityToQDot(k_cache, v), vdot;
  return x_dot;
}

} // namespace utils
} // namespace multibody
} // namespace dairlib

