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

template<typename T>
VectorX<T> CalcMVdot(const RigidBodyPlant<double>& plant,
                              VectorX<T> q,
                              VectorX<T> v,
                              VectorX<T> u, 
                              VectorX<T> lambda) {

  const RigidBodyTree<double>& tree = plant.get_rigid_body_tree();
  auto k_cache = tree.doKinematics(q, v);
  const MatrixX<T> M = tree.massMatrix(k_cache);

  const typename RigidBodyTree<T>::BodyToWrenchMap no_external_wrenches;

  VectorX<T> C =
    tree.dynamicsBiasTerm(k_cache, no_external_wrenches);
  MatrixX<T> J =
    tree.positionConstraintsJacobian(k_cache);
  VectorX<T> m_vdot =
    -C + tree.B*u + J.transpose()*lambda;

  return m_vdot;
}


template<typename T>
VectorX<T> CalcTimeDerivativesUsingLambda(const RigidBodyPlant<T>& plant, 
                                        VectorX<T> x,
                                        VectorX<T> u,
                                        VectorX<T> lambda) {

  const RigidBodyTree<double>& tree = plant.get_rigid_body_tree();
  const int num_positions = tree.get_num_positions();
  const int num_velocities = tree.get_num_velocities();
  const int num_actuators = tree.get_num_actuators();

  VectorX<T> q = x.topRows(num_positions);
  VectorX<T> v = x.bottomRows(num_velocities);

  auto k_cache = tree.doKinematics(q, v);
  const MatrixX<T> M = tree.massMatrix(k_cache);
  const typename RigidBodyTree<T>::BodyToWrenchMap no_external_wrenches;

  VectorX<T> right_hand_side = 
    -tree.dynamicsBiasTerm(k_cache, no_external_wrenches);

  if (num_actuators > 0) {
    right_hand_side += tree.B * u;
  }


  auto J = tree.positionConstraintsJacobian(k_cache);
  right_hand_side += J.transpose()*lambda;

  VectorX<T> vdot =
    M.completeOrthogonalDecomposition().solve(right_hand_side);

  VectorX<T> x_dot(plant.get_num_states());
  x_dot << tree.transformVelocityToQDot(k_cache, v), vdot;
  return x_dot;
}


// Instantiating templates
template VectorX<double> CalcTimeDerivativesUsingLambda<double>(const RigidBodyPlant<double>&,
                                                        VectorX<double>,
                                                        VectorX<double>,
                                                        VectorX<double>);

template VectorX<AutoDiffXd> CalcTimeDerivativesUsingLambda<AutoDiffXd>(const RigidBodyPlant<AutoDiffXd>&,
                                                        VectorX<AutoDiffXd>,
                                                        VectorX<AutoDiffXd>,
                                                        VectorX<AutoDiffXd>);

template VectorX<double> CalcMVdot<double>(const RigidBodyPlant<double>&,
                                           VectorX<double>,
                                           VectorX<double>,
                                           VectorX<double>,
                                           VectorX<double>);

template VectorX<AutoDiffXd> CalcMVdot<AutoDiffXd>(const RigidBodyPlant<double>&,
                                                   VectorX<AutoDiffXd>,
                                                   VectorX<AutoDiffXd>,
                                                   VectorX<AutoDiffXd>,
                                                   VectorX<AutoDiffXd>);



} // namespace utils
} // namespace multibody
} // namespace dairlib



