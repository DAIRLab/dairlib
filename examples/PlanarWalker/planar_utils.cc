#include "examples/PlanarWalker/planar_utils.h"
#include "common/find_resource.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/multibody/joints/revolute_joint.h"

using Eigen::VectorXd;
using Eigen::Vector3d;
using drake::solvers::Constraint;
using drake::AutoDiffVecXd;
using drake::solvers::MathematicalProgram;

namespace dairlib {


int GetBodyIndexFromNamePlanar(const RigidBodyTree<double>& tree, 
                         string name) {

  for(int i=0; i<tree.get_num_bodies(); i++) {

    if(!tree.get_body(i).get_name().compare(name)) {
      return i;
    }
  }
  return -1;
}


VectorXd ComputePlanarJointLimitForces(RigidBodyPlant<double>* plant,
                                       VectorXd x_init) {

  VectorXd joint_forces = VectorXd::Zero(plant->get_num_positions());
  VectorXd q_init = x_init.head(plant->get_num_positions());
  VectorXd v_init = x_init.tail(plant->get_num_velocities());
  {
    for (auto const& b : plant->get_rigid_body_tree().get_bodies()) {
      if(!b->has_parent_body()) continue;
      auto const& joint = b->getJoint();

      if(joint.get_num_positions() == 1 && joint.get_num_velocities() == 1) {
        const double limit_force = 
          plant->JointLimitForce(joint, q_init(b->get_position_start_index()), 
                                  v_init(b->get_velocity_start_index()));
        joint_forces(b->get_velocity_start_index()) += limit_force;
      }
    }
  }

  return joint_forces;
}



bool PlanarJointsWithinLimits(const RigidBodyTree<double>& tree, 
                              VectorXd x,
                              double tolerance, 
                              bool print_debug_messages) {

  map<string, int> position_map =
    tree.computePositionNameToIndexMap();

  bool is_within_limits = true;

  for (auto const& b : tree.get_bodies()) {
    if(!b->has_parent_body()) continue;
    auto const& joint = b->getJoint();
    if(joint.get_num_positions() == 1 && joint.get_num_velocities() == 1) {

      auto joint_lim_min_vec = joint.getJointLimitMin();
      auto joint_lim_max_vec = joint.getJointLimitMax();
      const int ind = position_map.at(joint.get_name());

      // Checking if the value is within limits
      if (x(ind) < (joint_lim_min_vec(0) + tolerance)
          || x(ind) > (joint_lim_max_vec(0) - tolerance)) {
        is_within_limits = false;

        if (print_debug_messages) {
          std::cout << "Joint " << joint.get_name() <<
            " with index " << ind << " is outside limits." << std::endl;
          std::cout << "Min limit: " << joint_lim_min_vec(0) << std::endl;
          std::cout << "Max limit: " << joint_lim_max_vec(0) << std::endl;
          std::cout << "Value: " << x(ind) << std::endl;
          std::cout << "_________________" << std::endl;
        }

      }

    }
  }

  return is_within_limits;
}


template<typename T>
MatrixX<T> PlanarPlant<T>::CalcContactJacobianPlanar(VectorX<T> q,
                                                     VectorX<T> v,
                                                     int num_contact_constraints) const {


  const int num_contacts = 2;
  const int num_constraints_per_contact = num_contact_constraints/num_contacts;

  //Computing double versions
  VectorX<double> q_double = DiscardGradient(q);
  VectorX<double> v_double = DiscardGradient(v);

  KinematicsCache<T> k_cache = tree_.doKinematics(q, v);
  KinematicsCache<double> k_cache_double = tree_.doKinematics(q_double, v_double);


  // Collision detect 
  VectorXd phi_total;
  Matrix3Xd normal_total, xA_total, xB_total;
  vector<int> idxA_total, idxB_total;


  // This (const cast) is an ugly way of doing it. Change it later if a better method is available
  const_cast<RigidBodyTree<double>&>(tree_).collisionDetect(
      k_cache_double, phi_total, normal_total, xA_total, xB_total, idxA_total, idxB_total);

  VectorXd phi(num_contacts);
  Matrix3Xd normal(3, num_contacts);
  Matrix3Xd xA(3, num_contacts);
  Matrix3Xd xB(3, num_contacts);
  vector<int> idxA(num_contacts), idxB(num_contacts);

  const int num_total_contacts = normal_total.cols();

  //  Getting the indices of the world and toes
  const int world_ind = GetBodyIndexFromNamePlanar(tree_, "world");
  const int toe_left_ind = GetBodyIndexFromNamePlanar(tree_, "left_lower_leg");
  const int toe_right_ind = GetBodyIndexFromNamePlanar(tree_, "right_lower_leg");

  // Obtaining the indices of valid collisions
  vector<int> contact_ind(num_contacts);
  int k=0;
  for (int i=0; i<num_total_contacts; i++) {
    int ind_a = idxA_total.at(i);
    int ind_b = idxB_total.at(i);
    if ((ind_a == world_ind && ind_b == toe_left_ind) ||
        (ind_a == world_ind && ind_b == toe_right_ind) ||
        (ind_a == toe_left_ind && ind_b == world_ind) ||
        (ind_a == toe_right_ind && ind_b == world_ind)) {

      contact_ind.at(k) = i;
      k++;

    }
  }

  for (int i = 0; i < num_contacts; i++) {
    phi(i) = phi_total(contact_ind.at(i));
    normal.col(i) = normal_total.col(contact_ind.at(i));
    xA.col(i) = xA_total.col(contact_ind.at(i));
    xB.col(i) = xB_total.col(contact_ind.at(i));
    idxA.at(i) = idxA_total.at(contact_ind.at(i));
    idxB.at(i) = idxB_total.at(contact_ind.at(i));
  }
  
  const Map<Matrix3Xd> normal_map(
      normal.data(), 3, num_contacts);

  vector<Map<Matrix3Xd>> tangents_map_vector;

  Matrix3Xd tmp_mat1 = Matrix3Xd::Zero(3, num_contacts);
  Map<Matrix3Xd> tmp_map1(tmp_mat1.data(), 3, num_contacts);
  Matrix3Xd tmp_mat2 = Matrix3Xd::Zero(3, num_contacts);
  Map<Matrix3Xd> tmp_map2(tmp_mat2.data(), 3, num_contacts);
  tangents_map_vector.push_back(tmp_map1);
  tangents_map_vector.push_back(tmp_map2);

  tree_.surfaceTangents(normal_map, tangents_map_vector);


  //Computing the position Jacobian
  vector<MatrixX<T>> Jd(num_contacts);


  for (int i=0; i<num_contacts; i++) {
    auto tmp_JA = tree_.transformPointsJacobian(k_cache,
                                                xA.col(i), 
                                                idxA.at(i),
                                                world_ind, 
                                                true);
    auto tmp_JB = tree_.transformPointsJacobian(k_cache,
                                                xB.col(i), 
                                                idxB.at(i),
                                                world_ind, 
                                                true);
    Jd.at(i) = tmp_JA - tmp_JB;
  }


  //Computing the 3 jacobians for each contact point
  MatrixX<T> J(num_contact_constraints, tree_.get_num_positions());

  for (int i=0; i<num_contacts; i++) {

    MatrixX<T> J_pt(num_contact_constraints, tree_.get_num_positions());

    auto normal_pt = normal.col(i);
    auto tangent1_pt = tangents_map_vector.at(0).col(i);
    auto tangent2_pt = tangents_map_vector.at(1).col(i);

    J_pt.row(0) = normal_pt.transpose()*Jd.at(i);
    J_pt.row(1) = tangent1_pt.transpose()*Jd.at(i);
    //std::cout << tangent1_pt << std::endl;
    //std::cout << "t--------------------------------t" << std::endl;
    //J_pt.row(2) = tangent2_pt.transpose()*Jd.at(i);

    J.block(i*num_constraints_per_contact, 0, num_constraints_per_contact, tree_.get_num_positions()) =  J_pt;
            
  }

  return J;


}


//template<typename T>
//MatrixX<T> PlanarPlant<T>::CalcContactJacobianPlanar(VectorX<T> q,
//                                                     VectorX<T> v,
//                                                     int num_contact_constraints,
//                                                     ContactInfo2 contact_info) const {
//
//  const int num_contacts = 4;
//  const int num_constraints_per_contact = num_contact_constraints/num_contacts;
//
//  //Computing double versions
//  VectorX<double> q_double = DiscardGradient(q);
//  VectorX<double> v_double = DiscardGradient(v);
//
//  KinematicsCache<T> k_cache = tree_.doKinematics(q, v);
//  KinematicsCache<double> k_cache_double = tree_.doKinematics(q_double, v_double);
//
//
//  //Computing the position Jacobian
//  vector<MatrixX<T>> Jd(num_contacts);
//
//  const int world_ind = GetBodyIndexFromName(tree_, "world");
//
//  for (int i=0; i<num_contacts; i++) {
//    auto tmp_JA = tree_.transformPointsJacobian(k_cache,
//                                                contact_info.xA.col(i), 
//                                                contact_info.idxA.at(i),
//                                                world_ind, 
//                                                true);
//    auto tmp_JB = tree_.transformPointsJacobian(k_cache,
//                                                contact_info.xB.col(i), 
//                                                contact_info.idxB.at(i),
//                                                world_ind, 
//                                                true);
//    Jd.at(i) = tmp_JA - tmp_JB;
//  }
//
//
//  //Computing the 3 jacobians for each contact point
//  MatrixX<T> J(num_contact_constraints, tree_.get_num_positions());
//
//  for (int i=0; i<num_contacts; i++) {
//
//    MatrixX<T> J_pt(num_contact_constraints, tree_.get_num_positions());
//
//    auto normal_pt = contact_info.normal.col(i);
//    auto tangent1_pt = contact_info.tangents_map_vector.at(0).col(i);
//    auto tangent2_pt = contact_info.tangents_map_vector.at(1).col(i);
//
//    J_pt.row(0) = normal_pt.transpose()*Jd.at(i);
//    J_pt.row(1) = tangent1_pt.transpose()*Jd.at(i);
//    J_pt.row(2) = tangent2_pt.transpose()*Jd.at(i);
//
//    J.block(i*num_constraints_per_contact, 0, num_constraints_per_contact, tree_.get_num_positions()) =  J_pt;
//            
//  }
//
//  return J;
//
//
//}




// Time derivative computation with zero alpha
//template<typename T>
//void PlanarPlant<T>::CalcTimeDerivativesPlanar(VectorX<T> x, 
//                                               VectorX<T> u,
//                                               ContinuousState<T>* x_dot) const {
//
//  const int num_positions = tree_.get_num_positions();
//  const int num_velocities = tree_.get_num_velocities();
//  const int num_efforts = tree_.get_num_actuators();
//
//  VectorX<T> q = x.topRows(num_positions);
//  VectorX<T> v = x.bottomRows(num_velocities);
//
//  auto k_cache = tree_.doKinematics(q, v);
//  const MatrixX<T> M = tree_.massMatrix(k_cache);
//  const typename RigidBodyTree<T>::BodyToWrenchMap no_external_wrenches;
//
//  VectorX<T> right_hand_side = 
//    -tree_.dynamicsBiasTerm(k_cache, no_external_wrenches);
//
//  if (num_efforts > 0) {
//    right_hand_side += tree_.B * u;
//  }
//
//
//  VectorX<T> vdot;
//  if (tree_.getNumPositionConstraints()) {
//    const T alpha = 0.0;
//    auto phi = tree_.positionConstraints(k_cache);
//    auto J = tree_.positionConstraintsJacobian(k_cache, false);
//    auto Jdotv = tree_.positionConstraintsJacDotTimesV(k_cache);
//
//    MatrixX<T> A(M.rows() + J.rows(), 
//                 M.cols() + J.rows());
//    VectorX<T> b(M.rows() + J.rows());
//    A << M, -J.transpose(), 
//         J, MatrixX<T>::Zero(J.rows(), J.rows());
//    b << right_hand_side, 
//         -(Jdotv + 2 * alpha * J * v + alpha * alpha * phi);
//    const VectorX<T> vdot_f = 
//      A.completeOrthogonalDecomposition().solve(b);
//    vdot = vdot_f.head(tree_.get_num_velocities());
//  } else {
//    vdot = M.llt().solve(right_hand_side);
//  }
//
//  VectorX<T> x_dot_sol(plant_.get_num_states());
//  x_dot_sol << tree_.transformVelocityToQDot(k_cache, v), vdot;
//  x_dot->SetFromVector(x_dot_sol);
//}



// Time derivative computation with zero alpha (Overloaded function)
//template<typename T>
//VectorX<T> PlanarPlant<T>::CalcTimeDerivativesPlanar(VectorX<T> x, 
//                                                     VectorX<T> u) const {
//
//  const int num_positions = tree_.get_num_positions();
//  const int num_velocities = tree_.get_num_velocities();
//  const int num_efforts = tree_.get_num_actuators();
//
//  VectorX<T> q = x.topRows(num_positions);
//  VectorX<T> v = x.bottomRows(num_velocities);
//
//  auto k_cache = tree_.doKinematics(q, v);
//  const MatrixX<T> M = tree_.massMatrix(k_cache);
//  const typename RigidBodyTree<T>::BodyToWrenchMap no_external_wrenches;
//
//  VectorX<T> right_hand_side = 
//    -tree_.dynamicsBiasTerm(k_cache, no_external_wrenches);
//
//  if (num_efforts > 0) {
//    right_hand_side += tree_.B * u;
//  }
//
//
//  VectorX<T> vdot;
//  if (tree_.getNumPositionConstraints()) {
//    const T alpha = 0.0;
//    auto phi = tree_.positionConstraints(k_cache);
//    auto J = tree_.positionConstraintsJacobian(k_cache, false);
//    auto Jdotv = tree_.positionConstraintsJacDotTimesV(k_cache);
//
//    MatrixX<T> A(M.rows() + J.rows(), 
//                 M.cols() + J.rows());
//    VectorX<T> b(M.rows() + J.rows());
//    A << M, -J.transpose(), 
//         J, MatrixX<T>::Zero(J.rows(), J.rows());
//    b << right_hand_side, 
//         -(Jdotv + 2 * alpha * J * v + alpha * alpha * phi);
//    const VectorX<T> vdot_f = 
//      A.completeOrthogonalDecomposition().solve(b);
//    vdot = vdot_f.head(tree_.get_num_velocities());
//  } else {
//    vdot = M.llt().solve(right_hand_side);
//  }
//
//  VectorX<T> x_dot(plant_.get_num_states());
//  x_dot << tree_.transformVelocityToQDot(k_cache, v), vdot;
//  return x_dot;
//}


//template<typename T>
//void PlanarPlant<T>::CalcTimeDerivativesPlanar(VectorX<T> x, 
//                                               VectorX<T> u, 
//                                               VectorX<T> lambda,
//                                               ContinuousState<T>* x_dot) const {
//
//  const int num_positions = tree_.get_num_positions();
//  const int num_velocities = tree_.get_num_velocities();
//  const int num_efforts = tree_.get_num_actuators();
//
//  VectorX<T> q = x.topRows(num_positions);
//  VectorX<T> v = x.bottomRows(num_velocities);
//
//  auto k_cache = tree_.doKinematics(q, v);
//  const MatrixX<T> M = tree_.massMatrix(k_cache);
//  const typename RigidBodyTree<T>::BodyToWrenchMap no_external_wrenches;
//
//  VectorX<T> right_hand_side = 
//    -tree_.dynamicsBiasTerm(k_cache, no_external_wrenches);
//
//  if (num_efforts > 0) {
//    right_hand_side += tree_.B * u;
//  }
//
//  auto J = tree_.positionConstraintsJacobian(k_cache);
//  right_hand_side += J.transpose()*lambda;
//
//  VectorX<T> vdot =
//    M.completeOrthogonalDecomposition().solve(right_hand_side);
//
//  VectorX<T> x_dot_sol(plant_.get_num_states());
//  x_dot_sol << tree_.transformVelocityToQDot(k_cache, v), vdot;
//  x_dot->SetFromVector(x_dot_sol);
//}


//template<typename T>
//VectorX<T> PlanarPlant<T>::CalcTimeDerivativesPlanar(VectorX<T> x, 
//                                                     VectorX<T> u, 
//                                                     VectorX<T> lambda) const {
//
//  const int num_positions = tree_.get_num_positions();
//  const int num_velocities = tree_.get_num_velocities();
//  const int num_efforts = tree_.get_num_actuators();
//
//  VectorX<T> q = x.topRows(num_positions);
//  VectorX<T> v = x.bottomRows(num_velocities);
//
//  auto k_cache = tree_.doKinematics(q, v);
//  const MatrixX<T> M = tree_.massMatrix(k_cache);
//  const typename RigidBodyTree<T>::BodyToWrenchMap no_external_wrenches;
//
//  VectorX<T> right_hand_side = 
//    -tree_.dynamicsBiasTerm(k_cache, no_external_wrenches);
//
//  if (num_efforts > 0) {
//    right_hand_side += tree_.B * u;
//  }
//
//  if (tree_.getNumPositionConstraints()) {
//
//    auto J = tree_.positionConstraintsJacobian(k_cache);
//    right_hand_side += J.transpose()*lambda;
//  }
//
//  VectorX<T> vdot =
//    M.completeOrthogonalDecomposition().solve(right_hand_side);
//
//  VectorX<T> x_dot(plant_.get_num_states());
//  x_dot << tree_.transformVelocityToQDot(k_cache, v), vdot;
//  return x_dot;
//}
          


//template<typename T>
//void PlanarPlant<T>::CalcTimeDerivativesPlanarStanding(VectorX<T> x, 
//                                                       VectorX<T> u, 
//                                                       VectorX<T> lambda,
//                                                       ContinuousState<T>* x_dot) const {
//
//  const int num_positions = tree_.get_num_positions();
//  const int num_velocities = tree_.get_num_velocities();
//  const int num_efforts = tree_.get_num_actuators();
//  const int num_tree_constraints = tree_.getNumPositionConstraints();
//  const int num_total_constraints = lambda.size();
//  const int num_contact_constraints = num_total_constraints - num_tree_constraints;
//
//  VectorX<T> q = x.head(num_positions);
//  VectorX<T> v = x.tail(num_velocities);
//  VectorX<T> lambda_tree = lambda.head(num_tree_constraints);
//  VectorX<T> lambda_contact = lambda.tail(num_contact_constraints);
//
//
//  //Computing double versions
//  VectorX<double> q_double = DiscardGradient(q);
//  VectorX<double> v_double = DiscardGradient(v);
//
//  KinematicsCache<T> k_cache = tree_.doKinematics(q, v);
//
//  const MatrixX<T> M = tree_.massMatrix(k_cache);
//  const typename RigidBodyTree<T>::BodyToWrenchMap no_external_wrenches;
//
//  VectorX<T> right_hand_side = 
//    -tree_.dynamicsBiasTerm(k_cache, no_external_wrenches);
//
//  if (num_efforts > 0) {
//    right_hand_side += tree_.B * u;
//  }
//
//  // Compliant contact forces wont be added here
//  // Computing accelerations without the collision information
//
//  VectorX<T> v_dot1;
//  if (num_tree_constraints) {
//
//    auto J_tree = tree_.positionConstraintsJacobian(k_cache);
//    right_hand_side += J_tree.transpose()*lambda_tree;
//  }
//
//  MatrixX<T> J_contact = CalcContactJacobianPlanar(q, 
//                                                   v,
//                                                   num_contact_constraints);
//
//  right_hand_side += J_contact.transpose()*lambda_contact;
//
//  VectorX<T> x_dot_sol(plant_.get_num_states());
//  VectorX<T> v_dot =
//    M.completeOrthogonalDecomposition().solve(right_hand_side);
//
//  DRAKE_DEMAND(v_dot.size() == num_velocities);
//
//  x_dot_sol << tree_.transformVelocityToQDot(k_cache, v), v_dot;
//  x_dot->SetFromVector(x_dot_sol);
//}


template<typename T>
VectorX<T> PlanarPlant<T>::CalcTimeDerivativesPlanarStanding(VectorX<T> x, 
                                                             VectorX<T> u, 
                                                             VectorX<T> lambda) const {

  const int num_positions = tree_.get_num_positions();
  const int num_velocities = tree_.get_num_velocities();
  const int num_efforts = tree_.get_num_actuators();
  const int num_total_constraints = lambda.size();
  const int num_contact_constraints = num_total_constraints;

  VectorX<T> q = x.head(num_positions);
  VectorX<T> v = x.tail(num_velocities);
  VectorX<T> lambda_contact = lambda.tail(num_contact_constraints);

  //Computing double versions
  VectorX<double> q_double = DiscardGradient(q);
  VectorX<double> v_double = DiscardGradient(v);

  KinematicsCache<T> k_cache = tree_.doKinematics(q, v);

  const MatrixX<T> M = tree_.massMatrix(k_cache);
  const typename RigidBodyTree<T>::BodyToWrenchMap no_external_wrenches;

  VectorX<T> right_hand_side = 
    -tree_.dynamicsBiasTerm(k_cache, no_external_wrenches);

  if (num_efforts > 0) {
    right_hand_side += tree_.B * u;
  }

  // Compliant contact forces wont be added here
  // Computing accelerations without the collision information
 
  MatrixX<T> J_contact = CalcContactJacobianPlanar(q, 
                                                   v,
                                                   num_contact_constraints);

  
  right_hand_side += J_contact.transpose()*lambda_contact;


  // Adding the vdots for the no contact and the contact case
  VectorX<T> x_dot(plant_.get_num_states());
  VectorX<T> v_dot =
    M.completeOrthogonalDecomposition().solve(right_hand_side);

  DRAKE_DEMAND(v_dot.size() == num_velocities);

  x_dot << tree_.transformVelocityToQDot(k_cache, v), v_dot;
  return x_dot;

}


template<typename T>
VectorX<T> PlanarPlant<T>::CalcMVdotPlanarStanding(VectorX<T> x, 
                                                   VectorX<T> u, 
                                                   VectorX<T> lambda) const {

  const int num_positions = tree_.get_num_positions();
  const int num_velocities = tree_.get_num_velocities();
  const int num_efforts = tree_.get_num_actuators();
  const int num_tree_constraints = tree_.getNumPositionConstraints();
  const int num_total_constraints = lambda.size();
  const int num_contact_constraints = num_total_constraints - num_tree_constraints;

  VectorX<T> q = x.head(num_positions);
  VectorX<T> v = x.tail(num_velocities);
  VectorX<T> lambda_tree = lambda.head(num_tree_constraints);
  VectorX<T> lambda_contact = lambda.tail(num_contact_constraints);

  // Making sure that the number of contact constraint forces are valid
  //DRAKE_THROW_UNLESS(num_tree_constraints >= 1);

  //Computing double versions
  VectorX<double> q_double = DiscardGradient(q);
  VectorX<double> v_double = DiscardGradient(v);

  KinematicsCache<T> k_cache = tree_.doKinematics(q, v);

  const MatrixX<T> M = tree_.massMatrix(k_cache);
  const typename RigidBodyTree<T>::BodyToWrenchMap no_external_wrenches;

  VectorX<T> right_hand_side = 
    -tree_.dynamicsBiasTerm(k_cache, no_external_wrenches);

  if (num_efforts > 0) {
    right_hand_side += tree_.B * u;
  }

  // Compliant contact forces wont be added here
  // Computing accelerations without the collision information

  if (num_tree_constraints) {

    auto J_tree = tree_.positionConstraintsJacobian(k_cache);
    right_hand_side += J_tree.transpose()*lambda_tree;

  }

  MatrixX<T> J_contact = CalcContactJacobianPlanar(q, 
                                                   v,
                                                   num_contact_constraints);

 
  right_hand_side += J_contact.transpose()*lambda_contact;

  return right_hand_side;

}





}  // namespace dairlib

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::PlanarPlant);





