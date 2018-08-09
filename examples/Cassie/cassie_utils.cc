#include "examples/Cassie/cassie_utils.h"
#include "common/find_resource.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/multibody/joints/revolute_joint.h"

using Eigen::VectorXd;
using Eigen::Vector3d;
using drake::solvers::Constraint;
using drake::AutoDiffVecXd;
using drake::solvers::MathematicalProgram;

namespace dairlib {

std::unique_ptr<RigidBodyTree<double>> makeFixedBaseCassieTreePointer(
    std::string filename) {
  auto tree = std::make_unique<RigidBodyTree<double>>();
  buildFixedBaseCassieTree(*tree.get(), filename);
  return tree;
}

void buildFixedBaseCassieTree(RigidBodyTree<double>& tree,
                              std::string filename) {
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow(filename),
      drake::multibody::joints::kFixed, &tree);

  // Add distance constraints for the two legs
  double achilles_length = .5012;
  int heel_spring_left = tree.FindBodyIndex("heel_spring_left");
  int thigh_left = tree.FindBodyIndex("thigh_left");

  int heel_spring_right = tree.FindBodyIndex("heel_spring_right");
  int thigh_right = tree.FindBodyIndex("thigh_right");

  Vector3d rod_on_heel_spring;  // symmetric left and right
  rod_on_heel_spring << .11877, -.01, 0.0;

  Vector3d rod_on_thigh_left;
  rod_on_thigh_left << 0.0, 0.0, 0.045;

  Vector3d rod_on_thigh_right;
  rod_on_thigh_right << 0.0, 0.0, -0.045;


  tree.addDistanceConstraint(heel_spring_left, rod_on_heel_spring,
                             thigh_left, rod_on_thigh_left,
                             achilles_length);

  tree.addDistanceConstraint(heel_spring_right, rod_on_heel_spring,
                           thigh_right, rod_on_thigh_right,
                           achilles_length);

  //// Add spring forces
  //int body_index = tree.FindIndexOfChildBodyOfJoint("knee_joint_left");
  //auto body = tree.get_mutable_body(body_index);
  //RevoluteJoint& knee_joint_left = dynamic_cast<RevoluteJoint&>(
  //      body->get_mutable_joint());
  //// stiffness is 2300 in URDF,these #s from gazebo
  //knee_joint_left.SetSpringDynamics(15.0, 0.0);

  //body_index = tree.FindIndexOfChildBodyOfJoint("knee_joint_right");
  //body = tree.get_mutable_body(body_index);
  //RevoluteJoint& knee_joint_right = dynamic_cast<RevoluteJoint&>(
  //      body->get_mutable_joint());
  //knee_joint_right.SetSpringDynamics(15.0, 0.0);  // 2300 in URDF

  //body_index = tree.FindIndexOfChildBodyOfJoint("ankle_spring_joint_left");
  //body = tree.get_mutable_body(body_index);
  //RevoluteJoint& ankle_spring_joint_left = dynamic_cast<RevoluteJoint&>(
  //      body->get_mutable_joint());
  //ankle_spring_joint_left.SetSpringDynamics(12.0, 0.0);  // 2000 in URDF

  //body_index = tree.FindIndexOfChildBodyOfJoint("ankle_spring_joint_right");
  //body = tree.get_mutable_body(body_index);
  //RevoluteJoint& ankle_spring_joint_right = dynamic_cast<RevoluteJoint&>(
  //      body->get_mutable_joint());
  //ankle_spring_joint_right.SetSpringDynamics(12.0, 0.0);  // 2300 in URDF
}



std::unique_ptr<RigidBodyTree<double>> makeFloatingBaseCassieTreePointer(
    std::string filename) {
  auto tree = std::make_unique<RigidBodyTree<double>>();
  buildFloatingBaseCassieTree(*tree.get());
  return tree;
}

void buildFloatingBaseCassieTree(RigidBodyTree<double>& tree,
                              std::string filename) {
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow(filename),
      drake::multibody::joints::kRollPitchYaw, &tree);

  // Add distance constraints for the two legs
  double achilles_length = .5012;
  int heel_spring_left = tree.FindBodyIndex("heel_spring_left");
  int thigh_left = tree.FindBodyIndex("thigh_left");

  int heel_spring_right = tree.FindBodyIndex("heel_spring_right");
  int thigh_right = tree.FindBodyIndex("thigh_right");

  Vector3d rod_on_heel_spring;  // symmetric left and right
  rod_on_heel_spring << .11877, -.01, 0.0;

  Vector3d rod_on_thigh_left;
  rod_on_thigh_left << 0.0, 0.0, 0.045;

  Vector3d rod_on_thigh_right;
  rod_on_thigh_right << 0.0, 0.0, -0.045;


  tree.addDistanceConstraint(heel_spring_left, rod_on_heel_spring,
                             thigh_left, rod_on_thigh_left,
                             achilles_length);

  tree.addDistanceConstraint(heel_spring_right, rod_on_heel_spring,
                           thigh_right, rod_on_thigh_right,
                           achilles_length);

  // Add spring forces
  int body_index = tree.FindIndexOfChildBodyOfJoint("knee_joint_left");
  auto body = tree.get_mutable_body(body_index);
  RevoluteJoint& knee_joint_left = dynamic_cast<RevoluteJoint&>(
        body->get_mutable_joint());
  // stiffness is 2300 in URDF,these #s from gazebo
  knee_joint_left.SetSpringDynamics(1500.0, 0.0);

  body_index = tree.FindIndexOfChildBodyOfJoint("knee_joint_right");
  body = tree.get_mutable_body(body_index);
  RevoluteJoint& knee_joint_right = dynamic_cast<RevoluteJoint&>(
        body->get_mutable_joint());
  knee_joint_right.SetSpringDynamics(1500.0, 0.0);  // 2300 in URDF

  body_index = tree.FindIndexOfChildBodyOfJoint("ankle_spring_joint_left");
  body = tree.get_mutable_body(body_index);
  RevoluteJoint& ankle_spring_joint_left = dynamic_cast<RevoluteJoint&>(
        body->get_mutable_joint());
  ankle_spring_joint_left.SetSpringDynamics(1250.0, 0.0);  // 2000 in URDF

  body_index = tree.FindIndexOfChildBodyOfJoint("ankle_spring_joint_right");
  body = tree.get_mutable_body(body_index);
  RevoluteJoint& ankle_spring_joint_right = dynamic_cast<RevoluteJoint&>(
        body->get_mutable_joint());
  ankle_spring_joint_right.SetSpringDynamics(1250.0, 0.0);  // 2300 in URDF
}


VectorXd ComputeCassieControlInputAnalytical(const RigidBodyTree<double>& tree, VectorXd x) {

  bool debug_flag = true;

  MatrixXd B = tree.B;
  auto k_cache = tree.doKinematics(x.head(tree.get_num_positions()), x.tail(tree.get_num_velocities()));
  MatrixXd M = tree.massMatrix(k_cache);

  const typename RigidBodyTree<double>::BodyToWrenchMap no_external_wrenches;
  VectorXd C = tree.dynamicsBiasTerm(k_cache, no_external_wrenches, true);
  //VectorXd u = B.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(C);
  MatrixXd J = tree.positionConstraintsJacobian(k_cache, true);


  // Computing lambda using the zero rows of B
  int num_zero_rows = 0;
  vector<int> zero_row_indices;
  for (int i=0; i<B.rows(); i++) {
    if (B.row(i).isZero()) {
      num_zero_rows++;
      zero_row_indices.push_back(i);
    }
  }

  // Block of J and C used to compute lambda
  MatrixXd Jb = MatrixXd::Zero(J.rows(), num_zero_rows);
  VectorXd Cb = VectorXd::Zero(num_zero_rows);
  for (int i=0; i<num_zero_rows; i++) {
    Jb.col(i) = J.col(zero_row_indices.at(i));
    Cb(i) = C(zero_row_indices.at(i));
  }

  Jb.transposeInPlace();

  VectorXd lambda = Jb.completeOrthogonalDecomposition().solve(Cb);

  MatrixXd CJ = C - J.transpose()*lambda;

  // Computing u
  VectorXd u = B.completeOrthogonalDecomposition().solve(CJ);

  if (debug_flag) {

    std::cout << "*****************C******************" << std::endl;
    std::cout << C << std::endl;
    std::cout << "*****************B******************" << std::endl;
    std::cout << B << std::endl;
    std::cout << "*****************J******************" << std::endl;
    std::cout << J << std::endl;
    std::cout << "*****************Jb*****************" << std::endl;
    std::cout << Jb << std::endl;
    std::cout << "*****************Cb*****************" << std::endl;
    std::cout << Cb << std::endl;
    std::cout << "**************lambda****************" << std::endl;
    std::cout << lambda << std::endl;
    std::cout << "****************CJ******************" << std::endl;
    std::cout << CJ << std::endl;
    std::cout << "****************u*******************" << std::endl;
    std::cout << u << std::endl;
    std::cout << "**************Check*****************" << std::endl;
    std::cout << C - J.transpose()*lambda - B*u << std::endl;

  }


  return u;

}


int GetBodyIndexFromName(const RigidBodyTree<double>& tree, 
                         string name) {

  for(int i=0; i<tree.get_num_bodies(); i++) {

    if(!tree.get_body(i).get_name().compare(name)) {
      return i;
    }
  }
  return -1;
}


VectorXd ComputeCassieJointLimitForces(RigidBodyPlant<double>* plant,
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



bool CassieJointsWithinLimits(const RigidBodyTree<double>& tree, 
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

// Time derivative computation with zero alpha
template<typename T>
void CassiePlant<T>::CalcTimeDerivativesCassie(VectorX<T> x, 
                                               VectorX<T> u,
                                               ContinuousState<T>* x_dot) const {

  const int nq = tree_.get_num_positions();
  const int nv = tree_.get_num_velocities();
  const int num_actuators = tree_.get_num_actuators();

  VectorX<T> q = x.topRows(nq);
  VectorX<T> v = x.bottomRows(nv);

  auto k_cache = tree_.doKinematics(q, v);
  const MatrixX<T> M = tree_.massMatrix(k_cache);
  const typename RigidBodyTree<T>::BodyToWrenchMap no_external_wrenches;

  VectorX<T> right_hand_side = 
    -tree_.dynamicsBiasTerm(k_cache, no_external_wrenches);

  if (num_actuators > 0) {
    right_hand_side += tree_.B * u;
  }

  {
    for (auto const& b : tree_.get_bodies()) {
      if (!b->has_parent_body()) continue;
      auto const& joint = b->getJoint();
      if (joint.get_num_positions() == 1 && joint.get_num_velocities() == 1) {
        const T limit_force = 
          plant_->JointLimitForce(joint, q(b->get_position_start_index()),
                                  v(b->get_velocity_start_index()));
        right_hand_side(b->get_velocity_start_index()) += limit_force;
      }
    }
  }

  VectorX<T> vdot;
  if (tree_.getNumPositionConstraints()) {
    const T alpha = 0.0;
    auto phi = tree_.positionConstraints(k_cache);
    auto J = tree_.positionConstraintsJacobian(k_cache, false);
    auto Jdotv = tree_.positionConstraintsJacDotTimesV(k_cache);

    MatrixX<T> A(M.rows() + J.rows(), 
                 M.cols() + J.rows());
    VectorX<T> b(M.rows() + J.rows());
    A << M, -J.transpose(), 
         J, MatrixX<T>::Zero(J.rows(), J.rows());
    b << right_hand_side, 
         -(Jdotv + 2 * alpha * J * v + alpha * alpha * phi);
    const VectorX<T> vdot_f = 
      A.completeOrthogonalDecomposition().solve(b);
    vdot = vdot_f.head(tree_.get_num_velocities());
  } else {
    vdot = M.llt().solve(right_hand_side);
  }

  VectorX<T> xdot_sol(plant_->get_num_states());
  xdot_sol << tree_.transformVelocityToQDot(k_cache, v), vdot;
  x_dot->SetFromVector(xdot_sol);
}



// Time derivative computation with zero alpha (Overloaded function)
template<typename T>
VectorX<T> CassiePlant<T>::CalcTimeDerivativesCassie(VectorX<T> x, 
                                                     VectorX<T> u) const {

  const int nq = tree_.get_num_positions();
  const int nv = tree_.get_num_velocities();
  const int num_actuators = tree_.get_num_actuators();

  VectorX<T> q = x.topRows(nq);
  VectorX<T> v = x.bottomRows(nv);

  auto k_cache = tree_.doKinematics(q, v);
  const MatrixX<T> M = tree_.massMatrix(k_cache);
  const typename RigidBodyTree<T>::BodyToWrenchMap no_external_wrenches;

  VectorX<T> right_hand_side = 
    -tree_.dynamicsBiasTerm(k_cache, no_external_wrenches);

  if (num_actuators > 0) {
    right_hand_side += tree_.B * u;
  }

  {
    for (auto const& b : tree_.get_bodies()) {
      if (!b->has_parent_body()) continue;
      auto const& joint = b->getJoint();
      if (joint.get_num_positions() == 1 && joint.get_num_velocities() == 1) {
        const T limit_force = 
          plant_->JointLimitForce(joint, q(b->get_position_start_index()),
                                  v(b->get_velocity_start_index()));
        right_hand_side(b->get_velocity_start_index()) += limit_force;
      }
    }
  }

  VectorX<T> vdot;
  if (tree_.getNumPositionConstraints()) {
    const T alpha = 0.0;
    auto phi = tree_.positionConstraints(k_cache);
    auto J = tree_.positionConstraintsJacobian(k_cache, false);
    auto Jdotv = tree_.positionConstraintsJacDotTimesV(k_cache);

    MatrixX<T> A(M.rows() + J.rows(), 
                 M.cols() + J.rows());
    VectorX<T> b(M.rows() + J.rows());
    A << M, -J.transpose(), 
         J, MatrixX<T>::Zero(J.rows(), J.rows());
    b << right_hand_side, 
         -(Jdotv + 2 * alpha * J * v + alpha * alpha * phi);
    const VectorX<T> vdot_f = 
      A.completeOrthogonalDecomposition().solve(b);
    vdot = vdot_f.head(tree_.get_num_velocities());
  } else {
    vdot = M.llt().solve(right_hand_side);
  }

  VectorX<T> xdot(plant_->get_num_states());
  xdot << tree_.transformVelocityToQDot(k_cache, v), vdot;
  return xdot;
}
          




// The function is not templated as we need the double versions of
// x, u and lambda (for collision detect) which is difficult to obtain during compile time
template<typename T>
void CassiePlant<T>::CalcTimeDerivativesCassieDuringContact(VectorX<T> x, 
                                                            VectorX<T> u, 
                                                            VectorX<T> lambda,
                                                            ContinuousState<T>* x_dot) const {

  const int nq = tree_.get_num_positions();
  const int nv = tree_.get_num_velocities();
  const int num_actuators = tree_.get_num_actuators();

  VectorX<T> q = x.topRows(nq);
  VectorX<T> v = x.bottomRows(nv);

  //Computing double versions
  VectorX<double> q_double = DiscardGradient(q);
  VectorX<double> v_double = DiscardGradient(v);

  KinematicsCache<T> k_cache = tree_.doKinematics(q, v);
  KinematicsCache<double> k_cache_double = tree_.doKinematics(q_double, v_double);

  const MatrixX<T> M = tree_.massMatrix(k_cache);
  const typename RigidBodyTree<T>::BodyToWrenchMap no_external_wrenches;

  VectorX<T> right_hand_side = 
    -tree_.dynamicsBiasTerm(k_cache, no_external_wrenches);

  if (num_actuators > 0) {
    right_hand_side += tree_.B * u;
  }

  {
    for (auto const& b : tree_.get_bodies()) {
      if(!b->has_parent_body()) continue;
      auto const& joint = b->getJoint();

      if(joint.get_num_positions() == 1 && joint.get_num_velocities() == 1) {
        const T limit_force = 
          plant_->JointLimitForce(joint, q(b->get_position_start_index()), 
                                  v(b->get_velocity_start_index()));
        right_hand_side(b->get_velocity_start_index()) += limit_force;
      }
    }
  }

  // Compliant contact forces wont be added here
  // Computing accelerations without the collision information

  VectorX<T> v_dot1;
  if (tree_.getNumPositionConstraints()) {

    const T alpha = 0.5;
    auto phi = tree_.positionConstraints(k_cache);
    auto J = tree_.positionConstraintsJacobian(k_cache, false);
    auto Jdotv = tree_.positionConstraintsJacDotTimesV(k_cache);

    MatrixX<T> A(M.rows() + J.rows(), 
                 M.cols() + J.rows());
    VectorX<T> b(M.rows() + J.rows());

    A << M, -J.transpose(), 
         J, MatrixX<T>::Zero(J.rows(), J.rows());
    b << right_hand_side, 
         -(Jdotv + 2 * alpha * J * v + alpha * alpha * phi);

    const VectorX<T> v_dot_f = 
      A.completeOrthogonalDecomposition().solve(b);

    v_dot1 = v_dot_f.head(tree_.get_num_velocities());
  } else {

    v_dot1 = M.llt().solve(right_hand_side);
  }


  //Collision detect (double template as AutoDiff doesnt work)
  VectorXd phi_total;
  Matrix3Xd normal_total, xA_total, xB_total;
  vector<int> idxA_total, idxB_total;

  // This (const cast) is an ugly way of doing it. Change it later if a better method is available
  std::cout << const_cast<RigidBodyTree<double>&>(tree_).collisionDetect(
      k_cache_double, phi_total, normal_total, xA_total, xB_total, idxA_total, idxB_total);
  std::cout << std::endl;

  const int num_total_contacts = normal_total.cols();
  // 4 contacts for Cassie (2 in each toe)
  const int num_contacts = 4;

  // Making sure that the number of constraint forces are valid
  DRAKE_DEMAND(lambda.size() == num_contacts*3);

  //Getting the indices of the world and toes
  const int world_ind = GetBodyIndexFromName(tree_, "world");
  const int toe_left_ind = GetBodyIndexFromName(tree_, "toe_left");
  const int toe_right_ind = GetBodyIndexFromName(tree_, "toe_right");

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

  Matrix3Xd normal = Matrix3Xd::Zero(normal_total.rows(), num_contacts);
  for (int i=0; i<num_contacts; i++) {
    normal.col(i) = normal_total.col(contact_ind.at(i));
  }
  
  const Map<Matrix3Xd> normal_map(
      normal.data(), normal_total.rows(), num_contacts);

  vector<Map<Matrix3Xd>> tangents;

  Matrix3Xd tmp_mat1 = Matrix3Xd::Zero(3, 4);
  Map<Matrix3Xd> tmp_map1(tmp_mat1.data(), 3, 4);
  Matrix3Xd tmp_mat2 = Matrix3Xd::Zero(3, 4);
  Map<Matrix3Xd> tmp_map2(tmp_mat2.data(), 3, 4);
  tangents.push_back(tmp_map1);
  tangents.push_back(tmp_map2);

  tree_.surfaceTangents(normal_map, tangents);


  //Computing the position Jacobian
  vector<MatrixX<T>> Jd(num_contacts);

  for (int i=0; i<num_contacts; i++) {
    auto tmp_JA = tree_.transformPointsJacobian(k_cache,
                                                xA_total.col(contact_ind.at(i)), 
                                                idxA_total.at(contact_ind.at(i)),
                                                world_ind, 
                                                true);
    auto tmp_JB = tree_.transformPointsJacobian(k_cache,
                                                xB_total.col(contact_ind.at(i)), 
                                                idxB_total.at(contact_ind.at(i)),
                                                world_ind, 
                                                true);
    Jd.at(i) = tmp_JA - tmp_JB;
  }

  std::cout << Jd.at(0) << std::endl;

  //Computing the 3 jacobians for each contact point
  MatrixX<T> J(num_contacts*3, tree_.get_num_positions());

  for (int i=0; i<num_contacts; i++) {

    MatrixX<T> J_pt(3, tree_.get_num_positions());

    auto normal_pt = normal.col(contact_ind.at(i));
    auto tangent1_pt = tangents.at(0).col(contact_ind.at(i));
    auto tangent2_pt = tangents.at(1).col(contact_ind.at(i));

    J_pt.row(0) = normal_pt.transpose()*Jd.at(i);
    J_pt.row(1) = tangent1_pt.transpose()*Jd.at(i);
    J_pt.row(2) = tangent2_pt.transpose()*Jd.at(i);

    J.block(i*3, 0, 3, tree_.get_num_positions()) =  J_pt;
            
  }


  // Adding the vdots for the no contact and the contact case
  VectorX<T> x_dot_sol(plant_->get_num_states());
  VectorX<T> v_dot2;
  v_dot2 = M.llt().solve(J.transpose()*lambda);

  DRAKE_DEMAND(v_dot1.size() == tree_.get_num_velocities());
  DRAKE_DEMAND(v_dot2.size() == tree_.get_num_velocities());

  VectorX<T> v_dot(plant_->get_num_velocities());
  v_dot = v_dot1 + v_dot2;

  x_dot_sol << tree_.transformVelocityToQDot(k_cache, v), v_dot;
  x_dot->SetFromVector(x_dot_sol);
}




}  // namespace dairlib

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::CassiePlant)
