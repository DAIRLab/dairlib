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
  buildFixedBaseCassieTree(*tree.get());
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

  if(num_actuators > 0) {
    right_hand_side += tree_.B*u;
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

  VectorX<T> vdot;
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

    const VectorX<T> vdot_f = 
      A.completeOrthogonalDecomposition().solve(b);

    vdot = vdot_f.head(tree_.get_num_velocities());
  } else {

    vdot = M.llt().solve(right_hand_side);
  }


  //Collision detect (double template as AutoDiff doesnt work)
  VectorXd phi_collision;
  Matrix3Xd normal_collision, xA_collision, xB_collision;
  vector<int> idxA_collision, idxB_collision;
  

  // This is an ugly way of doing it. Change it later if a better method is available
  std::cout << const_cast<RigidBodyTree<double>&>(tree_).collisionDetect(
      k_cache_double, phi_collision, normal_collision, xA_collision, xB_collision, idxA_collision, idxB_collision);
  std::cout << std::endl;

  // 4 contacts for Cassie (2 in each toe)
  const int num_contacts = 4;

  //const Map<Matrix3Xd> normals(normal_collision.data(), 3, num_contacts);
  //vector<Map<Matrix3Xd>> tangents;
  //tree_.surfaceTangents(normals, tangents);







  VectorX<T> x_dot_sol(plant_->get_num_states());
  x_dot_sol << tree_.transformVelocityToQDot(k_cache, v), vdot;
  x_dot->SetFromVector(x_dot_sol);
}




}  // namespace dairlib

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::CassiePlant)
