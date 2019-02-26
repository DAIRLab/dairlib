#include "examples/Cassie/cassie_utils.h"
#include "common/find_resource.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/multibody/joints/revolute_joint.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"

#include "drake/multibody/tree/revolute_spring.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/parsers/urdf_parser.h"

namespace dairlib {

using Eigen::VectorXd;
using Eigen::Vector3d;
using drake::solvers::Constraint;
using drake::AutoDiffVecXd;
using drake::solvers::MathematicalProgram;
using drake::multibody::MultibodyPlant;
using drake::geometry::SceneGraph;
using drake::multibody::Parser;
using drake::multibody::RevoluteSpring;
using drake::multibody::joints::FloatingBaseType;

/// Add a fixed base cassie to the given multibody plant and scene graph
/// These methods are to be used rather that direct construction of the plant
/// from the URDF to centralize any modeling changes or additions
void addCassieMultibody(MultibodyPlant<double>* plant,
    SceneGraph<double>* scene_graph, bool floating_base, std::string filename) {
  std::string full_name = FindResourceOrThrow(filename);
  Parser parser(plant, scene_graph);
  parser.AddModelFromFile(full_name);

  plant->AddForceElement<drake::multibody::UniformGravityFieldElement>(
      -9.81 * Eigen::Vector3d::UnitZ());

  if (!floating_base) {
    plant->WeldFrames(
      plant->world_frame(), plant->GetFrameByName("pelvis"),
      drake::math::RigidTransform<double>(Vector3d::Zero()).GetAsIsometry3());
  }

  // Add springss
  // stiffness is 2300 in URDF, 1500 from gazebo
  plant->AddForceElement<RevoluteSpring>(
      dynamic_cast<const drake::multibody::RevoluteJoint<double>&>(
          plant->GetJointByName("knee_joint_left")),
      0, 1500);
  plant->AddForceElement<RevoluteSpring>(
      dynamic_cast<const drake::multibody::RevoluteJoint<double>&>(
          plant->GetJointByName("knee_joint_right")),
      0, 1500);
  plant->AddForceElement<RevoluteSpring>(
      dynamic_cast<const drake::multibody::RevoluteJoint<double>&>(
          plant->GetJointByName("ankle_spring_joint_left")),
      0, 1250);
  plant->AddForceElement<RevoluteSpring>(
      dynamic_cast<const drake::multibody::RevoluteJoint<double>&>(
          plant->GetJointByName("ankle_spring_joint_right")),
      0, 1250);

  // TOOO(mposa): add loop closures when implemented in Drake
}

std::unique_ptr<RigidBodyTree<double>> makeCassieTreePointer(
    std::string filename, FloatingBaseType base_type) {
  auto tree = std::make_unique<RigidBodyTree<double>>();
  buildCassieTree(*tree.get(), filename, base_type);
  return tree;
}

void buildCassieTree(RigidBodyTree<double>& tree, std::string filename,
                     FloatingBaseType base_type) {
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow(filename), base_type, &tree);

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

  tree.addDistanceConstraint(heel_spring_left, rod_on_heel_spring, thigh_left,
                             rod_on_thigh_left, achilles_length);

  tree.addDistanceConstraint(heel_spring_right, rod_on_heel_spring, thigh_right,
                             rod_on_thigh_right, achilles_length);

  // Add spring forces
  int body_index = tree.FindIndexOfChildBodyOfJoint("knee_joint_left");
  auto body = tree.get_mutable_body(body_index);
  RevoluteJoint& knee_joint_left =
      dynamic_cast<RevoluteJoint&>(body->get_mutable_joint());
  // stiffness is 2300 in URDF,these #s from gazebo
  knee_joint_left.SetSpringDynamics(1500.0, 0.0);

  body_index = tree.FindIndexOfChildBodyOfJoint("knee_joint_right");
  body = tree.get_mutable_body(body_index);
  RevoluteJoint& knee_joint_right =
      dynamic_cast<RevoluteJoint&>(body->get_mutable_joint());
  knee_joint_right.SetSpringDynamics(1500.0, 0.0);  // 2300 in URDF

  body_index = tree.FindIndexOfChildBodyOfJoint("ankle_spring_joint_left");
  body = tree.get_mutable_body(body_index);
  RevoluteJoint& ankle_spring_joint_left =
      dynamic_cast<RevoluteJoint&>(body->get_mutable_joint());
  ankle_spring_joint_left.SetSpringDynamics(1250.0, 0.0);  // 2000 in URDF

  body_index = tree.FindIndexOfChildBodyOfJoint("ankle_spring_joint_right");
  body = tree.get_mutable_body(body_index);
  RevoluteJoint& ankle_spring_joint_right =
      dynamic_cast<RevoluteJoint&>(body->get_mutable_joint());
  ankle_spring_joint_right.SetSpringDynamics(1250.0, 0.0);  // 2300 in URDF
}

VectorXd solvePositionConstraints(const RigidBodyTree<double>& tree,
                                  VectorXd q_init,
                                  std::vector<int> fixed_joints) {
  MathematicalProgram prog;
  auto q = prog.NewContinuousVariables(tree.get_num_positions(), "q");
  auto constraint = std::make_shared<TreePositionConstraint>(tree);
  prog.AddConstraint(constraint, q);
  for (uint i = 0; i < fixed_joints.size(); i++) {
    int j = fixed_joints[i];
    prog.AddConstraint(q(j) == q_init(j));
  }
  prog.AddQuadraticCost((q - q_init).dot(q - q_init));
  prog.SetInitialGuessForAllVariables(q_init);
  prog.Solve();
  return prog.GetSolution(q);
}

TreePositionConstraint::TreePositionConstraint(
    const RigidBodyTree<double>& tree, const std::string& description)
    : Constraint(tree.getNumPositionConstraints(), tree.get_num_positions(),
                 VectorXd::Zero(tree.getNumPositionConstraints()),
                 VectorXd::Zero(tree.getNumPositionConstraints()),
                 description) {
  tree_ = &tree;
}

void TreePositionConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                    Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  Eval(drake::math::initializeAutoDiff(x), &y_t);
  *y = drake::math::autoDiffToValueMatrix(y_t);
}

void TreePositionConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                    AutoDiffVecXd* y) const {
  const AutoDiffVecXd q = x.head(tree_->get_num_positions());
  KinematicsCache<drake::AutoDiffXd> cache = tree_->doKinematics(q);
  *y = tree_->positionConstraints(cache);
}

void TreePositionConstraint::DoEval(
    const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>& x,
    drake::VectorX<drake::symbolic::Expression>* y) const {
  throw std::logic_error(
      "TreePositionConstraint does not support symbolic evaluation.");
}

}  // namespace dairlib
